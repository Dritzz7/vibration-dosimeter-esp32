# Catatan Teknis: Pemrosesan Sensor & Analisis Metrik Getaran (*Vibration Analysis*)

Dokumen ini mengonsolidasikan seluruh catatan teknis, metodologi komputasi, penurunan rumus matematis, standar regulasi internasional (ISO 5349-1 dan ISO 2631-1), integrasi perangkat keras sensor ADXL345, serta ketahanan algoritma pasca-pemrosesan pada skrip `util/calc_dosimeter.py`.

---

## 1. Integrasi Perangkat Keras Sensor ADXL345

Pengukuran getaran dilakukan menggunakan akselerometer triaksial digital **Analog Devices ADXL345** pada antarmuka I2C.

### A. Pengkabelan Kritis Sensor (HAV Node & Main Unit)
Pada pengujian awal integrasi HAV Node, Serial Monitor sempat menampilkan error `No I2C devices found on bus!` meskipun jalur I2C berstatus normal. Ditemukan ketentuan pengkabelan wajib sebagai berikut:
1. **Pin CS (Chip Select)**:
   * **Wajib dihubungkan ke 3.3V**. Jika dibiarkan melayang (*floating*), chip ADXL345 dapat secara acak masuk ke mode SPI dan menonaktifkan transceiver I2C internal.
2. **Pin SDO (I2C Address Select)**:
   * Dihubungkan ke **GND** untuk menetapkan alamat primer `0x53` (atau ke **VCC** untuk alamat sekunder `0x1D`).
3. **Pin VCC Modul Sensor**:
   * Sebaiknya dihubungkan ke rel **5V / VIN** ESP32 jika modul sensor memiliki regulator onboard 3.3V (tipe RT9161/6206) untuk mencegah *undervoltage* sensor akibat penurunan tegangan beruntun (*double-regulation*).
4. **Mekanisme Simulasi Mandiri (*Automatic Fallback*)**:
   * Firmware HAV Node dilengkapi fallback otomatis: jika sensor fisik tidak terdeteksi, firmware mengaktifkan simulasi sinusoidal agar pengujian fungsionalitas nirkabel dan pengolahan data BLE tidak terhenti.

---

## 2. Metodologi Komputasi Metrik Getaran ISO

Data getaran yang direkam pada berkas `dosimeter.csv` merupakan data RMS hasil pembobotan frekuensi yang diagregasikan setiap epoch 1 detik (1 Hz).

### A. Hand-Arm Vibration (HAV — ISO 5349-1)
Mengevaluasi risiko paparan getaran yang ditransmisikan dari stang kemudi ke sistem tangan-lengan pekerja:
1. **Filter Frekuensi $W_h$**: Filter IIR biquad cascade diterapkan pada ketiga sumbu ($X, Y, Z$) pada laju sampling 3200 Hz.
2. **Total Vektor RMS Akselerasi ($a_{hv,eq}$)**:
   $$a_{hv,eq} = \sqrt{\frac{1}{N} \sum_{i=1}^{N} \left( a_{hwx,i}^2 + a_{hwy,i}^2 + a_{hwz,i}^2 \right)}$$
3. **Paparan Harian Ekivalen 8 Jam ($A_{HAV}(8)$)**:
   Dihitung berdasarkan prinsip kesetaraan energi kuadratik (*Root-Mean-Square Energy Equivalence*):
   $$A_{HAV}(8) = a_{hv,eq} \times \sqrt{\frac{T_{exp}}{28.800}} = \sqrt{\frac{\sum_{i=1}^{N} a_{hv,i}^2}{28.800}}$$
   di mana $T_{exp} = N \times 1\text{ detik}$ dan $T_0 = 8\text{ jam} = 28.800\text{ detik}$.
4. **Ambang Batas Keselamatan (ISO 5349-1 / EU Directive 2002/44/EC)**:
   * **EAV (*Exposure Action Value*) = $2{,}50\text{ m/s}^2$**: Batas di mana tindakan pengendalian teknis/administratif wajib dimulai.
   * **ELV (*Exposure Limit Value*) = $5{,}00\text{ m/s}^2$**: Batas mutlak harian yang tidak boleh dilampaui demi mencegah *Hand-Arm Vibration Syndrome* (HAVS) atau *Vibration White Finger* (VWF).

---

### B. Whole-Body Vibration (WBV — ISO 2631-1)
Mengevaluasi getaran dari jok/alas duduk yang disalurkan ke tulang belakang dan seluruh tubuh pengendara:
1. **Faktor Pengali Arah Postur Duduk (*Directional Weighting*)**:
   Sesuai ISO 2631-1 pasal 6.1, tubuh manusia pada posisi duduk jauh lebih rentan terhadap getaran horizontal ($X, Y$) dibanding vertikal ($Z$):
   * Sumbu $X$ (maju-mundur): $k_x = 1{,}4$ (filter $W_d$)
   * Sumbu $Y$ (kiri-kanan): $k_y = 1{,}4$ (filter $W_d$)
   * Sumbu $Z$ (vertikal)   : $k_z = 1{,}0$ (filter $W_k$)
2. **Sumbu Dominan Paparan Harian ($A_{WBV}(8)$)**:
   Berbeda dengan HAV yang menjumlahkan ketiga sumbu secara vektor spasial, evaluasi kesehatan WBV **wajib mengacu pada nilai sumbu tertinggi (dominan)**:
   $$A_x(8) = 1{,}4 \cdot a_{wx,eq} \sqrt{\frac{T_{exp}}{28.800}}, \quad A_y(8) = 1{,}4 \cdot a_{wy,eq} \sqrt{\frac{T_{exp}}{28.800}}, \quad A_z(8) = 1{,}0 \cdot a_{wz,eq} \sqrt{\frac{T_{exp}}{28.800}}$$
   $$A_{WBV}(8) = \max \left( A_x(8),\, A_y(8),\, A_z(8) \right)$$
3. **Vibration Dose Value ($VDV$) Metode Pangkat-4**:
   Diakumulasikan menggunakan integral pangkat empat untuk memberikan bobot penalti tinggi terhadap sentakan kejut (*shock/peaks*):
   $$VDV_{sumbu} = k_{sumbu} \times \left( \sum_{i=1}^N a_{w,i}^4 \times \Delta t \right)^{0{,}25}$$
   $$VDV_{total} = \left( VDV_x^4 + VDV_y^4 + VDV_z^4 \right)^{0{,}25}$$
4. **Estimated VDV ($eVDV$)**:
   Menurut ISO 2631-1 Lampiran B, jika getaran relatif stasioner (*low crest factor*), nilai dosis getaran dapat diestimasi langsung dari RMS akselerasi total $a_{v,eq}$:
   $$eVDV = 1{,}4 \times a_{v,eq} \times \left(T_{exp}\right)^{0{,}25} \quad [\text{m/s}^{1{,}75}]$$
5. **Ambang Batas Keselamatan WBV**:
   * **EAV = $0{,}50\text{ m/s}^2$** atau **$VDV = 9{,}1\text{ m/s}^{1{,}75}$**
   * **ELV = $1{,}15\text{ m/s}^2$** atau **$VDV = 21{,}0\text{ m/s}^{1{,}75}$**

---

## 3. Landasan Teoretis: Mengapa VDV Hanya Diterapkan pada WBV?

Penerapan metrik pangkat empat ($VDV$) secara eksklusif pada evaluasi WBV didasari oleh biomekanika tubuh dan ketentuan regulasi:

| Parameter | Hand-Arm Vibration (HAV) | Whole-Body Vibration (WBV) |
| :--- | :--- | :--- |
| **Standar Acuan** | ISO 5349-1 | ISO 2631-1 / ISO 8041 |
| **Organ Kritis** | Saraf perifer & mikrovaskular jari | Diskus intervertebralis tulang belakang |
| **Karakter Kerusakan** | Akumulasi energi jangka panjang (kelelahan vaskular) | Kejut mekanis instan & kompresi bantalan tulang |
| **Sistem Redaman Alami** | Otot lengan, siku, dan bahu bertindak sebagai suspensi alami | Beban aksial langsung diteruskan melalui tulang panggul |
| **Metode Evaluasi** | Cukup berbasis kuadratik RMS $\rightarrow \mathbf{A(8)}$ | Kuadratik $\mathbf{A(8)}$ + Pangkat-4 $\rightarrow \mathbf{VDV}$ |
| **Satuan Dosis** | $\text{m/s}^2$ | $\text{m/s}^{1{,}75}$ |

---

## 4. Penurunan Matematis "Time Allowed" (Batas Waktu Paparan)

Metrik prediktif **"Time allowed"** pada laporan analisis menjawab pertanyaan:
> *"Jika pekerja terus berkendara dengan tingkat intensitas getaran rata-rata saat ini ($a_{eq}$), berapa lama total waktu kerja yang diperbolehkan sebelum menyentuh batas EAV atau ELV?"*

### Penurunan Rumus:
Dari formula normalisasi paparan 8 jam:
$$\text{Limit} = a_{eq} \times \sqrt{\frac{T_{allowed}}{28.800}}$$

Kuadratkan kedua sisi:
$$\text{Limit}^2 = a_{eq}^2 \times \frac{T_{allowed}}{28.800}$$

Diperoleh formula waktu izin paparan ($T_{allowed}$):
$$T_{allowed} = 28.800 \times \left( \frac{\text{Limit}}{a_{eq}} \right)^2 \quad [\text{detik}]$$

* **Penerapan HAV**:
  $$T_{EAV, HAV} = 28.800 \times \left( \frac{2{,}50}{a_{hv,eq}} \right)^2, \quad T_{ELV, HAV} = 28.800 \times \left( \frac{5{,}00}{a_{hv,eq}} \right)^2$$
* **Penerapan WBV**:
  $$T_{EAV, WBV} = 28.800 \times \left( \frac{0{,}50}{a_{w,max}} \right)^2, \quad T_{ELV, WBV} = 28.800 \times \left( \frac{1{,}15}{a_{w,max}} \right)^2$$

---

## 5. Ketahanan Algoritma Pasca-Pemrosesan (`util/calc_dosimeter.py`)

Skrip utilitas pasca-pemrosesan dirancang tahan terhadap berbagai anomali berkas log akibat pencabutan daya atau pengujian lapangan:

1. **Durasi Berbasis Akumulasi Sampel**:
   Durasi paparan dihitung murni dari $T_{total} = N \times 1\text{ detik}$, bukan dari selisih waktu dinding (`t_akhir - t_awal`). Jika kartu SD dicabut selama 5 menit lalu dipasang lagi, jeda 5 menit tersebut tidak akan menggelembungkan atau mendistorsi nilai $A(8)$ dan $VDV$.
2. **Toleransi Terhadap Baris Rusak (*Truncated Rows*)**:
   Baris terpotong akibat pencabutan kartu di tengah operasi penulisan secara otomatis dilewati melalui validasi `if len(row) < 9: continue` dan blok `try-except ValueError`.
3. **Penyaringan Header Ganda (*Duplicate Headers*)**:
   Jika berkas berisi header baru di tengah data akibat restart sistem, baris tersebut otomatis diabaikan karena parsing `float(row[1])` memicu `ValueError`.
4. **Visualisasi Berkelanjutan**:
   Sumbu horizontal pada grafik 4-panel Matplotlib menggunakan indeks waktu terakumulasi (`np.arange(N)`), memastikan visualisasi mulus tanpa celah kosong ratusan jam.
