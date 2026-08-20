# Laporan Teknis & Analisis Dosimeter Getaran
**Proyek**: Wearable Vibration Dosimeter ESP32 (ISO 5349-1 HAV & ISO 2631-1 WBV)  
**Tanggal**: 20 Agustus 2026  

---

## 1. Panduan Analisis Laporan Skrip Utilitas (`util/calc_dosimeter.py`)

Skrip utilitas pasca-pemrosesan membaca berkas log `dosimeter.csv` (1 Hz) dan menghasilkan laporan terstruktur yang mengevaluasi risiko paparan getaran harian sesuai standar ISO 5349-1 dan ISO 2631-1.

### A. Informasi Umum Sesi (*General Information*)
* **Total Logged Time**: Menunjukkan durasi riil pengukuran (contoh: `00h 00m 54s` = 54 sampel data pada laju 1 Hz).
* **Catatan Durasi**: Pada durasi pengukuran yang singkat, nilai paparan harian $A(8)$ akan tampak kecil karena dinormalisasi terhadap 8 jam (28.800 detik). Parameter *Time allowed* (waktu aman) memberikan estimasi batas waktu operasional jika tingkat getaran rata-rata konstan seperti saat pengukuran.

---

### B. Hand-Arm Vibration (HAV - ISO 5349-1)
Bagian ini mengevaluasi risiko getaran yang disalurkan melalui stang kemudi ke telapak tangan dan lengan pengendara.

1. **RMS Percepatan per Sumbu ($a_{hwx,eq}, a_{hwy,eq}, a_{hwz,eq}$)**:
   * Menunjukkan rata-rata kuadrat percepatan terbobot frekuensi $W_h$ pada sumbu X (maju-mundur), Y (kiri-kanan), dan Z (atas-bawah).
2. **Total Vektor RMS ($a_{hv,eq}$)**:
   * Total energi getaran gabungan ketiga sumbu secara spasial:
     $$a_{hv,eq} = \sqrt{a_{hwx,eq}^2 + a_{hwy,eq}^2 + a_{hwz,eq}^2}$$
3. **Paparan Harian Ekivalen 8 Jam ($A_{HAV}(8)$)**:
   * Dihitung berdasarkan prinsip kesetaraan energi kuadrat:
     $$A_{HAV}(8) = a_{hv,eq} \sqrt{\frac{T_{total}}{28800}}$$
4. **Evaluasi Ambang Batas Keselamatan (ISO 5349-1 / EU Directive 2002/44/EC)**:
   * **EAV (*Exposure Action Value*) = 2,50 m/s²**: Batas di mana tindakan pengendalian risiko harus mulai diimplementasikan.
     * *Time allowed to EAV*: Batas durasi kerja harian sebelum menyentuh nilai 2,50 m/s².
   * **ELV (*Exposure Limit Value*) = 5,00 m/s²**: Batas mutlak harian yang tidak boleh dilampaui demi mencegah *Hand-Arm Vibration Syndrome* (HAVS) / *Vibration White Finger* (VWF).
     * *Time allowed to ELV*: Batas waktu berkendara maksimal sebelum melewati batas bahaya.

---

### C. Whole-Body Vibration (WBV - ISO 2631-1)
Bagian ini mengevaluasi getaran dari jok/alas duduk yang disalurkan ke tulang belakang dan seluruh tubuh pengendara.

1. **Pembobotan Arah (*Directional Weighting*)**:
   * Sesuai ISO 2631-1 untuk posisi duduk, tubuh manusia lebih rentan terhadap getaran horizontal (sumbu X dan Y) dibanding vertikal (sumbu Z).
   * Faktor pengali arah: $k_x = 1{,}4$, $k_y = 1{,}4$, dan $k_z = 1{,}0$.
2. **Sumbu Dominan Paparan Harian ($A_{WBV}(8)$)**:
   * Berbeda dengan HAV yang menggunakan penjumlahan vektor, evaluasi risiko kesehatan WBV **wajib mengacu pada nilai sumbu tertinggi (dominan)**:
     $$A_{WBV}(8) = \max \left( A_x(8),\, A_y(8),\, A_z(8) \right)$$
3. **Vibration Dose Value ($VDV$) Metode Pangkat-4**:
   * Mengakumulasikan getaran berbasis integral pangkat empat:
     $$VDV = \left( \int_0^T a_w(t)^4 \, dt \right)^{1/4}$$
   * Sangat sensitif terhadap guncangan mendadak (*shock/peaks*) seperti jalan berlubang atau polisi tidur.
4. **Evaluasi Ambang Batas Keselamatan (ISO 2631-1)**:
   * **A(8) EAV = 0,50 m/s²** atau **VDV EAV = 9,1 m/s¹·⁷⁵**.
   * **A(8) ELV = 1,15 m/s²** atau **VDV ELV = 21,0 m/s¹·⁷⁵**.

---

## 2. Mengapa Kalkulasi VDV Hanya Diperuntukkan untuk WBV?

Penerapan *Vibration Dose Value* (VDV) secara eksklusif pada evaluasi WBV didasari oleh tiga landasan utama:

### A. Mekanisme Biomekanika & Fisiologi Tubuh Manusia
* **Pada WBV (Tulang Belakang & Organ Dalam)**:
  * Getaran dan kejutan mekanis langsung ditransmisikan melalui tulang panggul ke diskus intervertebralis tulang belakang (*spine*).
  * Beban kejut tunggal yang tinggi (*high crest factor*) dapat menyebabkan mikrotrauma mekanis seketika pada bantalan tulang belakang.
  * Metode kuadrat RMS ($A(8)$) meratakan (*underestimate*) dampak kejut singkat yang destruktif ini, sehingga **metode pangkat-4 pada VDV diperlukan untuk memberikan penalti bobot yang sangat besar terhadap puncak kejut**.
* **Pada HAV (Tangan & Lengan)**:
  * Fleksibilitas otot lengan, sendi pergelangan, dan siku berfungsi sebagai sistem suspensi/peredam alami (*biomechanical damper*).
  * Patologi klinis HAV (seperti kerusakan mikrovaskular dan neuropati perifer pada jari) berkorelasi langsung dengan **akumulasi total energi getaran kontinu jangka panjang**, bukan oleh kejut tunggal sesaat. Oleh karena itu, kesetaraan energi kuadrat ($A(8)$) sudah memadai dan paling representatif.

### B. Ketentuan Standar Regulasi Internasional
* **ISO 5349-1 (HAV)**: Hanya mendefinisikan satu metrik tunggal berbasis kuadrat ($A(8)$) dan tidak mendefinisikan rumus maupun batas VDV.
* **ISO 2631-1 & ISO 8041 (WBV)**: Secara eksplisit menetapkan VDV sebagai metode evaluasi wajib jika getaran memiliki faktor puncak (*crest factor*) $> 9$ atau mengandung kejut berulang.

### C. Ringkasan Perbandingan

| Parameter | Hand-Arm Vibration (HAV) | Whole-Body Vibration (WBV) |
| :--- | :--- | :--- |
| **Standar Acuan** | ISO 5349-1 | ISO 2631-1 / ISO 8041 |
| **Organ Kritis** | Saraf perifer & pembuluh darah kapiler jari | Diskus intervertebralis tulang belakang |
| **Karakter Kerusakan** | Akumulasi energi getaran jangka panjang | Kejut mekanis dan resonansi organ dalam |
| **Metode Evaluasi Utama** | Kesetaraan Energi Kuadrat $\rightarrow \mathbf{A(8)}$ | Kesetaraan Energi Kuadrat $\rightarrow \mathbf{A(8)}$ |
| **Metode Tambahan (Kejut)** | *Tidak Diterapkan* | **Pangkat-4 $\rightarrow \mathbf{VDV}$** |
| **Satuan Dosis** | $\text{m/s}^2$ | $\text{m/s}^{1{,}75}$ |

---

## 3. Analisis Variasi Perilaku *Brownout Detector* pada Papan ESP-32D

Perbedaan perilaku antara dua papan modul ESP32-WROOM-32D yang identik (satu normal, satu memicu *brownout reset*) adalah fenomena klasik **variansi batch manufaktur perangkat keras (*hardware component variance*)**.

### A. Faktor Penyebab Utama

1. **Perbedaan Kualitas IC Regulator LDO 3.3V Onboard**:
   * Papan pertama kemungkinan menggunakan IC LDO berkualitas tinggi (misal: AMS1117-3.3 original) dengan kapasitas arus puncak hingga 800 mA - 1000 mA.
   * Papan kedua sering kali menggunakan IC LDO kloningan murah atau tipe berkapasitas rendah (misal: ME6211 150–300 mA). Saat modul radio BLE aktif (`BLEDevice::init()`), terjadi lonjakan arus mendadak (*transient current spike*) sebesar 150–250 mA yang menyebabkan penurunan tegangan sesaat (*voltage dip*) di bawah 2,43V–2,80V.
2. **Kapasitor Penyaring (*Decoupling Capacitor*) Onboard yang Minim**:
   * Papan kloning sering kali memangkas kapasitor filter elektrolit/tantalum pada jalur catu daya 3.3V untuk menekan biaya produksi, sehingga tidak ada tandon energi lokal untuk menyangga lonjakan arus RF.
3. **Toleransi Pabrik Ambang Deteksi Silikon (*eFuse Brownout Threshold*)**:
   * Ambang batas komparator brownout pada silikon ESP32 memiliki toleransi pabrik berkisar antara **2,43V hingga 2,80V**. Papan kedua dapat memiliki ambang batas yang sedikit lebih sensitif dibanding papan pertama.
4. **Jatuh Tegangan (*Forward Voltage Drop*) Dioda Proteksi USB**:
   * Dioda Schottky pada jalur daya USB dengan jatuh tegangan lebih besar ($V_f \approx 0{,}6\text{ V}$) menyebabkan tegangan input regulator LDO turun menjadi $\approx 4{,}4\text{ V}$, mempercepat terjadinya kondisi *dropout*.

---

### B. Solusi yang Diterapkan

#### 1. Solusi Perangkat Lunak (*Software Bypass*)
Menonaktifkan sirkuit detektor brownout pada baris pertama fungsi `setup()` di `HAV-Node/src/main.cpp`:
```cpp
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Nonaktifkan detektor brownout
    // ... inisialisasi sensor dan BLE ...
}
```

#### 2. Solusi Perangkat Keras (*Hardware Fix - Direkomendasikan*)
* **Kapasitor Penyangga**: Pasang kapasitor elektrolit (elco) **100 µF – 470 µF (16V)** secara paralel di antara pin **`3V3`** dan **`GND`** sedekat mungkin dengan modul ESP32.
* **Catu Daya Eksternal**: Gunakan port USB 3.0 / adaptor *charger* 5V 2A dengan kabel data resistansi rendah untuk memastikan tegangan suplai stabil.
