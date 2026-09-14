# Catatan Teknis: Subsistem Catu Daya & Manajemen Daya (*Main Power*)

Dokumen ini mengonsolidasikan seluruh catatan teknis, perhitungan beban (*power budget*), seleksi komponen regulator, mitigasi *brownout reset*, dan rangkaian proteksi kelistrikan untuk sistem dosimeter getaran (*HAV Node* dan *Main Unit*).

---

## 1. Perhitungan Kebutuhan Kapasitas Baterai (*Power Budget Calculation*)

Sistem dosimeter getaran dirancang untuk beroperasi secara kontinu **minimal selama 8 jam** sesuai standar durasi kerja harian internasional (ISO 5349-1 dan ISO 2631-1). Kedua subsistem menggunakan sumber daya mandiri berbasis sel **Lithium-Ion 18650 / Li-Po 3.7V**.

### A. HAV Node (Node Stang Kemudi)
* **Konsumsi Arus Rata-rata Komponen**:
  * ESP32 (Akuisisi 3200 Hz, komputasi filter IIR biquad $W_h$, transmisi BLE Server 1 Hz): $\approx 90\text{ mA} - 110\text{ mA}$
  * Sensor ADXL345: $\approx 0{,}14\text{ mA}$
  * **Total Arus Rata-rata ($I_{avg}$)**: $\approx \mathbf{100\text{ mA}}$
* **Perhitungan Kapasitas Teoretis (8 Jam)**:
  $$C = I_{avg} \times t = 100\text{ mA} \times 8\text{ jam} = 800\text{ mAh}$$
* **Dengan *Safety Margin* 25% (Faktor Penuaan & Efisiensi LDO)**:
  $$C_{rekomendasi} \ge 800\text{ mAh} \times 1{,}25 = \mathbf{1000\text{ mAh} - 1200\text{ mAh}}$$
* **Pilihan Bentuk Baterai**:
  * **Li-Po 3.7V 1000–1200 mAh** (Bentuk pipih/kotak berbobot $\approx 20 - 25$ gram, sangat ideal agar total bobot HAV Node tetap di bawah batas **50 gram** pada stang kemudi).
  * Atau **1x Sel Li-Ion 18650 (2000–2600 mAh)** jika menggunakan *housing* silinder standar (daya tahan $> 18 - 20$ jam).

---

### B. Main Unit (Node Jok Sepeda Motor)
* **Konsumsi Arus Rata-rata Komponen**:
  * ESP32 Dual-Core (Akuisisi WBV 200 Hz, BLE Central RX aktif, FreeRTOS scheduler): $\approx 120\text{ mA} - 140\text{ mA}$
  * Sensor ADXL345 (WBV): $\approx 0{,}14\text{ mA}$
  * Modul RTC DS3231: $\approx 0{,}20\text{ mA}$
  * Modul MicroSD (Operasi penulisan SPI 1 Hz & *flush*): $\approx 20\text{ mA} - 30\text{ mA}$
  * Layar OLED 1.3" SH1106: $\approx 15\text{ mA}$ saat aktif (turun menjadi $< 2\text{ mA}$ saat mode *screensaver* titik 1 Hz aktif)
  * **Total Arus Rata-rata ($I_{avg}$)**: $\approx \mathbf{150\text{ mA}} - \mathbf{170\text{ mA}}$
* **Perhitungan Kapasitas Teoretis (8 Jam)**:
  $$C = I_{avg} \times t = 160\text{ mA} \times 8\text{ jam} = 1280\text{ mAh}$$
* **Dengan *Safety Margin* 30% (Lonjakan Transien SPI SD Card & BLE RX)**:
  $$C_{rekomendasi} \ge 1280\text{ mAh} \times 1{,}30 = \mathbf{1700\text{ mAh} - 2200\text{ mAh}}$$
* **Pilihan Bentuk Baterai**:
  * **1x Sel Li-Ion 18650 (2200–3000 mAh, 3.7V)** (Sangat direkomendasikan dan pas dengan batas bobot maksimum **150 gram** di bawah jok). Mampu memberikan waktu operasional kontinu **12–15 jam**.

### Ringkasan Kapasitas Baterai:
| Unit | Arus Rata-rata ($I_{avg}$) | Kapasitas Min (8 Jam) | Rekomendasi Baterai | Estimasi Durasi Nyata |
| :--- | :---: | :---: | :---: | :---: |
| **HAV Node** (Stang) | $\approx 100\text{ mA}$ | **1000 mAh** | Li-Po 3.7V 1000–1200 mAh / 18650 2000 mAh | **10–18 Jam** |
| **Main Unit** (Jok) | $\approx 160\text{ mA}$ | **1800 mAh** | 1x Li-Ion 18650 (2200–2600 mAh) | **12–15 Jam** |

---

## 2. Kriteria & Seleksi Regulator Tegangan LDO (*Low-Dropout*)

Untuk mencatu daya sistem ESP32 dari baterai Lithium (3,7V nominal, rentang 4,2V penuh s.d. 3,3V habis), regulator yang digunakan harus berkinerja tinggi. **Regulator linier umum seperti AMS1117-3.3 TIDAK COCOK untuk baterai Lithium.**

### 🔍 4 Kriteria Utama Regulator:
1. **Tegangan Jatuh Sangat Rendah (*Ultra-Low Dropout Voltage*)**:
   * AMS1117 memiliki *dropout* tinggi **1,1V – 1,3V**. Jika baterai turun ke 3,7V, output anjlok ke $3{,}7\text{ V} - 1{,}2\text{ V} = 2{,}5\text{ V}$ $\rightarrow$ ESP32 langsung mati (*brownout*) padahal kapasitas baterai masih tersisa 70%!
   * LDO ideal membutuhkan *dropout* $\le \mathbf{150\text{ mV} - 250\text{ mV}}$ pada arus beban penuh, menjaga output stabil **3,3V** hingga baterai mendekati **3,45V – 3,40V** (>90% kapasitas terpakai).
2. **Kapasitas Arus Puncak Cukup (*Peak Current $\ge 500\text{ mA} - 1\text{ A}$*)**:
   * ESP32 menarik arus kontinu 100–160 mA, namun transmisi radio BLE (*burst transmission*) atau penulisan MicroSD memicu lonjakan transien hingga **350 mA – 500 mA**.
   * Regulator harus mampu memasok arus kontinu minimal **500 mA – 600 mA** (arus puncak 800 mA – 1A).
3. **Arus Diam Sangat Rendah (*Ultra-Low Quiescent Current / $I_q$*)**:
   * $I_q$ harus kecil ($< \mathbf{10\text{ µA} - 60\text{ µA}}$). Sebagai perbandingan, AMS1117 membuang arus diam hingga 5–10 mA secara sia-sia.
4. **Derau Rendah (*Low Output Noise & High PSRR*)**:
   * Sensor akselerometer ADXL345 sangat sensitif terhadap riak (*ripple*). LDO linier menghasilkan tegangan DC bersih tanpa derau *switching*.

### 🏆 Evaluasi IC Regulator:
| Tipe IC Regulator | Arus Maks ($I_{out}$) | *Dropout Voltage* ($V_{drop}$) | Arus Diam ($I_q$) | Catatan Penggunaan |
| :--- | :---: | :---: | :---: | :--- |
| **AP2112K-3.3** *(Sangat Direkomendasikan)* | **600 mA** | **250 mV** @ 600 mA | $\approx 55\text{ µA}$ | Standar modul ESP32 komersial (Adafruit Feather). Sangat stabil menyuplai BLE & SD Card. |
| **XC6220B331MR** | **1000 mA (1A)** | **150 mV** @ 1A | $\approx 8\text{ µA}$ | Efisiensi tinggi dengan *dropout* ultra-rendah dan kapasitas arus besar. |
| **MCP1700-3302E** | **250 mA** | **178 mV** @ 250 mA | **1,6 µA** | Sangat hemat daya; **wajib elco 100–470 µF** pada output 3.3V untuk menyerap lonjakan BLE. |
| **RT9013-33GB / ME6211** | **500 mA** | **250 mV** @ 500 mA | $\approx 40\text{ µA}$ | Murah, ringkas (SOT-23-5), dan mudah didapatkan di pasaran lokal. |

---

## 3. Investigasi & Mitigasi *Brownout Reset* pada ESP32

### A. Gejala & Pemicu Utama
* **Gejala**: ESP32 mengalami *infinite loop reset* berulang setelah booting atau saat mulai menyalakan radio BLE.
* **Pesan Error Serial**: `Brownout detector was triggered`.
* **Pemicu**: Pengaktifan subsistem nirkabel BLE (`BLEDevice::init()`) menarik lonjakan arus (*current spike*) mendadak sebesar **150 mA – 250 mA**, menyebabkan penurunan tegangan sesaat (*voltage dip*) di bawah ambang komparator internal ESP32 (2,43V – 2,80V).

### B. Analisis Variasi Batch Papan ESP32-WROOM-32D
Perbedaan perilaku antar papan yang tampak identik (satu normal, satu memicu brownout) disebabkan oleh variansi manufaktur perangkat keras:
1. **Kualitas IC LDO Onboard**: Papan kloning sering menggunakan LDO murah (ME6211 150-300 mA) alih-alih AMS1117-3.3 original (800-1000 mA).
2. **Kapasitor Decoupling Onboard Minim**: Pemangkasan kapasitor tantalum/elco pada jalur catu daya 3.3V modul untuk menekan biaya produksi.
3. **Toleransi Silikon eFuse Brownout**: Ambang batas komparator brownout ESP32 memiliki rentang toleransi pabrik **2,43V hingga 2,80V**.
4. **Jatuh Tegangan Dioda Schottky USB**: Dioda proteksi USB dengan $V_f \approx 0{,}6\text{ V}$ menurunkan tegangan masukan regulator ke $\approx 4{,}4\text{ V}$, mempercepat terjadinya kondisi *dropout*.

### C. Solusi Pemecahan Masalah
1. **Solusi Perangkat Keras (Rekomendasi Utama)**:
   * Pasang kapasitor elektrolit (elco) / tantalum **100 µF – 470 µF (16V Low-ESR)** paralel di antara pin **`3V3`** dan **`GND`** sedekat mungkin dengan modul ESP32.
   * Gunakan kabel USB pendek berkualitas baik pada port USB 3.0 / adaptor catu daya 5V 2A eksternal.
2. **Solusi Perangkat Lunak (Bypass Darurat)**:
   Menonaktifkan sirkuit pendeteksi brownout pada baris pertama fungsi `setup()` di `main.cpp`:
   ```cpp
   #include "soc/soc.h"
   #include "soc/rtc_cntl_reg.h"

   void setup() {
       WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Nonaktifkan detektor brownout
       // ... inisialisasi sistem ...
   }
   ```
   *(Catatan: Metode software bypass ini berisiko membuat ESP32 hang jika tegangan catu daya anjlok terlalu dalam).*

---

## 4. Rangkaian Tambahan Pendukung (*Hardware Support Circuitry*)

Untuk menjamin keandalan sistem di lingkungan lapangan sepeda motor:

### 1. 🔋 Rangkaian Pengisian & Proteksi Baterai
* **Komponen**: Modul **TP4056** terintegrasi IC proteksi **DW01** dan Dual MOSFET **FS8205A** (port USB Type-C).
* **Fungsi Proteksi**:
  * *Over-Charge*: Memutus pengisian saat baterai mencapai 4,20V ± 0,05V.
  * *Over-Discharge*: Memutus beban otomatis saat baterai turun ke 2,4V – 2,5V agar sel Lithium tidak rusak permanen.
  * *Over-Current & Short-Circuit*: Memutus arus saat terjadi hubung singkat (> 3A).

### 2. 📊 Rangkaian Pembaca Level Baterai (*Voltage Divider*)
* **Skema**: Pembagi tegangan resistor 1:2 dari pin positif baterai ke pin ADC ESP32 (**GPIO 34** — ADC1, aman dari interferensi RF BLE).
* **Komponen**:
  * $R_1 = 100\text{ k}\Omega$ (1%) dan $R_2 = 100\text{ k}\Omega$ (1%) membagi tegangan baterai maks (4,2V $\rightarrow$ 2,1V). Arus bocor pemantau sangat minim ($< 21\text{ µA}$).
  * Kapasitor keramik **$100\text{ nF}$ (0,1 µF)** paralel dengan $R_2$ ke GND sebagai peredam derau ADC.

### 3. ⚡ Rangkaian Filter Decoupling (*Power Rail Filtering*)
1. **Bulk Capacitor**: Kapasitor elco **$100\text{ µF} - 470\text{ µF}$** antara `3V3` dan `GND`.
2. **High-Frequency Bypass**: Kapasitor keramik MLCC **$10\text{ µF}$** dan **$0{,}1\text{ µF}$** paralel.
3. **Isolasi Sensor (Opsional)**: *Ferrite Bead* ($100\ \Omega\ @\ 100\text{ MHz}$) pada jalur $V_{CC}$ ADXL345.

### 4. 🔌 Rangkaian Pull-Up & Stabilisasi Bus I2C / SPI
* **Pull-Up I2C (SDA & SCL)**: Resistor pull-up **$2{,}2\text{ k}\Omega - 4{,}7\text{ k}\Omega$** dari SDA (GPIO 21) dan SCL (GPIO 22) ke jalur `3V3`.
* **Pull-Up Chip Select (CS)**: Resistor **$10\text{ k}\Omega$** ke `3V3` pada pin CS MicroSD (GPIO 5) dan CS ADXL345 untuk mencegah pin melayang (*floating*) saat booting.

### 5. 🛡️ Rangkaian Sakelar & Proteksi Statis (*ESD*)
* **Sakelar Daya**: Sakelar geser (*slide switch*) mini pada jalur positif baterai atau tombol *soft-power* MOSFET P-Channel (AO3401).
* **Proteksi ESD**: Dioda TVS (*USBLC6-2SC6* / *ESD5V0*) pada port USB dan tombol fisik HMI guna mencegah lonjakan elektrostatik dari sentuhan tangan saat berkendara.

### Ringkasan Prioritas Rangkaian:
| No | Rangkaian Pendukung | Urgensi | Manfaat Utama |
| :---: | :--- | :---: | :--- |
| **1** | **TP4056 + DW01 Protection** | **Wajib** | Pengisian aman & pencegahan baterai rusak/meledak. |
| **2** | **Kapasitor Elco 100–470 µF (3V3-GND)** | **Wajib** | Mengeliminasi 100% masalah *Brownout Reset* akibat BLE. |
| **3** | **Pembagi Tegangan ADC ($100\text{ k}\Omega + 100\text{ k}\Omega$)** | **Sangat Dianjurkan** | Pemantauan persentase baterai & peringatan *Low Battery*. |
| **4** | **Pull-Up I2C ($4{,}7\text{ k}\Omega$) & SPI CS ($10\text{ k}\Omega$)** | **Sangat Dianjurkan** | Menjaga integritas sinyal sensor ADXL345 dan MicroSD. |
| **5** | **Dioda TVS / Proteksi ESD** | **Opsional** | Perlindungan elektrostatik di lingkungan operasional lapangan. |
