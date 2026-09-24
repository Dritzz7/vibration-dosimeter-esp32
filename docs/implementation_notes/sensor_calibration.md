# Catatan Implementasi: Kalibrasi Sensor Getaran (RION VE-10)

Dokumen ini menjelaskan metodologi, formulasi matematis, prosedur pengujian praktis, serta penerapan faktor koreksi kalibrasi pada sensor percepatan **ADXL345** menggunakan kalibrator getaran standar **RION VE-10** untuk **HAV Node** dan **Main Unit Vibration Dosimeter**.

---

## 1. Spesifikasi Kalibrator Standar (RION VE-10)

Kalibrator getaran elektromekanis **RION VE-10** menghasilkan sinyal sinusoidal murni dengan parameter acuan metrologi standar:

| Parameter | Nilai Acuan Metrologi | Catatan Teknis |
| :--- | :---: | :--- |
| **Frekuensi Eksitasi ($f_0$)** | **159,2 Hz** (±1%) | Tepat setara dengan kecepatan sudut $\omega = 1000\text{ rad/s}$ ($2\pi \times 159{,}155$). |
| **Amplitudo Percepatan ($a_{\text{ref}}$)** | **10,00 m/s² RMS** | Setara dengan **1,020 g RMS** (nilai efektif *Root Mean Square*). |
| **Amplitudo Puncak (*Peak Value*)** | **14,14 m/s² Peak** | Secara teoretis $a_{\text{peak}} = a_{\text{RMS}} \times \sqrt{2}$ (1,442 g Peak). |
| **Kecepatan Getaran (*Velocity*)** | **10 mm/s RMS** | Sesuai hubungan $v = a / \omega = 10 / 1000\text{ m/s}$. |
| **Perpindahan Getaran (*Displacement*)** | **10 µm RMS** | Sesuai hubungan $d = a / \omega^2 = 10 / 10^6\text{ m}$. |
| **Beban Maksimum Transduser** | **&le; 70 gram** | Sensor ADXL345 *breakout board* (~3–5 gram) sangat ringan dan ideal. |

---

## 2. Landasan Teori & Pemisahan Komponen DC (Gravitasi Bumi)

### A. Masalah Komponen DC Statis
Saat sensor ADXL345 dipasang pada dudukan silinder kalibrator, sensor berada di bawah medan gravitasi bumi ($1\text{ g} \approx 9{,}807\text{ m/s}^2$). Sumbu penginderaan yang sejajar dengan arah getar kalibrator (misal sumbu Z vertikal) akan membaca superposisi dua sinyal sekaligus:
1. **Sinyal AC Dinamis:** Getaran sinusoidal murni kalibrator dengan amplitudo acuan $10{,}00\text{ m/s}^2\text{ RMS}$.
2. **Sinyal DC Statis:** Komponen proyeksi percepatan gravitasi bumi $g_{\text{proj}}$ ditambah *zero-g offset* bawaan silikon sensor.

Jika nilai RMS dihitung langsung dari sinyal total mentah ($a_{\text{raw}}$), nilai RMS yang diperoleh akan mengalami galat bias yang sangat besar:
$$a_{\text{RMS, mentah}} = \sqrt{a_{\text{DC}}^2 + a_{\text{AC, RMS}}^2} = \sqrt{(9{,}81)^2 + (10{,}00)^2} \approx \mathbf{14{,}01\text{ m/s}^2} \quad (\text{Galat } +40\%!)$$

### B. Solusi Matematika: Eliminasi Komponen DC
Komponen rata-rata DC ($\bar{a}$) harus dihitung dan dieliminasi terlebih dahulu untuk setiap $N$ sampel data akuisisi:
$$\bar{a} = \frac{1}{N}\sum_{i=1}^{N} a_i$$

Sinyal getaran murni yang diisolasi adalah:
$$a_{\text{AC}, i} = a_i - \bar{a}$$

Maka nilai percepatan getaran RMS yang sebenarnya adalah nilai standar deviasi dari sinyal terukur:
$$a_{\text{vib, RMS}} = \sqrt{\frac{1}{N}\sum_{i=1}^{N} (a_{\text{AC}, i})^2} = \sqrt{\frac{1}{N}\sum_{i=1}^{N} (a_i - \bar{a})^2} = \operatorname{std}(a)$$

### C. Penentuan Faktor Pengali Kalibrasi ($K_{\text{cal}}$)
Faktor koreksi kalibrasi ($K_{\text{cal}}$) dihitung dengan membandingkan nilai referensi kalibrator terhadap nilai RMS dinamik yang diukur oleh sensor:
$$K_{\text{cal}} = \frac{a_{\text{ref, RMS}}}{a_{\text{vib, RMS}}} = \frac{10{,}000\text{ m/s}^2}{a_{\text{vib, RMS}}}$$

Persentase galat awal sensor (*uncalibrated error*):
$$\text{Error} (\%) = \left(\frac{a_{\text{vib, RMS}} - a_{\text{ref, RMS}}}{a_{\text{ref, RMS}}}\right) \times 100\%$$

---

## 3. Fitur Kalibrasi Otomatis pada `Sensor-Visualizer`

Firmware [`tests/Sensor-Visualizer/src/main.cpp`](file:///d:/Users/Rafi%20Ananta%20Alden/Documents/Kuliah/Semester%208/EL4060/vibration-dosimeter-esp32/tests/Sensor-Visualizer/src/main.cpp) telah dilengkapi dengan fungsi kalibrasi presisi tinggi: `runRionCalibrationRoutine()`.

### Mekanisme Kerja Internal Firmware:
1. **Konfigurasi Sampling Presisi:**
   Firmware otomatis menaikkan laju sampling (ODR) ADXL345 ke **$1600\text{ Hz}$** ($10\times$ lipat frekuensi eksitasi $159{,}2\text{ Hz}$) untuk menjamin margin Nyquist yang sangat tinggi dan bebas distorsi.
2. **Akuisisi Deterministik:**
   Mengakuisisi tepat **$3200\text{ sampel}$** (durasi $2{,}0\text{ detik}$, mewakili $\approx 318$ siklus gelombang penuh $159{,}2\text{ Hz}$).
3. **Pemisahan DC & Kalkulasi RMS 3 Sumbu:**
   Menghitung $\text{DC Mean}$, $\text{AC RMS}$, dan $\text{AC Peak}$ secara simultan pada sumbu X, Y, dan Z.
4. **Verifikasi Frekuensi Riil:**
   Mendeteksi jumlah perlintasan titik nol (*zero-crossings*) untuk memvalidasi bahwa getaran yang diterima sensor benar-benar berada di kisaran $159{,}2\text{ Hz}$.
5. **Pemeriksaan *Crest Factor*:**
   Menghitung rasio $C_f = a_{\text{peak}} / a_{\text{RMS}}$ (sinus murni bernilai $\approx 1{,}41$). Jika $C_f$ mendekati 1.41, pemasangan sensor dipastikan kokoh tanpa resonansi liar atau pergeseran mekanis (*mechanical looseness*).
6. **Ekstraksi Kode Otomatis:**
   Menghasilkan baris kode `#define CAL_FACTOR_...` yang siap disalin langsung ke firmware `HAV-Node` atau `Main-Unit`.

---

## 4. Prosedur Praktis Pelaksanaan Kalibrasi

### Langkah 1: Persiapan Perangkat Keras
1. Rekatkan atau pasang sensor ADXL345 (Main Unit atau HAV Node) secara kokoh pada silinder dudukan kalibrator RION VE-10 menggunakan sekrup ulir tengah atau pita perekat berkekuatan tinggi (*high-stiffness stud/mounting tape*).
2. Hubungkan pin I2C sensor (SDA, SCL, VCC 3.3V, GND) ke ESP32.
3. Hubungkan ESP32 ke PC melalui kabel USB.

### Langkah 2: Menjalankan Program Pengujian
Buka terminal pada direktori proyek dan jalankan:
```powershell
python tests/Sensor-Visualizer/visualizer.py
```
*(Atau buka Serial Monitor: `pio device monitor -d tests/Sensor-Visualizer -b 921600`)*.

### Langkah 3: Menjalankan Rutinitas Kalibrasi
1. Tekan tombol pada kalibrator RION VE-10 (tekan 1 kali untuk operasi 1 menit). Kalibrator akan bergetar pada $159{,}2\text{ Hz}$.
2. Pada antarmuka `visualizer.py` (atau serial monitor), tekan tombol keyboard:
   ```text
   c
   ```
3. Sistem akan menghentikan streaming sejenak dan melakukan akuisisi presisi selama 2 detik.
4. Periksa sertifikat kalibrasi yang langsung tercetak pada terminal:

```text
================================================================
   RION VE-10 STANDARD VIBRATION CALIBRATION ROUTINE            
   Standard Reference: 159.2 Hz (1000 rad/s) @ 10.00 m/s^2 RMS  
================================================================
# [CAL] Preparing high-precision sampling (ODR = 1600 Hz)...
# [CAL] Please ensure RION VE-10 is mounted firmly and vibrating!
# [CAL] Sampling 3200 points (2.0 seconds duration)...

----------------------------------------------------------------
          RION VE-10 CALIBRATION CERTIFICATE & RESULTS          
----------------------------------------------------------------
 Active Dominant Axis  : Axis Z
 Estimated Frequency   : 159.0 Hz  (Target Ref: 159.2 Hz)
 Reference Standard    : 10.000 m/s^2 (RMS)
 Measured Dynamic RMS  : 9.728 m/s^2
 Measured Dynamic Peak : 13.785 m/s^2  (Crest Factor: 1.42, Ideal Sine: 1.41)
 Measured Static DC    : +9.814 m/s^2 (Earth Gravity Component)
 Initial Sensor Error  : -2.72 %
----------------------------------------------------------------
 >>> RECOMMENDED CORRECTION FACTOR (K_cal) = 1.0280 <<<
----------------------------------------------------------------
 Summary per Axis:
   Axis X: RMS =  0.112 m/s^2 | DC = -0.045 m/s^2 | Peak =  0.198
   Axis Y: RMS =  0.098 m/s^2 | DC = +0.120 m/s^2 | Peak =  0.155
   Axis Z: RMS =  9.728 m/s^2 | DC = +9.814 m/s^2 | Peak = 13.785
----------------------------------------------------------------
 C++ Code Snippet for HAV-Node & Main-Unit firmware:

#define CAL_FACTOR_Z  1.0280f

================================================================
```

5. Ulangi prosedur untuk sumbu X dan Y dengan mengubah orientasi pemasangan sensor pada kalibrator jika diperlukan.

---

## 5. Penerapan Hasil Kalibrasi pada Firmware Produksi

Nilai faktor koreksi $K_{\text{cal}}$ yang diperoleh dari pengujian dimasukkan ke dalam firmware sistem utama:

### A. Pada `HAV-Node/src/main.cpp`
Tambahkan definisi faktor kalibrasi di bawah konstanta ADXL345:
```cpp
// Factory calibration factors (RION VE-10: 159.2 Hz @ 10.00 m/s^2 RMS)
#define CAL_FACTOR_X  1.0000f
#define CAL_FACTOR_Y  1.0000f
#define CAL_FACTOR_Z  1.0280f // Nilai dari hasil kalibrasi
```
Dan terapkan pada fungsi konversi percepatan:
```cpp
ax = rawX * ADXL_SCALE_G_PER_LSB * G_TO_MPS2 * CAL_FACTOR_X;
ay = rawY * ADXL_SCALE_G_PER_LSB * G_TO_MPS2 * CAL_FACTOR_Y;
az = rawZ * ADXL_SCALE_G_PER_LSB * G_TO_MPS2 * CAL_FACTOR_Z;
```

### B. Pada `Main-Unit/src/main.cpp`
Tambahkan definisi yang sama untuk sensor Whole-Body Vibration (WBV):
```cpp
// Factory calibration factors (RION VE-10: 159.2 Hz @ 10.00 m/s^2 RMS)
#define ADXL_CAL_FACTOR_X  1.0000f
#define ADXL_CAL_FACTOR_Y  1.0000f
#define ADXL_CAL_FACTOR_Z  1.0280f // Nilai dari hasil kalibrasi
```
Dan terapkan pada fungsi konversi pembacaan WBV:
```cpp
ax = rawX * ADXL_SCALE_G_PER_LSB * G_TO_MPS2 * ADXL_CAL_FACTOR_X;
ay = rawY * ADXL_SCALE_G_PER_LSB * G_TO_MPS2 * ADXL_CAL_FACTOR_Y;
az = rawZ * ADXL_SCALE_G_PER_LSB * G_TO_MPS2 * ADXL_CAL_FACTOR_Z;
```

---

## 6. Ringkasan Validitas Kalibrasi untuk HAV vs WBV

* **HAV Node (ISO 5349-1):**
  Rentang frekuensi evaluasi HAV adalah $8\text{--}1000\text{ Hz}$. Frekuensi $159{,}2\text{ Hz}$ kalibrator RION VE-10 berada tepat di dalam pita ukur HAV, sehingga kalibrasi ini memberikan validasi langsung terhadap respon sensor pada frekuensi kerja riilnya.
* **Main Unit (ISO 2631-1):**
  Rentang frekuensi evaluasi WBV adalah $0{,}5\text{--}80\text{ Hz}$ dengan laju akuisisi normal $200\text{ Hz}$. Karena transduser kapasitif ADXL345 memiliki kurva respon amplitudo yang sangat datar (*flat frequency response*) dari DC hingga $\approx 400\text{ Hz}$, faktor skala sensitivitas ($K_{\text{cal}}$) yang ditentukan pada $159{,}2\text{ Hz}$ ini sepenuhnya berlaku dan valid untuk seluruh rentang frekuensi kerja WBV.
