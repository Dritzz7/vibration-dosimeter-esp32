# Catatan Teknis: Subsistem Catu Daya & Manajemen Daya Main Unit (*Main Power*)

Dokumen ini mengonsolidasikan seluruh catatan teknis, perhitungan beban (*power budget*), seleksi komponen regulator, mitigasi *brownout reset*, rangkaian proteksi kelistrikan, serta implementasi *firmware* pemantauan baterai dan indikator daya rendah (*Low Battery Indicator*) khusus untuk **Main Unit (Node Jok Sepeda Motor)**.

> [!NOTE]
> Untuk dokumentasi mendalam mengenai subsistem daya dan manajemen baterai pada **HAV Node (Node Stang Kemudi)**, silakan merujuk ke dokumen khusus:
> 🔗 [`docs/implementation_notes/hav_power.md`](<file:///d:/Users/Rafi%20Ananta%20Alden/Documents/Kuliah/Semester%208/EL4060/vibration-dosimeter-esp32/docs/implementation_notes/hav_power.md>).

---

## 1. Perhitungan Kebutuhan Kapasitas Baterai (*Power Budget Calculation*)

Main Unit bertindak sebagai pusat agregasi data (*Central Hub & Data Logger*) yang diletakkan di bawah jok atau alas duduk pengendara. Sesuai standar ISO 2631-1 dan durasi kerja harian, sistem dirancang untuk beroperasi secara kontinu **minimal selama 8 jam**.

### A. Rincian Konsumsi Arus Komponen Main Unit:

* **ESP32-WROOM-32D Dual-Core**:
  * Core 0: `vTaskDataLogger` (Agregasi HAV BLE + WBV, penulisan MicroSD, sinkronisasi waktu RTC, pembacaan GPS).
  * Core 1: `vTaskSensorWBV` (Akuisisi ADXL345 200 Hz & filter $W_d/W_k$) + `vTaskHMIAndController` (Refresh OLED SH1106, pemindaian tombol HMI, deteksi baterai, kontrol LED status).
  * Radio BLE Central (RX aktif mendengarkan notifikasi HAV Node 1 Hz).
  * Rata-rata konsumsi ESP32 + BLE RX: $\approx 120\text{ mA} - 140\text{ mA}$.
* **Sensor Akselerometer WBV (ADXL345)**:
  * Pengambilan sampel 200 Hz ODR: $\approx 0{,}14\text{ mA}$.
* **Modul RTC DS3231 (I2C)**:
  * Operasi pencacahan waktu: $\approx 0{,}20\text{ mA}$.
* **Modul MicroSD Card (SPI)**:
  * Penulisan SPI berkala 1 Hz + operasi flush *FATFS*: $\approx 20\text{ mA} - 30\text{ mA}$ (dengan arus lonjakan transien hingga $\approx 100\text{ mA} - 150\text{ mA}$ saat penulisan blok flash).
* **Layar OLED 1.3" SH1106 (I2C)**:
  * Mode aktif penuh (rendering UI 8 baris): $\approx 15\text{ mA}$.
  * Mode hemat daya *screensaver* titik 1 Hz (setelah 30 detik tanpa interaksi tombol): $< 2\text{ mA}$.
* **Modul GPS u-blox NEO-6M (Opsional/UART2)**:
  * Saat *tracking/fix*: $\approx 45\text{ mA} - 55\text{ mA}$.
* **Total Arus Rata-rata ($I_{avg}$)**:
  * Tanpa GPS: $\approx \mathbf{150\text{ mA}} - \mathbf{170\text{ mA}}$.
  * Dengan GPS aktif: $\approx \mathbf{195\text{ mA}} - \mathbf{225\text{ mA}}$.

### B. Perhitungan Kapasitas Baterai (Target Operasional 8 Jam):

* **Kebutuhan Teoretis (Dengan GPS, $I_{avg} = 225\text{ mA}$)**:
  $$
  C = I_{avg} \times t = 225\text{ mA} \times 8\text{ jam} = 1800\text{ mAh}
  $$
* **Dengan *Safety Margin* 30%** (Mengantisipasi lonjakan SPI SD Card, BLE reconnection, dan efisiensi konversi LDO):
  $$
  C_{rekomendasi} \ge 1800\text{ mAh} \times 1{,}30 = \mathbf{2340\text{ mAh}} \approx \mathbf{2200\text{ mAh} - 2600\text{ mAh}}
  $$
* **Pemilihan Sel Baterai**:
  * **1x Sel Li-Ion 18650 (2200–2600 mAh, 3.7V nominal)**.
  * Bobot sel 18650 standar berkisar $\approx 45 - 48\text{ gram}$, sangat mematuhi batas beban maksimum Main Unit (**$< 150\text{ gram}$**).
  * Waktu operasional nyata yang dihasilkan: **12 hingga 15 jam kontinu**.

---

## 2. Kriteria & Seleksi Regulator Tegangan LDO (*Low-Dropout*)

ESP32 membutuhkan tegangan kerja $3{,}3\text{ V} \pm 10\%$ ($3{,}0\text{ V} - 3{,}6\text{ V}$). Karakteristik pengosongan sel Lithium berkisar dari **$4{,}20\text{ V}$** (kondisi penuh), **$3{,}70\text{ V}$** (kondisi nominal datar/plato), hingga **$3{,}40\text{ V} - 3{,}20\text{ V}$** (kondisi habis).

> [!WARNING]
> Regulator linier umum seperti **AMS1117-3.3 TIDAK BOLEH DIGUNAKAN** untuk sistem bertenaga baterai Lithium tunggal. AMS1117 memiliki tegangan jatuh (*dropout voltage*) sebesar **$1{,}1\text{ V} - 1{,}3\text{ V}$**. Jika baterai berada pada tegangan nominal $3{,}70\text{ V}$, tegangan keluarannya anjlok menjadi $3{,}7\text{ V} - 1{,}2\text{ V} = 2{,}5\text{ V}$, memicu *brownout reset* pada ESP32 ketika kapasitas baterai masih tersisa $> 70\%$!

### 🔍 4 Kriteria Utama Seleksi Regulator:

1. **Ultra-Low Dropout Voltage ($V_{drop} \le 250\text{ mV}$ pada beban penuh)**:
   * Menjaga rel tegangan tetap stabil di $3{,}3\text{ V}$ hingga tegangan baterai turun mendekati $3{,}45\text{ V} - 3{,}40\text{ V}$ (memanfaatkan $> 90\%$ kapasitas energi baterai).
2. **Kapasitas Arus Puncak Memadai ($I_{peak} \ge 800\text{ mA} - 1\text{ A}$)**:
   * Menyuplai lonjakan sesaat (*burst current*) saat inisialisasi radio BLE dan operasi tulis flash MicroSD.
3. **Arus Diam Rendah (*Ultra-Low Quiescent Current* / $I_q < 60\ \mu\text{A}$)**:
   * Mengurangi pemborosan energi saat ESP32 berada pada mode siaga atau saat layar screensaver aktif.
4. **Respon Transien Cepat & PSRR Tinggi**:
   * Mencegah riak daya merambat ke konverter ADC dan sensor getaran ADXL345.

### 🏆 Komparasi IC Regulator:

| Tipe IC Regulator              | Arus Maks ($I_{out}$) | *Dropout Voltage* ($V_{drop}$) | Arus Diam ($I_q$) |  Rekomendasi Penggunaan  |                            |                                                                                                                  |
| :----------------------------- | :--------------------------------------------------------------------------------: | :-----------------------: | :-------------------------: | :--------------------------------------------------------------------------------------------------------------- |
| **AP2112K-3.3**          |                                  **600 mA**                                  | **250 mV** @ 600 mA | $\approx 55\ \mu\text{A}$ | **Sangat Direkomendasikan**. Standar papan Adafruit Feather ESP32; sangat andal untuk beban BLE + SD Card. |
| **XC6220B331MR**         |                               **1000 mA (1A)**                               |   **150 mV** @ 1A   | $\approx 8\ \mu\text{A}$ | Sangat efisien,*dropout* sangat rendah, kapasitas arus besar.                                                  |
| **RT9013-33GB / ME6211** |                                  **500 mA**                                  | **250 mV** @ 500 mA | $\approx 40\ \mu\text{A}$ | Ekonomis, kemasan ringkas SOT-23-5, sangat mudah diperoleh di pasaran.                                           |
| **MCP1700-3302E**        |                                  **250 mA**                                  | **178 mV** @ 250 mA |      **1,6 µA**      | Arus kontinu mepet; membutuhkan elco besar ($\ge 470\ \mu\text{F}$) untuk menyerap beban puncak BLE.           |

---

## 3. Investigasi & Mitigasi *Brownout Reset* pada ESP32

### A. Mekanisme Pemicu

ESP32 memiliki rangkaian komparator tegangan internal (*Brownout Detector*) yang terhubung ke domain daya RTC. Ambang batas pemicu bawaan silikon berada pada rentang **$2{,}43\text{ V} - 2{,}80\text{ V}$**.

* **Pemicu Utama**: Penulisan serentak pada kartu MicroSD bersamaan dengan pertukaran paket nirkabel BLE memicu penarikan arus transien sebesar **$150\text{ mA} - 250\text{ mA}$** dalam orde mikrodetik. Jika impedansi jalur catu daya atau resistansi internal baterai/kabel terlalu tinggi, tegangan pada rel $3{,}3\text{ V}$ akan anjlok (*sag/dip*) melintasi ambang batas komparator.
* **Gejala**: ESP32 me-restart berulang kali (*infinite boot loop*) dengan pesan log UART:
  `rst:0x10 (RTCWDT_RTC_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT) ... Brownout detector was triggered`.

### B. Mitigasi Perangkat Keras (*Hardware Level - Mandatory*)

1. **Pemasangan Bulk Decoupling Capacitor**:
   * Pasang kapasitor elektrolit (elco) / tantalum tipe *Low-ESR* sebesar **$100\ \mu\text{F} - 470\ \mu\text{F}$ (16V)** paralel di antara pin **`3V3`** dan **`GND`** sedekat mungkin ($< 10\text{ mm}$) dengan pin modul ESP32.
   * Pasang kapasitor keramik MLCC **$10\ \mu\text{F}$** dan **$100\text{ nF}$ (0,1 µF)** tepat di kaki daya modul MicroSD dan pin VDD ESP32 untuk menekan derau frekuensi tinggi.
2. **Kualitas Jalur PCB / Pengkabelan**:
   * Lebar jalur rel daya $V_{BAT}$ dan $3\text{V}3$ pada tata letak PCB minimal berukuran **$25 - 40\text{ mil}$ ($0{,}6\text{ mm} - 1{,}0\text{ mm}$)** untuk meminimalkan rugi resistif ($I \times R$).

### C. Mitigasi Perangkat Lunak (*Software Bypass*)

Pada `Main-Unit/src/main.cpp`, fungsi proteksi brownout dinonaktifkan pada awal fungsi `setup()` untuk mencegah *false triggering* saat booting dingin (*cold boot*):

```cpp
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Nonaktifkan komparator brownout internal
    // Inisialisasi periferal ...
}
```

---

## 4. Rangkaian Pengisian & Proteksi Baterai

Modul daya Main Unit mengintegrasikan sistem proteksi terpadu berbasis modul **TP4056** dengan sirkuit pengaman sel Lithium:

* **IC Pengisi Daya**: TP4056 (Linear Constant-Current / Constant-Voltage Charger) dengan input USB Type-C. Arus pengisian disetel ke **$500\text{ mA} - 1000\text{ mA}$** via resistor $R_{PROG}$ ($1{,}2\text{ k}\Omega - 2\text{ k}\Omega$).
* **IC Proteksi**: **DW01A** yang mengendalikan sakelar ganda MOSFET **FS8205A**:
  * *Over-Charge Protection*: Memutus pengisian saat tegangan sel mencapai $4{,}28\text{ V} \pm 0{,}05\text{ V}$.
  * *Over-Discharge Protection*: Memutus beban secara otomatis saat tegangan sel anjlok di bawah **$2{,}40\text{ V} - 2{,}50\text{ V}$** untuk mencegah kristalisasi kimia Lithium.
  * *Short-Circuit & Over-Current*: Memutus sirkuit dalam $< 10\ \mu\text{s}$ jika arus beban melebihi **$3\text{ A}$**.

---

## 5. Implementasi Perangkat Keras Pemantauan Baterai Main Unit

Untuk memonitor status energi sel baterai 18650 secara akurat tanpa mengganggu subsistem lainnya:

```
          V_BAT (+ Sel Lithium 18650: 3.0V - 4.2V)
             │
             ├───[ R1: 100 kΩ, 1% ]───┐
             │                        │
            === C_filt                ├───> GPIO 35 (ADC1_CH7 ESP32)
            100nF                     │
             │                        ├───[ R2: 100 kΩ, 1% ]───┐
            GND                       │                        │
                                     === C_adc                GND
                                     100nF
                                      │
                                     GND
```

### A. Pemilihan Pin ADC1: GPIO 35 (ADC1_CH7)

* **Kritis & Esensial**: ESP32 memiliki dua unit SAR ADC (ADC1 dan ADC2). Unit **ADC2 secara perangkat keras digunakan bersama dengan modul radio nirkabel (Wi-Fi & Bluetooth LE)**. Saat stack BLE diinisialisasi (`BLEDevice::init()`), pengontrol arbiter internal mengunci ADC2, sehingga pembacaan pin ADC2 (GPIO 0, 2, 4, 12, 13, 14, 15, 25, 26, 27) akan gagal atau menghasilkan galat `ESP_ERR_INVALID_STATE`.
* **GPIO 35** merupakan bagian dari **ADC1 (ADC1_CH7)** yang sepenuhnya independen dari modul RF BLE, sehingga pembacaan analog dapat dilakukan kapan saja tanpa jeda atau konflik.
* **Input-Only (GPI)**: GPIO 35 adalah pin input murni tanpa rangkaian pull-up/pull-down internal ataupun fungsi output digital, menjadikannya pin analog berderau sangat rendah.

### B. Rasio Pembagi Tegangan & Filter Derau

* **Rasio Pembagi ($1:2$)**:
  $$
  V_{ADC} = V_{BAT} \times \frac{R_2}{R_1 + R_2} = V_{BAT} \times \frac{100\text{ k}\Omega}{100\text{ k}\Omega + 100\text{ k}\Omega} = \frac{V_{BAT}}{2}
  $$

  * Pada tegangan baterai penuh ($V_{BAT} = 4{,}20\text{ V}$), tegangan pin ADC adalah **$2{,}10\text{ V}$**. Nilai ini berada tepat di rentang linear terbaik konverter ADC ESP32 dengan redaman bawaan 11 dB ($0 - 3{,}1\text{ V}$).
* **Efisiensi Arus Bocor**:
  $$
  I_{leak} = \frac{4{,}20\text{ V}}{R_1 + R_2} = \frac{4{,}20\text{ V}}{200\text{ k}\Omega} = 21\ \mu\text{A}
  $$

  Arus bocor $21\ \mu\text{A}$ sangat kecil dan tidak akan menguras baterai bahkan jika alat disimpan selama berbulan-bulan.
* **Kapasitor Filter Derau ($C = 100\text{ nF}$)**:
  Membentuk filter pelewat-rendah (*Low-Pass Filter*) dengan frekuensi sudut:
  $$
  f_c = \frac{1}{2\pi (R_1 \parallel R_2) C} = \frac{1}{2\pi (50\text{ k}\Omega)(100\text{ nF})} \approx 31{,}8\text{ Hz}
  $$

  Filter ini secara efektif memblokir derau frekuensi tinggi akibat modulasi pulsa radio BLE dan riak transien penulisan SPI pada MicroSD.

---

## 6. Implementasi Perangkat Lunak Pemantauan Baterai pada Main Unit (`Main-Unit/src/main.cpp`)

### A. Parameter Konfigurasi & Ambang Batas Operasional

Didefinisikan pada konstanta perangkat keras `Main-Unit/src/main.cpp`:

```cpp
// --- Battery ADC Monitoring (ADC1, safe with BLE) ---
#define PIN_VBAT_SENSE           35     ///< GPIO 35 (ADC1_CH7) voltage divider
#define VBAT_DIVIDER_RATIO       2.0f   ///< R1=100k, R2=100k resistor divider
#define VBAT_MIN_VALID           2.80f  ///< Minimum valid battery voltage (filters floating pin on USB)
#define VBAT_LOW_THRESHOLD       3.40f  ///< Low battery cut-off warning (V)
#define VBAT_CRITICAL_THRESHOLD  3.20f  ///< Critical battery threshold (V)
```

| Ambang Batas                  |                                                             Rentang Tegangan                                                             |                                                                                            Status Sistem                                                                                            | Aksi Perangkat Lunak & Indikator HMI                                                                                                                                   |
| :---------------------------- | :--------------------------------------------------------------------------------------------------------------------------------------: | :--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------: | :--------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Normal**              |                                     $V_{bat} \ge 3{,}40\text{ V}$ | Daya Cukup ($> 0\% - 100\%$)                                     |                                                Menampilkan persentase & voltase normal pada Header Bar OLED; LED status berkedip normal sesuai mode.                                                |                                                                                                                                                                        |
| **Low Battery**         |                               $2{,}80\text{ V} \le V_{bat} < 3{,}40\text{ V}$ | Baterai Rendah ($0\%$)                               | **Header Bar OLED berkedip peringatan**, baris status menampilkan `(!BAT)`, **layar screensaver otomatis dibangunkan**, dan **LED status onboard (GPIO 2) berkedip cepat 5 Hz**. |                                                                                                                                                                        |
| **Critical**            | $V_{bat} < 3{,}20\text{ V}$ | Baterai Kritis | Peringatan berlanjut sebelum sistem diputus oleh IC proteksi DW01 ($2{,}5\text{ V}$). |                                                                                                                                                                                                      |                                                                                                                                                                        |
| **USB / Tanpa Baterai** |                                                      $V_{bat} < 2{,}80\text{ V}$                                                      |                                                                                         Daya USB Bangku Uji                                                                                         | Terdeteksi pin ADC melayang (*floating*) atau operasi murni USB; alarm baterai rendah ditekan agar tidak mengganggu pengujian (*Header: `VIB DOSIMETER (USB)`*). |

---

### B. Algoritma Pembacaan ADC Multi-Sampling

Untuk mengeliminasi fluktuasi transien saat kartu MicroSD menulis blok flash, pembacaan tegangan menggunakan teknik rata-rata 16 sampel (*multi-sample averaging*):

```cpp
static float readBatteryVoltage() {
    uint32_t rawSum = 0;
    const uint8_t SAMPLES = 16;
    for (uint8_t i = 0; i < SAMPLES; i++) {
        rawSum += analogRead(PIN_VBAT_SENSE);
        delay(2);
    }
    const float rawAvg = (float)rawSum / (float)SAMPLES;
    const float pinVoltage = (rawAvg / 4095.0f) * 3.3f;
    return pinVoltage * VBAT_DIVIDER_RATIO;
}
```

---

### C. Pemetaan Kapasitas Relatif (*State-of-Charge / SOC*)

Persentase kapasitas baterai dihitung menggunakan interpolasi linier tegangan pengosongan sel Lithium-Ion ($3{,}40\text{ V}$ sebagai $0\%$ dan $4{,}20\text{ V}$ sebagai $100\%$):

```cpp
static uint8_t calculateBatteryPercentage(float vbat) {
    if (vbat <= VBAT_LOW_THRESHOLD) return 0;
    if (vbat >= 4.20f) return 100;
    float pct = ((vbat - VBAT_LOW_THRESHOLD) / (4.20f - VBAT_LOW_THRESHOLD)) * 100.0f;
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    return (uint8_t)(pct + 0.5f);
}
```

---

### D. Siklus Pemantauan Berkala & *Screensaver Auto-Wake*

Pemeriksaan baterai dieksekusi secara berkala setiap **3 detik** di dalam task `vTaskHMIAndController` (Core 1):

```cpp
// Periodic Battery Check (every 3 seconds)
if (nowMs - lastBatteryCheckMs >= 3000) {
    lastBatteryCheckMs = nowMs;
    vbatVoltage = readBatteryVoltage();
    vbatValid = (vbatVoltage >= VBAT_MIN_VALID);
    if (vbatValid) {
        vbatPercent = calculateBatteryPercentage(vbatVoltage);
        vbatLow = (vbatVoltage < VBAT_LOW_THRESHOLD);
    } else {
        vbatPercent = 0;
        vbatLow = false; // Powered by USB / floating pin
    }

    // Safety: Auto-wake screensaver if battery drops to low level
    if (vbatLow && screenSaverActive) {
        screenSaverActive = false;
        lastInteractionTime = nowMs;
        display.powerOn();
        LOG_W("PWR", "Low battery detected (%.2fV)! Auto-waking OLED screensaver.", vbatVoltage);
    }
}
```

---

### E. Integrasi Antarmuka HMI (Layar OLED 1.3" SH1106)

Informasi baterai diintegrasikan secara optimal pada **Baris Header 0** layar OLED (128x64 piksel, font 5x7 monospasi, 21 karakter per baris), sehingga tidak mengurangi baris informasi periferal lainnya (HAV BLE, WBV Acc, SD Card, RTC, GPS):

1. **Kondisi Normal (Baterai Mencukupi)**:
   ```text
   +---------------------+
   |VIB DOSI   [85%] 4.0V|  <-- Baris 0: Header Dinamis Baterai
   |HAV BLE : CONNECTED  |
   |WBV ACC : ADXL345 OK |
   |SD CARD : LOGGING OK |
   |RTC CLK : 14:32:05 OK|
   |GPS FIX : 3D (8 SAT) |
   |A(8) WBV: 0.42 m/s2  |
   |STATUS:  LOGGING     |
   +---------------------+
   ```
2. **Kondisi Baterai Rendah ($V_{bat} < 3{,}40\text{ V}$)**:
   Header baris 0 berkedip selang-seling setiap 1 detik antara persentase peringatan dan tegangan aktual:
   * Detik Genap: `|VIB DOSI  [!  0% !] |`
   * Detik Ganjil: `|VIB DOSI  [!3.38V!] |`
   * Baris Status (Baris 7): Berganti dari `STATUS:  LOGGING` menjadi `STATUS:  LOG (!BAT)`.
3. **Kondisi Daya USB (Tanpa Baterai)**:
   Menampilkan teks statis: `|VIB DOSIMETER (USB)  |`.

---

### F. Indikasi Fisik: LED Status Onboard (GPIO 2)

Jika kondisi `vbatLow == true`, fungsi pengontrol LED mengesampingkan pola kedip normal dan beralih ke pola **kedip cepat 5 Hz (100 ms ON, 100 ms OFF)**:

```cpp
// Low Battery Override: Rapid 5 Hz flashing (100 ms ON, 100 ms OFF)
if (vbatLow) {
    bool ledOn = ((nowMs / 100) % 2) == 0;
    digitalWrite(PIN_LED_STATUS, ledOn ? HIGH : LOW);
} else {
    // Pola normal: 1 Hz kedip saat logging, solid saat stand-by, kedip cepat saat error
    ...
}
```

Pola kedip 5 Hz ini memberikan umpan balik visual yang sangat mencolok bagi pengendara sepeda motor meskipun layar OLED tidak berada dalam pandangan langsung.

---

### G. Prosedur Uji Mandiri Saat Booting (*Power-On Self-Test / POST*)

Pada fungsi `runSelfTest()`, ESP32 memeriksa keberadaan baterai dan mencatatnya ke konsol serial UART:

```cpp
// --- Check Battery Subsystem ---
vbatVoltage = readBatteryVoltage();
vbatValid = (vbatVoltage >= VBAT_MIN_VALID);
if (vbatValid) {
    vbatPercent = calculateBatteryPercentage(vbatVoltage);
    vbatLow = (vbatVoltage < VBAT_LOW_THRESHOLD);
    Serial.printf("[POST] Battery Sense : OK (%.2fV, %u%%)%s\n",
                  vbatVoltage, vbatPercent, vbatLow ? " [LOW BATTERY!]" : "");
} else {
    Serial.println("[POST] Battery Sense : USB / No Battery detected");
}
```

---

### H. Telemetri Serial & Integritas Format Berkas SD Card

* **Konsol Serial (1 Hz)**:
  Status baterai disematkan pada baris keluaran serial tanpa merusak pembacaan:
  `[LOG] 2026-06-17 14:30:00 | WBV: 0.3541 m/s² | HAV: 1.2504 m/s² | Vbat=3.95V (69%)`
* **Format Berkas CSV SD Card (`/dosimeter.csv`)**:
  Struktur berkas log CSV 17-kolom **tetap dipertahankan 100% tanpa perubahan**:
  ```csv
  Timestamp,SeqHAV,HAV_RMS_X,HAV_RMS_Y,HAV_RMS_Z,HAV_VectorSum,HAV_Valid,SeqWBV,WBV_RMS_X,WBV_RMS_Y,WBV_RMS_Z,WBV_VectorSum,WBV_Valid,GPS_Lat,GPS_Lon,GPS_Speed,GPS_Valid
  ```

  Hal ini menjamin kompatibilitas mutlak dengan skrip analisis pasca-proses [`calc_dosimeter.py`](<file:///d:/Users/Rafi%20Ananta%20Alden/Documents/Kuliah/Semester%208/EL4060/vibration-dosimeter-esp32/calc_dosimeter.py>).

---

## 7. Rangkuman Pengkabelan & Pinout Daya Main Unit

| Komponen / Sinyal              |     Pin ESP32     |        Jenis Pin        |        Komponen Terhubung        | Keterangan Rangkaian                                                                                           |
| :----------------------------- | :----------------: | :---------------------: | :-------------------------------: | :------------------------------------------------------------------------------------------------------------- |
| **Sensor Baterai**       | **GPIO 35** | Input Analog (ADC1_CH7) | Titik tengah pembagi$R_1 + R_2$ | $100\text{ k}\Omega$ ke $V_{BAT}$, $100\text{ k}\Omega$ ke GND, $100\text{ nF}$ filter paralel ke GND. |
| **LED Peringatan Fisik** |  **GPIO 2**  |     Output Digital     |     Anoda Status LED Onboard     | Seri resistor$330\ \Omega$ ke GND. Berkedip 5 Hz saat baterai rendah.                                        |
| **Rel Daya Masukan**     | **VIN / 5V** |        Power In        |       Pin OUT+ Modul TP4056       | Menerima daya$3{,}7\text{ V} - 4{,}2\text{ V}$ dari baterai atau $5\text{ V}$ USB.                         |
| **Rel Daya Teregulasi**  |   **3V3**   |       Power Rail       |      Output LDO AP2112K-3.3      | Menyuplai ESP32, OLED SH1106, ADXL345, DS3231, dan MicroSD. Wajib elco$470\ \mu\text{F}$.                    |
| **Ground**               |   **GND**   |         Ground         |          Ground Bersama          | Menghubungkan ground TP4056, baterai, LDO, dan periferal.                                                      |
