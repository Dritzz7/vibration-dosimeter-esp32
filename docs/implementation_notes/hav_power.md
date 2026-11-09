# Catatan Teknis: Subsistem Catu Daya & Manajemen Daya HAV Node (*HAV Power Management*)

Dokumen ini mengonsolidasikan arsitektur kelistrikan, perhitungan beban (*power budget*), seleksi komponen regulator LDO, mitigasi *brownout reset*, rangkaian proteksi pengisian baterai, serta implementasi *firmware* pemantauan baterai dan indikator visual daya rendah (*Low Battery Indicator*) khusus untuk **HAV Node (Node Stang Kemudi Sepeda Motor)**.

> [!NOTE]
> Untuk dokumentasi mengenai subsistem catu daya, baterai 18650, dan indikator HMI OLED pada **Main Unit (Node Jok)**, silakan merujuk ke:
> 🔗 [`docs/implementation_notes/main_power.md`](file:///d:/Users/Rafi%20Ananta%20Alden/Documents/Kuliah/Semester%208/EL4060/vibration-dosimeter-esp32/docs/implementation_notes/main_power.md).

---

## 1. Konteks Operasional & Batasan Desain Fisik

HAV Node bertindak sebagai modul pemroses getaran di sisi tepi (*edge computing node*) yang dipasang langsung pada stang kemudi (*handlebar*) sepeda motor dekat genggaman tangan pengendara:
* **Batasan Bobot Kritis ($< 50\text{ gram}$)**:
  Sesuai standar ergonomi dan dinamika berkendara, penambahan massa pada ujung stang kemudi tidak boleh memengaruhi kestabilan manuver kemudi ataupun mengubah frekuensi resonansi alami stang. Oleh karena itu, bobot total modul HAV Node termasuk baterai, penutup (*enclosure*), dan sensor dibatasi ketat di bawah **$50\text{ gram}$**.
* **Operasi Mandiri Nirkabel**:
  HAV Node beroperasi secara mandiri tanpa kabel penghubung ke jok (*wireless tetherless*) menggunakan protokol **Bluetooth Low Energy (BLE Server / GATT Peripheral)** untuk mengirimkan hasil kalkulasi percepatan RMS tertimbang frekuensi ($a_{hwx}, a_{hwy}, a_{hwz}, a_{hv}$) setiap 1 detik ke Main Unit.

---

## 2. Perhitungan Kebutuhan Kapasitas Baterai (*Power Budget Calculation*)

Sistem dosimeter getaran dirancang untuk beroperasi secara kontinu **minimal selama 8 jam** sesuai durasi shift kerja harian internasional (ISO 5349-1).

### A. Rincian Konsumsi Arus Komponen HAV Node:
* **ESP32 Core 1 (Akuisisi Frekuensi Tinggi 3200 Hz & Filter IIR Wh)**:
  * Pengambilan sampel ADXL345 via I2C 400 kHz setiap $312{,}5\ \mu\text{s}$.
  * Komputasi filter digital IIR Biquad Cascade (*Direct Form II Transposed*) 3 sumbu ($X, Y, Z$).
  * Rata-rata konsumsi CPU: $\approx 85\text{ mA} - 95\text{ mA}$.
* **Radio BLE Server (Transmisi Notifikasi 1 Hz)**:
  * Emisi paket notifikasi CSV GATT setiap 1 detik: $\approx 10\text{ mA} - 15\text{ mA}$ (rata-rata terintegrasi).
* **Sensor Akselerometer HAV (ADXL345)**:
  * Mode pengukuran kontinu pada kecepatan data tertinggi (3200 Hz ODR, $\pm 16g$): $\approx 0{,}14\text{ mA}$ ($140\ \mu\text{A}$).
* **Mini RGB LED Indikator (LEDC PWM / FreeRTOS Background Task)**:
  * Mode *Blinking Blue* (Iklan BLE) / *Breathing Green* (Logging Normal): $\approx 2\text{ mA} - 8\text{ mA}$ (rata-rata dinamis).
* **Total Arus Rata-rata ($I_{avg}$)**: $\approx \mathbf{100\text{ mA}}$.

### B. Perhitungan Kapasitas & Seleksi Bentuk Baterai:
* **Kapasitas Teoretis (8 Jam Operasi Kontinu)**:
  $$C = I_{avg} \times t = 100\text{ mA} \times 8\text{ jam} = 800\text{ mAh}$$
* **Dengan *Safety Margin* 25%** (Mengakomodasi penuaan baterai dan efisiensi konversi regulator LDO):
  $$C_{rekomendasi} \ge 800\text{ mAh} \times 1{,}25 = \mathbf{1000\text{ mAh} - 1200\text{ mAh}}$$
* **Perbandingan Bentuk & Bobot Sel Baterai**:
  | Jenis Sel Baterai | Kapasitas Nominal | Bobot Sel | Dimensi Fisik | Kesesuaian Beban Stang ($< 50\text{ g}$) |
  | :--- | :---: | :---: | :---: | :---: |
  | **Li-Po 3.7V (Pouch Cell)** | **1000 – 1200 mAh** | **$\approx 20 - 24\text{ gram}$** | Pipih ($50 \times 34 \times 6\text{ mm}$) | **Sangat Direkomendasikan (Lolos Batas)** |
  | Li-Ion 18650 Silinder | 2000 – 2600 mAh | $\approx 46 - 48\text{ gram}$ | Silinder ($\varnothing 18 \times 65\text{ mm}$) | Terlalu berat (total unit melebihi $70\text{ g}$) |

> [!TIP]
> Penggunaan sel kantong **Li-Po 3.7V 1000–1200 mAh** berbobot $\approx 22\text{ gram}$ memungkinkan total bobot modul HAV Node (PCB + komponen + baterai + casing 3D print) berada pada kisaran **$42 - 46\text{ gram}$**, sehingga sepenuhnya mematuhi batas keselamatan ergonomis stang kemudi sepeda motor.

* **Estimasi Daya Tahan Nyata**:
  $$t_{operasi} = \frac{1100\text{ mAh}}{100\text{ mA}} \approx \mathbf{11\text{ Jam}}$$
  Menyediakan margin aman $3\text{ jam}$ di atas target shift kerja standar $8\text{ jam}$.

---

## 3. Rangkaian Proteksi & Catu Daya PCB Rev A (`hardware-dosimeter`)

Pada implementasi PCB perangkat keras revisi A (**`hardware-dosimeter`**), arsitektur catu daya ditingkatkan dari prototipe 1S menjadi paket **2S Li-ion/Li-Po (nominal 7.4V, maks 8.4V)** untuk menjamin kestabilan tegangan tinggi dan efisiensi regulasi:

```
  [ Baterai 2S Li-ion ] (7.4V - 8.4V)
           │
      [ JST-XH J2 ] (Konektor 2-pin terpolarisasi)
           │
     [ PTC Fuse F1 ] (Bourns MF-MSMF075: I_hold = 0.75A, 13.2V)
           │ (BAT_PROTECTED)
     [ Sakelar SW3 ] (KCD11-101 Rocker Switch SPST)
           │ (SW_BAT)
           ├───[ R8: 330 kΩ ]───┬───> GPIO 34 (VBAT_SENSE ESP32)
           │                    ├───[ R9: 100 kΩ ]─── GND
           │                   === C11: 100nF
           │                    │
           │                   GND
           │
    [ Input Filter ] (Panasonic FR 47 µF / 25V + MLCC 100 nF)
           │
    [ Buck MP1584EN ] (Regulator Switching Step-Down Fixed 3.3V)
           │
   [ 3V3_SYS Rail ] (Panasonic FR 47 µF / 25V + MLCC 100 nF + 22 µF MLCC)
           ├─── ESP32-WROOM-32E (VDD)
           └─── ADXL345 (VS & VDD_IO)
```

* **Konektor Input Baterai (J2)**: Header vertikal terpolarisasi **JST-XH 2-pin (pitch 2.50 mm)** untuk koneksi paket baterai 2S.
* **Proteksi Arus Lebih / Hubung Singkat (F1)**:
  * Menggunakan *Polymer Positive Temperature Coefficient* (PTC) resettable fuse **Bourns MF-MSMF075-2** (kemasan SMD 1812).
  * Arus tahan (*Hold Current*): **$0{,}75\text{ A}$**.
  * Arus putus (*Trip Current*): **$1{,}50\text{ A}$**.
  * Rating tegangan maksimum: **$13{,}2\text{ V}$**.
  * Memberikan proteksi protektif otomatis apabila terjadi hubung singkat pada jalur kabel sakelar atau modul switching.
* **Sakelar Daya Fisik (SW3)**:
  * Sakelar mekanik rocker **KCD11-101 SPST** yang dipasang pada dinding *casing*, terhubung ke PCB via pad solder kawat diameter $\varnothing 1{,}7\text{ mm}$ pitch $4{,}2\text{ mm}$.

---

## 4. Regulasi Tegangan 3.3V & Mitigasi *Brownout Reset*

### A. Seleksi Regulator: Mengapa Switching Buck MP1584EN Menggantikan LDO Linier?
Pada arsitektur baterai 2S ($V_{bat} = 7{,}4\text{ V} - 8{,}4\text{ V}$), penggunaan regulator linier LDO (seperti AP2112K) akan menghasilkan disipasi daya termal yang sangat berbahaya di dalam *enclosure* tertutup:
$$V_{drop} = 8{,}4\text{ V} - 3{,}3\text{ V} = 5{,}1\text{ V}$$
$$P_{loss} = V_{drop} \times I_{load} = 5{,}1\text{ V} \times 0{,}10\text{ A} = \mathbf{0{,}51\text{ Watt (510 mW)}}$$
Daya panas sebesar $510\text{ mW}$ akan menyebabkan kenaikan suhu ekstrem pada *enclosure* stang kemudi.

Oleh karena itu, modul regulator beralih ke **Switching Step-Down Buck Converter MP1584EN (Fixed 3.3V Output)**:
* **Efisiensi Tinggi**: $\approx 88\% - 92\%$, sehingga disipasi daya panas berkurang hingga kurang dari $50\text{ mW}$.
* **Arus Output Kontinu**: Hingga $1{,}5\text{ A}$ (jauh melampaui kebutuhan beban puncak ESP32 saat radio BLE aktif sebesar $\approx 350\text{ mA}$).

### B. Jaringan Filter Riak (*Ripple Decoupling*) & Mitigasi *Brownout*
Regulator switching memiliki frekuensi switching tinggi ($\approx 1\text{ MHz} - 1{,}5\text{ MHz}$) yang memicu riak tegangan (*switching ripple*). Pada PCB `hardware-dosimeter`, diimplementasikan filter 2-tahap:
1. **Input Rail (`SW_BAT`)**:
   * Kapasitor radial ultra-low ESR **Panasonic FR Series $47\ \mu\text{F} / 25\text{ V}$ ($C_7$)** (dimensi $\varnothing 5\times 11\text{ mm}$, impedansi $0{,}08\ \Omega$).
   * Kapasitor keramik MLCC SMD 0805 **$100\text{ nF} / 16\text{ V}$ ($C_8$)** untuk tapis noise frekuensi tinggi.
2. **Output Rail (`3V3_SYS`)**:
   * Kapasitor radial ultra-low ESR **Panasonic FR Series $47\ \mu\text{F} / 25\text{ V}$ ($C_9$)** tepat di keluaran modul MP1584.
   * Kapasitor keramik MLCC SMD 0805 **$100\text{ nF}$ ($C_{10}$)**.
   * Kapasitor keramik MLCC SMD 1206 **$22\ \mu\text{F} / 16\text{ V}$ ($C_1$)** paralel dengan **$100\text{ nF}$ ($C_2$)** tepat di pin VDD ESP32.
3. **Mitigasi Perangkat Lunak**:
   * Pengecualian detektor brownout tetap dipertahankan pada awal `setup()` di `HAV-Node/src/main.cpp`:
   ```cpp
   WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Bypass brownout detector
   ```

---

## 5. Rangkaian Pemantauan Baterai Perangkat Keras (*Hardware ADC Sense*)

```
          V_BAT (Paket 2S Li-ion: 6.4V - 8.4V)
             │
             ├───[ R8: 330 kΩ, 1% ]───┐
             │                        │
            === C_filt                ├───> GPIO 34 (ADC1_CH6 ESP32)
            100nF                     │
             │                        ├───[ R9: 100 kΩ, 1% ]───┐
            GND                       │                        │
                                     === C11                  GND
                                     100nF
                                      │
                                     GND
```

### A. Alokasi Pin: GPIO 34 (ADC1_CH6)
* **Bebas Konflik BLE**: GPIO 34 terhubung ke unit periferal ADC1 internal ESP32, beroperasi tanpa hambatan saat radio Bluetooth LE memancarkan paket notifikasi.
* **Karakteristik Pin Input-Only (GPI)**: Bebas interferensi rangkaian output digital internal, menjamin akurasi sampling voltase.

### B. Nilai Komponen Pembagi Tegangan Aktual PCB Rev A:
* **Resistor Presisi $R_8 = 330\text{ k}\Omega$ (1%) dan $R_9 = 100\text{ k}\Omega$ (1%)**:
  * Rasio atenuasi pembagi tegangan:
    $$V_{\text{ADC}} = V_{\text{BAT}} \times \frac{R_9}{R_8 + R_9} = V_{\text{BAT}} \times \frac{100\text{ k}\Omega}{330\text{ k}\Omega + 100\text{ k}\Omega} = \frac{V_{\text{BAT}}}{4{,}30}$$
  * Faktor pengali pemulihan perangkat lunak: $\mathbf{4{,}30\mathbf{f}}$.
  * Saat baterai 2S terisi penuh ($8{,}40\text{ V}$):
    $$V_{\text{pin}} = \frac{8{,}40\text{ V}}{4{,}30} \approx \mathbf{1{,}95\text{ V}}$$
    Nilai ini berada tepat di zona linearitas tertinggi ADC ESP32 (rentang aman $0{,}1\text{ V} - 2{,}5\text{ V}$).
  * Arus bocor pemantau sangat kecil:
    $$I = \frac{8{,}4\text{ V}}{430\text{ k}\Omega} \approx 19{,}5\ \mu\text{A}$$
* **Kapasitor Filter $C_{11} = 100\text{ nF}$**:
  * Membentuk filter pelewat-rendah (*Low-Pass Filter*) dengan frekuensi potong $f_c \approx 20{,}8\text{ Hz}$ untuk meredam ripple switching dari MP1584 dan transmisi radio BLE.

---

## 6. Implementasi Perangkat Lunak Pemantauan Baterai pada HAV Node (`HAV-Node/src/main.cpp`)

### A. Parameter Konfigurasi (Telah Diselaraskan)
Didefinisikan pada `HAV-Node/src/main.cpp`:
```cpp
// --- Battery ADC Monitoring (2S Li-ion pack: nominal 7.4V, max 8.4V) ---
#define ENABLE_BATTERY_MONITOR 1     ///< Set 1 when battery divider is connected to GPIO34
#define PIN_VBAT_SENSE         34    ///< GPIO34 (ADC1_CH6) voltage divider
#define VBAT_DIVIDER_RATIO     4.30f ///< R8=330k (1%), R9=100k (1%) -> (330k+100k)/100k = 4.30
#define VBAT_MIN_VALID         5.50f ///< Minimum valid battery voltage (filters floating pin on USB test bench)
#define VBAT_LOW_THRESHOLD     6.60f ///< Low battery cut-off warning for 2S pack (3.30V per cell)
```

### B. Fungsi Pengambilan Sampel Tegangan
```cpp
static float readBatteryVoltage() {
    const uint16_t raw = analogRead(PIN_VBAT_SENSE);
    const float pinVoltage = (raw / 4095.0f) * 3.3f;
    return pinVoltage * VBAT_DIVIDER_RATIO;
}
```

### C. Siklus Pemantauan Berkala
Pemeriksaan kondisi tegangan baterai dilakukan setiap **5 detik** di dalam batas epoch 1 detik pada fungsi `loop()`:
```cpp
#if ENABLE_BATTERY_MONITOR
    // Periodic battery voltage monitoring (every 5 seconds)
    static uint32_t lastBatteryCheckMs = 0;
    const uint32_t nowMs = millis();
    if (nowMs - lastBatteryCheckMs >= 5000) {
        lastBatteryCheckMs = nowMs;
        const float vbat = readBatteryVoltage();
        if (vbat >= VBAT_MIN_VALID && vbat < VBAT_LOW_THRESHOLD) {
            LOG_W("PWR", "Low battery detected: %.2f V < %.2f V", vbat, VBAT_LOW_THRESHOLD);
            rgb_led_set_state(STATE_LOW_BATTERY);
        }
    }
#endif
```

* **Penyaring Pengujian USB (`VBAT_MIN_VALID = 5.50f`)**:
  Mencegah salah deteksi saat pengujian USB meja tanpa baterai 2S terpasang (tegangan terbaca di bawah $5{,}50\text{ V}$).
* **Ambang Batas Rendah 2S (`VBAT_LOW_THRESHOLD = 6.60f`)**:
  Menjamin sistem memberi peringatan ketika rata-rata tegangan sel mencapai $3{,}30\text{ V/sel}$, menyisakan margin waktu aman sebelum baterai diputus oleh rangkaian proteksi sel internal.

---

## 7. Indikator Visual HMI: Mini RGB LED (Pola *Red Double-Blink*)

Seluruh status operasional dan peringatan daya baterai disampaikan melalui **Mini RGB LED** (Common Cathode, tipe KY-016 eksternal via konektor J3):

### A. Alokasi Pin Mini RGB LED (PCB Rev A)
* **Kanal Merah (Red)**: **GPIO 32** (Pin 8 ESP32 via $R_{10} = 220\,\Omega$).
* **Kanal Hijau (Green)**: **GPIO 26** (Pin 11 ESP32 via $R_{11} = 220\,\Omega$).
* **Kanal Biru (Blue)**: **GPIO 27** (Pin 12 ESP32 via $R_{12} = 220\,\Omega$).
* Dilengkapi resistor pulldown $R_{13}, R_{14}, R_{15} = 100\text{ k}\Omega$ ke ground untuk mencegah status *floating* saat booting.

### B. Pola Peringatan Baterai Rendah (*Red Double-Blink*)
Ketika status FSM berpindah ke `HAV_STATE_LOW_BATTERY`, task FreeRTOS `RGB_LED_FSM` mengaktifkan pola **Kedip Ganda Warna Merah setiap 3 detik**:
```cpp
case HAV_STATE_LOW_BATTERY: {
    // Double-blink RED every 3 seconds (periode 150 tick x 20 ms = 3000 ms)
    const uint32_t phase = tickCount % 150;
    if (phase < 5 || (phase >= 10 && phase < 15)) {
        apply_hw_color(COLOR_RED.r, COLOR_RED.g, COLOR_RED.b); // Merah Menyala
    } else {
        apply_hw_color(COLOR_OFF.r, COLOR_OFF.g, COLOR_OFF.b); // Padam
    }
    break;
}
```

---

## 8. Rangkuman Spesifikasi Pinout & Jalur Kelistrikan HAV Node (PCB Rev A)

| Pin ESP32 | Nama Sinyal | Tipe Pin | Terhubung ke Komponen | Spesifikasi & Proteksi Rangkaian |
| :---: | :---: | :---: | :---: | :--- |
| **GPIO 34** | `VBAT_SENSE` | Input Analog (ADC1_CH6) | Titik tengah pembagi tegangan baterai | $R_8 = 330\text{ k}\Omega$ ke $V_{BAT}$, $R_9 = 100\text{ k}\Omega$ ke GND, filter keramik $C_{11} = 100\text{ nF}$ ke GND (rasio $4{,}30$). |
| **GPIO 32** | `HAV_LED_R` | Output Digital (PWM) | Anoda Merah Mini RGB LED (J3 pin 2) | Seri $R_{10} = 220\ \Omega$ + pulldown $R_{13} = 100\text{ k}\Omega$. |
| **GPIO 26** | `HAV_LED_G` | Output Digital (PWM) | Anoda Hijau Mini RGB LED (J3 pin 3) | Seri $R_{11} = 220\ \Omega$ + pulldown $R_{14} = 100\text{ k}\Omega$. |
| **GPIO 27** | `HAV_LED_B` | Output Digital (PWM) | Anoda Biru Mini RGB LED (J3 pin 4) | Seri $R_{12} = 220\ \Omega$ + pulldown $R_{15} = 100\text{ k}\Omega$. |
| **GPIO 18** | `SPI_ADXL_SCK` | Output Digital (SPI Clock)| Pin SCL/VCLOCK ADXL345 | Bus SPI 5 MHz; resistor redaman seri $R_4 = 33\ \Omega$. |
| **GPIO 19** | `SPI_ADXL_MISO`| Input Digital (SPI MISO) | Pin SDO/MISO ADXL345 | Bus SPI 5 MHz jalur balik data. |
| **GPIO 23** | `SPI_ADXL_MOSI`| Output Digital (SPI MOSI)| Pin SDA/MOSI ADXL345 | Bus SPI 5 MHz; resistor redaman seri $R_5 = 33\ \Omega$. |
| **GPIO 25** | `ADXL_CS` | Output Digital (SPI CS) | Pin CS ADXL345 | Seri $R_6 = 33\ \Omega$ + pull-up $R_7 = 10\text{ k}\Omega$ ke `3V3_SYS`. |
| **GPIO 35** | `ADXL_INT1` | Input Digital (Interrupt)| Pin INT1 ADXL345 | Sinyal interupsi hardware Data-Ready 3200 Hz (`RISING`). |
| **Konektor J2**| `BAT_IN` | Power Input Header | Paket Baterai 2S Li-ion | $7{,}4\text{ V} - 8{,}4\text{ V}$ terproteksi PTC Fuse $F_1 = 0{,}75\text{ A}$ via SW3. |
| **3V3_SYS** | `VCC_3V3` | Power Rail Regulasi | Output Modul MP1584EN Buck | Rel daya $3{,}3\text{ V}$ bersih untuk ESP32 dan ADXL345. Dilengkapi elco Panasonic FR $47\ \mu\text{F} / 25\text{ V}$ + MLCC $100\text{ nF}$ + $22\ \mu\text{F}$. |
| **GND** | `GND` | Ground Sistem | Ground Bersama (Star Ground) | Terhubung ke bidang tembaga ground plane dan net-tie sensor $R_{16} = 0\ \Omega$. |
