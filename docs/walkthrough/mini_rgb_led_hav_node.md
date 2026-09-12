# Catatan Teknis: Implementasi Mini RGB LED pada Edge Processor HAV Node

Dokumen ini mendokumentasikan implementasi indikator visual **Mini RGB LED** pada modul *Edge Processor* **HAV Node** (Hand-Arm Vibration, ISO 5349-1), mencakup desain arsitektur perangkat lunak, skema perkabelan, pemetaan *Finite State Machine* (FSM), serta temuan kritis konflik perangkat keras (*hardware conflict*) antara GPIO dan BLE stack.

---

## 1. Latar Belakang & Tujuan

Node sensor HAV dirancang beroperasi tanpa layar OLED maupun RTC lokal untuk meminimalkan dimensi fisik dan konsumsi daya saat dipasang pada setang kemudi sepeda motor. Sebagai antarmuka manusia-mesin (*Human-Machine Interface* / HMI) ringkas, sebuah modul **Mini RGB LED (KY-016 / SMD)** digunakan untuk memberikan umpan balik visual terkait status operasional sistem secara *real-time*.

---

## 2. Skema Pengkabelan (*Hardware Wiring*)

Modul yang digunakan adalah **KY-016 RGB LED (Common Cathode)**. 

### Tabel Pinout Akhir

| Pin Modul KY-016 | Pin ESP32 Node HAV | Deskripsi Fungsi | Karakteristik / Catatan |
| :--- | :--- | :--- | :--- |
| **R (Red)** | **GPIO 32** | Kanal LED Merah | ADC1_CH4 (Aman dari BLE & RTC) |
| **G (Green)** | **GPIO 26** | Kanal LED Hijau | DAC2 / GPIO umum |
| **B (Blue)** | **GPIO 27** | Kanal LED Biru | Touch7 / GPIO umum |
| **- (GND)** | **GND** | Ground bersama | Dihubungkan ke ground sistem ESP32 |

> **Konfigurasi Polaritas Firmware:**
> - Tipe: **Common Cathode** (`#define RGB_LED_COMMON_CATHODE 1`)
> - Sifat: **Active HIGH** (Duty cycle 255 = LED menyala penuh, 0 = LED padam).

---

## 3. Investigasi Temuan Kritis: Konflik Perangkat Keras GPIO 25 vs BLE Stack

Selama tahap verifikasi, ditemukan fenomena anomalus di mana LED merah tidak menyala sama sekali pada pengujian integrasi HAV Node, padahal kode unit test mandiri (`tests/Mini-RGB-LED`) dapat menyalakan warna merah dengan normal.

### Analisis Akar Masalah (*Root Cause Analysis*)
1. **Penggunaan Pin Awal**: Awalnya kanal Merah (R) dialokasikan pada **GPIO 25**.
2. **Karakteristik GPIO 25**: Pada arsitektur ESP32, GPIO 25 terhubung ke periferal **DAC1**, **ADC2_CH8**, dan domain daya **RTC IO 6**.
3. **Konflik Subsistem BLE**:
   - Firmware HAV Node mengaktifkan stack radio Bluetooth Low Energy (**ESP-IDF Bluedroid**).
   - Saat subsistem radio BLE aktif, register daya RTC dan periferal ADC2/DAC diambil alih oleh manajemen daya internal RF/Wi-Fi/BT.
   - Hal ini menyebabkan konfigurasi output digital/PWM pada GPIO 25 terdisosiasi dari GPIO Matrix, sehingga pin tetap berada pada logika *low* atau *floating* saat BLE aktif.
4. **Solusi Perbaikan**:
   - Memindahkan jalur kanal Merah dari **GPIO 25** ke **GPIO 32**.
   - **GPIO 32** berada di bawah domain periferal **ADC1 (ADC1_CH4)** yang tidak terpengaruh oleh aktivasi stack radio Wi-Fi/BLE pada ESP32.

---

## 4. Pemetaan Status Sistem (FSM) & Pola Kedip LED

Pola kedip dan warna LED merepresentasikan status internal mesin keadaan (*Finite State Machine* / FSM) dari HAV Node:

| Skenario | Status FSM HAV Node | Warna LED | Pola / Perilaku Kedip | Frekuensi / Waktu | Indikasi Lapangan |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **A** | `HAV_STATE_INIT`<br>`HAV_STATE_BLE_ADVERTISING` | **Biru** | Berkedip lambat (*Slow Blink*) | 1,0 Hz (500 ms ON, 500 ms OFF) | Sistem sedang booting dan memancarkan sinyal BLE, menunggu koneksi dari Main Unit. |
| **B** | `HAV_STATE_LOGGING_NORMAL` | **Hijau** | Bernapas halus (*Smooth Breathing*) | Siklus 2,0 detik (sinusoidal, 35–255 PWM) | Terhubung dengan Main Unit via BLE, sensor ADXL345 aktif mengambil sampel getaran pada 3200 Hz. |
| **D** | `HAV_STATE_COMM_LOST` | **Merah** | Berkedip cepat (*Rapid Blink*) | 2,0 Hz (250 ms ON, 250 ms OFF) | Koneksi BLE terputus saat pengukuran berlangsung. |
| **E** | `HAV_STATE_ERROR` | **Merah** | Berkedip cepat (*Rapid Blink*) | 2,0 Hz (250 ms ON, 250 ms OFF) | Kegagalan inisialisasi sensor ADXL345 atau galat perangkat keras I2C. |
| **-** | `HAV_STATE_LOW_BATTERY` | **Merah** | Kedip ganda (*Double Blink*) | Dua pulsa 100 ms setiap periode 3,0 detik | Tegangan baterai berada di bawah batas aman (< 3,40 V). |

### Rasionalisasi Desain:
* **Penghapusan Skenario C (Peringatan Paparan Getaran EAV/ELV)**: Sistem ini dikembangkan sebagai artefak penelitian dan instrumen dosimetri getaran ilmiah, bukan alat peringatan langsung (*warning dashboard*) untuk pengendara. Oleh karena itu, logika Skenario C dihapus agar tidak membebani komputasi node sensor dan memastikan status visual tetap intuitif bagi peneliti.
* **Konsolidasi Warna Peringatan ke Merah**: Skenario D (*Comm Lost*), Skenario E (*Hardware Fault*), dan *Low Battery* distandarisasi menggunakan warna **Merah** dengan pola kedip yang berbeda untuk mempertegas kondisi anomalus sistem.

---

## 5. Arsitektur Perangkat Lunak (*Non-Blocking FreeRTOS Task*)

Pengambilan data akselerasi ADXL345 berlangsung pada laju sampling tinggi **3200 Hz** (periode 312,5 µs) dengan perhitungan biquad filter *Wh* secara *real-time*. Pola kedip LED **sama sekali tidak boleh menggunakan fungsi pemblokir (*blocking delay*)** seperti `delay()` karena akan merusak *jitter* waktu sampling getaran.

### Fitur Implementasi Firmware:
1. **Dedicated FreeRTOS Task (`vTaskRgbLedPattern`)**:
   - Dijalankan secara independen pada **Core 1** dengan prioritas latar belakang (`priority = 1`).
   - Pembaruan kedip/efek dilakukan secara deterministik pada frekuensi **50 Hz** (interval 20 ms menggunakan `vTaskDelayUntil`).
2. **Sinkronisasi Thread-Safe**:
   - Perubahan status dari loop utama ke task LED diproteksi menggunakan FreeRTOS Mutex (`s_mutex_led`) via fungsi `rgb_led_set_state(new_state)`.
3. **Pembangkitan PWM Tanpa Konflik Timer**:
   - Pengendalian intensitas warna menggunakan `analogWrite()` bawaan Arduino ESP32 Core yang secara dinamis mengalokasikan kanal LEDC hardware tanpa memblokir CPU.
4. **Efek Breathing Sinusoidal**:
   - Diimplementasikan dengan rumus sinusoidal berbasis *tick*:
     $$\text{duty} = 35 + \left(\frac{1 - \cos(\omega t)}{2}\right) \times (255 - 35)$$
     Formula ini menghasilkan modulasi kecerahan yang lembut dan nyaman dilihat tanpa patah-patah.

---

## 6. Lokasi Berkas Sumber Terkait

- **Firmware Utama**: [`HAV-Node/src/main.cpp`](file:///d:/Users/Rafi%20Ananta%20Alden/Documents/Kuliah/Semester%208/EL4060/vibration-dosimeter-esp32/HAV-Node/src/main.cpp)
- **Unit Test Perangkat Keras LED**: [`tests/Mini-RGB-LED/src/main.cpp`](file:///d:/Users/Rafi%20Ananta%20Alden/Documents/Kuliah/Semester%208/EL4060/vibration-dosimeter-esp32/tests/Mini-RGB-LED/src/main.cpp)
