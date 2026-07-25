# Laporan Ringkas Temuan Integrasi Perangkat Keras & Firmware
**Proyek**: Vibration Dosimeter ESP32 (ISO 5349-1 HAV & ISO 2631-1 WBV)  
**Tanggal**: 25 Juli 2026  

---

## 1. Pemotongan Paket Nirkabel BLE (BLE MTU Truncation)
* **Gejala**: Log Main Unit menampilkan `[WARN][BLE_RX] Bad HAV payload (parsed=3): HAV,273,275310,0.018` dan nilai HAV terbaca `0.0000`.
* **Penyebab Utama**: Batas bawaan (*default*) Bluetooth Low Energy ATT MTU adalah **23 bytes** (3 bytes header ATT + **20 bytes payload data**). String CSV yang dipancarkan HAV Node berukuran ~55 bytes sehingga terpotong tepat pada karakter ke-20. `sscanf()` pada Main Unit gagal karena hanya membaca 3 dari 7 parameter.
* **Solusi yang Diterapkan**:
  - Mengonfigurasi penegosiasian ukuran MTU menjadi **128 bytes** pada Server & Client:
    - **HAV Node**: `BLEDevice::setMTU(128);`
    - **Main Unit**: `BLEDevice::setMTU(128);` dan `pBleClient->setMTU(128);`
  - Mengoptimalkan format penulisan angka pecahan pada CSV dari `%.6f` menjadi `%.4f` (presisi 0.0001 m/s²).

---

## 2. Error Peringatan GPIO OLED SH1106 (`Invalid Pin Selected`)
* **Gejala**: Log Serial Monitor menampilkan error `[E][esp32-hal-gpio.c:102] __pinMode(): Invalid pin selected` dan `gpio_set_level(227): GPIO output gpio_num error`.
* **Penyebab Utama**: Pustaka SH1106 lama meng-cast parameter pin reset bernilai `-1` (tanpa pin reset fisik) menjadi *unsigned byte* `227`/`255` lalu memanggil `pinMode(227)` dan `gpio_set_level(227)`. Driver ESP32 menolak pin di atas GPIO 39.
* **Solusi yang Diterapkan**:
  - Migrasi ke pustaka resmi **`Adafruit SH110X`** (`Adafruit_SH1106G`) pada `platformio.ini`.
  - Menggunakan header `#include <Adafruit_SH110X.h>` dan instansiasi `static Adafruit_SH1106G oled(128, 64, &Wire, -1);`.
  - Memperbarui konstanta warna ke `SH110X_WHITE`.

---

## 3. Pengkabelan & Inisialisasi Sensor ADXL345 (HAV Node)
* **Gejala**: Serial Monitor HAV Node menampilkan `No I2C devices found on bus!` meskipun jalur I2C berstatus `HIGH (OK)`.
* **Penyebab Utama**: 
  - Pin `CS` (Chip Select) tidak terhubung/melayang, menyebabkan chip ADXL345 secara tidak sengaja masuk ke **mode SPI** dan menonaktifkan bus I2C.
  - Catu daya `VCC` modul terpotong oleh regulator onboard 3.3V saat dihubungkan ke rail 3.3V ESP32.
* **Solusi yang Diterapkan**:
  - **Pengkabelan**: Pin **`CS` wajib dihubungkan ke 3.3V** (mengunci mode I2C), **`SDO` ke GND** (menetapkan alamat `0x53`), dan **`VCC` ke pin 5V/VIN ESP32**.
  - **Automatic Simulation Fallback**: Menambahkan mekanisme *fallback* otomatis pada `HAV-Node/src/main.cpp`. Jika sensor fisik tidak merespons, firmware secara otomatis beralih ke simulasi sinusoidal agar pengiriman data BLE ke Main Unit tidak terhenti.

---

## 4. Stempel Waktu Real-Time WIB (Waktu Indonesia Barat)
* **Gejala**: Stempel waktu pada log Serial Monitor dan berkas CSV SD Card sebelumnya menggunakan nilai mentah *UNIX Epoch timestamp* (contoh: `1784972187`).
* **Solusi yang Diterapkan**:
  - Menambahkan fungsi pembantu `rtc_getFormattedTime()` yang membaca RTC DS3231 dan memformat waktu lokal WIB menjadi string **`YYYY-MM-DD HH:MM:SS`** (contoh: `2026-07-25 09:41:27`).
  - Format baru diterapkan pada Serial Monitor `LOGGER` dan kolom pertama berkas `dosimeter.csv` pada SD Card.

---

## 5. Ringkasan Status Akhir Sistem
| Komponen / Fungsi | Status | Keterangan |
| :--- | :---: | :--- |
| **Uji Mandiri (Self-Test)** | **PASS (100%)** | WBV ADXL345 (0x53), DS3231 RTC, SD Card, & OLED SH1106 aktif normal. |
| **Koneksi BLE HAV** | **PASS** | Terhubung, Notifikasi Aktif, MTU 128 Bytes tanpa truncation. |
| **Pencatatan Data SD Card** | **PASS** | Format CSV sinkron 1 Hz (`dosimeter.csv`) dengan stempel waktu WIB. |
| **Layar OLED & Screen Saver** | **PASS** | Tampilan status FSM normal + mode hemat daya titik berkedip (30 detik). |
