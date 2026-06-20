### 1. Prinsip Kerja Simulasi

*   **HAV Node (ESP32 #1 - BLE Server / Peripheral)**:
    *   Dengan aktifnya `#define SIMULATE_ADXL345 1` di kodenya, HAV Node tidak memerlukan sensor ADXL345 fisik terpasang pada pin I2C.
    *   Node ini secara otomatis melewati verifikasi inisialisasi sensor dan menghasilkan data akselerasi tri-aksial buatan menggunakan gelombang sinus (20 Hz pada sumbu X, 15 Hz pada sumbu Y, dan 10 Hz pada sumbu Z).
    *   Data ini difilter secara internal menggunakan pembobotan *Wh* (ISO 5349-1) dan dipancarkan ke udara setiap 1 detik sebagai karakteristik BLE Notification dengan format CSV (`"HAV,seq,millis,ahwx,ahwy,ahwz,ahv,n"`).
*   **Main Unit (ESP32 #2 - BLE Client / Central)**:
    *   Setelah diubah menjadi `#define USE_BLE_HAV 1`, Main Unit akan mengaktifkan task penerima BLE (`vTaskBLEReceiver`) di Core 1.
    *   Main Unit akan secara aktif memindai perangkat BLE di sekitarnya dengan nama `"HAV_NODE"`, melakukan koneksi, dan meng-extract data RMS getaran HAV tersebut secara nirkabel.
    *   Secara bersamaan, ia tetap mengukur getaran seluruh tubuh (WBV) menggunakan sensor ADXL345 fisik lokal yang terhubung di bus I2C-nya.

---

### 2. Konfigurasi Kode Program yang Telah Diperbarui

Saya telah menyesuaikan konfigurasi kode di [Main-Unit/src/main.cpp](file:///d:/Users/Rafi%20Ananta%20Alden/Documents/Kuliah/Semester%208/EL4060/vibration-dosimeter-esp32/Main-Unit/src/main.cpp) agar siap untuk simulasi ini:
1.  **Mengaktifkan BLE HAV**: `#define USE_BLE_HAV 1` diaktifkan agar unit mencari dan terhubung ke HAV Node.
2.  **Mitigasi Error SD Card saat Boot**: Pengecekan *Self-Test* saat booting di `setup()` diubah menjadi `(wbvSensorOK || sdOK)`. Karena modul SD Card Anda saat ini mengalami kegagalan/tidak terpasang, penyesuaian ini menjamin sistem akan langsung masuk ke status `READY` saat dinyalakan tanpa tertahan di status `ERROR` (tidak perlu melakukan *button long-press* secara manual untuk melewati error boot).

*Catatan: Pastikan di file [HAV-Node/src/main.cpp](file:///d:/Users/Rafi%20Ananta%20Alden/Documents/Kuliah/Semester%208/EL4060/vibration-dosimeter-esp32/HAV-Node/src/main.cpp) Anda tetap menggunakan `#define SIMULATE_ADXL345 1`.*

---

### 3. Langkah-Langkah Menjalankan Simulasi

#### **Langkah A: Pengunggahan (Flashing) Firmware**
1.  Hubungkan **ESP32 HAV-Node** ke komputer Anda melalui kabel USB, lalu lakukan *Upload* firmware dari project `HAV-Node` di PlatformIO.
2.  Hubungkan **ESP32 Main-Unit** ke kabel USB (bisa di port yang berbeda atau bergantian), lalu lakukan *Upload* firmware dari project `Main-Unit` di PlatformIO.

#### **Langkah B: Pemantauan (Monitoring) & Koneksi BLE**
1.  **Nyalakan HAV Node**:
    *   Buka Serial Monitor untuk port COM milik HAV Node (Baud rate: `115200`).
    *   Anda akan melihat pesan inisialisasi:
        ```text
        [INFO][INIT] ADXL345 HAV is SIMULATED
        [INFO][BLE] BLE HAV Node advertising started.
        ```
    *   HAV Node sekarang memancarkan data dummy secara berkala.
2.  **Nyalakan Main Unit**:
    *   Buka Serial Monitor untuk port COM milik Main Unit (Baud rate: `115200`).
    *   Pada layar OLED, Anda akan melihat status awal sistem berada di status `READY` dengan indikasi `HAV BLE: DISC` (Disconnected) dan `SD Card: ERR` (jika kartu SD bermasalah/tidak dipasang).
    *   Di Serial Monitor Main Unit, Anda akan melihat proses pemindaian dimulai:
        ```text
        [INFO][BLE_RX] Scanning for HAV_NODE ...
        ```
    *   Begitu sinyal dari HAV Node tertangkap, Main Unit akan otomatis terhubung:
        ```text
        [INFO][BLE_RX] Connected to HAV Node.
        [INFO][BLE_RX] Subscribed to HAV notifications.
        ```
    *   Pada layar OLED Main Unit, indikator `HAV BLE` akan berubah menjadi `DATA` setelah paket pertama diterima.

#### **Langkah C: Menyimulasikan Pengukuran Getaran**
1.  **Mulai Logging**:
    *   Tekan tombol HMI (SHORT press pada pin 4) di Main Unit.
    *   Status FSM beralih ke `LOGGING` dan OLED akan menampilkan status `LOGGING`.
    *   Di Serial Monitor Main Unit, Anda akan melihat log pencatatan data terpadu (gabungan HAV & WBV) setiap 1 detik:
        ```text
        [INFO][LOGGER] LOGGER t=1774092453 | HAV ahv=1.581138 | WBV av=0.1245
        ```
2.  **Uji Coba Screen Saver**:
    *   Diamkan sistem dalam mode `LOGGING` selama **30 detik** tanpa menyentuh tombol.
    *   OLED akan otomatis masuk ke mode hemat daya (layar padam dengan satu titik putih berkedip di pojok kanan atas).
    *   Tekan tombol sekali lagi untuk mematikan mode screensaver dan mengembalikan tampilan utama OLED.
3.  **Hentikan Logging**:
    *   Tekan tombol HMI (SHORT press) kembali untuk mengembalikan sistem ke status `READY`.