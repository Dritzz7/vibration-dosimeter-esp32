# Catatan Teknis: Subsistem Transmisi Nirkabel (*BLE TX/RX*)

Dokumen ini mengonsolidasikan seluruh catatan teknis, arsitektur protokol komunikasi Bluetooth Low Energy (BLE), mitigasi pemotongan paket (*MTU truncation*), serta temuan kritis konflik perangkat keras antara stack radio BLE dan GPIO periferal pada sistem dosimeter getaran.

---

## 1. Arsitektur Komunikasi Nirkabel BLE

Untuk memisahkan node sensor tangan (stang kemudi) dan unit pemroses utama (di bawah jok) tanpa kabel yang mengganggu pengendara, komunikasi data getaran Hand-Arm Vibration (HAV) ditransmisikan melalui tautan nirkabel **Bluetooth Low Energy (BLE)** berbasis arsitektur **GATT Server-Client**.

### A. Peran & Konfigurasi Perangkat
* **HAV Node**: Berperan sebagai **BLE Server (GATT Peripheral)** dengan nama perangkat `"HAV_NODE"`.
  - Service UUID: `9b6f0001-5f5a-4f0d-9d7f-000000000001`
  - Characteristic UUID: `9b6f0002-5f5a-4f0d-9d7f-000000000002` (Property: `NOTIFY`)
* **Main Unit**: Berperan sebagai **BLE Client (GATT Central)** yang melakukan *scanning*, *connect*, dan *subscribe* (indikasi notifikasi) ke karakteristik HAV.

### B. Format Payload Transmisi
Data akselerasi HAV terbobot $W_h$ diakumulasikan setiap epoch 1 detik (3200 sampel) lalu dipancarkan dalam format string CSV:
```text
"HAV,<seq>,<millis_ms>,<ahwx>,<ahwy>,<ahwz>,<ahv>,<n_samples>"
```
* **Contoh Payload**: `HAV,627,627100,0.0179,0.0194,0.0229,0.0349,3200`
* Format angka pecahan dioptimalkan menggunakan presisi `%.4f` (resolusi $0{,}0001\text{ m/s}^2$) untuk memadatkan panjang string tanpa mengurangi akurasi metrik ISO 5349-1.

---

## 2. Investigasi Temuan Kritis: Pemotongan Paket BLE (*BLE MTU Truncation*)

### A. Gejala Masalah
Pada pengujian awal integrasi, Serial Monitor Main Unit menampilkan pesan peringatan berulang:
```text
[WARN][BLE_RX] Bad HAV payload (parsed=3): HAV,273,275310,0.018
```
Akibatnya, nilai getaran HAV pada Main Unit selalu terbaca `0.0000` dan fungsi `sscanf()` gagal mengekstrak parameter data.

### B. Analisis Akar Masalah (*Root Cause*)
1. **Batas Bawaan BLE ATT MTU**:
   Nilai standar *Maximum Transmission Unit* (MTU) pada protokol Bluetooth Low Energy bawaan adalah **23 bytes** (3 bytes untuk *header* ATT + **20 bytes payload data**).
2. **Ukuran String CSV**:
   String data HAV berukuran rata-rata **~55 bytes**. Akibat batas bawaan tersebut, paket data terpotong tepat pada karakter ke-20 (`...0.018`). Pemotongan ini merusak struktur string sehingga Main Unit hanya berhasil membaca 3 dari 7 parameter yang dikirimkan.

### C. Solusi yang Diterapkan
Mengonfigurasi penegosiasian ukuran buffer ATT MTU menjadi **128 bytes** pada kedua sisi (Server dan Client):
1. **Pada HAV Node (Server)**:
   ```cpp
   BLEDevice::setMTU(128);
   ```
2. **Pada Main Unit (Client)**:
   ```cpp
   BLEDevice::setMTU(128);
   pBleClient->setMTU(128); // Meminta negosiasi MTU saat koneksi terbentuk
   ```
* **Hasil**: Seluruh baris data string CSV (~55 bytes) diterima utuh 100% dalam satu transaksi paket notifikasi tanpa terpotong.

---

## 3. Investigasi Temuan Kritis: Konflik Perangkat Keras GPIO 25 vs Stack BLE

### A. Gejala Masalah
Saat menambahkan indikator visual LED pada HAV Node, kanal Merah yang dihubungkan ke **GPIO 25** sama sekali tidak menyala pada firmware utama. Namun, saat menjalankan kode pengujian mandiri sederhana (`tests/Mini-RGB-LED`), LED merah pada GPIO 25 menyala normal.

### B. Analisis Akar Masalah (*Root Cause*)
1. **Karakteristik Multi-fungsi GPIO 25**:
   Pada mikrokontroler ESP32, GPIO 25 terhubung ke periferal internal **DAC1**, **ADC2_CH8**, dan domain daya **RTC IO 6**.
2. **Perebutan Domain oleh Stack Bluedroid**:
   Firmware HAV Node menginisialisasi stack radio Bluetooth (**ESP-IDF Bluedroid**). Saat stack radio RF/BLE aktif, manajemen daya internal chip mengambil alih domain RTC dan register ADC2/DAC.
3. **Pelepasan GPIO Matrix**:
   Hal ini menyebabkan konfigurasi keluaran digital/PWM (LEDC) pada GPIO 25 terdisosiasi dari GPIO Matrix perangkat keras. Pin terkunci pada level rendah (*low*) atau melayang (*floating*) setiap kali stack BLE beroperasi.
4. **Bukti Validasi**:
   Pada unit test mandiri, stack BLE tidak diinisialisasi sehingga domain RTC bebas dan GPIO 25 berfungsi normal.

### C. Solusi yang Diterapkan
Memindahkan jalur keluaran kanal Merah dari **GPIO 25** ke **GPIO 32**:
```cpp
// Konfigurasi pinout HAV Node yang aman terhadap BLE
#define RGB_LED_PIN_RED    32  // ADC1_CH4 — Bebas dari domain RTC/DAC dan aman saat BLE aktif
#define RGB_LED_PIN_GREEN  26  // GPIO 26
#define RGB_LED_PIN_BLUE   27  // GPIO 27
```
* **GPIO 32** berada di bawah blok periferal **ADC1 (ADC1_CH4)** yang secara arsitektural independen dan tidak pernah diganggu oleh aktivasi radio Wi-Fi maupun Bluetooth pada ESP32.
* **Hasil**: Kanal LED Merah berfungsi sempurna dan stabil secara bersamaan dengan transmisi nirkabel BLE.
