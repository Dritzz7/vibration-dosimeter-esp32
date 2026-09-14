# Catatan Teknis: Subsistem Media Penyimpan (*MicroSD Card*)

Dokumen ini mengonsolidasikan seluruh catatan teknis, penanganan galat (*error handling*), mekanisme *fail-fast*, arsitektur FSM *safety interlock*, pencegahan penimpaan data (*append protection*), serta prosedur pemulihan (*hot-plug recovery*) pada modul **MicroSD** di Main Unit.

---

## 1. Arsitektur Antarmuka & Format Berkas Log

Modul MicroSD berfungsi sebagai media penyimpan data primer (*primary storage*) untuk merekam seluruh metrik paparan getaran pekerja secara persisten pada laju **1 Hz**.

### A. Pengkabelan Bus SPI & Pinout
* **Jalur Bus SPI**:
  * `SCK / CLK` : GPIO 18
  * `MISO`      : GPIO 19 (dikonfigurasi dengan `INPUT_PULLUP` di firmware)
  * `MOSI`      : GPIO 23
  * `CS` (Chip Select): GPIO 5 (dilengkapi resistor pull-up eksternal $10\text{ k}\Omega$ ke `3V3`)
* **Frekuensi Jam SPI**:
  * Inisialisasi awal dicoba pada laju **4 MHz**, dengan *fallback* otomatis ke **1 MHz** untuk menjamin kestabilan pada kabel jumper yang panjang.

### B. Struktur Berkas `dosimeter.csv`
Berkas disimpan pada direktori root (`/dosimeter.csv`) dengan baris header dan format 9 kolom:
```text
timestamp,ahwx,ahwy,ahwz,ahv,awx,awy,awz,av
2026-09-12 10:07:07,0.0182,0.0132,0.0282,0.0360,0.0000,0.0000,0.0000,0.0000
2026-09-12 10:07:08,0.1925,0.1184,0.4915,0.5409,0.5222,1.9034,2.4980,3.7250
```

---

## 2. Mitigasi Pencabutan Paksa Kartu SD (*Fail-Fast & Anti-Spam*)

### A. Gejala & Bahaya Awal
Saat kartu SD dicabut paksa ketika proses perekaman (`SYS_LOGGING`) sedang berlangsung:
1. **Banjir Pesan Error Tak Hingga (*Infinite Error Spam*)**:
   Firmware terus memanggil `SD.open("/dosimeter.csv", FILE_APPEND)` setiap detik. Driver SPI ESP-IDF (`sd_diskio.cpp`) mengirim perintah `GO_IDLE_STATE` dan mengalami *timeout* berulang kali karena tidak ada token yang diterima (`sdCommand(): no token received`).
2. **Pemblokiran CPU (*Blocking Timeout*)**:
   Setiap kegagalan memblokir CPU Core 1 selama ratusan milidetik, mengacaukan penjadwalan FreeRTOS dan memicu jitter pada task lain.
3. **Indikasi Palsu pada Pengguna**:
   Layar OLED tetap menampilkan `STATUS: LOGGING`, sehingga pengguna mengira data sedang tersimpan padahal kartu sudah lepas.

### B. Solusi yang Diterapkan: *Fail-Fast & FSM Transition*
Pada task `vTaskDataLogger`, ditambahkan penghitung kegagalan berurutan (`sdFailCount`). Jika penulisan berkas gagal **3 kali berturut-turut**:
```cpp
sdFailCount++;
LOG_E(TAG, "Cannot open CSV file for logging! (Fail %u/3)", sdFailCount);
if (sdFailCount >= 3) {
    LOG_E(TAG, "SD Card detached or unreadable! Unmounting and transitioning to SYS_ERROR.");
    sdOK = false;
    SD.end(); // Unmount filesystem untuk seketika memutus transaksi bus SPI
    setSystemState(SYS_ERROR);
}
```
* **Dampak**:
  - Panggilan `SD.end()` melepas mount point VFS dan mematikan sinyal SPI seketika.
  - Sistem berpindah ke status `SYS_ERROR`, menyebabkan task logger berhenti mencoba menulis ke SD card. **Banjir log error terminal terhenti seketika**.
  - Layar OLED otomatis keluar dari mode *screensaver* dan menampilkan status bahaya:
    ```text
    STATUS:  ERROR
    WBV Acc: OK
    SD Card: ERR
    ```

---

## 3. Arsitektur FSM: Penegakan Syarat Wajib & *Safety Interlock*

Untuk mencegah terjadinya sesi pencatatan kosong tanpa media penyimpanan:

### A. Kartu SD Sebagai Syarat Wajib pada `SYS_SELF_TEST`
Pada kode awal, pengujian periferal menggunakan logika `(wbvSensorOK || sdOK)` sehingga sistem tetap lolos ke `SYS_READY` meskipun kartu SD tidak ada. Logika ini diperbaiki secara ketat:
```cpp
// Pada vTaskHMIAndController (case SYS_SELF_TEST)
if (!sdOK) {
    LOG_I(TAG, "Re-probing SD card...");
    sd_reinit();
}
// WBV sensor dan SD card KEDUANYA WAJIB siap untuk memasuki status READY
const bool allOK = (wbvSensorOK && sdOK);
if (allOK) {
    setSystemState(SYS_READY);
} else {
    setSystemState(SYS_ERROR);
}
```
* Jika pengguna menekan tombol saat di `SYS_ERROR` tanpa memasang kembali kartu SD, sistem mencoba re-probe, gagal, dan **tetap bertahan di `SYS_ERROR`**.

### B. *Auto Re-probe* Latar Belakang di `SYS_READY`
Ketika sistem berada di `SYS_READY` dan mendeteksi `sdOK == false`:
```cpp
// Auto re-probe setiap 2 detik secara non-blocking
if (!sdOK && (now - lastSdCheckMs >= 2000UL)) {
    lastSdCheckMs = now;
    sd_reinit();
    if (sdOK) {
        LOG_I(TAG, "SD card auto-detected and mounted OK!");
    }
}
```
* Begitu kartu SD dicolokkan kembali ke soket, layar OLED seketika berganti dari `SD Card: ERR` $\rightarrow$ `SD Card: OK` dalam waktu maksimal 2 detik tanpa perlu menekan tombol apa pun.

### C. *Safety Interlock* Menuju `SYS_LOGGING`
Saat tombol ditekan di `SYS_READY` untuk memulai pengukuran:
```cpp
if (buttonEvent) {
    if (!sdOK) {
        sd_reinit(); // Verifikasi akhir keberadaan kartu
    }
    if (sdOK) {
        setSystemState(SYS_LOGGING);
    } else {
        setSystemState(SYS_ERROR); // Tolak merekam tanpa kartu SD!
        LOG_E(TAG, "FSM: Cannot start logging without SD Card! Returning to ERROR.");
    }
}
```

---

## 4. Perlindungan Riwayat Data Log (*Append Protection*)

### A. Masalah Penimpaan Berkas Awal
Pada implementasi awal, inisialisasi boot di `vTaskDataLogger` memanggil `SD.open("/dosimeter.csv", FILE_WRITE)`. Parameter `FILE_WRITE` berisiko memotong (*truncate*) dan menghapus seluruh data pengukuran sebelumnya setiap kali ESP32 di-restart atau mengalami mati daya mendadak di lapangan.

### B. Solusi Perlindungan Berkas
Inisialisasi diperbaiki dengan memeriksa keberadaan berkas terlebih dahulu:
```cpp
// Inisialisasi aman saat boot (Cold Boot & Reboot Protection)
if (sdOK) {
    if (xSemaphoreTake(xMutexSD, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (!SD.exists("/dosimeter.csv")) {
            File f = SD.open("/dosimeter.csv", FILE_WRITE);
            if (f) {
                f.println("timestamp,ahwx,ahwy,ahwz,ahv,awx,awy,awz,av");
                f.close();
            }
        }
        xSemaphoreGive(xMutexSD);
    }
}
```
* **Hasil**:
  1. Header CSV hanya ditulis satu kali saat kartu SD masih kosong / berkas belum ada.
  2. Jika berkas sudah ada dari sesi sebelumnya, berkas dibiarkan utuh.
  3. Seluruh penulisan data rutin selalu menggunakan mode `FILE_APPEND`, menjamin data baru tersambung secara kontinu di baris paling bawah.
