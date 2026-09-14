# Catatan Teknis: Subsistem Real-Time Clock (*RTC DS3231*)

Dokumen ini mengonsolidasikan seluruh catatan teknis, integrasi perangkat keras, format waktu, investigasi anomali daya, serta mekanisme kalibrasi otomatis modul **RTC DS3231** pada sistem dosimeter getaran (*Main Unit*).

---

## 1. Peran & Spesifikasi Subsistem RTC

Untuk menghasilkan berkas log data paparan getaran yang valid secara forensik dan standar K3 (ISO 5349-1 dan ISO 2631-1), Main Unit menggunakan modul RTC berbasis IC **Maxim DS3231**.
* **Antarmuka**: I2C (SDA=GPIO 21, SCL=GPIO 22) dengan alamat tetap `0x68`.
* **Akurasi**: Osilator kristal terkompensasi suhu internal (*TCXO - Temperature Compensated Crystal Oscillator*) dengan deviasi $\pm 2\text{ ppm}$ ($< 1\text{ menit per tahun}$).
* **Daya Cadangan**: Soket baterai koin **CR2032 (3V)** untuk mempertahankan pencatatan waktu saat ESP32 dimatikan.

---

## 2. Format Stempel Waktu Real-Time (WIB)

### A. Gejala Awal
Pada versi awal firmware, kolom pertama berkas log `dosimeter.csv` dan keluaran Serial Monitor menggunakan nilai mentah *UNIX Epoch timestamp* (contoh: `1784972187`). Nilai ini menyulitkan inspeksi langsung di lapangan oleh operator tanpa konversi perangkat lunak.

### B. Solusi yang Diterapkan
Diterapkan fungsi pembantu `rtc_getFormattedTime()` yang membaca register DS3231 dan memformat waktu lokal Waktu Indonesia Barat (WIB) menjadi string ISO yang mudah dibaca manusia:
* **Format**: **`YYYY-MM-DD HH:MM:SS`** (contoh: `2026-09-12 10:07:07`).
* **Implementasi**:
  ```cpp
  static void rtc_getFormattedTime(char *buf, size_t bufLen) {
      if (xSemaphoreTake(xMutexRTC, pdMS_TO_TICKS(5)) == pdTRUE) {
          if (rtcOK) {
              DateTime now = rtc.now();
              snprintf(buf, bufLen, "%04d-%02d-%02d %02d:%02d:%02d",
                       now.year(), now.month(), now.day(),
                       now.hour(), now.minute(), now.second());
          } else {
              uint32_t sec = (uint32_t)(millis() / 1000UL);
              snprintf(buf, bufLen, "U+%lu", sec); // Fallback relatif jika RTC gagal
          }
          xSemaphoreGive(xMutexRTC);
      } else {
          uint32_t sec = (uint32_t)(millis() / 1000UL);
          snprintf(buf, bufLen, "U+%lu", sec);
      }
  }
  ```
* **Penerapan**: Format ini digunakan serentak pada Serial Monitor `LOGGER` dan kolom pertama berkas `dosimeter.csv` pada kartu MicroSD pada laju 1 Hz.

---

## 3. Investigasi Temuan Kritis: Kegagalan Kalibrasi Waktu & OSF Flag

### A. Gejala Masalah
Saat menguji pencatatan data ke kartu SD pada tanggal **12 September 2026**, berkas `dosimeter.csv` tetap mencatat tanggal lama (**2026-08-28 11:51:xx**). Meskipun baterai koin diganti baru dan firmware di-flash ulang, waktu tetap tidak mau berpindah ke waktu saat ini.

### B. Analisis Akar Masalah (*Root Cause*)
1. **Ketergantungan Tunggal pada `rtc.lostPower()`**:
   Kode inisialisasi awal hanya memperbarui waktu jika fungsi `rtc.lostPower()` mengembalikan `true`:
   ```cpp
   // KODE LAMA (Bermasalah)
   rtcOK = rtc.begin();
   if (rtcOK && rtc.lostPower()) {
       rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
   }
   ```
2. **Karakteristik Bit Status OSF (*Oscillator Stop Flag*)**:
   `rtc.lostPower()` membaca bit 7 pada register status `0x0F` chip DS3231. Bit OSF ini bernilai `1` **hanya jika** kedua jalur daya (VCC utama dan VBAT baterai koin) turun di bawah batas kritis osilator (~1,5 V).
3. **Penyebab Kegagalan**:
   Saat pengguna mengganti baterai koin dalam kondisi ESP32 masih tersambung ke port USB, pin VCC modul RTC tetap teraliri tegangan 3,3V/5V. Osilator tidak pernah padam, sehingga flag OSF tetap bernilai `0`. Akibatnya, `rtc.lostPower()` bernilai `false`, pemanggilan `rtc.adjust(...)` **dilewati sepenuhnya**, dan jam DS3231 terus melanjutkan waktu lama dari registernya.

---

## 4. Solusi Kalibrasi Otomatis & Persistensi Waktu

### A. Logika Kalibrasi Cerdas (*Compile-Time Comparison*)
Logika inisialisasi diperbarui pada [`Main-Unit/src/main.cpp`](file:///d:/Users/Rafi%20Ananta%20Alden/Documents/Kuliah/Semester%208/EL4060/vibration-dosimeter-esp32/Main-Unit/src/main.cpp) dengan membandingkan waktu RTC saat ini (`now`) terhadap waktu kompilasi biner firmware (`compiled` dari makro `__DATE__` & `__TIME__`):

```cpp
// KODE BARU (Terkalibrasi Otomatis)
rtcOK = rtc.begin();
LOG_I("INIT", "DS3231 RTC init %s", rtcOK ? "OK" : "NOT FOUND");
if (rtcOK) {
    DateTime now = rtc.now();
    DateTime compiled = DateTime(F(__DATE__), F(__TIME__));
    if (now < compiled || rtc.lostPower()) {
        LOG_W("INIT", "RTC time (%04d-%02d-%02d %02d:%02d:%02d) is behind compile time! Updating to %04d-%02d-%02d %02d:%02d:%02d",
              now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second(),
              compiled.year(), compiled.month(), compiled.day(), compiled.hour(), compiled.minute(), compiled.second());
        rtc.adjust(compiled);
    }
}
```

### B. Keunggulan Arsitektur:
1. **Kalibrasi Otomatis Seketika**:
   Jika waktu internal RTC tertinggal dari waktu kompilasi biner (misal waktu RTC masih Agustus sedangkan biner dikompilasi September), kondisi `now < compiled` seketika terpenuhi dan jam otomatis disesuaikan ke waktu build saat ini.
2. **Aman Terhadap Reboot Berikutnya**:
   Setelah dikalibrasi, waktu jam terus berdetak maju berkat baterai koin. Pada reboot berikutnya, nilai `now >= compiled` terpenuhi sehingga waktu tidak akan pernah ter-reset mundur lagi ke waktu kompilasi lama.
3. **Ketahanan Lapangan**:
   Baterai koin CR2032 menjamin jam tetap berdetak saat ESP32 dicabut dari catu daya sepeda motor, menjamin integritas kronologis seluruh sesi pengukuran.
