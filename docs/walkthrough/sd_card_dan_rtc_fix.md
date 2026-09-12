# Catatan Teknis: Optimasi Penanganan Kartu SD dan Perbaikan Stempel Waktu RTC

Dokumen ini mendokumentasikan pemecahan masalah serta optimasi perangkat lunak pada unit pemroses utama (**Main Unit**) terkait sinkronisasi stempel waktu **RTC DS3231**, mitigasi pelepasan paksa media penyimpan **MicroSD**, serta ketahanan analisis pasca-pemrosesan pada berkas log `dosimeter.csv`.

---

## 1. Perbaikan Stempel Waktu RTC DS3231

### Gejala & Temuan Masalah
Pada pengujian pencatatan data ke kartu SD (`dosimeter.csv`), stempel waktu menunjukkan tanggal lama (**2026-08-28**) alih-alih tanggal aktual (**2026-09-12**). Meskipun baterai koin telah diganti baru dan firmware di-flash ulang, stempel waktu tetap tidak berubah.

### Analisis Akar Masalah (*Root Cause*)
1. **Ketergantungan Tunggal pada `rtc.lostPower()`**:
   Kode inisialisasi awal hanya menyetel waktu kompilasi jika `rtc.lostPower()` bernilai `true`.
   ```cpp
   // KODE LAMA (Bermasalah)
   if (rtcOK && rtc.lostPower()) {
       rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
   }
   ```
2. **Perilaku Daya Modul RTC**:
   `rtc.lostPower()` hanya membaca bit status **OSF (*Oscillator Stop Flag*)** pada register `0x0F` chip DS3231. Jika penggantian baterai koin dilakukan saat ESP32 masih tersambung ke USB, pin VCC modul RTC tetap dialiri tegangan 3,3V/5V. Osilator internal tidak pernah berhenti berdetak sehingga OSF tetap bernilai `0` (`lostPower() == false`).
3. Akibatnya, baris `rtc.adjust(...)` **dilewati sepenuhnya**, dan chip DS3231 terus melanjutkan hitungan dari waktu lama yang tersimpan di registernya.

### Solusi yang Diterapkan
Logika inisialisasi pada `Main-Unit/src/main.cpp` diubah dengan menambahkan komparasi waktu kompilasi biner:
```cpp
// KODE BARU (Terkalibrasi Otomatis)
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
* **Hasil**: Jika waktu pada register RTC lebih lampau daripada waktu biner dikompilasi (`now < compiled`), firmware langsung memperbarui jam RTC ke waktu saat ini.
* **Keandalan Boot**: Pada *reboot* berikutnya, karena waktu jam sudah berjalan maju (`now >= compiled`), jam tidak akan ter-reset ulang dan terus berdetak akurat menggunakan daya baterai koin cadangan.

---

## 2. Optimasi Penanganan Kartu SD (*Fail-Fast & Recovery*)

### Gejala & Temuan Masalah
Saat kartu MicroSD dicabut paksa ketika proses pengukuran (`SYS_LOGGING`) sedang berjalan:
* Terminal Serial Monitor mengalami **banjir pesan error tak hingga (*infinite spam*)** dari driver SPI ESP-IDF (`sdCommand(): no token received`, `GO_IDLE_STATE failed`, `fopen failed`).
* Bus SPI mengalami *blocking timeout* ratusan milidetik setiap detik pada Core 1.
* Status sistem pada layar OLED tetap menampilkan `LOGGING`, memberikan indikasi palsu kepada pengguna bahwa data sedang direkam.
* Saat kartu dipasang kembali, ESP32 tetap gagal menulis karena *mount point* VFS dalam kondisi rusak dan tidak pernah di-inisialisasi ulang.

### Solusi yang Diterapkan (Pendekatan 1: *Fail-Fast & FSM Error State Transition*)

#### A. Penghitung Kegagalan Berurutan & Unmount Mandiri (`vTaskDataLogger`)
Menambahkan variabel `sdFailCount` pada task logger. Jika pembukaan berkas gagal 3 kali berturut-turut:
```cpp
sdFailCount++;
LOG_E(TAG, "Cannot open CSV file for logging! (Fail %u/3)", sdFailCount);
if (sdFailCount >= 3) {
    LOG_E(TAG, "SD Card detached or unreadable! Unmounting and transitioning to SYS_ERROR.");
    sdOK = false;
    SD.end(); // Unmount filesystem untuk seketika memutus transaksi SPI
    setSystemState(SYS_ERROR);
}
```
* **Dampak**: Begitu masuk ke `SYS_ERROR`, task logger langsung menghentikan seluruh percobaan penulisan SPI. Banjir pesan error seketika berhenti, dan CPU terbebas dari *blocking timeout*.

#### B. Indikasi Visual Langsung pada OLED
Ketika FSM beralih ke `SYS_ERROR`:
* Mode *screensaver* otomatis dinonaktifkan.
* Layar OLED langsung menampilkan peringatan status:
  ```text
  STATUS:  ERROR
  WBV Acc: OK
  SD Card: ERR
  ```

#### C. Pemulihan Koneksi Bersih (*Re-probe Helper*)
Dibuat fungsi pembantu `sd_reinit()`:
```cpp
static bool sd_reinit(void) {
    if (xSemaphoreTake(xMutexSD, pdMS_TO_TICKS(100)) == pdTRUE) {
        SD.end();
        pinMode(PIN_SD_CS, OUTPUT);
        digitalWrite(PIN_SD_CS, HIGH);
        pinMode(PIN_SPI_MISO, INPUT_PULLUP);
        
        sdOK = SD.begin(PIN_SD_CS, SPI, 4000000);
        if (!sdOK) {
            sdOK = SD.begin(PIN_SD_CS, SPI, 1000000);
        }
        if (sdOK) {
            if (!SD.exists("/dosimeter.csv")) {
                File f = SD.open("/dosimeter.csv", FILE_WRITE);
                if (f) {
                    f.println("timestamp,ahwx,ahwy,ahwz,ahv,awx,awy,awz,av");
                    f.close();
                }
            }
        }
        xSemaphoreGive(xMutexSD);
    }
    return sdOK;
}
```
Saat pengguna memasang kembali kartu SD dan menekan tombol fisik, FSM masuk ke `SYS_SELF_TEST`. Jika `!sdOK`, fungsi `sd_reinit()` dipanggil secara otomatis. Jika kartu terdeteksi, status kembali ke `SD Card: OK` dan sistem siap digunakan kembali (`SYS_READY`).

---

## 3. Pencegahan Penghapusan Data Log Lama (*Append Protection*)

Pada implementasi lama, inisialisasi boot awal di `vTaskDataLogger` memanggil `SD.open("/dosimeter.csv", FILE_WRITE)`. Hal ini berisiko menghapus data pengukuran sebelumnya setiap kali ESP32 mati total atau di-restart di lapangan.

Logika inisialisasi diperbaiki menjadi:
```cpp
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
  - Berkas baru hanya dibuat jika `/dosimeter.csv` belum ada.
  - Jika berkas sudah ada dari sesi sebelumnya, berkas dibiarkan utuh.
  - Seluruh pencatatan data getaran selanjutnya selalu menggunakan mode `FILE_APPEND`, sehingga riwayat data tetap tersambung secara kontinu.

---

## 4. Ketahanan Utilitas Analisis (`calc_dosimeter.py`)

Skrip pasca-pemrosesan data `util/calc_dosimeter.py` terbukti mampu menangani diskontinuitas akibat pencabutan kartu maupun restart sistem secara andal:

| Karakteristik Log | Perilaku `calc_dosimeter.py` | Dampak terhadap Hasil |
| :--- | :--- | :--- |
| **Stempel Waktu Terputus / Melompat** | Durasi total dihitung berdasarkan jumlah sampel ($T_{total} = N \times 1\text{ s}$), bukan selisih waktu dinding (`t_akhir - t_awal`). | Nilai dosis paparan $A(8)$ dan $VDV$ tetap akurat dan tidak terdistorsi oleh lamanya kartu dicabut. |
| **Baris Terpotong (*Truncated Row*)** | Pengecekan `if len(row) < 9` dan blok `try-except ValueError` memfilter baris cacat. | Baris rusak akibat pencabutan daya diabaikan secara transparan tanpa menyebabkan program *crash*. |
| **Header Ganda di Tengah File** | Percobaan konversi kata `"ahwx"` ke *float* memicu `ValueError` dan baris dilewati. | Aman, tidak merusak parsing rekaman data numerik berikutnya. |
| **Visualisasi Grafik Matplotlib** | Sumbu horizontal menggunakan indeks waktu kontinu akumulatif (`np.arange(N)`). | Grafik 4-panel tampil mulus tanpa celah kosong ratusan jam. |

---

## 5. Ringkasan Status Akhir

| Komponen | Status | Hasil Pengujian |
| :--- | :---: | :--- |
| **RTC DS3231** | **PASS** | Terkalibrasi otomatis ke tanggal aktual (12 September 2026), persisten berkat baterai koin baru. |
| **Mitigasi Cabut Kartu SD** | **PASS** | Error spam berhenti seketika (3 kali gagal), FSM beralih ke `SYS_ERROR`, OLED menampilkan `SD Card: ERR`. |
| **Pemulihan Kartu SD** | **PASS** | Kartu dapat dipasang kembali dan pulih normal via tombol HMI (`SYS_SELF_TEST` $\rightarrow$ `SYS_READY`). |
| **Integritas Berkas CSV** | **PASS** | Data lama tidak tertimpa saat restart, data baru menyambung (*append*) dengan aman. |
| **Pasca-Pemrosesan Python** | **PASS** | Evaluasi metrik ISO 5349-1 (HAV) dan ISO 2631-1 (WBV) selesai < 0,1 detik tanpa kesalahan parsing. |
