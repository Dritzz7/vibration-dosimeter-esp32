# Subsistem GPS: u-blox NEO-6M (Main Unit)

Dokumen teknis ini mendokumentasikan secara komprehensif arsitektur perangkat keras, integrasi *multitasking* FreeRTOS, penanganan berkas log MicroSD, mitigasi galat (*error handling*), riwayat perbaikan masalah (*bug fixes*), serta hasil pengujian empiris subsistem GPS **u-blox NEO-6M** pada **Main Unit Vibration Dosimeter ESP32** berdasarkan skenario uji terstruktur pada [`docs/test_scenarios/gps.md`](file:///d:/Users/Rafi%20Ananta%20Alden/Documents/Kuliah/Semester%208/EL4060/vibration-dosimeter-esp32/docs/test_scenarios/gps.md).

---

## 1. Ikhtisar & Tujuan Integrasi

Integrasi modul GPS bertujuan untuk melengkapi pencatatan data getaran paparan manusia (*Hand-Arm Vibration* ISO 5349-1 dan *Whole-Body Vibration* ISO 2631-1) dengan konteks spasial dan kinematika kendaraan:
1. **Kecepatan Kendaraan (*Ground Speed*):** Menganalisis korelasi antara kecepatan laju sepeda motor dengan intensitas getaran mekanik atau guncangan transien jalan (*potholes*, polisi tidur, jalan bergelombang).
2. **Koordinat Geospasial (*Latitude, Longitude*):** Memetakan lokasi (*vibration spatial mapping*) titik-titik jalan dengan paparan getaran berlebih.
3. **Ketinggian (*Altitude*):** Memantau profil kontur medan jalan (*elevation profile*).
4. **Metrik Kualitas Sinyal (*HDOP & Satellites Tracked*):** Menyediakan indikator keandalan dan presisi geometri data posisi.

---

## 2. Riwayat Perbaikan Masalah & Solusi yang Diimplementasikan

Selama proses integrasi dan pengujian sistem, ditemukan beberapa kendala teknis yang telah dianalisis dan diselesaikan secara tuntas:

| No | Kendala yang Ditemukan | Akar Masalah (*Root Cause*) | Solusi & Perbaikan yang Diimplementasikan |
| :-: | :--- | :--- | :--- |
| **1** | Tidak ada header kolom GPS saat melanjutkan pencatatan pada kartu SD. | Ketidakkonsistenan antara struktur log lama (9 kolom) dan struktur log baru (17 kolom). | **Standardisasi format tunggal 17-kolom.** Seluruh proses logging kini menggunakan format standar 17-kolom secara eksklusif. |
| **2** | Tidak ada header sama sekali jika proses *logging* dimulai saat berkas `/dosimeter.csv` belum ada atau baru dibuat. | `vTaskDataLogger` membuka berkas langsung dengan mode `FILE_APPEND`. Pada berkas baru, penulisan langsung menambahkan baris data pertama tanpa menuliskan baris judul kolom (*header*). | **Penulisan Header Deterministik.** Pada setiap siklus penulisan (serta saat *booting* dan `sd_reinit()`), sistem mengecek `!SD.exists() \|\| checkF.size() == 0`. Jika berkas belum ada atau berukuran 0 byte, baris header 17-kolom dituliskan terlebih dahulu. |
| **3** | Saat kabel GPS (TX, RX, VCC, GND) terputus, OLED tetap menampilkan `GPS: NO FIX` (bukan `GPS: ERR`), bahkan setelah ESP32 di-restart. | 1. Pin GPIO 16 (RX2) berkarakteristik CMOS mengambang (*floating*), menangkap derau frekuensi tinggi yang disalahartikan UART sebagai byte framing error (0x00). Akibatnya `Serial2.available() > 0` terpenuhi.<br>2. Tidak ada mekanisme pemantauan aliran data saat sistem sedang berjalan (*runtime*). | **Tiga Lapis Mitigasi Galat:**<br>1. Mengaktifkan *pull-up* internal via `pinMode(PIN_GPS_RX, INPUT_PULLUP)` agar jalur RX terkunci pada logika HIGH (*idle*).<br>2. Probing *self-test* memvalidasi aliran karakter NMEA (`$`), bukan sembarang byte.<br>3. *Runtime Watchdog* 3 detik pada `vTaskGPS`: jika tidak ada kalimat NMEA valid $> 3000\text{ ms}$, flag `gpsHardwareOK` dipaksa `false` sehingga OLED seketika menampilkan `GPS: ERR`. |
| **4** | Parameter `altitude` dan `hdop` tidak muncul pada Serial Monitor (hanya ada `fix`, `sat`, dan `spd`). | Format string `LOG_I` pada *task* logger belum menyertakan argumen ketinggian dan HDOP. | **Pembaruan Telemetri Serial:** Ditambahkan `alt=%.1f m` (dikonversi dari integer milimeter ke meter) dan `hdop=%.2f` pada format keluaran serial logger. |
| **5** | Keinginan pengguna agar kecepatan kendaraan tidak memenuhi layar OLED saat mode *logging*. | Penambahan teks pada OLED berisiko membuat antarmuka terlalu padat (*cluttered*) saat berkendara. | **Penyederhanaan Tampilan HMI:** Kecepatan kendaraan tidak ditampilkan pada OLED saat mode `LOGGING`. Layar OLED tetap mempertahankan antarmuka minimalis dan kontras dengan indikator status `GPS: OK (xs)` / `GPS: ERR`. |

---

## 3. Antarmuka Perangkat Keras & Kelistrikan

Modul GPS NEO-6M dihubungkan ke pengontrol bawaan **Hardware UART2** pada ESP32:

| Pin Modul NEO-6M | Pin ESP32 | Mode I/O | Keterangan Kelistrikan & Penanganan Jalur |
| :--- | :--- | :--- | :--- |
| **VCC** | **3V3** (atau 5V/VIN) | Power Rail | Konsumsi arus rata-rata ~45–60 mA (akuisisi) dan ~35 mA (pelacakan). |
| **GND** | **GND** | Ground | Ground bersama sistem (*common ground*). |
| **TX** | **GPIO 16 (RX2)** | Input UART | Menerima aliran kalimat NMEA (`$GPRMC`, `$GPGGA`) 9600 baud. **Wajib pull-up internal.** |
| **RX** | **GPIO 17 (TX2)** | Output UART | Mengirim perintah konfigurasi (UBX) dari ESP32 jika diperlukan. |

---

## 4. Arsitektur Perangkat Lunak & Multitasking FreeRTOS

Integrasi perangkat lunak dirancang dengan prinsip deterministik agar tidak mengganggu komputasi digital *Whole-Body Vibration* (Core 0, 200 Hz):

```
┌─────────────────────────────────────────────────────────────────────────────┐
│ CORE 0 (CORE_DSP)                                                           │
│  └── vTaskWBVAcquisition (Prio 4, 200 Hz / 5 ms) ──> xQueueWBVData          │
├─────────────────────────────────────────────────────────────────────────────┤
│ CORE 1 (CORE_PERIPHERAL)                                                    │
│  ├── vTaskGPS           (Prio 1, 50 ms) ───────────> Mutex: xMutexGPS       │
│  ├── vTaskDataLogger    (Prio 2, 1000 ms) ─────────> SD Card (/dosimeter.csv│
│  ├── vTaskHMIController (Prio 1, 100 ms) ──────────> OLED SH1106 & Tombol   │
│  └── vTaskBLEReceiver   (Prio 3, Event-driven) ────> xQueueHAVData          │
└─────────────────────────────────────────────────────────────────────────────┘
```

1. **Task Terdedikasi `vTaskGPS` (Core 1, Priority 1, Stack 3072 byte):**
   * Berjalan setiap **50 ms** untuk mengosongkan buffer FIFO perangkat keras UART2 secara *non-blocking* (`gpsSerial.read()`) dan memasukkan karakter ke parser `TinyGPSPlus::encode()`.
   * Pada laju 9600 baud (960 byte/detik), jeda 50 ms hanya mengakumulasikan maksimum ~48 byte data, jauh di bawah kapasitas buffer FIFO silikon ESP32 (128 byte), menjamin **bebas risiko luapan buffer (*zero FIFO overflow*)**.
2. **Sinkronisasi Aman Antar-Thread (*Thread-Safe Data Snapshot*):**
   * Pembaruan data posisi disimpan dalam struktur data `GpsData_t` di bawah perlindungan mutex `xMutexGPS`.
   * `vTaskDataLogger` mengambil salinan data (*snapshot*) setiap 1 detik saat menulis ke kartu SD.
3. **Mekanisme Pengawas Waktu Nyata (*Runtime Watchdog 3 Detik*):**
   * Pada `vTaskGPS`, waktu penerimaan karakter NMEA valid dicatat (`lastNmeaTimeMs`).
   * Jika tidak ada data NMEA valid selama $> 3000\text{ ms}$ (akibat kabel putus atau modul kehilangan catu daya saat *runtime*), flag `gpsHardwareOK` secara otomatis diubah menjadi `false` dan `fixValid` disetel ke `false`.
   * Saat kabel dihubungkan kembali, penerimaan karakter NMEA langsung memulihkan `gpsHardwareOK` ke `true`.
4. **Kebijakan FSM (*Non-blocking Satellite Fix Policy*):**
   * Mengingat modul GPS membutuhkan waktu 30–90 detik untuk *cold start*, ketiadaan kuncian satelit (*no satellite fix*) **tidak memblokir** sistem untuk beralih dari `SYS_SELF_TEST` ke `SYS_READY` maupun memulai pencatatan `SYS_LOGGING`.

---

## 5. Struktur Data & Format Berkas Log CSV

### A. Struktur Data `GpsData_t`
```cpp
typedef struct {
    double   latitude;    // Lintang dalam derajat desimal (e.g. -6.867576)
    double   longitude;   // Bujur dalam derajat desimal (e.g. 107.538755)
    float    speedKmh;    // Kecepatan laju tanah [km/h]
    int32_t  altitudeMM;  // Ketinggian di atas permukaan laut [milimeter]
    float    hdop;        // Horizontal Dilution of Precision
    uint32_t satellites;  // Jumlah satelit yang terlacak
    uint32_t ageMs;       // Usia data posisi sejak pembaruan terakhir [ms]
    bool     fixValid;    // True jika kuncian koordinat valid dan usia < 2000 ms
} GpsData_t;
```

### B. Format Baris CSV (`/dosimeter.csv`)
Berkas log pada kartu MicroSD menggunakan format 17-kolom:
```csv
timestamp,ahwx,ahwy,ahwz,ahv,awx,awy,awz,av,lat,lon,speed_kmh,altitudeMM,hdop,satellites,ageMs,fix_valid
```

* **Indeks 0 s.d. 8:** Data percepatan getaran ISO 5349-1 (HAV) dan ISO 2631-1 (WBV).
* **Indeks 9 s.d. 16:** Parameter geospasial GPS.

---

## 6. Antarmuka Pengguna: Layar OLED (HMI) & Telemetri Serial

### A. Tampilan Status Layar OLED SH1106
Status subsistem GPS ditampilkan pada baris ke-8 antarmuka OLED:
* **`GPS: ERR`** : Terjadi kegagalan komunikasi perangkat keras UART (kabel RX/TX terputus, modul tidak merespons, atau *watchdog* 3 detik terpicu).
* **`GPS: NO FIX`** : Modul GPS terdeteksi dan berkomunikasi normal, namun belum mengunci konstelasi satelit yang memadai.
* **`GPS: OK (xs)`** : Posisi geospasial berhasil dikunci dengan $x$ satelit (misal: `GPS: OK (9s)`).

### B. Format Telemetri Serial Monitor
Format `LOG_I` pada *task* logger menyajikan informasi lengkap secara waktu nyata:
```text
[INFO][LOGGER] LOGGER t=2026-09-20 10:34:51 | HAV ahv=0.0356 | WBV av=6.2968 | GPS fix=1 sat=9 spd=0.0 km/h alt=777.9 m hdop=2.40
```

---

## 7. Integrasi Skrip Pasca-Pemrosesan (`util/calc_dosimeter.py`)

Skrip analisis Python [`util/calc_dosimeter.py`](file:///d:/Users/Rafi%20Ananta%20Alden/Documents/Kuliah/Semester%208/EL4060/vibration-dosimeter-esp32/util/calc_dosimeter.py) telah diperbarui untuk mengenali berkas log 17-kolom dan secara otomatis menghitung metrik geospasial:
* **Cakupan Kuncian Satelit (*Fix Coverage Rate*):** Persentase durasi kuncian satelit valid terhadap total durasi pencatatan.
* **Statistik Kecepatan:** Kecepatan laju tanah maksimum dan rata-rata (*km/h*).
* **Ketinggian Rata-Rata:** Ketinggian rata-rata pengendara di atas permukaan laut (*meters MSL*).
* **Kualitas Sinyal:** Rata-rata nilai HDOP dan rata-rata jumlah satelit aktif.
* **Koordinat Terakhir:** Posisi lintang dan bujur akhir pengujian.

---

## 8. Hasil Pengujian Subsistem GPS Berdasarkan Skenario Uji

Pengujian subsistem GPS dievaluasi berdasarkan protokol skenario resmi pada [`docs/test_scenarios/gps.md`](file:///d:/Users/Rafi%20Ananta%20Alden/Documents/Kuliah/Semester%208/EL4060/vibration-dosimeter-esp32/docs/test_scenarios/gps.md):

### 8.1 Skenario 1: Uji Komunikasi UART & Kebijakan FSM (*Bench / Indoor Test*)
* **Status:** **LULUS (PASS)**
* **Kondisi Pengujian:** Meja laboratorium di dalam ruangan, Main Unit terhubung ke PC via USB.
* **Hasil Pengamatan:**
  1. Booting dan *self-test* mendeteksi modul dengan sukses (`[INFO][INIT] GPS u-blox NEO-6M UART2 probe: OK (NMEA stream live)`).
  2. Layar OLED menampilkan `GPS: NO FIX` (karena sinyal satelit terhalang atap lab).
  3. FSM berhasil bertransisi `INIT` $\rightarrow$ `SELF_TEST` $\rightarrow$ `READY` tanpa terblokir.
  4. Penekanan tombol berhasil memulai pencatatan `LOGGING`. Kartu SD mencatat baris data dengan koordinat aman default:
     `...,0.000000,0.000000,0.00,0,99.99,0,4294967295,0`.
  5. **Uji Negatif (Kabel Putus):** Saat kabel jumper TX/RX/VCC/GND dicabut, OLED seketika berganti menampilkan **`GPS: ERR`** (berkat *pull-up* internal dan *watchdog* 3 detik), membuktikan galat terdeteksi secara akurat dan tidak tertukar dengan `NO FIX`.

---

### 8.2 Skenario 2: Uji Kuncian Satelit Statis di Luar Ruangan (*Outdoor Static Fix Test*)
* **Status:** **LULUS (PASS)**
* **Kondisi Pengujian:** Halaman terbuka dengan pandangan bebas ke langit, perangkat diletakkan diam selama 52 detik.
* **Bukti Berkas Log:** [`docs/results/dosimeter.csv`](file:///d:/Users/Rafi%20Ananta%20Alden/Documents/Kuliah/Semester%208/EL4060/vibration-dosimeter-esp32/docs/results/dosimeter.csv).
* **Hasil Pengamatan:**
  1. *Time-To-First-Fix* (TTFF) tercapai dalam $\approx 45\text{ detik}$, ditandai dengan LED 1-PPS pada modul NEO-6M yang mulai berkedip 1 Hz.
  2. Status OLED berubah dari `GPS: NO FIX` menjadi `GPS: OK (9s)` (terkunci dengan 9 satelit).
  3. **Verifikasi Data Riil ([`dosimeter.csv`](file:///d:/Users/Rafi%20Ananta%20Alden/Documents/Kuliah/Semester%208/EL4060/vibration-dosimeter-esp32/docs/results/dosimeter.csv)):**
     * **Baris Header:** Baris pertama memuat 17 kolom lengkap secara utuh.
     * **Cakupan Fix:** 52 dari 52 sampel berstatus `fix_valid = 1` (**100% cakupan kuncian**).
     * **Kecepatan Statis:** Nilai kecepatan terukur $0{,}00\text{--}0{,}22\text{ km/h}$ (rata-rata $0{,}02\text{ km/h}$), membuktikan stabilitas filter Doppler NEO-6M saat diam.
     * **Ketinggian (*Altitude*):** Terukur pada rentang $777{,}300\text{--}778{,}000\text{ mm}$ ($\approx 777{,}7\text{ m}$ di atas permukaan laut), sesuai dengan elevasi topografi lokasi pengujian.
     * **Kualitas Geometri:** Nilai HDOP stabil pada $2{,}40$ dengan 9 satelit konstan.
     * **Koordinat:** Terkunci presisi pada lintang $-6{,}867576^\circ$ s.d. $-6{,}867538^\circ$ dan bujur $107{,}538755^\circ$ s.d. $107{,}538794^\circ$.

---

### 8.3 Skenario 3: Uji Dinamis Berkendara (*Dynamic Road Test*)
* **Status:** **TERTUNDA (PENDING)**
* **Keterangan & Rencana Pelaksanaan:**
  * Pengujian ini melibatkan pemasangan Main Unit pada kompartemen jok sepeda motor di jalan raya berkecepatan dinamis ($30\text{--}50\text{ km/h}$) dan melintasi rintangan jalan (polisi tidur/lubang jalan).
  * Tahap ini diagendakan untuk dilaksanakan pada pengujian lapangan (*field trial*) tahap akhir bersamaan dengan pengujian getaran kendaraan menyeluruh.
  * Target validasi pada uji dinamis meliputi:
    1. Korelasi inversi/transien antara kecepatan laju (`speed_kmh`) dan lonjakan vektor percepatan $a_v$ saat melintasi polisi tidur.
    2. Pemetaan rute perjalanan (*spatial route mapping*) yang dapat divisualisasikan pada peta digital.
    3. Ketahanan *multitasking* FreeRTOS (Core 0 WBV 200 Hz tanpa *deadline miss*) saat bergerak dengan fluktuasi sinyal satelit.

---

### 8.4 Skenario 4: Verifikasi Pasca-Pemrosesan Data (*Post-Processing Verification*)
* **Status:** **LULUS (PASS)**
* **Perintah Uji:**
  ```powershell
  python util/calc_dosimeter.py docs/results/dosimeter.csv --no-plot
  ```
* **Hasil Eksekusi Laporan:**
  ```text
  ===============================================================================
                 VIBRATION DOSIMETER ANALYSIS REPORT
                 Standards: ISO 5349-1 (HAV) | ISO 2631-1 (WBV)
  ===============================================================================
  Input Log File   : docs/results/dosimeter.csv
  Total Logged Time: 00h 00m 52s (52 seconds / 52 samples @ 1 Hz)
  Report Generated : 2026-09-20 10:51:20
  -------------------------------------------------------------------------------

  1. HAND-ARM VIBRATION (HAV) ANALYSIS (ISO 5349-1)
  -------------------------------------------------------------------------------
    * RMS Acceleration (Equiv):
        ahwx_eq = 0.0188 m/s² | ahwy_eq = 0.0189 m/s² | ahwz_eq = 0.0220 m/s²
        ahv_eq  (Vector Total) = 0.0346 m/s²
    * 8-Hour Daily Exposure A_HAV(8):
        A_HAV(8) Vector Total  = 0.0015 m/s²
    * Exposure Compliance Status: OK (SAFE)

  2. WHOLE-BODY VIBRATION (WBV) ANALYSIS (ISO 2631-1)
  -------------------------------------------------------------------------------
    * RMS Acceleration (Unweighted Equiv):
        awx_eq = 0.5530 m/s² | awy_eq = 0.4396 m/s² | awz_eq = 0.1636 m/s²
    * Directional Weighted RMS (kx=1.4, ky=1.4, kz=1.0 for seated):
        1.4*awx = 0.7741 m/s² | 1.4*awy = 0.6154 m/s² | 1.0*awz = 0.1636 m/s²
    * 8-Hour Daily Exposure A_WBV(8):
        Dominant Axis A_WBV(8) = 0.0329 m/s² (Axis X)
        Vector Total A_v(8)   = 0.0426 m/s²
    * Vibration Dose Value (VDV) 4th-Power Method:
        VDV Total Combined    = 5.3455 m/s^1.75
        Estimated VDV (eVDV)  = 3.7684 m/s^1.75
    * Exposure Compliance Status: OK (SAFE)

  3. GEOSPATIAL & VEHICLE TELEMETRY (u-blox NEO-6M GPS)
  -------------------------------------------------------------------------------
    * Satellite Fix Coverage: 52/52 seconds (100.0% valid fix)
    * Vehicle Ground Speed  : Max = 0.22 km/h | Mean = 0.02 km/h
    * Mean Altitude (MSL)   : 777.69 meters
    * Signal Quality & Sats : Mean HDOP = 2.40 | Mean Satellites = 9.0
    * Latest Position Fix   : Lat = -6.867538, Lon = 107.538794
  ===============================================================================
  ```
* **Kesimpulan:** Algoritma kalkulasi dosis paparan getaran ISO 5349/2631 dan modul telemetri GPS bekerja secara harmonis tanpa konflik indeks baris kolom ataupun kesalahan *parsing* numerik.

---

## 9. Ringkasan Status Kesiapan Subsistem (*Subsystem Readiness Summary*)

| Komponen / Fitur | Status | Catatan Teknis |
| :--- | :---: | :--- |
| **Antarmuka Hardware UART2** | **SIAP** | GPIO 16 (RX) + internal pull-up, GPIO 17 (TX), baud rate 9600. |
| **Multitasking FreeRTOS** | **SIAP** | `vTaskGPS` pada Core 1 (50 ms) terlindung mutex `xMutexGPS`. |
| **Mitigasi Kabel Putus / Derau** | **SIAP** | Deteksi `GPS: ERR` terverifikasi akurat via pull-up, NMEA probing, & watchdog 3s. |
| **Penulisan Header Kartu SD** | **SIAP** | Deterministik 17-kolom pada berkas baru maupun kosong. |
| **Telemetri Serial & Layar OLED** | **SIAP** | OLED `GPS: OK (xs)` / `ERR` dan serial `alt=%.1f m hdop=%.2f`. |
| **Integrasi Analisis Data Python**| **SIAP** | `util/calc_dosimeter.py` memproses metrik ISO dan GPS simultan. |
| **Pengujian Dinamis Jalan Raya** | **TERTUNDA** | Menunggu uji coba berkendara lapangan (*field trial*). |
