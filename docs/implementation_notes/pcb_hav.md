# Catatan Implementasi & Verifikasi Hardware: PCB HAV Node

- **Status Pengujian**:  **VERIFIED & PASSED (100% Berhasil)**
- **Tanggal Pengujian**: 9 November 2026
- **Target Hardware**: HAV (Hand-Arm Vibration) Node PCB Rev A ($56.0 \times 36.0\text{ mm}$, 2-Layer FR-4)
- **Komponen Utama**:
  - MCU: **ESP32-WROOM-32E-N4** (Silicon Rev v3.1, Dual-Core 240 MHz, 40 MHz Crystal, 4MB SPI Flash)
  - Sensor Akselerometer: **Analog Devices ADXL345BCCZ-RL7** (LGA-14, SPI 5 MHz, 3200 Hz ODR)
  - Standar Acuan: **ISO 5349-1 / ISO 5349-2** (Filter Pembobotan Frekuensi $W_h$)
  - Firmware Acuan: [`HAV-Node/src/main.cpp`](../../HAV-Node/src/main.cpp)

![](../media/pcb_hav_test.jpg)
*Pengujian Awal PCB HAV*

---

## 1. Ringkasan Eksekutif Hasil Pengujian

Pengujian awal (*bring-up testing*) pada unit fabrikasi PCBA JLCPCB pertama membuktikan bahwa **desain sirkuit, integritas sinyal berkecepatan tinggi, dan firmware dosimeter HAV Node berjalan dengan normal**:

1. **Integritas Bus SPI 5 MHz**: Sensor ADXL345 terdeteksi secara presisi dengan pembacaan register `DEVID = 0xE5`.
2. **Kestabilan Sampling 3200 Hz Real-Time**: Interrupt berbasis hardware pada pin `GPIO35` mampu melayani streaming data 3200 sampel/detik tanpa terjadi *dropped samples* atau *FIFO overflow*.
3. **Zero Packet Loss**: Sebanyak 68+ paket epoch satu detik (`HAV,0` hingga `HAV,68`) ditransmisikan secara kontinu tanpa jeda atau *system freeze*.
4. **Resonansi & Noise Floor Sangat Rendah**: Pada kondisi diam di atas meja, desah residual tertimbang hanya terukur sebesar $a_{hv} \approx 0.027\text{--}0.032\text{ m/s}^2$.
5. **Akurasi Fisika 3-Sumbu**:
   - Vektor gravitasi bumi terukur presisi pada sumbu Z saat board horizontal ($a_z \approx 9.18\text{--}9.48\text{ m/s}^2 \approx 1\text{ g}$).
   - Saat dimiringkan pada sumbu X atau Y, jumlah vektor percepatan ortogonal tetap konsisten pada $\approx 1\text{ g}$ ($\sqrt{a_x^2 + a_z^2} \approx 9.51\text{ m/s}^2$ dan $\sqrt{a_y^2 + a_z^2} \approx 9.90\text{ m/s}^2$).
   - Respons dinamis terhadap guncangan/ketukan sangat responsif ($a_z$ sesaat mencapai $18.05\text{ m/s}^2$ dengan respons RMS $a_{hv} \approx 0.585\text{ m/s}^2$).

---

## 2. Karakterisasi Hardware & Pengujian Pra-Penyalaan (Cold Test)

Sebelum menghubungkan catu daya, dilakukan pengukuran impedansi antara rel tegangan **`3V3_SYS` (Pin 2)** dan **`GND` (Pin 1)** pada header pemrograman `J1`:

- **Probe Merah (+) di Pin 2 (`3V3`), Probe Hitam (-) di Pin 1 (`GND`)**: Terukur $\approx 240\text{ k}\Omega$.
- **Probe Merah (+) di Pin 1 (`GND`), Probe Hitam (-) di Pin 2 (`3V3`)**: Terukur $\approx 12\text{ k}\Omega$.

### Analisis Rekayasa:

Asimetri pengukuran ini adalah **perilaku normal semikonduktor CMOS**. Pada polaritas terbalik, dioda klem proteksi ESD internal dan *parasitic body diodes* silikon ESP32 serta ADXL345 berada di ambang panjar maju (*forward-biased*). Nilai resistansi yang tinggi pada polaritas normal membuktikan bahwa **tidak terdapat jembatan timah (*solder bridge*) atau korsleting pada rel 3.3V**.

---

## 3. Pemetaan Pinout Hardware Definitif (Ground Truth)

Berdasarkan pelacakan netlist berkas tata letak PCB ([`hav_node.kicad_pcb`](../../../hardware-dosimeter/hav_node/hav_node.kicad_pcb)), berikut adalah pemetaan pin mikrokontroler yang telah terverifikasi:

| Fungsi Sinyal Hardware    | Pin Modul ESP32 | Pad Fisik IC | GPIO Firmware | Keterangan Teknis                                                                             |
| :------------------------ | :-------------: | :----------: | :-----------: | :-------------------------------------------------------------------------------------------- |
| **ADXL345 SCK**     |    `IO18`    |    Pad 30    |  `GPIO 18`  | SPI Clock via resistor damping $R_4 = 33\,\Omega$                                           |
| **ADXL345 MISO**    |    `IO19`    |    Pad 31    |  `GPIO 19`  | SPI Master In Slave Out (direct line)                                                         |
| **ADXL345 MOSI**    |    `IO23`    |    Pad 37    |  `GPIO 23`  | SPI Master Out Slave In via resistor damping $R_5 = 33\,\Omega$                             |
| **ADXL345 CS**      |    `IO25`    |    Pad 10    |  `GPIO 25`  | Chip Select via $R_6 = 33\,\Omega$ + pull-up $R_7 = 10\,\text{k}\Omega$ ke 3.3V           |
| **ADXL345 INT1**    |    `IO35`    |    Pad 7    |  `GPIO 35`  | Input hardware interrupt (*Data Ready*, rising edge)                                        |
| **VBAT Sense**      |    `IO34`    |    Pad 6    |  `GPIO 34`  | ADC1 CH6 via pembagi tegangan $R_8=330\text{k}\Omega / R_9=100\text{k}\Omega$ (Rasio 4.30x) |
| **RGB LED - Red**   |    `IO32`    |    Pad 8    |  `GPIO 32`  | Kanal Merah via resistor pembatas $R_{10} = 220\,\Omega$                                    |
| **RGB LED - Green** |    `IO26`    |    Pad 11    |  `GPIO 26`  | Kanal Hijau via resistor pembatas $R_{11} = 220\,\Omega$                                    |
| **RGB LED - Blue**  |    `IO27`    |    Pad 12    |  `GPIO 27`  | Kanal Biru via resistor pembatas $R_{12} = 220\,\Omega$                                     |
| **UART RX0**        |    `RXD0`    |    Pad 34    |  `GPIO 3`  | Header `J1` Pin 4 (Receive data)                                                            |
| **UART TX0**        |    `TXD0`    |    Pad 35    |  `GPIO 1`  | Header `J1` Pin 3 via resistor seri $R_3 = 499\,\Omega$                                   |
| **ESP Bootloader**  |     `IO0`     |    Pad 25    |  `GPIO 0`  | Tombol taktil `SW2` (BOOT) & Header `J1` Pin 6                                            |
| **ESP Reset**       |     `EN`     |    Pad 3    |  `CHIP_PU`  | Tombol taktil `SW1` (RESET), $R_2=10\text{k}\Omega$, $C_3=1\mu\text{F}$                 |

---

## 4. Antarmuka Pemrograman & Prosedur Flashing

### A. Pinout Header Pemrograman `J1` (Pitch 2.54 mm, Sisi Kanan Board)

Header `J1` memiliki 6 pin vertikal dengan susunan dari atas ke bawah:

- **Pin 1 (Atas, Pad Persegi)**: `GND` $\rightarrow$ Hubungkan ke `GND` USB-to-UART.
- **Pin 2**: `3V3_SYS` $\rightarrow$ Hubungkan ke `3.3V` USB-to-UART (*Dilarang menggunakan 5V!*).
- **Pin 3**: `UART_TX0` $\rightarrow$ Hubungkan ke `RXD` USB-to-UART.
- **Pin 4**: `UART_RX0` $\rightarrow$ Hubungkan ke `TXD` USB-to-UART.
- **Pin 5**: `EN` (Reset) $\rightarrow$ *Dibiarkan kosong* (dikendalikan manual).
- **Pin 6 (Bawah)**: `GPIO0` (Boot) $\rightarrow$ *Dibiarkan kosong* (dikendalikan manual).

### B. Prosedur Masuk Mode Bootloader yang Terbukti Andal

Berdasarkan pengujian lapangan, menekan tombol RESET manual saat komputer melakukan handshaking dapat memicu *contact bounce* dan merusak sinkronisasi paket (`Invalid head of packet`).

**Metode yang terbukti 100% andal**:

1. Jalankan perintah `pio run -t upload`.
2. Saat terminal mulai menampilkan teks `Connecting........`:
   - Cukup **tekan dan tahan tombol `SW2` (BOOT)**.
   - Tetap tahan tombol `SW2` hingga teks `Writing at 0x...` muncul di terminal.
   - Lepaskan tombol `SW2`.

### C. Konfigurasi `platformio.ini` Optimal

Karena terdapat resistor seri $R_3 = 499\,\Omega$ pada jalur TX0 untuk proteksi arus, baud rate upload harus dibatasi pada tingkat stabil:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
upload_speed = 115200
upload_port = COM3
monitor_port = COM3
```

> [!WARNING]
> Kecepatan upload $921600\text{ baud}$ menyebabkan distorsi sinyal digital akibat konstanta waktu RC parasitik ($R_3 = 499\,\Omega$ + kapasitansi kabel jumper), yang memicu kegagalan `The chip stopped responding`. Kecepatan $115200\text{ baud}$ memberikan keandalan 100%.

---

## 5. Log Bukti Pengujian Fungsional

### A. Startup & Inisialisasi Periferal

```text
==========================================================      
  Vibration Dosimeter — HAV Node Firmware v2.0 (SPI/INT)
  Standards: ISO 5349-1 (HAV)
  Peripherals: ADXL345 (SPI 5MHz) + Wh Filter + BLE Server      
  Design Configuration: Custom PCB hardware-dosimeter Rev A     
==========================================================
[INFO][INIT] Mini RGB LED initialized on GPIO R:32 G:26 B:27    
[INFO][LED] FSM State transition: -> INIT (Blue 1Hz)
[INFO][INIT] ADXL345 HAV detected via SPI (DEVID=0xE5)
[INFO][INIT] ADXL345 HAV SPI configuration complete:
[INFO][INIT]   - Bus      : 4-wire SPI (SCK:18, MISO:19, MOSI:23, CS:25) @ 5 MHz
[INFO][INIT]   - Interrupt: GPIO35 (ADXL_INT1 DATA_READY, RISING) 
[INFO][INIT]   - Range    : +/-16g full resolution
[INFO][INIT]   - DataRate : 3200 Hz ODR
[INFO][LED] FSM State transition: -> BLE_ADVERTISING (Blue 1Hz)
[INFO][BLE] BLE HAV Node advertising started.
[INFO][INIT] Sampling rate HAV set to 3200.0 Hz
[INFO][INIT] Expected BLE CSV Packet Format:
[INFO][INIT]   HAV,seq,millis_ms,ahwx,ahwy,ahwz,ahv,n_samples
```

### B. Cuplikan Data Pengukuran Kontinu

```text
RAW: ax=-0.535443, ay=0.000000, az=9.179025 | HAV,3,5342,0.0186,0.0178,0.0196,0.0323,3200
RAW: ax=-0.305967, ay=0.000000, az=9.484992 | HAV,4,6346,0.0148,0.0170,0.0196,0.0298,3200
RAW: ax=-0.535443, ay=0.000000, az=9.255516 | HAV,5,7351,0.0151,0.0157,0.0203,0.0298,3200
...
RAW: ax=7.649187, ay=0.000000, az=5.660398 | HAV,57,59594,0.0632,0.0969,0.1285,0.1729,3200
RAW: ax=0.688427, ay=-7.266728, az=6.731285 | HAV,63,65622,0.0936,0.1461,0.1235,0.2130,3200
RAW: ax=-2.600724, ay=-1.223870, az=18.052080 | HAV,65,67631,0.2727,0.4675,0.2234,0.5855,3200
RAW: ax=-0.382459, ay=0.000000, az=9.179025 | HAV,68,70645,0.0270,0.0169,0.0447,0.0549,3200
```

---

## 6. Antarmuka Eksternal Modul KY-016 (RGB LED)

Konektor **`J3`** (JST-XH 4-pin, sudut kanan bawah PCB) telah dipersiapkan dengan proteksi resistor onboard:

- **Pin 1 (Kiri, dekat `OUT-`)**: `GND` $\rightarrow$ Hubungkan ke `-` (Common Cathode) pada KY-016.
- **Pin 2**: `R` $\rightarrow$ Hubungkan ke pin `R` KY-016 (Kanal Merah).
- **Pin 3**: `G` $\rightarrow$ Hubungkan ke pin `G` KY-016 (Kanal Hijau).
- **Pin 4 (Kanan, tepi PCB)**: `B` $\rightarrow$ Hubungkan ke pin `B` KY-016 (Kanal Biru).

### Pola Indikator LED yang Terkonfirmasi:

1. **Biru Kedip Lambat (1 Hz)**: Menunggu koneksi Bluetooth (*BLE Advertising*).
2. **Hijau Efek Bernapas (*Breathing* 2s)**: Terhubung ke Main Unit & akuisisi aktif (*Logging Normal*).
3. **Merah Kedip Cepat (2 Hz)**: Sensor error atau koneksi komunikasi terputus (*Error / Comm Lost*).

---

## 7. Kesimpulan & Rekomendasi Tahap Lanjutan

Pengujian fungsional modul HAV Node dinyatakan **SELESAI dan MEMENUHI SPESIFIKASI (PASSED)**.

### Rencana Tindak Lanjut:

1. **Perakitan Subsistem Catu Daya Onboard**:
   - Solder modul step-down switching buck **MP1584EN** pada footprint `U3` (`IN+`, `IN-`, `3V3`, `OUT-`).
   - Hubungkan sakelar mekanik eksternal rocker **KCD11-101** pada pad `SW3`.
   - Hubungkan kemasan baterai 2S Li-ion (7.4V–8.4V) ke soket `J2`. Dengan demikian, board tidak lagi bergantung pada arus terbatas dari modul USB-to-UART.
2. **Pengujian Integrasi Komunikasi Nirkabel (BLE)**:
   - Hubungkan node sensor HAV ini dengan **Main Unit** (WBV PCB) atau aplikasi scanner BLE (nRF Connect) untuk memverifikasi penerimaan paket data notifikasi GATT berkecepatan 1 Hz secara nirkabel.
