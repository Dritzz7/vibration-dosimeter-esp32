# Spesifikasi I/O Perangkat Keras ESP32 untuk Perancangan PCB (*Hardware Pinout & PCB Design Specification*)

Dokumen ini ditujukan bagi **perancang perangkat keras / PCB (*PCB Designer*)** sebagai acuan resmi pemetaan pin I/O, antarmuka komunikasi periferal, penanganan *strapping pins*, serta pedoman kelistrikan dan tata letak (*layout*) untuk sistem dosimeter getaran (*Vibration Dosimeter*).

---

## 1. Ikhtisar Sistem (*System Architecture*)

Sistem dosimeter getaran terdiri dari dua unit terpisah yang berbasis mikrokontroler **ESP32 (ESP32-WROOM-32D / ESP32-WROOM-32E / DevKit 38-Pin)**:

1. **HAV Node (Hand-Arm Vibration)**:
   * **Penempatan**: Stang kemudi sepeda motor (beban bobot kritis $< 50\text{ gram}$).
   * **Catu Daya**: Baterai Li-Po 3.7V (1000–1200 mAh) / 18650 dengan modul pengisi TP4056 + LDO 3.3V.
   * **Periferal**: 1x Akselerometer ADXL345 (I2C), 1x Mini RGB LED KY-016 (Common Cathode), 1x ADC Pembagi Tegangan Baterai.
   * **Konektivitas**: Nirkabel BLE Server (GATT Peripheral).
2. **Main Unit (Whole-Body Vibration & Logger)**:
   * **Penempatan**: Bawah jok / alas duduk pengendara (bobot $< 150\text{ gram}$).
   * **Catu Daya**: 1x Sel Li-Ion 18650 3.7V (2200–2600 mAh) dengan modul pengisi TP4056 + LDO 3.3V.
   * **Periferal**: 1x Akselerometer ADXL345 WBV (I2C), 1x RTC DS3231 (I2C), 1x Layar OLED SH1106 1.3" (I2C), 1x Modul MicroSD (SPI), 1x Push-Button HMI, 1x Status LED.
   * **Konektivitas**: BLE Client (GATT Central).

---

## 2. Rincian Pemetaan Pin I/O Unit 1: HAV Node (Stang Kemudi)

| No | Pin ESP32 | Nama Sinyal | Tipe Pin | Terhubung ke Komponen | Deskripsi & Persyaratan Rangkaian |
| :---: | :---: | :---: | :---: | :---: | :--- |
| **1** | **GPIO 21** | `I2C_SDA` | I/O Digital (Open-Drain) | ADXL345 (Pin SDA) | Jalur data I2C. Wajib resistor pull-up eksternal **$4{,}7\text{ k}\Omega$** ke `3V3`. |
| **2** | **GPIO 22** | `I2C_SCL` | Output Digital (Open-Drain)| ADXL345 (Pin SCL) | Jalur clock I2C (100–400 kHz). Wajib resistor pull-up eksternal **$4{,}7\text{ k}\Omega$** ke `3V3`. |
| **3** | **GPIO 32** | `LED_RED` | Output Digital (PWM) | Mini RGB LED (Anoda R) | **PENTING:** Kanal Merah dialokasikan ke GPIO 32 (ADC1_CH4) karena aman dari interferensi stack BLE. Pasang resistor pembatas arus **$220\ \Omega - 330\ \Omega$**. |
| **4** | **GPIO 26** | `LED_GREEN` | Output Digital (PWM) | Mini RGB LED (Anoda G) | Kanal Hijau. Pasang resistor pembatas arus **$220\ \Omega - 330\ \Omega$**. |
| **5** | **GPIO 27** | `LED_BLUE` | Output Digital (PWM) | Mini RGB LED (Anoda B) | Kanal Biru. Pasang resistor pembatas arus **$220\ \Omega - 330\ \Omega$**. |
| **6** | **GPIO 34** | `VBAT_SENSE`| Input Analog (ADC1_CH6) | Rangkaian Pembagi Tegangan Baterai | **Input-Only (GPI)**. Titik tengah pembagi tegangan ($R_1 = 100\text{ k}\Omega$ ke $V_{BAT}$, $R_2 = 100\text{ k}\Omega$ ke GND) paralel kapasitor keramik **$100\text{ nF}$** ke GND. |
| **7** | **GND** | `GND` | Ground | Ground Bersama | Katoda bersama (*Common Cathode*) RGB LED dan ground sensor. |
| **8** | **3V3** | `3V3` | Power Rail | VCC Periferal | Jalur tegangan teregulasi 3,3V bersih dari output LDO. |

> **⚠️ Catatan Khusus Sensor ADXL345 pada HAV Node:**
> * **Pin CS ADXL345**: **Wajib ditarik ke `3V3`** (mengunci mode I2C permanen). Jangan dibiarkan melayang (*floating*)!
> * **Pin SDO ADXL345**: Dihubungkan ke **GND** untuk menetapkan alamat I2C ke **`0x53`**.
> * **Pin VCC Modul ADXL345**: Jika menggunakan modul siap pakai dengan LDO onboard (tipe RT9161/6206), hubungkan ke rel **5V/VIN** atau langsung bypass ke pin 3.3V chip.

---

## 3. Rincian Pemetaan Pin I/O Unit 2: Main Unit (Jok Sepeda Motor)

| No | Pin ESP32 | Nama Sinyal | Tipe Pin | Terhubung ke Komponen | Deskripsi & Persyaratan Rangkaian |
| :---: | :---: | :---: | :---: | :---: | :--- |
| **1** | **GPIO 21** | `I2C_SDA` | I/O Digital (Open-Drain) | Bus Bersama: ADXL345, DS3231, OLED | Jalur data bus I2C bersama. Sediakan sepasang resistor pull-up eksternal **$2{,}2\text{ k}\Omega - 4{,}7\text{ k}\Omega$** ke `3V3`. |
| **2** | **GPIO 22** | `I2C_SCL` | Output Digital (Open-Drain)| Bus Bersama: ADXL345, DS3231, OLED | Jalur clock bus I2C bersama (100–400 kHz). Sediakan pull-up **$2{,}2\text{ k}\Omega - 4{,}7\text{ k}\Omega$** ke `3V3`. |
| **3** | **GPIO 18** | `SPI_CLK` | Output Digital | Modul MicroSD (CLK/SCK) | Jalur clock bus SPI (1–4 MHz). |
| **4** | **GPIO 19** | `SPI_MISO` | Input Digital | Modul MicroSD (DO/MISO) | Jalur data masuk SPI (Master In Slave Out). Firmware mengaktifkan `INPUT_PULLUP`. |
| **5** | **GPIO 23** | `SPI_MOSI` | Output Digital | Modul MicroSD (DI/MOSI) | Jalur data keluar SPI (Master Out Slave In). |
| **6** | **GPIO 5** | `SD_CS` | Output Digital | Modul MicroSD (CS) | **PERINGATAN STRAPPING PIN**: GPIO 5 menentukan timing SDIO saat booting. **WAJIB pasang resistor pull-up eksternal $10\text{ k}\Omega$ ke `3V3`** agar tidak tertahan LOW saat boot. |
| **7** | **GPIO 4** | `HMI_BUTTON` | Input Digital | Tombol Push-Button HMI | Rangkaian tombol aktif-rendah (*Active-Low*, tombol menghubungkan pin ke GND saat ditekan). Pasang resistor pull-up eksternal **$10\text{ k}\Omega$** ke `3V3`, kapasitor filter derau **$100\text{ nF}$** ke GND, dan dioda proteksi ESD (TVS). |
| **8** | **GPIO 2** | `STATUS_LED` | Output Digital | Indikator LED Onboard | **STRAPPING PIN**: Harus berlogika LOW atau mengambang saat proses flashing UART. Pasang LED seri dengan resistor $330\ \Omega$ ke GND. |
| **9** | **GPIO 35** *(Opsional)* | `VBAT_SENSE`| Input Analog (ADC1_CH7) | Rangkaian Pembagi Baterai | **Input-Only (GPI)**. Titik tengah pembagi tegangan 1:2 ($100\text{ k}\Omega + 100\text{ k}\Omega + 100\text{ nF}$) untuk pemantauan baterai Main Unit. |

> **📋 Alamat Bus I2C pada Main Unit:**
> * **Sensor ADXL345 (WBV)**: Alamat **`0x53`** (SDO=GND) atau **`0x1D`** (SDO=3V3). Pin CS wajib ditarik ke `3V3`.
> * **Layar OLED SH1106 1.3"**: Alamat **`0x3C`**.
> * **RTC Maxim DS3231**: Alamat **`0x68`** (sediakan soket baterai koin CR2032 3V pada PCB).

---

## 4. Panduan Desain Kelistrikan & Skematik (*Electrical Design Guidelines*)

### A. Seleksi & Regulasi Catu Daya LDO (3.3V Rail)
* **Regulator Linier yang Direkomendasikan**: **AP2112K-3.3**, **XC6220B331MR**, atau **RT9013-33GB**.
  * Arus kontinu minimal: **$600\text{ mA} - 1\text{ A}$**.
  * Tegangan jatuh (*dropout*): $\le \mathbf{250\text{ mV}}$ pada beban penuh.
  * **JANGAN GUNAKAN AMS1117-3.3** karena *dropout*-nya terlalu tinggi (1,1V–1,3V), yang akan mematikan ESP32 saat baterai Lithium masih tersisa 70%.

### B. Kapasitor Penyangga Anti-Brownout (*Decoupling Capacitors*)
Transmisi nirkabel BLE menarik lonjakan arus mendadak (*burst current spike*) sebesar **150 mA – 250 mA**. Jika kapasitor penyangga kurang, tegangan rel 3.3V akan drop (*voltage dip*) dan memicu *Brownout Detector Reset*.
* **Wajib pada Jalur 3V3 ESP32**:
  1. Pasang **1x Kapasitor Elektrolit (Elco) / Tantalum $100\text{ µF} - 470\text{ µF} \text{ (16V Low-ESR)}$** paralel sedekat mungkin dengan pin suplai VDD ESP32.
  2. Pasang **1x Kapasitor Keramik MLCC $10\text{ µF}$** dan **1x MLCC $0{,}1\text{ µF} \text{ (100 nF)}$** paralel untuk memfilter derau frekuensi tinggi.

### C. Proteksi Pengisian & Baterai Lithium (TP4056 + DW01)
* Tempatkan modul pengisi daya **TP4056** terintegrasi IC proteksi **DW01** dan Dual MOSFET **FS8205A** dengan konektor USB Type-C.
* Hambatan pemutus arus diatur pada ambang batas pemutus *over-discharge* 2,5V dan pemutus hubung singkat (*short-circuit*) > 3A.

### D. Rangkaian Proteksi ESD & Transien
* Pasang dioda TVS penekan transien (misal: **USBLC6-2SC6** atau **ESD5V0**) pada jalur data USB (D+, D-) dan jalur sinyal tombol push-button fisik guna menangkal listrik statis dari sentuhan tangan saat berkendara.

---

## 5. Pedoman Tata Letak PCB (*PCB Layout Guidelines*)

1. **Area Bebas Antena RF (*RF Antenna Keepout Area*)**:
   * Posisikan modul ESP32 di tepi terluar papan PCB dengan posisi antena menghadap keluar.
   * **DILARANG KERAS** meletakkan tembaga (*copper pour*), bidang ground (*ground plane*), komponen, atau jalur tembaga apa pun di area tepat di bawah dan di samping antena PCB ESP32 (berikan jarak bebas minimal **15 mm** di ketiga sisi antena).
2. **Perutean Jalur Komunikasi Bus Berkecepatan Tinggi**:
   * Jalur sinyal SPI MicroSD (`CLK`, `MOSI`, `MISO`, `CS`) harus dirutekan sependek mungkin dan sejajar dengan ground plane solid di bawahnya.
   * Jauhkan jalur sinyal I2C dan SPI dari area modul radio RF dan koil induktor pengisi daya.
3. **Lebar Jalur Catu Daya (Power Traces)**:
   * Jalur rel catu daya baterai ($V_{BAT}$) dan rel daya teregulasi ($3\text{V}3$) harus memiliki lebar minimal **$0{,}6\text{ mm} - 1{,}0\text{ mm}$ (25–40 mil)** untuk meminimalkan resistansi parasitik saat lonjakan arus RF.
4. **Bidang Ground (*Ground Plane*)**:
   * Gunakan bidang ground penuh (*solid ground pour*) pada lapisan bawah (*bottom layer*), hubungkan semua pin ground komponen menggunakan *via stitch* ganda ke bidang ground utama.

---

## 6. Ringkasan Pantangan Pin ESP32 (*Pin Restrictions Checklist*)

| Pin ESP32 | Status / Fungsi Bawaan | Rekomendasi untuk Desainer PCB |
| :--- | :--- | :--- |
| **GPIO 0** | Strapping Pin (Boot Mode) | Harus HIGH saat boot normal. Sediakan pull-up 10k ke 3.3V + tombol BOOT ke GND. |
| **GPIO 2** | Strapping Pin | Harus LOW atau mengambang saat flash. Digunakan untuk status LED ke GND. |
| **GPIO 5** | Strapping Pin | Wajib diberi resistor pull-up eksternal 10k ke 3.3V (Chip Select SD Card). |
| **GPIO 12** | Strapping Pin (Flash Voltage MTDI) | **JANGAN DIGUNAKAN / BIARKAN FLOATING**. Jika ditarik HIGH saat boot, chip flash ESP32 akan rusak (tegangan meloncat ke 1.8V). |
| **GPIO 15** | Strapping Pin (MTDO) | Ada pull-up internal. Jangan ditarik paksa ke GND saat boot. |
| **GPIO 25** | DAC1 / ADC2_CH8 / RTC_IO6 | **JANGAN DIGUNAKAN BERSAMAAN DENGAN BLE**. Radio BLE mematikan kontrol GPIO matrix pada pin ini (alasan mengapa LED Merah dipindah ke GPIO 32). |
| **GPIO 34, 35, 36, 39**| Input-Only (GPI) | Tidak memiliki pull-up/pull-down internal. Hanya dapat digunakan sebagai input (ideal untuk ADC pembagi baterai). |
