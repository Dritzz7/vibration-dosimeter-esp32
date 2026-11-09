# Catatan Teknis: Subsistem Sensor Getaran HAV (*Hand-Arm Vibration Sensor*)

Dokumen ini mendokumentasikan arsitektur perangkat keras, pemilihan bus komunikasi, sirkuit integritas sinyal, konfigurasi interupsi perangkat keras (*hardware interrupt*), serta implementasi driver *firmware* untuk sensor getaran lengan-tangan (**ADXL345**) pada modul **HAV Node**.

---

## 1. Konteks Operasional & Standar Pengukuran

Modul HAV Node berfungsi mengukur paparan getaran mekanis yang ditransmisikan dari mesin/stang kemudi kendaraan ke tangan dan lengan pekerja sesuai standar:
* **ISO 5349-1:2001**: *Mechanical vibration — Measurement and evaluation of human exposure to hand-transmitted vibration — Part 1: General requirements*.
* **ISO 5349-2:2001**: *Part 2: Practical guidance for measurement at the workplace*.
* **ISO 8041:2005**: *Human response to vibration — Measuring instrumentation*.

### Kebutuhan Sensor:
* **Bandwidth Frekuensi HAV**: Rentang frekuensi utama evaluasi getaran tangan-lengan adalah **$8\text{ Hz} - 1000\text{ Hz}$** (dengan kurva pembobotan frekuensi $W_h$).
* **Laju Pengambilan Sampel (*Sampling Rate*)**: Sesuai kriteria Nyquist dan oversampling filter digital, laju data sensor disetel pada **$f_s = 3200\text{ Hz}$** (ODR tertinggi pada ADXL345), dengan periode pengambilan sampel $T_s = 312{,}5\ \mu\text{s}$.
* **Rentang Skala Penuh (*Full-Scale Range*)**: **$\pm 16\text{g}$** dalam mode *Full Resolution* ($3{,}9\text{ mg/LSB}$ konstan di seluruh rentang skala).

---

## 2. Arsitektur Perangkat Keras PCB (`hardware-dosimeter`)

Sensor diimplementasikan langsung pada PCB kustom **`hardware-dosimeter`** (Revisi A) menggunakan chip kemasan permukaan **Analog Devices ADXL345BCCZ-RL7** (kemasan LGA-14 / CC-14-1, dimensi $3 \times 5 \times 1\text{ mm}$):

```
       Rel Daya 3V3_SYS
              │
         ┌────┴────┐
         │         │
       C4===     C5===  C6===
       100nF     100nF  1µF
         │         │     │
       GND_ADXL  GND_ADXL│
         │         │     │
      [Pin 6]   [Pin 1]  │
       (VDD)     (VS)────┘
         ┌───────────────────┐
         │      ADXL345      │
         │      (LGA-14)     │
         │                   │
         │ [Pin 14] SCL/SCK  │<───[ R4: 33 Ω ]─── GPIO 18 (SPI_ADXL_SCK)
         │ [Pin 13] SDA/MOSI │<───[ R5: 33 Ω ]─── GPIO 23 (SPI_ADXL_MOSI)
         │ [Pin 12] SDO/MISO │───────────────────> GPIO 19 (SPI_ADXL_MISO)
         │ [Pin 7]  CS       │<───[ R6: 33 Ω ]─── GPIO 25 (ADXL_CS)
         │                   │         │
         │                   │    [ R7: 10 kΩ Pullup ]─── 3V3_SYS
         │ [Pin 8]  INT1     │───────────────────> GPIO 35 (ADXL_INT1)
         │ [Pin 9]  INT2     │───────────────────> TP11 (Test Point INT2)
         │                   │
         │ [Pin 2,4,5,11]    │
         │     GND_ADXL      │
         └─────────┬─────────┘
                   │
              [ R16: 0 Ω ] (Net-tie / Jumper Bintang)
                   │
             GND (Sistem Utama)
```

### A. Integritas Sinyal & Peredaman Pantulan (*Damping Resistors*)
* Karena bus SPI beroperasi pada frekuensi tinggi ($5\text{ MHz}$) dengan waktu naik sinyal (*slew rate*) cepat dari ESP32, dipasang resistor redaman seri **$R_4, R_5, R_6 = 33\ \Omega$** (SMD 0805) sedekat mungkin dengan pin mikrokontroler.
* Resistor seri ini mencocokkan impedansi keluaran driver GPIO ESP32 dengan impedansi karakteristik *trace* PCB, mencegah derau *ringing*, *overshoot*, dan *undershoot* yang dapat memicu salah baca bit data.
* Jalur Chip Select (CS) dilengkapi resistor *pull-up* **$R_7 = 10\text{ k}\Omega$** ke `3V3_SYS` untuk menjamin pin CS selalu berlogika HIGH saat ESP32 berada dalam proses reset atau *bootloader*, mencegah sensor terakses secara tidak sengaja.

### B. Pemisahan Domain Ground (*Ground Isolation & Decoupling*)
* Bidang ground sensor dipisahkan menjadi poligon tembaga tersendiri (**`GND_ADXL`**) yang terhubung ke ground digital utama hanya melalui sebuah titik tunggal menggunakan resistor jumper net-tie **$R_{16} = 0\ \Omega$** (tipe *star grounding*).
* Hal ini mencegah arus derau frekuensi tinggi (*switching noise*) dari konverter buck MP1584 dan lonjakan arus modul RF BLE ESP32 mengalir melintasi ground sensor yang sensitif.
* Jalur catu daya dilengkapi kapasitor *decoupling* lokal:
  * **$C_4 = 100\text{ nF}$** pada pin $V_{DD}$ (catu daya antarmuka logika digital).
  * **$C_5 = 100\text{ nF}$** dan **$C_6 = 1\ \mu\text{F}$** paralel pada pin $V_S$ (catu daya inti transduser kapasitif analog).

---

## 3. Protokol Komunikasi: 4-Wire SPI @ 5 MHz

### Mengapa SPI Menggantikan I2C?
Pada tahap prototipe awal, sensor diakses menggunakan I2C pada kecepatan 400 kHz (Fast Mode). Namun, pada laju data $3200\text{ Hz}$ ($T_s = 312{,}5\ \mu\text{s}$), waktu bus I2C menjadi *bottleneck* kritis:

| Parameter Evaluasi | Bus I2C 400 kHz (Prototipe Lama) | Bus 4-Wire SPI 5 MHz (PCB Baru) | Keuntungan SPI |
| :--- | :---: | :---: | :--- |
| **Frekuensi Clock** | $400\text{ kHz}$ | **$5\text{ MHz}$** | **$12{,}5\times$ lebih cepat** |
| **Overhead Protokol** | START, Byte Address + ACK, Sub-address + ACK, Repeated START, 6-byte data + ACK, STOP ($\approx 82$ siklus clock) | CS assert, 1 Command Byte, 6 Data Bytes, CS de-assert (56 siklus clock) | Bebas latensi handshake ACK |
| **Durasi Pembacaan 6-Byte** | **$\approx 205\ \mu\text{s}$** | **$\approx 11{,}2\ \mu\text{s}$** | **Menghemat $\approx 194\ \mu\text{s}$ waktu CPU per sampel** |
| **Beban CPU pada 3200 Hz** | $\approx 65{,}6\%$ (Hanya untuk bus I2C) | **$\approx 3{,}6\%$** | CPU leluasa memproses filter IIR $W_h$ dan stack BLE |
| **Ketahanan Derau** | Jalur *open-drain* dengan pull-up (rentan derau EMI) | Jalur *push-pull* penuh dengan resistor damping | Integritas sinyal jauh lebih tinggi pada getaran ekstrem |

### Format Bit Transaksi SPI ADXL345:
Pada mode SPI, bit pertama byte perintah mendefinisikan operasi:
* **Bit 7 (R/W)**: `0` untuk Tulis (*Write*), `1` untuk Baca (*Read*).
* **Bit 6 (MB)**: `0` untuk Akses Tunggal (*Single-byte*), `1` untuk Pembacaan Beruntun (*Multi-byte burst read*).
* **Bit 5–0**: Alamat register (0x00 hingga 0x3F).
* **Pengaturan SPI ESP32**: `SPISettings(5000000, MSBFIRST, SPI_MODE3)` (Clock idle HIGH, data disampel pada tepi naik clock / *rising edge*).

---

## 4. Mekanisme Sinkronisasi Interupsi Hardware (GPIO 35)

Alih-alih menggunakan *busy-waiting* atau *software microsecond polling* yang rentan mengalami *jitter*, sistem memanfaatkan pin interupsi perangkat keras **INT1** pada ADXL345 yang dirutekan secara fisik ke **GPIO 35** ESP32:

```
[ ADXL345 ADC ] ──> Selesai Konversi ──> Pin INT1 Naik HIGH
                                                │
[ GPIO 35 ESP32 ] <─────────────────────────────┘ (Edge RISING)
       │
       ▼
[ ISR: isrADXLDataReady() ] (IRAM)
       │
       └──> vTaskNotifyGiveFromISR(s_mainTaskHandle)
                   │
                   ▼
[ loop() Core 1 ] <── Bangun Seketika (ulTaskNotifyTake)
       │
       ├──> SPI Burst Read (11.2 µs) ──> Otomatis me-reset INT1
       ├──> Filter IIR Wh (3 Sumbu)
       └──> Akumulasi RMS
```

1. **Konfigurasi Register Sensor**:
   * `REG_INT_MAP (0x2F)`: Ditulis `0x00` (semua fungsi interupsi, khususnya `DATA_READY` pada bit 7, dialokasikan ke pin fisik **INT1**).
   * `REG_INT_ENABLE (0x2E)`: Ditulis `0x80` (mengaktifkan bit 7: `DATA_READY`).
2. **Karakteristik GPIO 35 ESP32**:
   * GPIO 35 adalah pin input murni (*GPI*) pada domain ADC1, bebas konflik dari stack radio BLE dan tidak terbebani sirkuit output digital internal, menjamin latensi interupsi terendah ($< 2\ \mu\text{s}$).
3. **Pembersihan Otomatis Interupsi**:
   * Sesuai datasheet ADXL345, bit dan pin `DATA_READY` otomatis kembali ke status LOW ketika mikrokontroler membaca register data percepatan (`REG_DATAX0` hingga `REG_DATAZ1`), sehingga tidak memerlukan pembersihan manual tambahan.

---

## 5. Implementasi Perangkat Lunak (`HAV-Node/src/main.cpp`)

### A. Definisi Pinout & Konfigurasi SPI
```cpp
// --- ADXL345 4-wire SPI Bus ---
#define PIN_SPI_SCK          18    ///< GPIO18 (via R4 33R)
#define PIN_SPI_MISO         19    ///< GPIO19 (direct to MISO)
#define PIN_SPI_MOSI         23    ///< GPIO23 (via R5 33R)
#define PIN_ADXL_CS          25    ///< GPIO25 (via R6 33R + R7 10k pull-up to 3.3V)
#define PIN_ADXL_INT1        35    ///< GPIO35 (ADXL345 INT1 Data Ready interrupt)

static SPISettings adxlSpiSettings(5000000, MSBFIRST, SPI_MODE3);
```

### B. Fungsi Transaksi Register SPI
```cpp
bool writeRegisterSPI(uint8_t reg, uint8_t value) {
    SPI.beginTransaction(adxlSpiSettings);
    digitalWrite(PIN_ADXL_CS, LOW);
    SPI.transfer(reg & 0x3F); // R/W = 0, MB = 0
    SPI.transfer(value);
    digitalWrite(PIN_ADXL_CS, HIGH);
    SPI.endTransaction();
    return true;
}

bool readRegisterSPI(uint8_t reg, uint8_t &value) {
    SPI.beginTransaction(adxlSpiSettings);
    digitalWrite(PIN_ADXL_CS, LOW);
    SPI.transfer((reg & 0x3F) | 0x80); // R/W = 1, MB = 0
    value = SPI.transfer(0x00);
    digitalWrite(PIN_ADXL_CS, HIGH);
    SPI.endTransaction();
    return true;
}

bool readMultipleRegistersSPI(uint8_t startReg, uint8_t *buffer, uint8_t length) {
    SPI.beginTransaction(adxlSpiSettings);
    digitalWrite(PIN_ADXL_CS, LOW);
    SPI.transfer((startReg & 0x3F) | 0x80 | 0x40); // R/W = 1, MB = 1
    for (uint8_t i = 0; i < length; i++) {
        buffer[i] = SPI.transfer(0x00);
    }
    digitalWrite(PIN_ADXL_CS, HIGH);
    SPI.endTransaction();
    return true;
}
```

### C. Inisialisasi Sensor & Interupsi
```cpp
bool setupADXL345() {
    pinMode(PIN_ADXL_CS, OUTPUT);
    digitalWrite(PIN_ADXL_CS, HIGH);
    pinMode(PIN_ADXL_INT1, INPUT);

    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_ADXL_CS);

    uint8_t devid = 0;
    if (!readRegisterSPI(REG_DEVID, devid) || devid != ADXL345_DEVID_VALUE) {
        LOG_E("INIT", "ADXL345 SPI probe failed! DEVID=0x%02X (Expected 0xE5)", devid);
        return false;
    }

    writeRegisterSPI(REG_POWER_CTL, 0x00);                // Standby
    writeRegisterSPI(REG_BW_RATE, ADXL_BW_3200HZ);        // 3200 Hz ODR
    writeRegisterSPI(REG_DATA_FORMAT, ADXL_FORMAT_FULLRES_16G); // ±16g Full-Res
    writeRegisterSPI(REG_INT_MAP, 0x00);                  // Map DATA_READY ke INT1
    writeRegisterSPI(REG_INT_ENABLE, 0x80);               // Aktifkan DATA_READY
    writeRegisterSPI(REG_POWER_CTL, 0x08);                // Measurement Mode

    // Flush data awal & pasang ISR
    uint8_t dummy[6];
    readMultipleRegistersSPI(REG_DATAX0, dummy, 6);
    attachInterrupt(digitalPinToInterrupt(PIN_ADXL_INT1), isrADXLDataReady, RISING);

    return true;
}
```

### D. Konversi Data & Skala Percepatan
Nilai mentah 16-bit signed integer dikonversikan ke satuan baku SI percepatan ($\text{m/s}^2$):
$$\text{Faktor Skala} = 3{,}9\ \text{mg/LSB} = 0{,}0039\text{ g/LSB}$$
$$a = \text{raw} \times 0{,}0039 \times 9{,}80665\ \text{m/s}^2$$

```cpp
bool readADXL345(float &ax, float &ay, float &az) {
    if (!adxlAvailable) return false;

    uint8_t data[6];
    if (!readMultipleRegistersSPI(REG_DATAX0, data, 6)) return false;

    const int16_t rawX = (int16_t)((data[1] << 8) | data[0]);
    const int16_t rawY = (int16_t)((data[3] << 8) | data[2]);
    const int16_t rawZ = (int16_t)((data[5] << 8) | data[4]);

    ax = rawX * ADXL_SCALE_G_PER_LSB * G_TO_MPS2;
    ay = rawY * ADXL_SCALE_G_PER_LSB * G_TO_MPS2;
    az = rawZ * ADXL_SCALE_G_PER_LSB * G_TO_MPS2;
    return true;
}
```

---

## 6. Titik Uji Perangkat Keras (*Hardware Test Points Matrix*)

Pada layer bawah PCB (`B.Cu`), disediakan titik uji tembaga $\varnothing 1{,}5\text{ mm}$ untuk verifikasi integritas sinyal menggunakan osiloskop atau penganalisis logika (*logic analyzer*):

| Test Point | Nama Sinyal | Layer | Deskripsi & Verifikasi Karakteristik Sinyal |
| :---: | :---: | :---: | :--- |
| **TP6** | `SPI_ADXL_SCK` | B.Cu | Sinyal clock SPI $5\text{ MHz}$ (Duty cycle 50%, amplitudo $3{,}3\text{ V}$, verifikasi tidak ada ringing berlebih). |
| **TP7** | `SPI_ADXL_MOSI` | B.Cu | Jalur data Master-Out Slave-In (perintah tulis register). |
| **TP8** | `SPI_ADXL_MISO` | B.Cu | Jalur data Master-In Slave-Out (aliran data percepatan dari ADXL345). |
| **TP9** | `ADXL_CS` | B.Cu | Sinyal Chip Select (aktif LOW selama transaksi burst 6-byte). |
| **TP10** | `ADXL_INT1` | B.Cu | Pulsa interupsi `DATA_READY` berfrekuensi tepat $3200\text{ Hz}$ (lebar pulsa $\approx 312{,}5\ \mu\text{s}$). |
| **TP11** | `ADXL_INT2` | B.Cu | Jalur sekunder untuk opsi fungsi *Watermark / FIFO Overflow*. |
| **TP3** | `3V3_SYS` | B.Cu | Tegangan rel daya $3{,}3\text{ V}$ sensor (ripple $< 30\text{ mV}_{p-p}$). |
| **TP4** | `GND` | B.Cu | Titik acuan ground osiloskop. |
