# ADXL345 Vibration Sensor Visualizer & Validation Platform

Proyek uji mandiri (*standalone test project*) PlatformIO di bawah direktori `tests/Sensor-Visualizer` yang dirancang khusus untuk membuktikan **validitas, integritas, dan linearitas data percepatan mentah (*raw 3-axis acceleration*)** kepada dosen pembimbing dan peneliti sebelum data diproses oleh algoritma pembobotan frekuensi ISO 5349-1 / ISO 2631-1.

---

## 1. Fitur Utama

1. **Pengambilan Data Deterministik Berkecepatan Tinggi**:
   * Pengatur waktu mikrodetik (*hardware microsecond scheduler*) tanpa *jitter*.
   * Menggunakan bus **I2C Cepat 400 kHz** dengan pembacaan langsung 6-byte register (*burst read*) dalam satu transaksi.
   * Modus resolusi penuh (*Full Resolution Mode*) $\pm 16g$ ($3{,}9\text{ mg/LSB} = 0{,}0383\text{ m/s}^2\text{/LSB}$).
2. **Fleksibilitas Laju Cuplik (*Output Data Rate - ODR*)**:
   * Dapat diubah secara *real-time* via Serial: **100 Hz, 200 Hz (Standar WBV), 400 Hz, 800 Hz, 1600 Hz, hingga 3200 Hz (Standar HAV)**.
3. **Format Serial Plotter Universal**:
   * Menghasilkan *stream* data: `ax:VALUE,ay:VALUE,az:VALUE,amag:VALUE`.
   * Kompatibel langsung dengan **Arduino IDE Serial Plotter**, **VSCode Teleplot / Serial Plotter**, maupun **Serial Studio**.
4. **Alat Bantu Visualisasi Python Terintegrasi (`visualizer.py`)**:
   * Menampilkan **grafik domain waktu 4-kanal** ($a_x, a_y, a_z, |a|$).
   * Menampilkan **analisis spektrum frekuensi FFT secara waktu nyata (*Real-Time FFT Power Spectrum*)**.
   * Menghitung dan menampilkan frekuensi cuplik riil ($F_{actual}$), nilai puncak (*Peak*), nilai RMS, serta frekuensi dominan secara instan.
5. **Kontrol Interaktif (CLI)**:
   * Fitur **Tare / Zero Offset (`t`)** untuk meniadakan gravitasi statis sehingga osilasi getaran murni terpusat di angka nol.
   * Fitur **Reset Tare (`r`)** untuk kembali ke pembacaan gravitasi absolut bumi ($1g \approx 9{,}81\text{ m/s}^2$).
   * Fitur **Toggle Satuan (`u`)** antara $\text{m/s}^2$ dan $g$.

---

## 2. Pengkabelan Perangkat Keras (*Hardware Wiring*)

Hubungkan modul akselerometer **ADXL345** ke board ESP32 sesuai tabel berikut:

| Pin ADXL345 | Pin ESP32 | Keterangan |
| :--- | :--- | :--- |
| **VCC** | **3V3** (atau 5V jika modul memiliki regulator onboard) | Catu daya sensor |
| **GND** | **GND** | Ground bersama |
| **SDA** | **GPIO 21** | Jalur data I2C (disarankan pull-up 4.7k ke 3V3) |
| **SCL** | **GPIO 22** | Jalur clock I2C (disarankan pull-up 4.7k ke 3V3) |
| **CS** | **3V3** | **Wajib ditarik ke 3V3** untuk mengunci mode I2C |
| **SDO** | **GND** | Menetapkan alamat I2C ke `0x53` (jika ke 3V3 alamat `0x1D`) |

---

## 3. Kompilasi & Pengunggahan Firmware (*Flashing*)

Buka terminal di *root repository* atau masuk ke direktori proyek:

```bash
# 1. Kompilasi firmware
pio run -d tests/Sensor-Visualizer

# 2. Unggah firmware ke ESP32 (sesuaikan port COM jika perlu)
pio run -d tests/Sensor-Visualizer -t upload --upload-port COM3
```

---

## 4. Cara Menjalankan Visualisasi

### Metode A: Python Real-Time Visualizer + Analisis Spektrum FFT (Sangat Direkomendasikan untuk Dosen)

Metode ini memberikan pembuktian ilmiah paling komprehensif karena dosen dapat melihat gelombang waktu dan kandungan frekuensi FFT secara berdampingan.

1. Pasang pustaka pendukung (jika belum ada):
   ```bash
   pip install pyserial matplotlib numpy
   ```
2. Jalankan skrip visualizer:
   ```bash
   python tests/Sensor-Visualizer/visualizer.py --port COM3
   ```
   *(Port akan terdeteksi otomatis jika parameter `--port` tidak diisi).*

3. **Tombol Pintas pada Jendela Grafik**:
   * `T` : Melakukan *Tare* (menghilangkan offset DC gravitasi agar sinyal AC berpusat di 0).
   * `R` : Me-reset *Tare* kembali ke gravitasi absolut $1g$.
   * `U` : Mengubah satuan antara $\text{m/s}^2$ dan $g$.
   * `1`–`6` : Mengubah laju cuplik ODR (100 Hz hingga 3200 Hz).
   * `Spasi` : Menjeda (*pause*) grafik untuk menginspeksi bentuk gelombang.

---

### Metode B: Arduino IDE Serial Plotter / VSCode Serial Plotter

Tanpa perlu Python, Anda bisa langsung menggunakan *Serial Plotter* bawaan:

1. Buka **Arduino IDE** $\rightarrow$ Pilih Board **ESP32** & Port COM terkait.
2. Buka **Tools $\rightarrow$ Serial Plotter** (atau tekan `Ctrl + Shift + L`).
3. Atur *Baud Rate* pada Serial Plotter ke **115200**.
4. Empat kurva berwarna ($a_x, a_y, a_z, |a|$) akan langsung terplot secara halus (*smooth*).

---

### Metode C: PlatformIO Device Monitor (Teks / Telemetri)

Untuk melihat pesan diagnostik, frekuensi cuplik riil, atau mengirim perintah CLI:

```bash
pio device monitor -p COM3 -b 115200
```

Ketik `h` lalu tekan Enter untuk menampilkan menu bantuan interaktif.

---

## 5. Protokol Pengujian untuk Dosen Pembimbing (*Testing Demonstration Protocol*)

Berikut panduan langkah demi langkah saat mendemonstrasikan validitas sensor kepada dosen pembimbing:

### Pengujian 1: Uji Statis Gravitasi Bumi 1g ($9{,}81\text{ m/s}^2$) & Ortogonalitas Sumbu
* **Tujuan**: Membuktikan faktor skala kalibrasi, akurasi nilai nol, dan ortogonalitas sumbu $X, Y, Z$.
* **Prosedur**:
  1. Letakkan sensor mendatar di atas meja diam dengan sumbu $Z$ menghadap ke atas.
  2. Perhatikan nilai pembacaan:
     $$a_x \approx 0{,}0\text{ m/s}^2, \quad a_y \approx 0{,}0\text{ m/s}^2, \quad a_z \approx +9{,}81\text{ m/s}^2$$
     Magnitudo total $|a| = \sqrt{a_x^2 + a_y^2 + a_z^2} \approx 9{,}81\text{ m/s}^2$ ($1{,}0g$).
  3. Balik sensor menghadap ke bawah: $a_z$ berubah menjadi $-9{,}81\text{ m/s}^2$.
  4. Miringkan sensor ke arah sumbu $X$ atau $Y$: Komponen percepatan berpindah secara sinusoidal mengikuti hukum trigonometri, namun **magnitudo $|a|$ tetap konstan $9{,}81\text{ m/s}^2$**.
* **Kesimpulan Ilmiah untuk Dosen**: Sensor memiliki ortogonalitas tinggi dan sensitivitas absolut yang terkalibrasi tepat pada percepatan gravitasi bumi $1g$.

---

### Pengujian 2: Uji Respons Impuls (*Impulse / Shock Response*)
* **Tujuan**: Membuktikan kemampuan sensor menangkap transien cepat tanpa saturasi (*clipping*) dan melihat respons redaman sistem mekanis.
* **Prosedur**:
  1. Berikan ketukan tajam (*tap*) dengan jari atau ujung pulpen pada permukaan tempat sensor terpasang.
  2. Amati grafik domain waktu:
     * Terlihat lonjakan tajam (*spike*) seketika dengan waktu naik (*rise time*) fraksi milidetik, diikuti gelombang redaman eksponensial (*damped ringdown*).
  3. Amati spektrum FFT pada `visualizer.py`:
     * Muncul kenaikan energi serentak pada spektrum frekuensi lebar (*broadband excitation*), yang merupakan sifat fisis khas dari fungsi impuls Dirac $\delta(t)$.
* **Kesimpulan Ilmiah untuk Dosen**: Sensor dan jalur akuisisi ESP32 memiliki *bandwidth* transien yang cukup tinggi dan tidak mengalami distorsi saturasi pada rentang $\pm 16g$.

---

### Pengujian 3: Uji Respons Sinusoidal / Getaran Harmonik (*Harmonic Vibration*)
* **Tujuan**: Membuktikan linearitas dinamis dan kemampuan menangkap getaran frekuensi periodik secara akurat.
* **Prosedur**:
  1. Letakkan sensor di atas sumber getaran periodik (misalnya *vibration motor* smartphone dengan aplikasi generator frekuensi, atau bodi motor listrik/kipas angin).
  2. Amati grafik domain waktu:
     * Sinyal percepatan membentuk gelombang sinusoidal kontinu yang mulus tanpa cacat cacah (*quantization noise* yang minim).
  3. Amati spektrum FFT pada `visualizer.py`:
     * Terlihat satu puncak tajam (*sharp dominant peak*) tepat pada frekuensi kerja motor getar (misal pada frekuensi $50\text{ Hz}$, $120\text{ Hz}$, atau $175\text{ Hz}$).
* **Kesimpulan Ilmiah untuk Dosen**: Akuisisi data bebas dari distorsi harmonik yang signifikan (*low harmonic distortion*), membuktikan integritas data sebelum diproses oleh filter IIR pembobotan ISO.

---

### Pengujian 4: Integritas Laju Cuplik & Ketepatan Waktu (*Timing Determinism*)
* **Tujuan**: Membuktikan bahwa interval pencuplikan berjalan stabil dan tidak ada sampel data yang hilang (*zero sample drops*).
* **Prosedur**:
  1. Jalankan `pio device monitor` atau perhatikan teks *telemetry* pada `visualizer.py`.
  2. Amati nilai `Sampling Fs`:
     * Pada ODR 200 Hz, frekuensi terukur stabil pada rentang $200{,}0 \pm 0{,}5\text{ Hz}$.
     * Pada ODR 3200 Hz, frekuensi terukur stabil pada rentang $3200 \pm 10\text{ Hz}$.
* **Kesimpulan Ilmiah untuk Dosen**: Algoritma pencuplikan berbasis pengatur waktu mikrodetik dan pembacaan *burst* I2C 400 kHz menjamin integritas basis waktu (*time-base integrity*), syarat mutlak agar integrasi numerik RMS dan eVDV ISO valid.
