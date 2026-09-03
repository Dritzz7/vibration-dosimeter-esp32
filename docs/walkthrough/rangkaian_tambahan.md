
Untuk memastikan sistem dosimeter getaran bekerja dengan andal, aman dari gangguan listrik, dan terlindungi saat digunakan di lingkungan lapangan (pada sepeda motor), terdapat **5 opsi rangkaian tambahan pendukung** yang sangat direkomendasikan untuk diimplementasikan:

---

### 1. 🔋 Rangkaian Pengisian & Proteksi Baterai (*Battery Charger & Protection*)

Rangkaian ini adalah **lini pertahanan utama** untuk menjaga umur pakai dan keamanan baterai Lithium.

* **Komponen**: Modul **TP4056** terintegrasi IC proteksi **DW01** dan Dual MOSFET **FS8205A** dengan port USB Type-C.
* **Fungsi Proteksi**:
  * **Over-Charge Protection**: Menghentikan arus pengisian saat baterai menyentuh 4,20V ± 0,05V.
  * **Over-Discharge Protection**: Memutus aliran beban secara otomatis saat tegangan baterai turun ke ambang batas kritis (2,4V – 2,5V) agar sel Lithium tidak rusak permanen.
  * **Over-Current & Short-Circuit**: Memutus arus jika terjadi korsleting (> 3A).

---

### 2. 📊 Rangkaian Pembaca Level Baterai (*Battery Voltage Monitor / Voltage Divider*)

Rangkaian ini dibutuhkan agar ESP32 dapat membaca sisa persentase daya baterai secara real-time dan menampilkannya di layar OLED atau memicu peringatan *low-battery*.

* **Skema Rangkaian**:
  * Menggunakan **Pembagi Tegangan Resistor (*Voltage Divider*) 1:2** dari pin positif baterai ke pin ADC ESP32 (misalnya **GPIO 34, 35, atau 36** yang merupakan pin *Input-Only* dan bebas interferensi RF).
  * **Nilai Resistor**: $R_1 = 100\text{ k}\Omega$ (1%) dan $R_2 = 100\text{ k}\Omega$ (1%) untuk membagi tegangan baterai maksimum (4,2V $\rightarrow$ 2,1V, sangat aman di bawah batas input ADC ESP32 3,3V).
  * **Filter Derau**: Pasang kapasitor keramik **$100\text{ nF}$ (0,1 µF)** paralel dengan $R_2$ ke GND untuk meredam fluktuasi/derau ADC.
  * *Nilai resistor $100\text{ k}\Omega$ dipilih agar arus bocor pemantau sangat minim ($< 21\text{ µA}$).*

---

### 3. ⚡ Rangkaian Filter Decoupling & Anti-Brownout (*Power Rail Filtering*)

Rangkaian ini wajib ditambahkan untuk menyerap lonjakan arus transmisi radio BLE (*burst current spike*) dan penulisan MicroSD.

* **Komponen & Penempatan**:
  1. **Bulk Capacitor (Penyangga Arus Transien)**: Pasang kapasitor elektrolit (elco) / tantalum **$100\text{ µF} - 470\text{ µF} \text{ (16V Low ESR)}$** di antara jalur **`3V3`** dan **`GND`** sedekat mungkin dengan pin suplai ESP32.
  2. **High-Frequency Bypass**: Pasang kapasitor keramik MLCC **$10\text{ µF}$** dan **$0{,}1\text{ µF}$** paralel untuk menyaring derau frekuensi tinggi.
  3. **Isolasi Derau Sensor (Opsional)**: Pasang *Ferrite Bead* ($100\ \Omega\ @\ 100\text{ MHz}$) pada jalur $V_{CC}$ sensor ADXL345 agar pembacaan getaran tidak terganggu oleh derau RF BLE.

---

### 4. 🔌 Rangkaian Pull-Up & Stabilisasi Bus I2C / SPI

Untuk mencegah sinyal komunikasi sensor dan kartu SD macet (*hang/glitch*) akibat getaran kabel atau jalur PCB yang panjang:

* **Pull-Up Eksternal I2C (SDA & SCL)**:
  * Pasang sepasang resistor pull-up **$2{,}2\text{ k}\Omega - 4{,}7\text{ k}\Omega$** dari jalur SDA (GPIO 21) dan SCL (GPIO 22) ke jalur **`3V3`**. Ini menjamin bentuk gelombang sinyal jam I2C tajam pada laju 100–400 kHz.
* **Pull-Up Chip Select (CS) Pin**:
  * Pasang resistor pull-up **$10\text{ k}\Omega$** ke `3V3` pada pin CS SD Card (GPIO 5) dan CS ADXL345 untuk mencegah pin melayang (*floating*) saat fase booting ESP32.

---

### 5. 🛡️ Rangkaian Sakelar & Proteksi Statis (*ESD & Soft Power*)

* **Sakelar Daya**:
  * **Opsi A (Mekanis)**: Sakelar geser (*slide switch*) mini pada jalur positif baterai.
  * **Opsi B (Tombol Soft-Power / Push Button)**: Rangkaian P-Channel MOSFET (misal AO3401) + NPN BJT yang memungkinkan penyalaan melalui tombol tekan dan fitur *auto-shutdown* mandiri oleh ESP32 jika baterai mencapai kondisi darurat.
* **Proteksi ESD (Elektrostatik)**:
  * Pasang dioda TVS (misal *USBLC6-2SC6* atau *ESD5V0*) pada port USB dan pin tombol fisik HMI guna mencegah kerusakan chip akibat lonjakan listrik statis dari sentuhan tangan saat berkendara.

---

### 📋 Ringkasan Prioritas Rangkaian:

|     No     | Modul / Rangkaian                                                            |          Tingkat Urgensi          | Manfaat Utama                                                      |
| :---------: | :--------------------------------------------------------------------------- | :--------------------------------: | :----------------------------------------------------------------- |
| **1** | **TP4056 + DW01 Protection**                                           |          **Wajib**          | Pengisian aman & pencegahan baterai rusak/meledak.                 |
| **2** | **Kapasitor Elco 100–470 µF (3V3-GND)**                              |          **Wajib**          | Menghilangkan 100% masalah*Brownout Reset* akibat transmisi BLE. |
| **3** | **Pembagi Tegangan ADC ($100\text{ k}\Omega + 100\text{ k}\Omega$)** | **Sangat Direkomendasikan** | Menampilkan persentase baterai di OLED & peringatan baterai habis. |
| **4** | **Pull-Up I2C ($4{,}7\text{ k}\Omega$)**                             | **Sangat Direkomendasikan** | Stabilitas pembacaan ADXL345, OLED, dan RTC tanpa putus.           |
| **5** | **Dioda TVS / Proteksi ESD**                                           | **Opsional (Penyempurnaan)** | Perlindungan terhadap listrik statis lingkungan luar.              |
