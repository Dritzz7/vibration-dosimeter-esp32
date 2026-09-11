
Untuk mencatu daya sistem dosimeter getaran berbasis ESP32 dari baterai Lithium (3.7V nominal), regulator tegangan yang dibutuhkan adalah jenis **Low-Dropout Linear Regulator (LDO) berkinerja tinggi** dengan karakteristik spesifik.

Regulator linier biasa (seperti **AMS1117-3.3**) **TIDAK COCOK** untuk sistem bertenaga baterai Lithium.

---

### 🔍 4 Kriteria Utama Regulator yang Dibutuhkan:

#### 1. Tegangan Jatuh Sangat Rendah (*Ultra-Low Dropout Voltage*)

* **Masalah pada AMS1117**: AMS1117 memiliki *dropout voltage* tinggi sekitar **1,1V – 1,3V**. Jika tegangan baterai Lithium turun ke 3,7V, output AMS1117 akan anjlok ke $3{,}7\text{ V} - 1{,}2\text{ V} = 2{,}5\text{ V}$ $\rightarrow$ ESP32 langsung mati (*brownout reset*) padahal kapasitas baterai masih tersisa 70%!
* **Spesifikasi yang Dibutuhkan**: LDO dengan *dropout voltage* $\le \mathbf{150\text{ mV} - 250\text{ mV}}$ pada arus beban penuh. Dengan LDO jenis ini, regulator tetap mampu memasok tegangan stabil **3,3V** hingga tegangan baterai turun mendekati **3,45V – 3,40V** (kapasitas baterai terpakai $> 90\%$).

#### 2. Kapasitas Arus Puncak Cukup (*Peak Current $\ge 500\text{ mA} - 1\text{ A}$*)

* ESP32 menarik arus rata-rata 100–160 mA, namun saat memancarkan radio BLE (*burst transmission*) atau menulis ke SD Card, terjadi lonjakan arus transien sesaat hingga **350 mA – 500 mA**.
* Regulator harus mampu memasok arus kontinu minimal **$500\text{ mA} - 600\text{ mA}$** (arus puncak 800 mA – 1A) agar tegangan tidak mengalami *sagging/dip* yang memicu *brownout*.

#### 3. Arus Diam Sangat Rendah (*Ultra-Low Quiescent Current / $I_q$*)

* Regulator harus memiliki arus diam (*quiescent current*) kecil ($< \mathbf{10\text{ µA} - 60\text{ µA}}$) agar tidak membuang-buang energi baterai saat sistem beroperasi atau dalam kondisi siaga (*standby*). AMS1117 membuang arus diam hingga 5.000–10.000 µA (5–10 mA) hanya untuk dirinya sendiri.

#### 4. Derau Rendah (*Low Output Noise & High PSRR*)

* Sensor getaran ADXL345 sangat sensitif terhadap riak tegangan (*noise*). Regulator LDO linier menghasilkan tegangan DC yang jauh lebih bersih dan bebas derau switching dibanding konverter *buck regulator* switching murahan.

---

### 🏆 Rekomendasi IC Regulator Terbaik untuk Proyek Ini:

| Tipe IC Regulator                                    | Arus Maksimum ($I_{out}$) | *Dropout Voltage* ($V_{drop}$) | Arus Diam ($I_q$) |    Catatan Penggunaan    |                          |                                                                                                                                          |
| :--------------------------------------------------- | :------------------------------------------------------------------------------------: | :-----------------------: | :-----------------------: | :--------------------------------------------------------------------------------------------------------------------------------------- |
| **AP2112K-3.3** *(Sangat Direkomendasikan)*  |                                    **600 mA**                                    | **250 mV** @ 600 mA | $\approx 55\text{ µA}$ | Standar emas pada modul ESP32 komersial (Adafruit Feather). Sangat stabil menyuplai BLE & SD Card.                                       |
| **XC6220B331MR**                               |                                 **1000 mA (1A)**                                 |   **150 mV** @ 1A   | $\approx 8\text{ µA}$ | Sangat efisien dengan*dropout* ultra-rendah dan arus besar.                                                                            |
| **MCP1700-3302E** *(Sesuai Dokumen b300.md)* |                                    **250 mA**                                    | **178 mV** @ 250 mA |     **1,6 µA**     | Sangat hemat daya, namun**wajib ditambahkan kapasitor elektrolit 100–470 µF** pada pin output 3.3V untuk menyangga lonjakan BLE. |
| **RT9013-33GB / ME6211**                       |                                    **500 mA**                                    | **250 mV** @ 500 mA | $\approx 40\text{ µA}$ | Murah, ringkas (SOT-23-5), dan mudah ditemukan di pasaran lokal.                                                                         |

---

### 💡 Rekomendasi Rangkaian Tambahan (Filter & Proteksi):

1. **Kapasitor Input ($C_{in}$)**: Pasang kapasitor keramik **$10\text{ µF}$** di antara pin masukan baterai dan GND.
2. **Kapasitor Output ($C_{out}$)**: Pasang kapasitor keramik **$10\text{ µF} - 22\text{ µF}$** paralel dengan kapasitor elektrolit **$100\text{ µF} - 220\text{ µF}$** pada pin output 3.3V sedekat mungkin dengan pin ESP32 untuk menyerap lonjakan arus transmisi BLE.
