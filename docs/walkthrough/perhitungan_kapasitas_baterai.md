
Berdasarkan spesifikasi desain teknis pada dokumen [docs/ref/b200.md](<file:///d:/Users/Rafi%20Ananta%20Alden/Documents/Kuliah/Semester%208/EL4060/vibration-dosimeter-esp32/docs/ref/b200.md#L262>) dan [docs/ref/b300.md](<file:///d:/Users/Rafi%20Ananta%20Alden/Documents/Kuliah/Semester%208/EL4060/vibration-dosimeter-esp32/docs/ref/b300.md#L679-L683>), sistem dosimeter dirancang untuk beroperasi secara kontinu **minimal selama 8 jam** (sesuai standar durasi kerja harian internasional ISO 5349-1 dan ISO 2631-1).

Kedua subsistem menggunakan sumber catu daya mandiri berbasis sel **Lithium-Ion 18650 / Li-Po 3.7V**.

---

### ⚡ 1. Perhitungan Kebutuhan Kapasitas Baterai (*Power Budget Calculation*)

#### **A. HAV Node (Node Stang Kemudi)**

* **Konsumsi Arus Rata-rata Komponen**:
  * ESP32 (Akuisisi 3200 Hz, komputasi filter IIR biquad, transmisi BLE Server 1 Hz): $\approx 90\text{ mA} - 110\text{ mA}$
  * Sensor ADXL345: $\approx 0{,}14\text{ mA}$
  * **Total Arus Rata-rata ($I_{avg}$)**: $\approx \mathbf{100\text{ mA}}$
* **Perhitungan Kapasitas Teoretis (8 Jam)**:
  $$
  C = I_{avg} \times t = 100\text{ mA} \times 8\text{ jam} = 800\text{ mAh}
  $$
* **Dengan *Safety Margin* 25% (Faktor Penuaan & Efisiensi LDO)**:
  $$
  C_{rekomendasi} \ge 800\text{ mAh} \times 1{,}25 = \mathbf{1000\text{ mAh} - 1200\text{ mAh}}
  $$
* **Pilihan Bentuk Baterai**:
  * **Li-Po 3.7V 1000 – 1200 mAh** (Bentuk pipih/kotak berbobot $\approx 20 - 25$ gram, sangat ideal agar total bobot HAV Node tetap di bawah batas **50 gram** pada stang kemudi).
  * Atau **1x Sel Li-Ion 18650 (2000 – 2600 mAh)** jika menggunakan *housing* silinder standar (bisa bertahan $> 18 - 20$ jam).

---

#### **B. Main Unit (Node Jok Sepeda Motor)**

* **Konsumsi Arus Rata-rata Komponen**:
  * ESP32 Dual-Core (Akuisisi WBV 200 Hz, BLE Central RX aktif, FreeRTOS scheduler): $\approx 120\text{ mA} - 140\text{ mA}$
  * Sensor ADXL345 (WBV): $\approx 0{,}14\text{ mA}$
  * Modul RTC DS3231: $\approx 0{,}20\text{ mA}$
  * Modul MicroSD (Operasi penulisan berkas SPI 1 Hz & *flush*): $\approx 20\text{ mA} - 30\text{ mA}$
  * Layar OLED 1.3" SH1106: $\approx 15\text{ mA}$ saat aktif (turun menjadi $< 2\text{ mA}$ saat mode *screen saver* titik 1 Hz aktif)
  * **Total Arus Rata-rata ($I_{avg}$)**: $\approx \mathbf{150\text{ mA}} - \mathbf{170\text{ mA}}$
* **Perhitungan Kapasitas Teoretis (8 Jam)**:
  $$
  C = I_{avg} \times t = 160\text{ mA} \times 8\text{ jam} = 1280\text{ mAh}
  $$
* **Dengan *Safety Margin* 30% (Lonjakan SPI SD Card & BLE RX)**:
  $$
  C_{rekomendasi} \ge 1280\text{ mAh} \times 1{,}30 = \mathbf{1700\text{ mAh} - 2200\text{ mAh}}
  $$
* **Pilihan Bentuk Baterai**:
  * **1x Sel Li-Ion 18650 (2200 – 3000 mAh, 3.7V)** (Sangat direkomendasikan dan pas dengan batas bobot maksimum **150 gram** untuk penempatan di bawah jok). Baterai ini mampu memberikan waktu operasional kontinu hingga **12 – 15 jam**.

---

### 📋 Ringkasan Rekomendasi Kapasitas:

| Unit                       | Arus Rata-rata ($I_{avg}$) | Kapasitas Minimum (8 Jam) |                     Rekomendasi Baterai Terbaik                     | Estimasi Durasi Nyata |
| :------------------------- | :--------------------------: | :-----------------------: | :------------------------------------------------------------------: | :--------------------: |
| **HAV Node** (Stang) |  $\approx 100\text{ mA}$  |    **1000 mAh**    | **Li-Po 3.7V 1000–1200 mAh** atau **1x 18650 2000 mAh** | **10 – 18 Jam** |
| **Main Unit** (Jok)  |  $\approx 160\text{ mA}$  |    **1800 mAh**    |              **1x Li-Ion 18650 (2200–2600 mAh)**              | **12 – 15 Jam** |

*Catatan: Pastikan baterai dihubungkan melalui modul charger & proteksi **TP4056 + DW01** serta regulator LDO **MCP1700-3302E** (3.3V) sesuai diagram rangkaian kelistrikan pada dokumen desain.*
