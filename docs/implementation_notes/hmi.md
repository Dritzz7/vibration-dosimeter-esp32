# Catatan Teknis: Antarmuka Pengguna & Indikator Visual (*HMI & Visual Indicators*)

Dokumen ini mengonsolidasikan seluruh catatan teknis terkait perancangan antarmuka manusia-mesin (*Human-Machine Interface* / HMI), mencakup indikator visual **Mini RGB LED** pada HAV Node serta antarmuka layar **OLED SH1106** dan tombol fisik pada Main Unit.

---

## 1. Mini RGB LED pada HAV Node (KY-016)

Node sensor tangan (HAV Node) dirancang beroperasi tanpa layar OLED maupun RTC untuk meminimalkan dimensi fisik dan beban pada stang kemudi. Sebagai indikator visual ringkas, modul **Mini RGB LED (KY-016 / Common Cathode)** digunakan untuk memberikan umpan balik operasional *real-time*.

### A. Skema Pengkabelan (*Hardware Wiring*)
| Pin Modul KY-016 | Pin ESP32 HAV Node | Fungsi | Karakteristik / Catatan |
| :--- | :--- | :--- | :--- |
| **R (Red)** | **GPIO 32** | Kanal LED Merah | ADC1_CH4 (Aman dari domain RTC & BLE) |
| **G (Green)** | **GPIO 26** | Kanal LED Hijau | DAC2 / GPIO umum |
| **B (Blue)** | **GPIO 27** | Kanal LED Biru | Touch7 / GPIO umum |
| **- (GND)** | **GND** | Ground bersama | Dihubungkan ke sistem ground ESP32 |

* **Polaritas**: Common Cathode (`#define RGB_LED_COMMON_CATHODE 1`), Active HIGH (Duty cycle 255 = menyala penuh, 0 = padam).

---

### B. Pemetaan Status Sistem (FSM) & Pola Kedip LED
| Skenario | Status FSM HAV Node | Warna LED | Pola / Perilaku Kedip | Frekuensi / Waktu | Indikasi Operasional |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **A** | `HAV_STATE_INIT`<br>`HAV_STATE_BLE_ADVERTISING` | **Biru** | Berkedip lambat (*Slow Blink*) | 1,0 Hz (500 ms ON, 500 ms OFF) | Sistem sedang booting dan memancarkan sinyal iklan BLE, menunggu koneksi Main Unit. |
| **B** | `HAV_STATE_LOGGING_NORMAL` | **Hijau** | Bernapas halus (*Smooth Breathing*) | Siklus 2,0 detik (sinusoidal, 35–255 PWM) | Terhubung dengan Main Unit via BLE, sensor ADXL345 aktif mengambil sampel getaran pada 3200 Hz. |
| **D** | `HAV_STATE_COMM_LOST` | **Merah** | Berkedip cepat (*Rapid Blink*) | 2,0 Hz (250 ms ON, 250 ms OFF) | Tautan nirkabel BLE terputus di tengah pengukuran. |
| **E** | `HAV_STATE_ERROR` | **Merah** | Berkedip cepat (*Rapid Blink*) | 2,0 Hz (250 ms ON, 250 ms OFF) | Kegagalan inisialisasi sensor ADXL345 atau galat bus I2C. |
| **-** | `HAV_STATE_LOW_BATTERY` | **Merah** | Kedip ganda (*Double Blink*) | Dua pulsa 100 ms setiap periode 3,0 detik | Tegangan baterai berada di bawah batas aman (< 3,40 V). |

### C. Rasionalisasi Desain:
* **Penghapusan Skenario C (Peringatan Paparan Getaran EAV/ELV)**: Sistem ini dikembangkan sebagai artefak penelitian dan instrumen dosimetri ilmiah, bukan alat peringatan langsung (*warning dashboard*) untuk pengendara. Logika Skenario C dihapus agar tidak membebani komputasi node sensor dan memastikan status visual tetap intuitif bagi peneliti.
* **Standarisasi Warna Merah**: Kondisi anomali (*Comm Lost*, *Hardware Error*, dan *Low Battery*) distandarisasi menggunakan warna **Merah** dengan pola kedip yang berbeda untuk mempertegas kondisi yang membutuhkan perhatian operator.

---

### D. Arsitektur Perangkat Lunak (*Non-Blocking FreeRTOS Task*)
Pengambilan data akselerasi ADXL345 berlangsung pada laju sampling tinggi **3200 Hz** (periode 312,5 µs). Pola kedip LED **sama sekali tidak boleh menggunakan fungsi pemblokir (*blocking delay*)** seperti `delay()`.
1. **Dedicated Background Task (`vTaskRgbLedPattern`)**:
   * Berjalan di **Core 1** dengan prioritas latar belakang (`priority = 1`).
   * Pembaruan efek dilakukan secara deterministik pada frekuensi **50 Hz** (interval 20 ms menggunakan `vTaskDelayUntil`).
2. **Sinkronisasi Thread-Safe**:
   * Transisi status dilindungi menggunakan FreeRTOS Mutex (`s_mutex_led`) via fungsi `rgb_led_set_state(new_state)`.
3. **Efek Breathing Sinusoidal**:
   * Diimplementasikan dengan rumus sinusoidal berbasis *tick*:
     $$\text{duty} = 35 + \left(\frac{1 - \cos(\omega t)}{2}\right) \times (255 - 35)$$
     menghasilkan modulasi kecerahan yang halus tanpa menyita waktu siklus CPU.

---

## 2. Layar OLED SH1106 pada Main Unit

Main Unit dilengkapi layar grafis **OLED 1,3 inci (SH1106 I2C, alamat 0x3C)** untuk menampilkan status diagnostik sistem.

### A. Investigasi Temuan Kritis: Error Driver GPIO SH1106
* **Gejala**: Log Serial Monitor menampilkan error berkelanjutan:
  ```text
  [E][esp32-hal-gpio.c:102] __pinMode(): Invalid pin selected
  gpio_set_level(227): GPIO output gpio_num error
  ```
* **Penyebab**: Pustaka lama meng-cast nilai pin reset `-1` (tanpa pin reset fisik) menjadi *unsigned byte* `227`/`255` lalu memanggil `pinMode(227)`. Driver ESP32 menolak nomor GPIO di atas 39.
* **Solusi**: Migrasi ke pustaka resmi **`Adafruit SH110X`** (`Adafruit_SH1106G`) dengan instansiasi:
  ```cpp
  static Adafruit_SH1106G oled(128, 64, &Wire, -1);
  ```
  serta memperbarui konstanta warna menjadi `SH110X_WHITE`.

---

### B. Tata Letak Informasi Tampilan OLED
Pada pengoperasian normal, layar diperbarui pada frekuensi 1 Hz di `vTaskHMIAndController`:
```text
┌────────────────────┐
│VIBRATION DOSIMETER │
│====================│
│STATUS:  LOGGING    │
│HAV BLE: DATA       │
│WBV Acc: OK         │
│SD Card: OK         │
│RTC:     OK         │
└────────────────────┘
```
* **Status FSM**: Menampilkan `INIT`, `SELF TEST`, `READY`, `LOGGING`, atau `ERROR`.
* **Status Periferal**:
  - `HAV BLE`: `DATA` (menerima paket), `CONN` (terhubung, menunggu data), atau `DISC` (terputus).
  - `WBV Acc`, `SD Card`, `RTC`: `OK` atau `ERR`.

---

### C. Mode Hemat Daya (*Screensaver*)
Untuk menghemat konsumsi daya baterai hingga $\approx 13\text{ mA}$ saat pengukuran berlangsung lama:
* Jika sistem berada pada status `SYS_LOGGING` selama lebih dari **30 detik** tanpa ada penekanan tombol fisik (`OLED_SCREENSAVER_S = 30`), layar OLED dipadamkan.
* Layar hanya menyisakan satu **titik kecil berkedip 1 Hz** di sudut kanan atas layar (`x=124, y=4, r=2`) sebagai indikator bahwa dosimeter tetap hidup dan aktif merekam getaran.
* Setiap penekanan tombol fisik atau transisi ke status `SYS_ERROR` seketika membangunkan layar ke tampilan diagnostik penuh.

---

### D. Penanganan Tombol Fisik HMI
* **Pin**: GPIO 0 (Push-Button terpasang dengan rangkaian penarik internal/eksternal active-low).
* **Debounce Non-blocking**: Menggunakan algoritma jendela geser (*sliding window*) 50 ms (`BUTTON_DEBOUNCE_MS`) berbasis `millis()` tanpa pemblokiran eksekusi FreeRTOS.
