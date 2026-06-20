# Temuan Penting: Brownout Reset pada ESP32 Main Unit

## 1. Gejala & Temuan Utama
* **Gejala**: ESP32 Main Unit mengalami *infinite loop reset* setelah berhasil di-flash.
* **Pesan Error pada Serial Monitor**: `Brownout detector was triggered`.
* **Pemicu**: Pengaktifan task `vTaskBLEReceiver` (modul nirkabel BLE) yang menarik lonjakan arus (*current spike*) mendadak sekitar **150 mA - 250 mA**. Tegangan catu daya mengalami penurunan sesaat (*dip*) di bawah batas aman sehingga memicu pengaman internal mikrokontroler.

---

## 2. Solusi Pemecahan Masalah

### A. Solusi Hardware (Rekomendasi Utama)
1. **Ganti Kabel & Port USB**:
   * Gunakan kabel USB pendek berkualitas baik (resistansi rendah).
   * Pindahkan koneksi ke port USB 3.0 (warna biru) atau gunakan adapter charger HP (5V 1A/2A) eksternal.
2. **Kapasitor Decoupling**:
   * Pasang kapasitor elektrolit (elco) **100 uF s.d. 1000 uF** secara paralel di antara pin **`3V3` dan `GND`** sedekat mungkin dengan modul ESP32 untuk menyangga lonjakan arus.

### B. Solusi Software (Bypass Sementara)
Nonaktifkan sensor pendeteksi brownout dengan menambahkan kode berikut pada bagian atas fungsi `setup()` di `main.cpp`:
```cpp
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Matikan detektor brownout
    // ... sisa kode setup ...
}
```
*Catatan: Metode software bypass ini berisiko membuat ESP32 hang atau freeze jika tegangan drop terlalu dalam.*
