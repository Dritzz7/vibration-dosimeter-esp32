# RENCANA KERJA AKSELERASI PoC (3-DAY ULTRA-SPRINT)

**Proyek Tugas Akhir:** Vibration Dosimeter Portabel pada Pengendara Sepeda Motor  
**Target Presentasi & Demo:** 9 Juni 2026  
**Tenggat Pengumpulan Dokumen:** 8 Juni 2026  
**Kelompok:** TA2526.02.003

---

## 1. STRUKTUR WORK BREAKDOWN STRUCTURE (WBS) AKSELERASI & ALOKASI PERAN

Untuk sisa 3 hari ini, alokasi peran bergeser menjadi mode full-parallel engineering:

- **Rafi Ananta Alden (Firmware & Multitasking):**
  - Fokus penuh pada integrasi kode FreeRTOS dual-core di ESP32.
  - Mengamankan kestabilan queue data dari Core 0 ke Core 1.
  - Memastikan penulisan non-blocking FatFs ke SD Card berjalan tanpa menghalangi sampling 200 Hz.

- **Fachri Ananda Hauzan (DSP, Matematika, & Slide Prep):**
  - Mengekspor koefisien filter IIR (a dan b) dari MATLAB ke format array C++ untuk diserahkan ke Rafi.
  - Melakukan kalkulasi galat (error tuning).
  - Memimpin penyusunan draf presentasi serta visualisasi grafik hasil filter.

- **Nicholas Darren (Hardware Integrator & Enclosure):**
  - Menyelesaikan sirkuit fisik di perfboard/solderan rapi agar siap pakai.
  - Mengemas rangkaian dalam box pelindung darurat.
  - Memastikan kestabilan tegangan LDO dan memasang kapasitor pengaman.

---

## 2. LINIMASA SPRINT 3 HARI TERAKHIR (6 Juni – 9 Juni 2026)

| Aktivitas Utama / Garis Waktu       | Sab, 6 Jun (Hari Ini) | Ming, 7 Jun           | Sen, 8 Jun (Tenggat) | Sel, 9 Jun (H-Day) |
|-------------------------------------|-----------------------|-----------------------|----------------------|-------------------|
| Fase 1 & 2: Finalisasi HW & Driver  | [====================] |                       |                      |                   |
| Fase 3: FreeRTOS Dual-Core & IIR    | [=========]            | [==================]  |                      |                   |
| Fase 4: Pengujian & Validasi Galat  |                       | [=========]           | [==================] |                   |
| Fase 5: Box Packaging & Slide Prep  |                       |                       | [==================] | [====] (Pagi)     |
| Fase 6: Demo & Presentasi Sempro    |                       |                       |                      | [==================] |

---

## 3. RENCANA AKSI DETAIL HARIAN

### Hari 1: Sabtu, 6 Juni 2026 (Finalisasi Hardware & Integrasi Dasar)
- **Pagi - Siang (Target: 13.00 WIB):**
  - Darren: Memindahkan rangkaian ke perfboard solderan, pasang kapasitor decoupling 100 µF dekat pin VCC SD Card.
  - Fachri: Menuntaskan pembobotan frekuensi filter \(W_d\) dan \(W_k\) di MATLAB, ekspor koefisien array a dan b (float32).
  - Rafi: Menyiapkan kerangka dasar kode FreeRTOS di ESP32 dengan task minimal.

- **Sore - Malam (Target: 22.00 WIB):**
  - Darren & Rafi: Uji coba pembacaan sensor ADXL345 dan penyimpanan SD Card. Pastikan RTC DS3231 menulis timestamp ke log CSV.

- **Milestone Hari 1:** Perangkat keras kokoh tersolder, ESP32 sukses membaca data sensor 200 Hz dan mencatat ke SD Card.

---

### Hari 2: Minggu, 7 Juni 2026 (FreeRTOS Dual-Core & Porting Filter IIR)
- **Pagi - Siang (Target: 14.00 WIB):**
  - Rafi: Aktifkan arsitektur Dual-Core ESP32.
    - Core 0: Interupsi timer 5 ms (200 Hz) → baca sensor, filter IIR, hitung RMS, masukkan ke xQueue.
    - Core 1: Ambil data tiap 1 s, sematkan timestamp RTC, buffer flushing ke SD Card.
  - Fachri: Mulai draf presentasi PPT (Slide 1–8).

- **Sore - Malam (Target: 22.00 WIB):**
  - Rafi & Fachri: Uji coba pembacaan getaran motor statis 1 menit, validasi file CSV di MATLAB.

- **Milestone Hari 2:** Sistem multitasking dual-core berjalan tanpa frame drop, filter terintegrasi.

---

### Hari 3: Senin, 8 Juni 2026 (Validasi Galat, Enclosure, & Slide Prep) – Tenggat Dokumen
- **Pagi - Siang (Target: 13.00 WIB):**
  - Fachri: Bandingkan grafik filter ESP32 vs MATLAB, tuning gain jika galat RMS > 5%.
  - Darren: Masukkan sirkuit ke box pelindung, pastikan tombol HMI & OLED kokoh.

- **Sore - Malam (Target: 23.59 WIB):**
  - Rafi: Mengunci Safe-Logging Protocol (file tetap aman jika SD Card dicabut).
  - Kelompok: Rampungkan slide (1–16), sisipkan grafik perbandingan filter & foto alat. Kirim dokumen B100-B300 ke pembimbing.

- **Milestone Hari 3:** Produk PoC 100% siap fisik & program; slide rampung; galat < 5%.

---

### Hari 4: Selasa, 9 Juni 2026 (H-Day: Demo & Seminar Proposal)
- **Pagi (07.00 - 08.30 WIB):**
  - Final rehearsal tanya jawab di Labtek V.
  - Pastikan baterai 18650 penuh 100% dan bawa SD cadangan.

- **Seminar (20 Menit Maksimal):**
  - Presentasi slide secara lugas.
  - Demo fungsionalitas langsung di depan dosen penguji.

---

## 4. PROTOKOL MITIGASI DARURAT (Crash-Handling Sprint)

- **Jika Integrasi IIR C++ di ESP32 Gagal/Bug:**
  - Simpan data mentah akselerasi (ax, ay, az) ke SD Card.
  - Lakukan filter digital post-processing di laptop saat demo.

- **Jika Komunikasi Nirkabel BLE Bermasalah:**
  - Gunakan kabel data sementara untuk koneksi node stang–unit jok.
  - Jelaskan kepada penguji bahwa BLE masih dalam fase penyempurnaan.

- **Jika Terjadi Korupsi File FAT32 di SD Card:**
  - Gunakan instruksi `f_sync()` setelah setiap penulisan baris data.
  - Hindari file terbuka terus-menerus tanpa penyimpanan berkala.
