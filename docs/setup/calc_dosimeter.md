# Vibration Dosimeter Utility Scripts

Folder ini berisi skrip utilitas pasca-pemrosesan (*post-processing*) untuk menganalisis berkas log `dosimeter.csv` hasil rekaman Main Unit.

## 📄 Skrip: `calc_dosimeter.py`

Skrip Python ini membaca log CSV 1 Hz dan menghitung nilai-nilai paparan getaran harian sesuai standar internasional **ISO 5349-1** (Hand-Arm Vibration / HAV) dan **ISO 2631-1** (Whole-Body Vibration / WBV).

### 🔢 Parameter yang Dihitung:
1. **Hand-Arm Vibration (HAV)**:
   - $a_{hwx}, a_{hwy}, a_{hwz}$ (RMS Terbobot $W_h$ per Sumbu)
   - $a_{hv,eq}$ (Total Vektor RMS Terbobot)
   - **$A_{HAV}(8)$** (Dosis Paparan Harian Ekivalen 8 Jam)
   - Estimasi sisa waktu aman sebelum mencapai **EAV (2.5 m/s²)** dan **ELV (5.0 m/s²)**

2. **Whole-Body Vibration (WBV)**:
   - $a_{wx}, a_{wy}, a_{wz}$ (RMS Terbobot $W_d/W_k$ per Sumbu)
   - Percepatan Terbobot Arah ($1.4 \cdot a_{wx}, 1.4 \cdot a_{wy}, 1.0 \cdot a_{wz}$)
   - $A_x(8), A_y(8), A_z(8)$ (Dosis Paparan Harian per Sumbu)
   - **$A_{WBV}(8)$** (Nilai Paparan Harian Sumbu Dominan $\max(A_x, A_y, A_z)$)
   - **$VDV$** (*Vibration Dose Value* Metode Pangkat-4 per sumbu & total combined $m/s^{1.75}$)
   - **$eVDV$** (*Estimated Vibration Dose Value*)
   - Estimasi sisa waktu aman sebelum mencapai **EAV (0.50 m/s² / 9.1 VDV)** dan **ELV (1.15 m/s² / 21.0 VDV)**

---

### 💻 Cara Penggunaan:

```powershell
# Jalankan kalkulasi metrik sekaligus menampilkan jendela grafik visualisasi
python util/calc_dosimeter.py

# Atau tentukan path file secara eksplisit
python util/calc_dosimeter.py "f:\dosimeter.csv"

# Simpan grafik visualisasi ke file gambar (PNG)
python util/calc_dosimeter.py "f:\dosimeter.csv" -s dosimeter_plot.png

# Simpan laporan teks dan file gambar secara simultan
python util/calc_dosimeter.py "f:\dosimeter.csv" -o report.txt -s dosimeter_plot.png

# Jalankan hanya kalkulasi teks di terminal tanpa membuka jendela grafik
python util/calc_dosimeter.py "f:\dosimeter.csv" --no-plot
```

### 📊 Panel Grafik Visualisasi:
1. **Panel 1 (HAV Time Series)**: Profil getaran 1 Hz RMS sumbu X, Y, Z, total vektor $a_{hv}$, dan garis rata-rata ekivalen $a_{hv,eq}$.
2. **Panel 2 (WBV Time Series)**: Profil getaran 1 Hz RMS sumbu X, Y, Z, total vektor $a_v$, dan garis rata-rata ekivalen $a_{v,eq}$.
3. **Panel 3 (Progressive A(8) vs ISO Limits)**: Kurva akumulasi dosis harian $A_{HAV}(8)$ dan $A_{WBV}(8)$ terhadap garis batas ambang **EAV** dan **ELV**.
4. **Panel 4 (Progressive VDV)**: Akumulasi dosis getaran metode pangkat-4 ($VDV_x, VDV_y, VDV_z, VDV_{total}$) terhadap batas EAV ($9{,}1\text{ m/s}^{1{,}75}$) dan ELV ($21{,}0\text{ m/s}^{1{,}75}$).
