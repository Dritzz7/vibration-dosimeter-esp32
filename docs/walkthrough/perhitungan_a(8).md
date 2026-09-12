# Perhitungan A(8)

### 1. Perhitungan $A(8)$ dan $eVDV$ Meski Pengukuran Belum 8 Jam

Sesuai standar **ISO 5349-1** (HAV) dan **ISO 2631-1** (WBV), metrik **$A(8)$** adalah **nilai ekivalen energi getaran yang dinormalisasi ke durasi kerja standar 8 jam ($T_0 = 8\text{ jam} = 28.800\text{ detik}$)**.

Artinya, instrumen **tidak perlu menunggu pengukuran berjalan selama 8 jam penuh**. Algoritma menggunakan prinsip **kesetaraan energi (*Root-Mean-Square Energy Equivalence*)**:

$$
E \propto A(8)^2 \times T_0 = a_{eq}^2 \times T_{exp}
$$

Dengan $T_0 = 28.800\text{ detik}$ dan $T_{exp} = N \times \Delta t$ ($N$ adalah jumlah sampel detik yang telah terekam sejauh ini):

$$
A(8) = a_{eq} \times \sqrt{\frac{T_{exp}}{28.800}}
$$

#### A. Hand-Arm Vibration ($A_{HAV}(8)$ — ISO 5349-1)

1. Nilai RMS akselerasi total vektor getaran ($a_{hv}$) dihitung dari $N$ sampel yang terkumpul:
   $$
   a_{hv,eq} = \sqrt{\frac{1}{N} \sum_{i=1}^{N} a_{hv,i}^2}
   $$
2. Nilai paparan harian $A_{HAV}(8)$ dihitung dengan:
   $$
   A_{HAV}(8) = a_{hv,eq} \times \sqrt{\frac{N \times 1\text{ detik}}{28.800}} = \sqrt{\frac{\sum_{i=1}^{N} a_{hv,i}^2}{28.800}}
   $$

   *Perhatikan bahwa secara progresif setiap detik data baru masuk ($a_{hv,i}$), kuadrat energinya dijumlahkan dan dibagi $28.800$. Nilai $A(8)$ akan terakumulasi secara bertahap seiring berjalannya waktu.*

#### B. Whole-Body Vibration ($A_{WBV}(8)$ — ISO 2631-1)

1. Diterapkan faktor pengali arah postur duduk ($k_x = 1{,}4$, $k_y = 1{,}4$, $k_z = 1{,}0$):
   $$
   A_x(8) = 1{,}4 \cdot a_{wx,eq} \sqrt{\frac{T_{exp}}{28.800}}, \quad A_y(8) = 1{,}4 \cdot a_{wy,eq} \sqrt{\frac{T_{exp}}{28.800}}, \quad A_z(8) = 1{,}0 \cdot a_{wz,eq} \sqrt{\frac{T_{exp}}{28.800}}
   $$
2. Standar ISO 2631-1 menetapkan bahwa paparan harian ditentukan oleh **sumbu dominan (terbesar)**:
   $$
   A_{WBV}(8) = \max\left(A_x(8), A_y(8), A_z(8)\right)
   $$

#### C. Estimated Vibration Dose Value ($eVDV$)

Menurut ISO 2631-1 Lampiran B, jika getaran bersifat stasioner tanpa sentakan impulsif ekstrem (*low crest factor*), nilai dosis getaran dapat diestimasi langsung dari durasi berjalan $T_{exp}$ dan RMS getaran total $a_{v,eq}$:

$$
eVDV = 1{,}4 \times a_{v,eq} \times \left(T_{exp}\right)^{0{,}25} \quad [\text{m/s}^{1{,}75}]
$$

Karena $T_{exp}$ diketahui dari jumlah sampel detik ($N$), nilai $eVDV$ dapat dihitung secara *real-time* untuk durasi berapa pun.

---

### 2. Perhitungan "Time Allowed" Berdasarkan EAV dan ELV

Metrik **"Time allowed"** menjawab pertanyaan prediktif:

> *"Jika pekerja terus berkendara dengan tingkat intensitas getaran rata-rata saat ini ($a_{eq}$), berapa lama total waktu kerja yang diperbolehkan sebelum menyentuh batas aman EAV atau batas bahaya ELV?"*

#### Penurunan Rumus:

Dari rumus $A(8)$:

$$
\text{Limit} = a_{eq} \times \sqrt{\frac{T_{allowed}}{28.800}}
$$

Jika kedua sisi dikuadratkan:

$$
\text{Limit}^2 = a_{eq}^2 \times \frac{T_{allowed}}{28.800}
$$

Maka diperoleh formula waktu yang diizinkan ($T_{allowed}$):

$$
T_{allowed} = 28.800 \times \left( \frac{\text{Limit}}{a_{eq}} \right)^2 \quad [\text{detik}]
$$

---

#### Penerapan pada Kode `calc_dosimeter.py`:

1. **Untuk HAV (ISO 5349-1)**:

   - **EAV (*Exposure Action Value*) = $2{,}50\text{ m/s}^2$**:
     $$
     T_{EAV, HAV} = 28.800 \times \left( \frac{2{,}50}{a_{hv,eq}} \right)^2
     $$
   - **ELV (*Exposure Limit Value*) = $5{,}00\text{ m/s}^2$**:
     $$
     T_{ELV, HAV} = 28.800 \times \left( \frac{5{,}00}{a_{hv,eq}} \right)^2
     $$
2. **Untuk WBV (ISO 2631-1)**:
   Menggunakan nilai akselerasi terbobot dari sumbu terdominan ($a_{w,max} = \max(1{,}4 a_{wx,eq}, 1{,}4 a_{wy,eq}, 1{,}0 a_{wz,eq})$):

   - **EAV = $0{,}50\text{ m/s}^2$**:
     $$
     T_{EAV, WBV} = 28.800 \times \left( \frac{0{,}50}{a_{w,max}} \right)^2
     $$
   - **ELV = $1{,}15\text{ m/s}^2$**:
     $$
     T_{ELV, WBV} = 28.800 \times \left( \frac{1{,}15}{a_{w,max}} \right)^2
     $$

---

### Contoh Interpretasi Fisik:

* **Jika $a_{hv,eq} = 2{,}50\text{ m/s}^2$ (sama dengan batas EAV)**:
  $$
  T_{EAV} = 28.800 \times \left(\frac{2{,}50}{2{,}50}\right)^2 = 28.800\text{ detik} = \mathbf{8\text{ jam}}.
  $$
* **Jika $a_{hv,eq} = 5{,}00\text{ m/s}^2$ (2 kali lebih kuat dari EAV)**:
  $$
  T_{EAV} = 28.800 \times \left(\frac{2{,}50}{5{,}00}\right)^2 = 28.800 \times \frac{1}{4} = 7.200\text{ detik} = \mathbf{2\text{ jam}}.
  $$

  *(Karena energi getaran berlipat ganda secara kuadratik, waktu kerja aman terpangkas menjadi seperempatnya).*
* **Jika getaran sangat kecil / mesin mati ($a_{eq} \approx 0$)**:
  $T_{allowed} \rightarrow \infty$ (pada skrip diformat menjadi `"N/A (> 1 year)"`).
