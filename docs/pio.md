Viewed main.cpp:1121-1148

Berikut adalah panduan lengkap perintah **PlatformIO CLI (`pio`)** untuk meng-compile, mengunggah (*upload*), dan memantau Serial Monitor baik dari dalam folder proyek maupun dari root workspace:

---

### 1. Memeriksa Port COM yang Terhubung
Sebelum mengunggah, Anda dapat mengecek port COM mana saja yang terdeteksi di komputer Anda:
```powershell
pio device list
```

---

### 2. Meng-compile (Build) Firmware

* **Jika terminal berada di dalam direktori sub-proyek** (misal `cd Main-Unit` atau `cd HAV-Node`):
  ```powershell
  pio run
  ```
* **Jika terminal berada di root workspace** (`vibration-dosimeter-esp32`):
  ```powershell
  # Compile Main Unit
  pio run -d Main-Unit

  # Compile HAV Node
  pio run -d HAV-Node
  ```

---

### 3. Mengunggah (Upload) Firmware ke ESP32

* **Otomatis mendeteksi port COM**:
  ```powershell
  # Jika di dalam direktori Main-Unit
  pio run -t upload

  # Jika dari root workspace
  pio run -d Main-Unit -t upload
  ```

* **Menentukan port COM secara spesifik** (contoh: `COM12` atau `COM11`):
  ```powershell
  # Mengunggah ke COM12
  pio run -t upload --upload-port COM12

  # Dari root workspace ke COM12
  pio run -d Main-Unit -t upload --upload-port COM12
  ```

---

### 4. Memantau Serial Monitor

* **Memuka Serial Monitor (kecepatan baud rate akan otomatis mengikuti `platformio.ini` yaitu 115200)**:
  ```powershell
  # Jika di dalam direktori Main-Unit
  pio device monitor

  # Menentukan port tertentu (misal COM12)
  pio device monitor -p COM12
  ```

* **Dari root workspace**:
  ```powershell
  pio device monitor -d Main-Unit
  ```

* **Keluar dari Serial Monitor**: Tekan `Ctrl + C` atau `Ctrl + ]`.

---

### 5. PERINTAH SAPU JAGAT (Build + Upload + Monitor Sekaligus)

Anda dapat menggabungkan proses *compile*, *upload*, dan *serial monitor* dalam **satu baris perintah**:

```powershell
# Di dalam folder Main-Unit
pio run -t upload -t monitor

# Dengan menentukan port COM12 secara spesifik
pio run -t upload -t monitor --upload-port COM12 -p COM12
```

---

### 6. Perintah Tambahan yang Berguna

* **Membersihkan hasil build lama (Clean)**:
  ```powershell
  pio run -t clean
  ```
* **Memasang / memperbarui pustaka (`lib_deps`) secara manual**:
  ```powershell
  pio lib install
  ```