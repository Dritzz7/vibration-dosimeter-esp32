/**
 * @file main.cpp
 * @brief GPS u-blox NEO-6M Diagnostic & Debugging Sketch for ESP32
 *
 * Fitur Debugging:
 *  1. Deteksi hardware otomatis pada saat startup (NMEA framing probe).
 *  2. Pull-up internal pada RXD2 (GPIO 16) untuk mencegah derau floating.
 *  3. Dashboard telemetri berkala (1 Hz) dengan metrik lengkap:
 *     - Status Fix & Akurasi (HDOP & Satellites)
 *     - Koordinat Geospasial, Kecepatan, dan Ketinggian
 *     - Waktu UTC & Waktu Lokal (WIB / UTC+7)
 *     - Statistik parsing NMEA (Bytes processed, Passed Checksum, Failed Checksum)
 *  4. Mode Raw NMEA toggle via Serial Monitor (Ketik 'r' untuk melihat $GPRMC, $GPGGA mentah).
 *  5. Peringatan dini kesalahan kabel (No Data / RX-TX terbalik / Baudrate mismatch).
 */

#include <Arduino.h>
#include <TinyGPS++.h>

// =============================================================================
// KONFIGURASI PIN & SERIAL UART2
// =============================================================================
#define PIN_GPS_RX 16   // ESP32 RX2 <- Modul GPS TX
#define PIN_GPS_TX 17   // ESP32 TX2 -> Modul GPS RX (Opsional)
#define GPS_BAUD   9600 // Baudrate default u-blox NEO-6M

// Objek Parser TinyGPS++ dan HardwareSerial UART2
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

// Status & Variabel Debugging
bool showRawNmea = false;
bool showDashboard = true;
uint32_t lastDashboardTime = 0;
uint32_t lastByteReceivedTime = 0;
uint32_t totalBytesReceived = 0;
uint8_t spinnerIdx = 0;
const char spinnerChars[] = "|/-\\";

// Prototipe Fungsi
void printHelp();
void printDiagnostics();
void handleSerialCommands();

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n==================================================");
  Serial.println("   ESP32 u-blox NEO-6M GPS Diagnostic Suite");
  Serial.println("==================================================");
  Serial.printf("[INIT] RX Pin: GPIO %d (Internal Pull-Up AKTIF)\n", PIN_GPS_RX);
  Serial.printf("[INIT] TX Pin: GPIO %d\n", PIN_GPS_TX);
  Serial.printf("[INIT] Baudrate GPS: %d bps\n", GPS_BAUD);

  // Aktifkan internal pull-up untuk meredam derau frekuensi tinggi saat kabel mengambang
  pinMode(PIN_GPS_RX, INPUT_PULLUP);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

  // Probing awal: deteksi aliran karakter NMEA ($) selama 3 detik
  Serial.print("[PROBE] Memeriksa komunikasi serial GPS...");
  uint32_t probeStart = millis();
  bool probeDetected = false;

  while (millis() - probeStart < 3000) {
    if (gpsSerial.available()) {
      char c = (char)gpsSerial.read();
      if (c == '$') {
        probeDetected = true;
        break;
      }
    }
    delay(10);
  }

  if (probeDetected) {
    Serial.println(" [OK] Modul GPS terdeteksi mengirim NMEA!");
  } else {
    Serial.println(" [PERINGATAN]");
    Serial.println("   -> Belum menerima karakter NMEA '$'.");
    Serial.println("   -> Periksa kabel: GPS TX -> ESP32 GPIO 16, GPS VCC (3.3V/5V), GND.");
    Serial.println("   -> Pastikan modul GPS mendapat daya yang stabil.");
  }

  printHelp();
  lastDashboardTime = millis();
  lastByteReceivedTime = millis();
}

void loop() {
  // 1. Baca byte dari GPS UART2
  while (gpsSerial.available() > 0) {
    char c = (char)gpsSerial.read();
    totalBytesReceived++;
    lastByteReceivedTime = millis();

    // Jika mode Raw NMEA aktif, teruskan langsung karakter ke Serial Monitor
    if (showRawNmea) {
      Serial.write(c);
    }

    // Masukkan ke parser TinyGPS++
    gps.encode(c);
  }

  // 2. Baca perintah interaktif dari pengguna di Serial Monitor
  handleSerialCommands();

  // 3. Tampilkan Dashboard Telemetri setiap 1 detik
  if (showDashboard && (millis() - lastDashboardTime >= 1000)) {
    lastDashboardTime = millis();
    printDiagnostics();
  }
}

/**
 * @brief Menampilkan ringkasan status diagnostik GPS secara berkala
 */
void printDiagnostics() {
  spinnerIdx = (spinnerIdx + 1) % 4;
  uint32_t silenceTime = millis() - lastByteReceivedTime;
  bool isHardwareConnected = (silenceTime < 3000);

  Serial.println("--------------------------------------------------------------------------------");
  Serial.printf("[%c] STATUS HARDWARE : %s", 
                spinnerChars[spinnerIdx],
                isHardwareConnected ? "TERHUBUNG (Menerima Data)" : "TIDAK ADA DATA! (>3 detik)");
  if (!isHardwareConnected) {
    Serial.printf(" (Diam selama %lu ms)", silenceTime);
  }
  Serial.println();

  // Status Fix Satelit
  bool fixValid = gps.location.isValid() && (gps.location.age() < 2500);
  Serial.printf("    Status Fix       : %s\n", fixValid ? ">> FIX BERHASIL (LOCKED) <<" : "MENCARI SATELIT (NO FIX)");
  Serial.printf("    Satelit Terlacak : %u satelit\n", gps.satellites.isValid() ? gps.satellites.value() : 0);

  // Kualitas Geometri Sinyal (HDOP)
  if (gps.hdop.isValid()) {
    float hdopVal = gps.hdop.hdop();
    const char *hdopQuality = (hdopVal < 1.0f) ? "Ideal" :
                              (hdopVal < 2.0f) ? "Sangat Baik" :
                              (hdopVal < 5.0f) ? "Baik" :
                              (hdopVal < 10.0f) ? "Sedang" : "Buruk";
    Serial.printf("    Presisi (HDOP)   : %.2f (%s)\n", hdopVal, hdopQuality);
  } else {
    Serial.println("    Presisi (HDOP)   : N/A");
  }

  // Posisi Geospasial
  if (gps.location.isValid()) {
    Serial.printf("    Koordinat        : Lat: %.6f, Lng: %.6f (Umur data: %lu ms)\n",
                  gps.location.lat(), gps.location.lng(), gps.location.age());
  } else {
    Serial.println("    Koordinat        : N/A (Menunggu sinyal satelit)");
  }

  // Kecepatan & Ketinggian
  if (gps.speed.isValid()) {
    Serial.printf("    Kecepatan        : %.2f km/h (%.2f knot)\n", gps.speed.kmph(), gps.speed.knots());
  } else {
    Serial.println("    Kecepatan        : N/A");
  }

  if (gps.altitude.isValid()) {
    Serial.printf("    Ketinggian       : %.2f meter\n", gps.altitude.meters());
  } else {
    Serial.println("    Ketinggian       : N/A");
  }

  // Waktu UTC & WIB (UTC+7)
  if (gps.time.isValid() && gps.date.isValid()) {
    int hourWIB = (gps.time.hour() + 7) % 24;
    Serial.printf("    Waktu UTC        : %04d-%02d-%02d %02d:%02d:%02d\n",
                  gps.date.year(), gps.date.month(), gps.date.day(),
                  gps.time.hour(), gps.time.minute(), gps.time.second());
    Serial.printf("    Waktu WIB (UTC+7): %04d-%02d-%02d %02d:%02d:%02d\n",
                  gps.date.year(), gps.date.month(), gps.date.day(),
                  hourWIB, gps.time.minute(), gps.time.second());
  } else {
    Serial.println("    Waktu / Tanggal  : N/A");
  }

  // Statistik Parsing
  Serial.printf("    Statistik Parser : Total Byte: %lu | Chars: %lu | Passed CS: %lu | Failed CS: %lu\n",
                totalBytesReceived,
                gps.charsProcessed(),
                gps.passedChecksum(),
                gps.failedChecksum());

  // Indikasi / Catatan Bantuan jika Belum Ada Fix
  if (!fixValid) {
    if (gps.satellites.isValid() && gps.satellites.value() == 0) {
      Serial.println("    [TIPS] 0 satelit terlihat. Jika sedang di dalam ruangan, pindahkan antena ke dekat jendela/luar.");
    } else {
      Serial.println("    [TIPS] Sinyal satelit sedang dikumpulkan. Cold start memerlukan waktu ~1-3 menit.");
    }
  }
}

/**
 * @brief Memproses input karakter dari Serial Monitor untuk fitur interaktif
 */
void handleSerialCommands() {
  while (Serial.available() > 0) {
    char cmd = (char)Serial.read();
    if (cmd == '\r' || cmd == '\n') continue;

    if (cmd == 'r' || cmd == 'R') {
      showRawNmea = !showRawNmea;
      Serial.printf("\n>>> Mode Raw NMEA: %s <<<\n", showRawNmea ? "AKTIF" : "NONAKTIF");
    } else if (cmd == 's' || cmd == 'S') {
      showDashboard = !showDashboard;
      Serial.printf("\n>>> Dashboard Berkala: %s <<<\n", showDashboard ? "AKTIF" : "NONAKTIF");
    } else if (cmd == 'h' || cmd == 'H' || cmd == '?') {
      printHelp();
    }
  }
}

/**
 * @brief Menampilkan menu bantuan interaktif
 */
void printHelp() {
  Serial.println("\n--- Perintah Serial Interaktif ---");
  Serial.println("  'r' : Toggle Tampilan Raw NMEA stream ($GPRMC, $GPGGA, dll)");
  Serial.println("  's' : Toggle Dashboard Telemetri 1 detik");
  Serial.println("  'h' : Tampilkan menu bantuan ini");
  Serial.println("----------------------------------\n");
}
