"""
Real-time ADXL345 Vibration Visualizer & FFT Spectral Analyzer
==============================================================
Companion tool for validating raw sensor data integrity and dynamic response
(Static 1g, Impulse / Shock, and Sinusoidal Vibration).

Features:
  - Real-time 4-channel Time Domain Plot (ax, ay, az, |a|)
  - Real-time Fast Fourier Transform (FFT) Power Spectrum (0 to Fs/2)
  - Live statistical telemetry: Fs, Peak, RMS, Dominant Frequency
  - Auto-port detection (CP210x, CH340, FTDI) or manual --port argument
  - Hotkey controls forwarded to ESP32 (Tare, ODR switch, Unit toggle)

Usage:
  python visualizer.py [--port COM3] [--baud 115200] [--window 1000]

Dependencies:
  pip install pyserial matplotlib numpy
"""

import sys
import time
import argparse
import threading
from collections import deque
import numpy as np
import serial
import serial.tools.list_ports

try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    from matplotlib.widgets import Button
except ImportError:
    print("[ERR] matplotlib is required! Run: pip install matplotlib numpy pyserial")
    sys.exit(1)


def auto_detect_port():
    """Find most likely ESP32 serial COM port."""
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None
    for p in ports:
        desc = (p.description or "").lower()
        if any(keyword in desc for keyword in ["cp210", "ch340", "ch9102", "usb to uart", "esp32", "silicon labs"]):
            return p.device
    return ports[0].device


class VibrationVisualizer:
    def __init__(self, port, baud=115200, buffer_size=800):
        self.port_name = port
        self.baud_rate = baud
        self.buffer_size = buffer_size

        # Ring buffers for time domain
        self.t_buf = deque(maxlen=buffer_size)
        self.ax_buf = deque(maxlen=buffer_size)
        self.ay_buf = deque(maxlen=buffer_size)
        self.az_buf = deque(maxlen=buffer_size)
        self.am_buf = deque(maxlen=buffer_size)

        self.ser = None
        self.running = True
        self.paused = False
        self.start_time = None
        self.sample_count = 0
        self.measured_fs = 200.0

        # Lock for thread-safe buffer access
        self.data_lock = threading.Lock()

        # Connect to serial port
        self.connect_serial()

        # Start serial reader background thread
        self.reader_thread = threading.Thread(target=self._serial_worker, daemon=True)
        self.reader_thread.start()

    def connect_serial(self):
        try:
            print(f"[INFO] Connecting to {self.port_name} at {self.baud_rate} baud...")
            self.ser = serial.Serial(self.port_name, self.baud_rate, timeout=1.0)
            time.sleep(0.5)
            self.ser.reset_input_buffer()
            print(f"[SUCCESS] Connected to {self.port_name}!")
        except Exception as e:
            print(f"[ERR] Failed to open port {self.port_name}: {e}")
            sys.exit(1)

    def send_command(self, cmd_char):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(cmd_char.encode("utf-8"))
                print(f"[CMD] Sent command: '{cmd_char}'")
            except Exception as e:
                print(f"[ERR] Serial write failed: {e}")

    def _serial_worker(self):
        self.start_time = time.time()
        last_calc_t = self.start_time
        calc_samples = 0

        while self.running:
            try:
                if not self.ser or not self.ser.is_open:
                    time.sleep(0.1)
                    continue

                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                # Print diagnostic or log messages directly
                if line.startswith("#") or line.startswith("[") or line.startswith("="):
                    print(f"ESP32: {line}")
                    continue

                # Parse: ax:%.3f,ay:%.3f,az:%.3f,amag:%.3f
                # Also handles space or comma separation
                tokens = line.replace(",", " ").split()
                data_map = {}
                for tok in tokens:
                    if ":" in tok:
                        k, v = tok.split(":", 1)
                        try:
                            data_map[k.strip()] = float(v.strip())
                        except ValueError:
                            pass

                if "ax" in data_map and "ay" in data_map and "az" in data_map:
                    ax = data_map["ax"]
                    ay = data_map["ay"]
                    az = data_map["az"]
                    amag = data_map.get("amag", np.sqrt(ax * ax + ay * ay + az * az))

                    now_t = time.time() - self.start_time
                    calc_samples += 1

                    # Measure actual frequency every 0.5s
                    dt = time.time() - last_calc_t
                    if dt >= 0.5:
                        self.measured_fs = calc_samples / dt
                        calc_samples = 0
                        last_calc_t = time.time()

                    with self.data_lock:
                        self.t_buf.append(now_t)
                        self.ax_buf.append(ax)
                        self.ay_buf.append(ay)
                        self.az_buf.append(az)
                        self.am_buf.append(amag)

            except Exception as e:
                if self.running:
                    print(f"[WARN] Serial read error: {e}")
                    time.sleep(0.05)

    def start_gui(self):
        plt.style.use("dark_background")
        fig = plt.figure(figsize=(13, 8))
        fig.canvas.manager.set_window_title(f"ADXL345 Vibration Visualizer — {self.port_name}")

        gs = fig.add_gridspec(2, 1, height_ratios=[1.2, 1.0], hspace=0.35)

        # 1. Time-Domain Plot (Top)
        ax_time = fig.add_subplot(gs[0])
        ax_time.set_title("Time-Domain Acceleration Response (Impulse & Sinusoidal Waveforms)", fontsize=11, fontweight="bold", color="#58a6ff")
        ax_time.set_xlabel("Time (seconds)", fontsize=9)
        ax_time.set_ylabel("Acceleration (m/s² / g)", fontsize=9)
        ax_time.grid(True, linestyle="--", alpha=0.3)

        (line_ax,) = ax_time.plot([], [], label="X-Axis", color="#ff4757", lw=1.2)
        (line_ay,) = ax_time.plot([], [], label="Y-Axis", color="#2ed573", lw=1.2)
        (line_az,) = ax_time.plot([], [], label="Z-Axis", color="#1e90ff", lw=1.2)
        (line_am,) = ax_time.plot([], [], label="|a| Magnitude", color="#ffa502", lw=1.8)
        ax_time.legend(loc="upper right", framealpha=0.7, fontsize=8)

        # Telemetry text box inside time plot
        stats_text = ax_time.text(0.015, 0.95, "", transform=ax_time.transAxes,
                                  verticalalignment="top", fontsize=9,
                                  fontfamily="monospace",
                                  bbox=dict(boxstyle="round,pad=0.5", facecolor="#161b22", alpha=0.85, edgecolor="#30363d"))

        # 2. Real-time FFT Frequency Spectrum (Bottom)
        ax_freq = fig.add_subplot(gs[1])
        ax_freq.set_title("Real-Time FFT Frequency Spectrum (Harmonic Peak & Broadband Content)", fontsize=11, fontweight="bold", color="#7ee787")
        ax_freq.set_xlabel("Frequency (Hz)", fontsize=9)
        ax_freq.set_ylabel("Spectral Amplitude (m/s²)", fontsize=9)
        ax_freq.grid(True, linestyle="--", alpha=0.3)

        (line_fft,) = ax_freq.plot([], [], color="#a371f7", lw=1.5, label="Magnitude Spectrum (|a|)")
        peak_marker, = ax_freq.plot([], [], "ro", markersize=6, label="Dominant Frequency")
        ax_freq.legend(loc="upper right", framealpha=0.7, fontsize=8)

        # Instructions bar at the bottom
        fig.text(0.5, 0.01,
                 "Hotkeys: [T] Tare Zero  |  [R] Reset Tare (1g)  |  [U] Toggle Units  |  [1-6] Switch ODR (100-3200Hz)  |  [Space] Pause",
                 ha="center", fontsize=9, color="#8b949e")

        def update_frame(frame):
            if self.paused:
                return line_ax, line_ay, line_az, line_am, line_fft, peak_marker, stats_text

            with self.data_lock:
                if len(self.t_buf) < 16:
                    return line_ax, line_ay, line_az, line_am, line_fft, peak_marker, stats_text
                t = np.array(self.t_buf)
                ax = np.array(self.ax_buf)
                ay = np.array(self.ay_buf)
                az = np.array(self.az_buf)
                am = np.array(self.am_buf)

            # Update Time-Domain Curves
            line_ax.set_data(t, ax)
            line_ay.set_data(t, ay)
            line_az.set_data(t, az)
            line_am.set_data(t, am)

            # Auto-scroll time axis (trailing window)
            t_min = t[0]
            t_max = max(t[-1], t_min + 1.0)
            ax_time.set_xlim(t_min, t_max)

            # Auto-scale Y with headroom
            y_min = min(np.min(ax), np.min(ay), np.min(az), np.min(am))
            y_max = max(np.max(ax), np.max(ay), np.max(az), np.max(am))
            margin = max((y_max - y_min) * 0.15, 1.0)
            ax_time.set_ylim(y_min - margin, y_max + margin)

            # Compute Live FFT on Magnitude (AC component without DC)
            am_ac = am - np.mean(am)
            n = len(am_ac)
            fs = max(self.measured_fs, 10.0)

            # Windowing to prevent spectral leakage
            window = np.hanning(n)
            fft_vals = np.abs(np.fft.rfft(am_ac * window)) * (2.0 / n)
            freqs = np.fft.rfftfreq(n, d=1.0 / fs)

            line_fft.set_data(freqs, fft_vals)
            ax_freq.set_xlim(0, fs / 2.0)

            dom_freq = 0.0
            dom_amp = 0.0
            if len(freqs) > 2:
                # Exclude near-DC bin (first 2 bins)
                valid_idx = np.argmax(fft_vals[2:]) + 2
                dom_freq = freqs[valid_idx]
                dom_amp = fft_vals[valid_idx]
                peak_marker.set_data([dom_freq], [dom_amp])
                ax_freq.set_ylim(0, max(np.max(fft_vals) * 1.25, 0.5))

            # Statistics summary
            cur_ax, cur_ay, cur_az, cur_am = ax[-1], ay[-1], az[-1], am[-1]
            peak_val = np.max(np.abs(am_ac))
            rms_val = np.sqrt(np.mean(am_ac ** 2))

            stats_text.set_text(
                f"Sampling Fs : {fs:.1f} Hz\n"
                f"Instantaneous: X={cur_ax:+.2f} | Y={cur_ay:+.2f} | Z={cur_az:+.2f} | Mag={cur_am:.2f}\n"
                f"Dynamics (AC): Peak={peak_val:.2f} | RMS={rms_val:.2f}\n"
                f"Dominant Freq: {dom_freq:.1f} Hz (Amp={dom_amp:.2f})"
            )

            return line_ax, line_ay, line_az, line_am, line_fft, peak_marker, stats_text

        def on_key(event):
            if not event.key:
                return
            k = event.key.lower()
            if k == " ":
                self.paused = not self.paused
                print(f"[PLOT] {'PAUSED' if self.paused else 'RESUMED'}")
            elif k in ["1", "2", "3", "4", "5", "6", "t", "r", "u", "p", "s", "h"]:
                self.send_command(k)

        fig.canvas.mpl_connect("key_press_event", on_key)
        ani = animation.FuncAnimation(fig, update_frame, interval=40, blit=False)
        plt.show()

        # Teardown on window close
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()


def main():
    parser = argparse.ArgumentParser(description="Real-time Vibration Sensor Visualizer & FFT Spectrum")
    parser.add_argument("--port", type=str, default=None, help="Serial COM port (e.g. COM3 or /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--window", type=int, default=800, help="Rolling sample buffer size (default: 800)")
    args = parser.parse_args()

    port = args.port or auto_detect_port()
    if not port:
        print("[ERR] No serial port detected! Connect your ESP32 or specify --port COMx")
        sys.exit(1)

    app = VibrationVisualizer(port=port, baud=args.baud, buffer_size=args.window)
    app.start_gui()


if __name__ == "__main__":
    main()
