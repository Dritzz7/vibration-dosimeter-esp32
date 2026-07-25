/**
 * =============================================================================
 * Vibration Dosimeter Post-Processing & Analysis Utility (Modern C++17 Native)
 * Standards: ISO 5349-1 (HAV) | ISO 2631-1 (WBV) | ISO 8041
 * =============================================================================
 *
 * High-Performance Native CLI Tool for ISO Vibration Dosimetry.
 * Reads `dosimeter.csv` and computes A(8), VDV, eVDV, and safety limits.
 *
 * Compilation:
 *   g++ -O3 -std=c++17 util/calc_dosimeter.cpp -o util/calc_dosimeter.exe
 *
 * Usage:
 *   ./util/calc_dosimeter.exe [path_to_dosimeter.csv]
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <limits>
#include <chrono>

// =============================================================================
// CONSTANTS & SAFETY THRESHOLDS (ISO 5349-1, ISO 2631-1, EU Directive 2002/44/EC)
// =============================================================================
constexpr double T_8H_SEC = 28800.0; // 8 hours in seconds

// Seated Whole-Body Vibration multipliers (ISO 2631-1 section 6.1)
constexpr double KX_WBV = 1.4;
constexpr double KY_WBV = 1.4;
constexpr double KZ_WBV = 1.0;

// HAV Thresholds (ISO 5349-1)
constexpr double HAV_EAV_A8 = 2.50; // m/s²
constexpr double HAV_ELV_A8 = 5.00; // m/s²

// WBV Thresholds (ISO 2631-1)
constexpr double WBV_EAV_A8  = 0.50;  // m/s²
constexpr double WBV_ELV_A8  = 1.15;  // m/s²
constexpr double WBV_EAV_VDV = 9.10;  // m/s^1.75
constexpr double WBV_ELV_VDV = 21.00; // m/s^1.75

struct DosimeterRecord {
    std::string timestamp;
    double ahwx, ahwy, ahwz, ahv;
    double awx, awy, awz, av;
};

// Helper to format seconds into HHh MMm SSs
std::string formatDuration(double seconds) {
    if (std::isinf(seconds) || seconds > 86400.0 * 365.0) {
        return "N/A (> 1 year)";
    }
    long sec = static_cast<long>(seconds);
    long hrs = sec / 3600;
    long mins = (sec % 3600) / 60;
    long secs = sec % 60;

    std::ostringstream oss;
    oss << std::setfill('0') << std::setw(2) << hrs << "h "
        << std::setfill('0') << std::setw(2) << mins << "m "
        << std::setfill('0') << std::setw(2) << secs << "s";
    return oss.str();
}

int main(int argc, char* argv[]) {
    std::string filePath = "f:\\dosimeter.csv";
    if (argc > 1) {
        filePath = argv[1];
    }

    std::ifstream file(filePath);
    if (!file.is_open()) {
        filePath = "dosimeter.csv";
        file.open(filePath);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open dosimeter CSV file at " << argv[1] << " or ./dosimeter.csv\n";
            return 1;
        }
    }

    std::vector<DosimeterRecord> records;
    std::string line;
    bool isHeader = true;

    while (std::getline(file, line)) {
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string col;
        std::vector<std::string> cols;

        while (std::getline(ss, col, ',')) {
            // Trim whitespace
            col.erase(0, col.find_first_not_of(" \t\r\n"));
            col.erase(col.find_last_not_of(" \t\r\n") + 1);
            cols.push_back(col);
        }

        if (cols.empty()) continue;

        if (isHeader) {
            isHeader = false;
            if (cols[0].find("timestamp") != std::string::npos || cols[0].find("ahwx") != std::string::npos) {
                continue; // Skip header line
            }
        }

        if (cols.size() < 9) continue;

        try {
            DosimeterRecord rec;
            rec.timestamp = cols[0];
            rec.ahwx = std::stod(cols[1]);
            rec.ahwy = std::stod(cols[2]);
            rec.ahwz = std::stod(cols[3]);
            rec.ahv  = std::stod(cols[4]);
            rec.awx  = std::stod(cols[5]);
            rec.awy  = std::stod(cols[6]);
            rec.awz  = std::stod(cols[7]);
            rec.av   = std::stod(cols[8]);
            records.push_back(rec);
        } catch (...) {
            continue; // Ignore corrupt lines
        }
    }

    file.close();

    if (records.empty()) {
        std::cerr << "Error: No valid measurement data found in CSV file.\n";
        return 1;
    }

    const double N = static_cast<double>(records.size());
    const double dt = 1.0; // 1 Hz
    const double T_total = N * dt;

    // -------------------------------------------------------------------------
    // 1. HAV CALCULATION (ISO 5349-1)
    // -------------------------------------------------------------------------
    double sum_sq_ahwx = 0.0, sum_sq_ahwy = 0.0, sum_sq_ahwz = 0.0, sum_sq_ahv = 0.0;
    for (const auto& r : records) {
        sum_sq_ahwx += r.ahwx * r.ahwx;
        sum_sq_ahwy += r.ahwy * r.ahwy;
        sum_sq_ahwz += r.ahwz * r.ahwz;
        sum_sq_ahv  += r.ahv  * r.ahv;
    }

    double ahwx_eq = std::sqrt(sum_sq_ahwx / N);
    double ahwy_eq = std::sqrt(sum_sq_ahwy / N);
    double ahwz_eq = std::sqrt(sum_sq_ahwz / N);
    double ahv_eq  = std::sqrt(sum_sq_ahv  / N);

    double A8_hav = ahv_eq * std::sqrt(T_total / T_8H_SEC);
    double A8_hav_x = ahwx_eq * std::sqrt(T_total / T_8H_SEC);
    double A8_hav_y = ahwy_eq * std::sqrt(T_total / T_8H_SEC);
    double A8_hav_z = ahwz_eq * std::sqrt(T_total / T_8H_SEC);

    double t_eav_hav = (ahv_eq > 0) ? T_8H_SEC * std::pow(HAV_EAV_A8 / ahv_eq, 2.0) : std::numeric_limits<double>::infinity();
    double t_elv_hav = (ahv_eq > 0) ? T_8H_SEC * std::pow(HAV_ELV_A8 / ahv_eq, 2.0) : std::numeric_limits<double>::infinity();

    std::string hav_status = "OK (SAFE)";
    if (A8_hav >= HAV_ELV_A8) hav_status = "DANGER (EXCEEDS ELV 5.0 m/s²)";
    else if (A8_hav >= HAV_EAV_A8) hav_status = "WARNING (EXCEEDS EAV 2.5 m/s²)";

    // -------------------------------------------------------------------------
    // 2. WBV CALCULATION (ISO 2631-1)
    // -------------------------------------------------------------------------
    double sum_sq_awx = 0.0, sum_sq_awy = 0.0, sum_sq_awz = 0.0, sum_sq_av = 0.0;
    double sum_4th_awx = 0.0, sum_4th_awy = 0.0, sum_4th_awz = 0.0;

    for (const auto& r : records) {
        sum_sq_awx  += r.awx * r.awx;
        sum_sq_awy  += r.awy * r.awy;
        sum_sq_awz  += r.awz * r.awz;
        sum_sq_av   += r.av  * r.av;

        sum_4th_awx += std::pow(r.awx, 4.0);
        sum_4th_awy += std::pow(r.awy, 4.0);
        sum_4th_awz += std::pow(r.awz, 4.0);
    }

    double awx_eq = std::sqrt(sum_sq_awx / N);
    double awy_eq = std::sqrt(sum_sq_awy / N);
    double awz_eq = std::sqrt(sum_sq_awz / N);
    double av_eq  = std::sqrt(sum_sq_av  / N);

    double awx_w = KX_WBV * awx_eq;
    double awy_w = KY_WBV * awy_eq;
    double awz_w = KZ_WBV * awz_eq;

    double A8_wbv_x = awx_w * std::sqrt(T_total / T_8H_SEC);
    double A8_wbv_y = awy_w * std::sqrt(T_total / T_8H_SEC);
    double A8_wbv_z = awz_w * std::sqrt(T_total / T_8H_SEC);

    double A8_wbv_max = std::max({A8_wbv_x, A8_wbv_y, A8_wbv_z});
    std::string dom_axis = (A8_wbv_max == A8_wbv_x) ? "X" : ((A8_wbv_max == A8_wbv_y) ? "Y" : "Z");
    double max_w_rms = std::max({awx_w, awy_w, awz_w});

    double A8_wbv_vec = av_eq * std::sqrt(T_total / T_8H_SEC);

    double vdv_x = KX_WBV * std::pow(sum_4th_awx * dt, 0.25);
    double vdv_y = KY_WBV * std::pow(sum_4th_awy * dt, 0.25);
    double vdv_z = KZ_WBV * std::pow(sum_4th_awz * dt, 0.25);

    double vdv_total = std::pow(std::pow(vdv_x, 4.0) + std::pow(vdv_y, 4.0) + std::pow(vdv_z, 4.0), 0.25);
    double evdv = 1.4 * av_eq * std::pow(T_total, 0.25);

    double t_eav_wbv = (max_w_rms > 0) ? T_8H_SEC * std::pow(WBV_EAV_A8 / max_w_rms, 2.0) : std::numeric_limits<double>::infinity();
    double t_elv_wbv = (max_w_rms > 0) ? T_8H_SEC * std::pow(WBV_ELV_A8 / max_w_rms, 2.0) : std::numeric_limits<double>::infinity();

    std::string wbv_status = "OK (SAFE)";
    if (A8_wbv_max >= WBV_ELV_A8 || vdv_total >= WBV_ELV_VDV) wbv_status = "DANGER (EXCEEDS ELV 1.15 m/s² / 21 VDV)";
    else if (A8_wbv_max >= WBV_EAV_A8 || vdv_total >= WBV_EAV_VDV) wbv_status = "WARNING (EXCEEDS EAV 0.50 m/s² / 9.1 VDV)";

    // -------------------------------------------------------------------------
    // 3. PRINT REPORT
    // -------------------------------------------------------------------------
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "===============================================================================\n";
    std::cout << "          VIBRATION DOSIMETER ANALYSIS REPORT (Native C++17 Engine)\n";
    std::cout << "               Standards: ISO 5349-1 (HAV) | ISO 2631-1 (WBV)\n";
    std::cout << "===============================================================================\n";
    std::cout << "Input Log File   : " << filePath << "\n";
    std::cout << "Total Logged Time: " << formatDuration(T_total) << " (" << static_cast<long>(N) << " seconds / " << static_cast<long>(N) << " samples @ 1 Hz)\n";
    std::cout << "-------------------------------------------------------------------------------\n\n";

    std::cout << "1. HAND-ARM VIBRATION (HAV) ANALYSIS (ISO 5349-1)\n";
    std::cout << "-------------------------------------------------------------------------------\n";
    std::cout << "  * RMS Acceleration (Equiv):\n";
    std::cout << "      ahwx_eq = " << ahwx_eq << " m/s² | ahwy_eq = " << ahwy_eq << " m/s² | ahwz_eq = " << ahwz_eq << " m/s²\n";
    std::cout << "      ahv_eq  (Vector Total) = " << ahv_eq << " m/s²\n";
    std::cout << "  * 8-Hour Daily Exposure A_HAV(8):\n";
    std::cout << "      A_HAV(8) Vector Total  = " << A8_hav << " m/s²\n";
    std::cout << "      [Per-axis: Ax(8)=" << A8_hav_x << ", Ay(8)=" << A8_hav_y << ", Az(8)=" << A8_hav_z << "]\n";
    std::cout << "  * ISO 5349-1 Safety Limits:\n";
    std::cout << "      EAV Threshold (Action) = 2.50 m/s² | Time allowed = " << formatDuration(t_eav_hav) << "\n";
    std::cout << "      ELV Threshold (Limit)  = 5.00 m/s² | Time allowed = " << formatDuration(t_elv_hav) << "\n";
    std::cout << "  * Exposure Compliance Status: " << hav_status << "\n\n";

    std::cout << "2. WHOLE-BODY VIBRATION (WBV) ANALYSIS (ISO 2631-1)\n";
    std::cout << "-------------------------------------------------------------------------------\n";
    std::cout << "  * RMS Acceleration (Unweighted Equiv):\n";
    std::cout << "      awx_eq = " << awx_eq << " m/s² | awy_eq = " << awy_eq << " m/s² | awz_eq = " << awz_eq << " m/s²\n";
    std::cout << "  * Directional Weighted RMS (kx=1.4, ky=1.4, kz=1.0 for seated):\n";
    std::cout << "      1.4*awx = " << awx_w << " m/s² | 1.4*awy = " << awy_w << " m/s² | 1.0*awz = " << awz_w << " m/s²\n";
    std::cout << "  * 8-Hour Daily Exposure A_WBV(8):\n";
    std::cout << "      A_x(8) = " << A8_wbv_x << " m/s² | A_y(8) = " << A8_wbv_y << " m/s² | A_z(8) = " << A8_wbv_z << " m/s²\n";
    std::cout << "      Dominant Axis A_WBV(8) = " << A8_wbv_max << " m/s² (Axis " << dom_axis << ")\n";
    std::cout << "      Vector Total A_v(8)   = " << A8_wbv_vec << " m/s²\n";
    std::cout << "  * Vibration Dose Value (VDV) 4th-Power Method:\n";
    std::cout << "      VDV_x = " << vdv_x << " m/s^1.75 | VDV_y = " << vdv_y << " m/s^1.75 | VDV_z = " << vdv_z << " m/s^1.75\n";
    std::cout << "      VDV Total Combined    = " << vdv_total << " m/s^1.75\n";
    std::cout << "      Estimated VDV (eVDV)  = " << evdv << " m/s^1.75\n";
    std::cout << "  * ISO 2631-1 Safety Limits:\n";
    std::cout << "      A(8) EAV = 0.50 m/s²  | Time allowed = " << formatDuration(t_eav_wbv) << "\n";
    std::cout << "      A(8) ELV = 1.15 m/s²  | Time allowed = " << formatDuration(t_elv_wbv) << "\n";
    std::cout << "      VDV  EAV = 9.1 m/s^1.75 | VDV ELV = 21.0 m/s^1.75\n";
    std::cout << "  * Exposure Compliance Status: " << wbv_status << "\n";
    std::cout << "===============================================================================\n";

    return 0;
}
