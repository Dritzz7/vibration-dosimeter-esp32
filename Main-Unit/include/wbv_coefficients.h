// wbv_coefficients.h
// Auto-generated from MATLAB (export_all_coefficients.m)
// Standard: ISO 2631-1 / ISO 8041 - Whole-Body Vibration (WBV) Frequency Weighting
// Sampling rate: 400 Hz
// Filter topology: IIR Cascaded Biquad (Wd: 3 sections, Wk: 4 sections)
// Format per section: {b0, b1, b2, a1, a2}
// Equation: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]

#ifndef WBV_COEFFICIENTS_H
#define WBV_COEFFICIENTS_H

// WBV Sampling Frequency
static constexpr float FS_WBV = 400.0f;

// Wd filter for X and Y axes: 3 biquad sections
// Section 1: High-pass band-limiting (f1=0.4 Hz, Q=1/√2)
// Section 2: Low-pass band-limiting (f2=100 Hz, Q=1/√2)
// Section 3: A-V transition Wd (f3=f4=2 Hz, Q4=0.63)
static constexpr int NUM_SECTIONS_WD = 3;
static constexpr float coeff_wd[NUM_SECTIONS_WD][5] = {
  {9.955669865693e-01f, -1.991133973139e+00f, 9.955669865693e-01f, -1.991114321434e+00f, 9.911536248432e-01f},
  {2.261536997186e-01f,  4.523073994373e-01f, 2.261536997186e-01f, -2.809457378615e-01f, 1.855605367361e-01f},
  {1.556283105638e-02f,  4.813595785786e-04f,-1.508147147780e-02f, -1.950395530789e+00f, 9.513582499457e-01f}
};

// Wk filter for Z axis: 4 biquad sections
// Section 1: High-pass band-limiting (f1=0.4 Hz, Q=1/√2)
// Section 2: Low-pass band-limiting (f2=100 Hz, Q=1/√2)
// Section 3: A-V transition Wk (f3=f4=12.5 Hz, Q4=0.63)
// Section 4: Upward-step Wk (f5=2.37 Hz, f6=3.35 Hz, Q5=Q6=0.91)
static constexpr int NUM_SECTIONS_WK = 4;
static constexpr float coeff_wk[NUM_SECTIONS_WK][5] = {
  {9.955669865693e-01f, -1.991133973139e+00f, 9.955669865693e-01f, -1.991114321434e+00f, 9.911536248432e-01f},
  {2.261536997186e-01f,  4.523073994373e-01f, 2.261536997186e-01f, -2.809457378615e-01f, 1.855605367361e-01f},
  {9.250597606639e-02f,  1.653972247007e-02f,-7.596625359632e-02f, -1.699504317500e+00f, 7.325837624402e-01f},
  {9.914492321300e-01f, -1.941818968783e+00f, 9.517158005911e-01f, -1.941147290810e+00f, 9.438367106940e-01f}
};

#endif // WBV_COEFFICIENTS_H
