// hav_coefficients.h
// Auto-generated from MATLAB (export_all_coefficients.m)
// Standard: ISO 5349-1 / ISO 8041 - Hand-Arm Vibration (HAV) Wh Frequency Weighting
// Sampling rate: 3200 Hz
// Filter topology: 8th-order IIR (3 cascaded biquad sections)
// Format per section: {b0, b1, b2, a1, a2}
// Equation: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]

#ifndef HAV_COEFFICIENTS_H
#define HAV_COEFFICIENTS_H

// HAV Sampling Frequency
static constexpr float FS_HAV = 3200.0f;

// Wh filter: 3 biquad sections
// Section 1: High-pass band-limiting (f1=6.31 Hz, Q=0.71)
// Section 2: Low-pass band-limiting (f2=1258.9 Hz, Q=0.71)
// Section 3: Frequency-weighting Wh (f3=f4=15.915 Hz, Q4=0.64)
static constexpr int NUM_SECTIONS_WH = 3;
static constexpr float coeff_wh[NUM_SECTIONS_WH][5] = {
  {9.913126457328e-01f, -1.982625291466e+00f, 9.913126457328e-01f, -1.982549206447e+00f, 9.827013764839e-01f},
  {3.578767741404e-01f,  7.157535482807e-01f, 3.578767741404e-01f,  2.471762393794e-01f, 1.843308571820e-01f},
  {1.548677608063e-02f,  4.765016149668e-04f,-1.501027446567e-02f, -1.951395355291e+00f, 9.523483585214e-01f}
};

#endif // HAV_COEFFICIENTS_H
