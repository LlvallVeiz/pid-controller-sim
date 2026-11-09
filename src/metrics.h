// metrics.h — Metrik respons step dari deret waktu hasil simulasi.
//
// Definisi (standar buku teks kontrol, mis. Nise / Ogata):
//   * Overshoot %  : (puncak - setpoint) / |setpoint| * 100, diukur ke
//                    arah setpoint. 0 bila respons tidak pernah melampaui.
//   * Rise time    : waktu dari 10% ke 90% setpoint (interpolasi linear).
//   * Settling time: waktu terakhir |y - setpoint| keluar dari pita 2%.
//   * Steady-state error: setpoint - rata-rata y pada 10% akhir durasi.
//   * IAE          : integral |error| dt (trapezoid atas sampel).
#pragma once

#include <vector>

struct Sample {
    double t;       // waktu (s)
    double speed;   // kecepatan sudut (rad/s)
    double current; // arus jangkar (A)
    double voltage; // tegangan kontrol (V)
};

struct StepMetrics {
    double overshoot_pct = 0.0;  // %
    double rise_time = -1.0;     // s, -1 bila tidak pernah mencapai 90%
    double settling_time = -1.0; // s, -1 bila tidak pernah menetap
    double steady_state_error = 0.0;  // rad/s
    double final_value = 0.0;         // rad/s (rata-rata 10% akhir)
    double peak_value = 0.0;          // rad/s
    double peak_time = 0.0;           // s
    double iae = 0.0;                 // rad (integral |error| dt)
    double max_voltage = 0.0;         // |V| maksimum (V)
    double saturated_fraction = 0.0;  // fraksi sampel saat |V| == vmax
};

// Hitung metrik dari deret sampel terhadap `setpoint`.
// `vmax` dipakai hanya untuk fraksi saturasi (<= 0 = tidak dihitung).
StepMetrics compute_metrics(const std::vector<Sample>& samples, double setpoint,
                            double vmax);
