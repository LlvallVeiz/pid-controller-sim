// tuning.cpp — Implementasi identifikasi kurva reaksi + Z-N (lihat tuning.h).
#include "tuning.h"

#include <cmath>
#include <vector>

ReactionCurve identify_reaction_curve(const MotorParams& params, double step_voltage,
                                      double duration, double dt) {
    ReactionCurve rc;
    if (step_voltage == 0.0 || duration <= 0.0 || dt <= 0.0) return rc;

    DcMotor motor(params);
    std::vector<double> t;
    std::vector<double> w;
    const size_t n = static_cast<size_t>(duration / dt) + 1;
    t.reserve(n);
    w.reserve(n);
    for (size_t k = 0; k < n; ++k) {
        const double time = k * dt;
        if (k > 0) motor.step(step_voltage, dt);
        t.push_back(time);
        w.push_back(motor.speed());
    }

    const double w_ss = motor_steady_state_speed(params, step_voltage);
    rc.K = w_ss / step_voltage;

    // Titik belok = kemiringan maksimum. Kemiringan dihitung dengan beda
    // pusat pada sampel kecepatan (data simulasi tanpa derau, jadi beda
    // pusat cukup; tidak ada pemulusan yang bisa menggeser titik belok).
    double max_slope = 0.0;
    size_t k_inflect = 0;
    for (size_t k = 1; k + 1 < w.size(); ++k) {
        const double slope = (w[k + 1] - w[k - 1]) / (2.0 * dt);
        if (slope > max_slope) {
            max_slope = slope;
            k_inflect = k;
        }
    }
    if (max_slope <= 0.0) return rc;

    rc.t_inflect = t[k_inflect];
    const double w_inflect = w[k_inflect];
    // Garis singgung di titik belok: w = w_inflect + m*(t - t_inflect).
    // Memotong w = 0 pada t = L; mencapai w_ss pada t = L + T.
    rc.L = rc.t_inflect - w_inflect / max_slope;
    rc.T = w_ss / max_slope;
    rc.ok = (rc.L > 0.0 && rc.T > 0.0 && rc.K != 0.0);
    return rc;
}

PidGains ziegler_nichols_pid(const ReactionCurve& rc) {
    PidGains g;
    if (!rc.ok) return g;
    const double ti = 2.0 * rc.L;
    const double td = 0.5 * rc.L;
    g.kp = 1.2 * rc.T / (rc.K * rc.L);
    g.ki = g.kp / ti;
    g.kd = g.kp * td;
    return g;
}

PidGains ziegler_nichols_pi(const ReactionCurve& rc) {
    PidGains g;
    if (!rc.ok) return g;
    const double ti = rc.L / 0.3;
    g.kp = 0.9 * rc.T / (rc.K * rc.L);
    g.ki = g.kp / ti;
    g.kd = 0.0;
    return g;
}
