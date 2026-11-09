// metrics.cpp — Implementasi metrik respons step (lihat metrics.h).
#include "metrics.h"

#include <algorithm>
#include <cmath>

namespace {

// Waktu saat sinyal pertama kali melewati `level` (interpolasi linear
// antar sampel). Mengembalikan -1 bila tidak pernah tercapai.
double crossing_time(const std::vector<Sample>& s, double level, bool rising) {
    for (size_t k = 1; k < s.size(); ++k) {
        const double y0 = s[k - 1].speed;
        const double y1 = s[k].speed;
        const bool crossed = rising ? (y0 < level && y1 >= level)
                                    : (y0 > level && y1 <= level);
        if (crossed) {
            const double frac = (level - y0) / (y1 - y0);
            return s[k - 1].t + frac * (s[k].t - s[k - 1].t);
        }
    }
    return -1.0;
}

}  // namespace

StepMetrics compute_metrics(const std::vector<Sample>& samples, double setpoint,
                            double vmax) {
    StepMetrics m;
    if (samples.empty() || setpoint == 0.0) return m;

    const double sign = (setpoint > 0.0) ? 1.0 : -1.0;
    const double target = std::fabs(setpoint);

    // Nilai akhir = rata-rata 10% sampel terakhir.
    const size_t n_tail = std::max<size_t>(1, samples.size() / 10);
    double tail_sum = 0.0;
    for (size_t k = samples.size() - n_tail; k < samples.size(); ++k) {
        tail_sum += samples[k].speed;
    }
    m.final_value = tail_sum / static_cast<double>(n_tail);
    m.steady_state_error = setpoint - m.final_value;

    // Puncak ke arah setpoint.
    m.peak_value = samples.front().speed;
    m.peak_time = samples.front().t;
    for (const auto& s : samples) {
        if (sign * s.speed > sign * m.peak_value) {
            m.peak_value = s.speed;
            m.peak_time = s.t;
        }
    }
    const double excess = sign * (m.peak_value - setpoint);
    m.overshoot_pct = (excess > 0.0) ? 100.0 * excess / target : 0.0;

    // Rise time 10% -> 90% (level ditandatangani mengikuti setpoint).
    const double t10 = crossing_time(samples, sign * 0.10 * target, setpoint > 0.0);
    const double t90 = crossing_time(samples, sign * 0.90 * target, setpoint > 0.0);
    if (t10 >= 0.0 && t90 >= 0.0) m.rise_time = t90 - t10;

    // Settling time: waktu terakhir keluar dari pita 2%.
    const double band = 0.02 * target;
    double last_out = -1.0;
    for (const auto& s : samples) {
        if (std::fabs(s.speed - setpoint) > band) last_out = s.t;
    }
    m.settling_time = last_out;

    // IAE + tegangan maksimum + fraksi saturasi.
    double iae = 0.0;
    size_t n_sat = 0;
    for (size_t k = 1; k < samples.size(); ++k) {
        const double e0 = std::fabs(setpoint - samples[k - 1].speed);
        const double e1 = std::fabs(setpoint - samples[k].speed);
        iae += 0.5 * (e0 + e1) * (samples[k].t - samples[k - 1].t);
        m.max_voltage = std::max(m.max_voltage, std::fabs(samples[k].voltage));
        if (vmax > 0.0 && std::fabs(samples[k].voltage) >= vmax * 0.999) ++n_sat;
    }
    m.iae = iae;
    if (vmax > 0.0) m.saturated_fraction = static_cast<double>(n_sat) / samples.size();
    return m;
}
