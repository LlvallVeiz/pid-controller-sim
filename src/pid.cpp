// pid.cpp — Implementasi kontroler PID (lihat pid.h).
#include "pid.h"

#include <cmath>

PidController::PidController(PidGains gains, double vmax, bool derivative_on_measurement)
    : gains_(gains), vmax_(vmax), derivative_on_measurement_(derivative_on_measurement) {}

void PidController::reset() {
    integral_ = 0.0;
    prev_measurement_ = 0.0;
    prev_error_ = 0.0;
    first_ = true;
    saturated_ = false;
}

double PidController::update(double setpoint, double measurement, double dt) {
    const double error = setpoint - measurement;

    double derivative = 0.0;
    if (!first_) {
        if (derivative_on_measurement_) {
            // d(e)/dt = -d(y)/dt saat setpoint konstan; bentuk ini identik
            // dengan turunan pada error untuk setpoint tetap, tapi tidak
            // melompat saat setpoint berubah mendadak.
            derivative = -(measurement - prev_measurement_) / dt;
        } else {
            derivative = (error - prev_error_) / dt;
        }
    }
    prev_measurement_ = measurement;
    prev_error_ = error;
    first_ = false;

    const double p_term = gains_.kp * error;
    const double d_term = gains_.kd * derivative;

    // Anti-windup: integrasi kondisional. Integrator hanya diakumulasi
    // bila (a) keluaran belum jenuh, atau (b) error berlawanan arah
    // dengan kejenuhan sehingga membantu keluar dari saturasi.
    double u_unsat = p_term + gains_.ki * integral_ + d_term;
    bool would_saturate = (vmax_ > 0.0) && (std::fabs(u_unsat) > vmax_);
    bool error_pushes_deeper =
        (u_unsat > 0.0 && error > 0.0) || (u_unsat < 0.0 && error < 0.0);
    if (!would_saturate || !error_pushes_deeper) {
        integral_ += error * dt;
    }

    double u = p_term + gains_.ki * integral_ + d_term;
    saturated_ = false;
    if (vmax_ > 0.0 && std::fabs(u) > vmax_) {
        u = (u > 0.0) ? vmax_ : -vmax_;
        saturated_ = true;
    }
    return u;
}
