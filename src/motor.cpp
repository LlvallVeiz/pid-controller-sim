// motor.cpp — Implementasi model motor DC (lihat motor.h).
#include "motor.h"

double motor_steady_state_speed(const MotorParams& p, double voltage) {
    return p.Kt * voltage / (p.R * p.B + p.Kt * p.Ke);
}

double motor_dc_gain(const MotorParams& p) {
    return p.Kt / (p.R * p.B + p.Kt * p.Ke);
}

DcMotor::DcMotor(const MotorParams& params) : p_(params) {}

void DcMotor::reset() {
    i_ = 0.0;
    w_ = 0.0;
}

void DcMotor::derivatives(double i, double w, double v, double& di, double& dw) const {
    di = (v - p_.R * i - p_.Ke * w) / p_.L;
    dw = (p_.Kt * i - p_.B * w) / p_.J;
}

void DcMotor::step(double voltage, double dt) {
    // Integrator Runge-Kutta orde 4 (klasik). Tegangan dianggap konstan
    // selama satu langkah (zero-order hold), sesuai keluaran kontroler
    // digital yang diperbarui tiap dt.
    double k1i, k1w, k2i, k2w, k3i, k3w, k4i, k4w;
    derivatives(i_, w_, voltage, k1i, k1w);
    derivatives(i_ + 0.5 * dt * k1i, w_ + 0.5 * dt * k1w, voltage, k2i, k2w);
    derivatives(i_ + 0.5 * dt * k2i, w_ + 0.5 * dt * k2w, voltage, k3i, k3w);
    derivatives(i_ + dt * k3i, w_ + dt * k3w, voltage, k4i, k4w);
    i_ += (dt / 6.0) * (k1i + 2.0 * k2i + 2.0 * k3i + k4i);
    w_ += (dt / 6.0) * (k1w + 2.0 * k2w + 2.0 * k3w + k4w);
}
