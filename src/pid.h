// pid.h — Kontroler PID bentuk paralel (ISA standard form):
//   u = Kp*e + Ki*int(e) + Kd*d(e)/dt
// dengan dua fitur praktis yang penting untuk plant nyata:
//   * Turunan pada pengukuran (derivative on measurement) sebagai default,
//     supaya perubahan setpoint mendadak tidak menimbulkan "derivative kick".
//   * Anti-windup conditional integration: integrator dibekukan saat
//     keluaran jenuh DAN error mendorong lebih jauh ke arah jenuh.
#pragma once

struct PidGains {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
};

class PidController {
public:
    // vmax: batas saturasi aktuator |u| <= vmax (volt). Gunakan nilai
    // <= 0 untuk menonaktifkan saturasi.
    // derivative_on_measurement: true = turunan dari -d(y)/dt (default,
    // tanpa kick); false = turunan dari d(e)/dt (bentuk buku teks).
    PidController(PidGains gains, double vmax, bool derivative_on_measurement = true);

    void reset();

    // Satu langkah kontrol. `setpoint` dan `measurement` dalam rad/s,
    // `dt` dalam detik. Mengembalikan tegangan kontrol (sudah dijenuhkan).
    double update(double setpoint, double measurement, double dt);

    double integral() const { return integral_; }
    bool saturated() const { return saturated_; }
    PidGains gains() const { return gains_; }

private:
    PidGains gains_;
    double vmax_;
    bool derivative_on_measurement_;
    double integral_ = 0.0;
    double prev_measurement_ = 0.0;  // untuk turunan pada pengukuran
    double prev_error_ = 0.0;        // untuk turunan pada error
    bool first_ = true;
    bool saturated_ = false;
};
