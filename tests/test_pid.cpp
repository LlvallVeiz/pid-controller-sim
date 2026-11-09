// test_pid.cpp — Tes unit pid-controller-sim (tanpa framework eksternal).
//
// Konvensi: tiap tes mengembalikan true bila lolos; main() menghitung
// yang lolos dan mengembalikan 0 hanya bila semuanya lolos.
//
// Nilai harapan dihitung dari teori, bukan dari output program:
//   * Kecepatan tunak open-loop 1 V: w_ss = Kt/(R*B + Kt*Ke)
//     = 0.01 / (1*0.1 + 0.01*0.01) = 0.01/0.1001 = 0.099900... rad/s
//     (nilai yang sama dikutip CTMS: 1 V -> 0.1 rad/s).
//   * Arus saat start (w = 0, V = 1): i = V/R = 1 A.
//   * P-only Kp = 100: gain loop DC = Kp*K = 100*0.0999 = 9.99, jadi
//     w_ss = 9.99/10.99 = 0.9090 rad/s -> ess = 0.0910 rad/s (9.1%).
//   * Metrik: deret sintetis dengan nilai yang dihitung tangan di bawah.
#include <cmath>
#include <cstdio>
#include <vector>

#include "../src/metrics.h"
#include "../src/motor.h"
#include "../src/pid.h"
#include "../src/tuning.h"

namespace {

int g_passed = 0;
int g_failed = 0;

void check(bool ok, const char* name) {
    if (ok) {
        ++g_passed;
        std::printf("  PASS  %s\n", name);
    } else {
        ++g_failed;
        std::printf("  FAIL  %s\n", name);
    }
}

bool near(double got, double want, double tol) {
    return std::fabs(got - want) <= tol;
}

// Simulasi loop tertutup; mengembalikan sampel (duplikasi kecil dari
// main.cpp agar tes tidak bergantung pada CLI).
std::vector<Sample> simulate(const PidGains& gains, double setpoint, double duration,
                             double dt, double vmax, bool deriv_on_meas = true) {
    DcMotor motor;
    PidController pid(gains, vmax, deriv_on_meas);
    std::vector<Sample> out;
    const size_t n = static_cast<size_t>(duration / dt) + 1;
    double u = 0.0;
    for (size_t k = 0; k < n; ++k) {
        const double t = k * dt;
        if (k > 0) motor.step(u, dt);
        u = pid.update(setpoint, motor.speed(), dt);
        out.push_back(Sample{t, motor.speed(), motor.current(), u});
    }
    return out;
}

void test_motor_steady_state() {
    DcMotor motor;
    const double dt = 0.001;
    for (double t = 0.0; t < 10.0; t += dt) motor.step(1.0, dt);
    // Harapan teori: 0.01/0.1001 = 0.0999001 rad/s.
    check(near(motor.speed(), 0.0999001, 0.0005), "motor: w_ss 1 V = 0.0999 rad/s");
}

void test_motor_initial_current() {
    DcMotor motor;
    motor.step(1.0, 0.001);
    // Arus naik dengan konstanta waktu listrik L/R = 0.5 s, jadi setelah
    // 1 ms arusnya baru ~ (V/R)*(dt/tau_e) = 1*0.001/0.5 = 0.002 A
    // (back-EMF masih ~0 karena w ~ 0).
    check(near(motor.current(), 0.002, 0.0002), "motor: arus setelah 1 ms ~ 0.002 A");
    // Arus tunak pada 1 V: i_ss = (V - Ke*w_ss)/R = 1 - 0.01*0.0999.
    DcMotor motor2;
    const double dt = 0.001;
    for (double t = 0.0; t < 10.0; t += dt) motor2.step(1.0, dt);
    check(near(motor2.current(), 0.9990, 0.002), "motor: arus tunak 1 V ~ 0.999 A");
}

void test_motor_negative_voltage() {
    DcMotor motor;
    const double dt = 0.001;
    for (double t = 0.0; t < 10.0; t += dt) motor.step(-1.0, dt);
    check(near(motor.speed(), -0.0999001, 0.0005), "motor: simetri tegangan negatif");
}

void test_motor_dc_gain() {
    const MotorParams p;
    check(near(motor_dc_gain(p), 0.0999001, 1e-6), "motor: gain DC = 0.0999 rad/s/V");
    check(near(motor_steady_state_speed(p, 5.0), 0.4995005, 1e-6),
          "motor: w_ss 5 V = 0.4995 rad/s");
}

void test_pid_zero_gains() {
    const auto s = simulate(PidGains{0.0, 0.0, 0.0}, 1.0, 2.0, 0.001, 12.0);
    bool all_zero = true;
    for (const auto& x : s) {
        if (x.speed != 0.0 || x.voltage != 0.0) all_zero = false;
    }
    check(all_zero, "pid: gain nol -> motor diam, u = 0");
}

void test_pid_p_only_steady_state_error() {
    // P-only Kp=100: ess teori = 1/(1+Kp*K) = 1/10.99 = 0.0910 rad/s.
    const auto s = simulate(PidGains{100.0, 0.0, 0.0}, 1.0, 4.0, 0.001, 12.0);
    const StepMetrics m = compute_metrics(s, 1.0, 12.0);
    check(near(m.steady_state_error, 0.0910, 0.005),
          "pid: P-only Kp=100 ess ~ 0.091 rad/s (9.1%)");
    check(m.overshoot_pct < 5.0, "pid: P-only overshoot kecil (< 5%)");
}

void test_pid_integral_removes_error() {
    // Preset default CTMS: Kp=100, Ki=200, Kd=10 -> ess < 1% (CTMS).
    const auto s = simulate(PidGains{100.0, 200.0, 10.0}, 1.0, 4.0, 0.001, 12.0);
    const StepMetrics m = compute_metrics(s, 1.0, 12.0);
    check(std::fabs(m.steady_state_error) < 0.01,
          "pid: PID default ess < 1% (integral menghilangkan error)");
    check(m.overshoot_pct < 10.0, "pid: PID default overshoot < 10%");
    check(m.settling_time > 0.0 && m.settling_time < 2.0,
          "pid: PID default settling < 2 s (kriteria CTMS)");
    check(m.rise_time > 0.0 && m.rise_time < 1.0, "pid: PID default rise time < 1 s");
}

void test_pid_pi_more_overshoot_than_p() {
    // Tanpa saturasi (vmax = 0) perilaku buku teks muncul murni: suku I
    // menghilangkan ess tapi menaikkan overshoot; suku D mempercepat
    // settling dan menurunkan IAE. (Dengan saturasi +/-12 V kedua
    // varian sama-sama terjepit di 0% overshoot, jadi perbandingan
    // dilakukan tanpa saturasi.)
    const auto s_p = simulate(PidGains{100.0, 0.0, 0.0}, 1.0, 4.0, 0.001, 0.0);
    const auto s_pi = simulate(PidGains{100.0, 200.0, 0.0}, 1.0, 4.0, 0.001, 0.0);
    const auto s_pid = simulate(PidGains{100.0, 200.0, 10.0}, 1.0, 4.0, 0.001, 0.0);
    const StepMetrics m_p = compute_metrics(s_p, 1.0, 0.0);
    const StepMetrics m_pi = compute_metrics(s_pi, 1.0, 0.0);
    const StepMetrics m_pid = compute_metrics(s_pid, 1.0, 0.0);
    check(std::fabs(m_pi.steady_state_error) < 0.01,
          "pid: PI ess -> 0 (integral bekerja)");
    check(m_pi.overshoot_pct > m_p.overshoot_pct,
          "pid: PI overshoot > P-only (integral menambah overshoot)");
    check(m_pid.overshoot_pct < m_pi.overshoot_pct,
          "pid: PID overshoot < PI (derivatif meredam overshoot)");
    // Dengan saturasi +/-12 V, derivatif juga mempercepat settling:
    // PID 1.243 s vs PI 1.601 s (nilai dari run; lihat examples/).
    const auto s_pi_sat = simulate(PidGains{100.0, 200.0, 0.0}, 1.0, 4.0, 0.001, 12.0);
    const auto s_pid_sat =
        simulate(PidGains{100.0, 200.0, 10.0}, 1.0, 4.0, 0.001, 12.0);
    const StepMetrics m_pi_sat = compute_metrics(s_pi_sat, 1.0, 12.0);
    const StepMetrics m_pid_sat = compute_metrics(s_pid_sat, 1.0, 12.0);
    check(m_pid_sat.settling_time < m_pi_sat.settling_time,
          "pid: PID settling lebih cepat dari PI saat tersaturasi");
    check(m_pid_sat.iae < m_pi_sat.iae, "pid: PID IAE lebih kecil dari PI saat tersaturasi");
}

void test_pid_negative_setpoint() {
    const auto s = simulate(PidGains{100.0, 200.0, 10.0}, -1.0, 4.0, 0.001, 12.0);
    const StepMetrics m = compute_metrics(s, -1.0, 12.0);
    check(std::fabs(m.steady_state_error) < 0.01, "pid: setpoint -1 rad/s tercapai");
    check(m.overshoot_pct < 10.0, "pid: setpoint negatif overshoot < 10%");
}

void test_pid_saturation_and_antiwindup() {
    // Setpoint 5 rad/s butuh ~50 V tunak; aktuator mentok 12 V -> tidak
    // tercapai, tapi integrator tidak boleh meledak (anti-windup).
    DcMotor motor;
    PidController pid(PidGains{100.0, 200.0, 10.0}, 12.0, true);
    double u = 0.0, max_u = 0.0;
    const double dt = 0.001;
    for (size_t k = 0; k <= 4000; ++k) {
        if (k > 0) motor.step(u, dt);
        u = pid.update(5.0, motor.speed(), dt);
        max_u = std::max(max_u, std::fabs(u));
    }
    check(near(max_u, 12.0, 1e-9), "pid: tegangan dijenuhkan tepat di 12 V");
    // Tanpa anti-windup, integral akan membesar tanpa batas selama jenuh;
    // dengan conditional integration nilainya tetap dalam orde wajar.
    check(std::fabs(pid.integral()) < 5.0,
          "pid: anti-windup menjaga integral tetap terbatas saat jenuh");
    check(motor.speed() < 5.0 && motor.speed() > 0.5,
          "pid: saat jenuh motor bergerak tapi setpoint tak tercapai");
}

void test_metrics_synthetic() {
    // Deret sintetis hitung-tangan: naik linear 0 -> 1.2 dalam 0.6 s,
    // lalu turun linear ke 1.0 dalam 0.1 s, lalu menetap di 1.0.
    // Setpoint 1.0. Puncak 1.2 -> overshoot 20%. Level 10% (0.1) dicapai
    // pada t = 0.05 s, level 90% (0.9) pada t = 0.45 s -> rise time 0.40 s.
    // Terakhir keluar pita 2% saat turun ke 1.02: (1.2-1.02)/2 = 0.09 s
    // setelah t = 0.6 -> settling 0.69 s.
    std::vector<Sample> s;
    const double dt = 0.01;
    for (int k = 0; k <= 100; ++k) {
        const double t = k * dt;
        double w;
        if (t <= 0.6) {
            w = 2.0 * t;  // 0 -> 1.2
        } else if (t <= 0.7) {
            w = 1.2 - 2.0 * (t - 0.6);  // 1.2 -> 1.0
        } else {
            w = 1.0;
        }
        s.push_back(Sample{t, w, 0.0, 0.0});
    }
    const StepMetrics m = compute_metrics(s, 1.0, 0.0);
    check(near(m.overshoot_pct, 20.0, 0.01), "metrics: overshoot sintetis 20%");
    check(near(m.rise_time, 0.40, 0.02), "metrics: rise time sintetis 0.40 s");
    // Pita 2%: |w-1| <= 0.02 -> w <= 1.02; turun dari 1.2 ke 1.02 dalam
    // (1.2-1.02)/2 = 0.09 s setelah t=0.6 -> settling 0.69 s.
    check(near(m.settling_time, 0.69, 0.02), "metrics: settling sintetis 0.69 s");
    check(near(m.steady_state_error, 0.0, 1e-9), "metrics: ess sintetis 0");
    check(near(m.final_value, 1.0, 1e-9), "metrics: nilai akhir sintetis 1.0");
}

void test_zn_identification() {
    const MotorParams p;
    const ReactionCurve rc = identify_reaction_curve(p, 5.0, 5.0, 0.001);
    check(rc.ok, "zn: identifikasi kurva reaksi berhasil");
    check(near(rc.K, 0.0999001, 1e-4), "zn: K teridentifikasi = gain DC");
    check(rc.L > 0.0 && rc.L < 0.5, "zn: L semu dalam rentang wajar (0-0.5 s)");
    check(rc.T > 0.0 && rc.T < 2.0, "zn: T semu dalam rentang wajar (0-2 s)");
    const PidGains g = ziegler_nichols_pid(rc);
    check(g.kp > 0.0 && g.ki > 0.0 && g.kd > 0.0, "zn: gain Z-N positif");
    // Loop tertutup dengan gain Z-N harus stabil dan mencapai setpoint.
    const auto s = simulate(g, 1.0, 6.0, 0.001, 12.0);
    const StepMetrics m = compute_metrics(s, 1.0, 12.0);
    check(std::fabs(m.steady_state_error) < 0.02,
          "zn: loop Z-N stabil, ess < 2%");
    check(m.overshoot_pct < 60.0, "zn: loop Z-N overshoot < 60%");
}

void test_zn_pi_identification() {
    const MotorParams p;
    const ReactionCurve rc = identify_reaction_curve(p, 5.0, 5.0, 0.001);
    const PidGains g = ziegler_nichols_pi(rc);
    check(g.kp > 0.0 && g.ki > 0.0 && g.kd == 0.0, "zn-pi: gain Z-N PI valid");
    const auto s = simulate(g, 1.0, 6.0, 0.001, 12.0);
    const StepMetrics m = compute_metrics(s, 1.0, 12.0);
    check(std::fabs(m.steady_state_error) < 0.02, "zn-pi: loop Z-N PI ess < 2%");
}

}  // namespace

int main() {
    std::printf("pid-controller-sim tests\n");
    test_motor_steady_state();
    test_motor_initial_current();
    test_motor_negative_voltage();
    test_motor_dc_gain();
    test_pid_zero_gains();
    test_pid_p_only_steady_state_error();
    test_pid_integral_removes_error();
    test_pid_pi_more_overshoot_than_p();
    test_pid_negative_setpoint();
    test_pid_saturation_and_antiwindup();
    test_metrics_synthetic();
    test_zn_identification();
    test_zn_pi_identification();
    std::printf("%d passed, %d failed\n", g_passed, g_failed);
    return (g_failed == 0) ? 0 : 1;
}
