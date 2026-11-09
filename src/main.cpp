// main.cpp — CLI pid-controller-sim.
//
// Simulasi loop tertutup kecepatan motor DC dengan kontroler PID:
//   setpoint (rad/s) -> PID -> tegangan (dijenuhkan di +/-vmax) -> motor DC
//   -> kecepatan (rad/s) -> kembali ke PID.
//
// Contoh:
//   ./pid-sim                                  # preset default (CTMS)
//   ./pid-sim --preset zn --csv out.csv        # tuning Ziegler-Nichols
//   ./pid-sim --kp 100 --ki 0 --kd 0           # P-only, tunjukkan ess
//   ./pid-sim --identify                       # identifikasi kurva reaksi
//   ./pid-sim --setpoint 5                     # uji saturasi aktuator
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "metrics.h"
#include "motor.h"
#include "pid.h"
#include "tuning.h"

namespace {

void print_usage(const char* prog) {
    std::cout
        << "Usage: " << prog << " [options]\n"
        << "\n"
        << "Simulasi PID kecepatan motor DC (parameter CTMS: J=0.01, B=0.1,\n"
        << "Ke=Kt=0.01, R=1, L=0.5). Setpoint default 1 rad/s, durasi 4 s.\n"
        << "\n"
        << "Options:\n"
        << "  --kp X --ki X --kd X   gain PID (default preset CTMS 100/200/10)\n"
        << "  --preset NAME          default | zn | zn-pi | p-only | pi\n"
        << "                         (--kp/--ki/--kd menang atas preset bila\n"
        << "                          dipakai bersamaan)\n"
        << "  --setpoint X           kecepatan target, rad/s (default 1)\n"
        << "  --duration X           durasi simulasi, detik (default 4)\n"
        << "  --dt X                 langkah integrasi, detik (default 0.001)\n"
        << "  --vmax X               saturasi aktuator, volt (default 12; 0 = tanpa batas)\n"
        << "  --deriv-on-error       turunan pada error (default: turunan pada pengukuran)\n"
        << "  --csv FILE             tulis deret waktu t,omega,current,voltage ke CSV\n"
        << "  --plot                 tampilkan plot ASCII respons step\n"
        << "  --identify             hanya identifikasi kurva reaksi + gain Z-N, lalu keluar\n"
        << "  --help                 tampilkan bantuan ini\n";
}

// Jalankan satu simulasi loop tertutup; kembalikan deret sampel.
// Pada tiap langkah: hitung u dari keadaan saat ini, catat sampel,
// lalu majukan motor dengan u itu (zero-order hold) ke langkah berikut.
std::vector<Sample> run_closed_loop(const MotorParams& params, const PidGains& gains,
                                    double setpoint, double duration, double dt,
                                    double vmax, bool deriv_on_measurement) {
    DcMotor motor(params);
    PidController pid(gains, vmax, deriv_on_measurement);
    std::vector<Sample> samples;
    const size_t n = static_cast<size_t>(duration / dt) + 1;
    samples.reserve(n);
    double u = 0.0;
    for (size_t k = 0; k < n; ++k) {
        const double t = k * dt;
        if (k > 0) motor.step(u, dt);
        u = pid.update(setpoint, motor.speed(), dt);
        samples.push_back(Sample{t, motor.speed(), motor.current(), u});
    }
    return samples;
}

void write_csv(const std::string& path, const std::vector<Sample>& samples) {
    std::ofstream out(path);
    out << "t,omega_rad_s,current_A,voltage_V\n";
    char buf[256];
    for (const auto& s : samples) {
        std::snprintf(buf, sizeof(buf), "%.4f,%.6f,%.6f,%.6f\n", s.t, s.speed,
                      s.current, s.voltage);
        out << buf;
    }
}

// Plot ASCII: 70 kolom x 18 baris, kecepatan vs waktu + garis setpoint.
void print_ascii_plot(const std::vector<Sample>& samples, double setpoint) {
    const int cols = 70, rows = 18;
    double ymin = 0.0, ymax = setpoint;
    for (const auto& s : samples) {
        ymin = std::min(ymin, s.speed);
        ymax = std::max(ymax, s.speed);
    }
    if (ymax - ymin < 1e-9) ymax = ymin + 1.0;
    const double tmax = samples.back().t;
    std::vector<std::string> grid(rows, std::string(cols, ' '));
    const int sp_row = static_cast<int>((ymax - setpoint) / (ymax - ymin) * (rows - 1));
    for (int c = 0; c < cols; ++c) grid[sp_row][c] = '-';
    for (const auto& s : samples) {
        const int c = static_cast<int>(s.t / tmax * (cols - 1));
        const int r = static_cast<int>((ymax - s.speed) / (ymax - ymin) * (rows - 1));
        if (r >= 0 && r < rows && c >= 0 && c < cols) grid[r][c] = '*';
    }
    std::cout << "\nASCII plot: kecepatan (rad/s) vs waktu (s), '-' = setpoint "
              << setpoint << " rad/s\n";
    for (int r = 0; r < rows; ++r) {
        const double val = ymax - r * (ymax - ymin) / (rows - 1);
        char label[16];
        std::snprintf(label, sizeof(label), "%6.2f |", val);
        std::cout << label << grid[r] << "|\n";
    }
    std::cout << "       +";
    for (int c = 0; c < cols; ++c) std::cout << "-";
    std::cout << "+\n       0";
    for (int c = 0; c < cols - 8; ++c) std::cout << " ";
    std::cout << "t = " << tmax << " s\n";
}

void print_metrics(const StepMetrics& m, double setpoint, double duration) {
    std::printf("Step-response metrics (setpoint %.3f rad/s, durasi %.2f s)\n",
                setpoint, duration);
    std::printf("  Overshoot          : %7.2f %%\n", m.overshoot_pct);
    if (m.rise_time >= 0.0)
        std::printf("  Rise time (10-90%%) : %7.3f s\n", m.rise_time);
    else
        std::printf("  Rise time (10-90%%) : tidak pernah mencapai 90%%\n");
    if (m.settling_time >= 0.0)
        std::printf("  Settling time (2%%)  : %7.3f s\n", m.settling_time);
    else
        std::printf("  Settling time (2%%)  : tidak pernah menetap\n");
    std::printf("  Steady-state error : %7.4f rad/s (%.2f %%)\n", m.steady_state_error,
                100.0 * m.steady_state_error / setpoint);
    std::printf("  Nilai akhir        : %7.4f rad/s (puncak %.4f rad/s @ %.3f s)\n",
                m.final_value, m.peak_value, m.peak_time);
    std::printf("  IAE                : %7.4f rad.s\n", m.iae);
    std::printf("  |V| maks           : %7.2f V (jenuh %.1f %% sampel)\n", m.max_voltage,
                100.0 * m.saturated_fraction);
}

bool parse_double(const char* arg, double& out) {
    char* end = nullptr;
    out = std::strtod(arg, &end);
    return end != arg && *end == '\0';
}

}  // namespace

int main(int argc, char** argv) {
    MotorParams params;  // default CTMS
    PidGains gains{100.0, 200.0, 10.0};  // preset default (CTMS, terverifikasi)
    std::string preset;
    // Gain dari CLI dicatat terpisah agar menang atas preset bila keduanya dipakai.
    bool kp_given = false, ki_given = false, kd_given = false;
    double kp_cli = 0.0, ki_cli = 0.0, kd_cli = 0.0;
    double setpoint = 1.0;
    double duration = 4.0;
    double dt = 0.001;
    double vmax = 12.0;
    bool deriv_on_measurement = true;
    bool identify_only = false;
    bool do_plot = false;
    std::string csv_path;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto need_value = [&](double& dst, const char* name) {
            if (i + 1 >= argc) {
                std::cerr << "error: " << name << " butuh nilai\n";
                return false;
            }
            if (!parse_double(argv[++i], dst)) {
                std::cerr << "error: nilai tidak valid untuk " << name << ": " << argv[i]
                          << "\n";
                return false;
            }
            return true;
        };
        if (a == "--help" || a == "-h") {
            print_usage(argv[0]);
            return 0;
        } else if (a == "--kp") {
            if (!need_value(kp_cli, "--kp")) return 1;
            kp_given = true;
        } else if (a == "--ki") {
            if (!need_value(ki_cli, "--ki")) return 1;
            ki_given = true;
        } else if (a == "--kd") {
            if (!need_value(kd_cli, "--kd")) return 1;
            kd_given = true;
        } else if (a == "--setpoint") {
            if (!need_value(setpoint, "--setpoint")) return 1;
        } else if (a == "--duration") {
            if (!need_value(duration, "--duration")) return 1;
        } else if (a == "--dt") {
            if (!need_value(dt, "--dt")) return 1;
        } else if (a == "--vmax") {
            if (!need_value(vmax, "--vmax")) return 1;
        } else if (a == "--preset") {
            if (i + 1 >= argc) {
                std::cerr << "error: --preset butuh nama\n";
                return 1;
            }
            preset = argv[++i];
        } else if (a == "--deriv-on-error") {
            deriv_on_measurement = false;
        } else if (a == "--identify") {
            identify_only = true;
        } else if (a == "--plot") {
            do_plot = true;
        } else if (a == "--csv") {
            if (i + 1 >= argc) {
                std::cerr << "error: --csv butuh path file\n";
                return 1;
            }
            csv_path = argv[++i];
        } else {
            std::cerr << "error: opsi tidak dikenal: " << a << "\n";
            print_usage(argv[0]);
            return 1;
        }
    }
    if (duration <= 0.0 || dt <= 0.0 || dt > duration) {
        std::cerr << "error: durasi dan dt harus > 0 dan dt <= durasi\n";
        return 1;
    }

    // Identifikasi kurva reaksi (dipakai preset zn / zn-pi dan --identify).
    ReactionCurve rc;
    bool rc_needed = identify_only || preset == "zn" || preset == "zn-pi";
    if (rc_needed) {
        rc = identify_reaction_curve(params, 5.0, 5.0, dt);
        if (!rc.ok) {
            std::cerr << "error: identifikasi kurva reaksi gagal\n";
            return 1;
        }
        std::printf("Identifikasi kurva reaksi (step open-loop 5 V, 5 s):\n");
        std::printf("  Gain DC K = %.4f rad/s per V\n", rc.K);
        std::printf("  Lag semu L = %.4f s, konstanta waktu T = %.4f s\n", rc.L, rc.T);
        std::printf("  Titik belok @ %.4f s\n", rc.t_inflect);
        const PidGains zn = ziegler_nichols_pid(rc);
        const PidGains zn_pi = ziegler_nichols_pi(rc);
        std::printf("  Z-N PID: Kp=%.2f Ki=%.2f Kd=%.2f\n", zn.kp, zn.ki, zn.kd);
        std::printf("  Z-N PI : Kp=%.2f Ki=%.2f Kd=%.2f\n", zn_pi.kp, zn_pi.ki, zn_pi.kd);
        if (identify_only) return 0;
        gains = (preset == "zn-pi") ? zn_pi : zn;
    } else if (!preset.empty()) {
        if (preset == "default") {
            gains = PidGains{100.0, 200.0, 10.0};
        } else if (preset == "p-only") {
            gains = PidGains{100.0, 0.0, 0.0};
        } else if (preset == "pi") {
            gains = PidGains{100.0, 200.0, 0.0};
        } else {
            std::cerr << "error: preset tidak dikenal: " << preset
                      << " (default|zn|zn-pi|p-only|pi)\n";
            return 1;
        }
    }
    // Gain dari CLI menang atas preset.
    if (kp_given) gains.kp = kp_cli;
    if (ki_given) gains.ki = ki_cli;
    if (kd_given) gains.kd = kd_cli;

    std::printf("PID motor DC — Kp=%.2f Ki=%.2f Kd=%.2f | setpoint=%.3f rad/s | "
                "durasi=%.2f s dt=%.4f s | vmax=%.1f V | turunan pada %s\n",
                gains.kp, gains.ki, gains.kd, setpoint, duration, dt, vmax,
                deriv_on_measurement ? "pengukuran" : "error");

    const std::vector<Sample> samples =
        run_closed_loop(params, gains, setpoint, duration, dt, vmax,
                        deriv_on_measurement);
    const StepMetrics m = compute_metrics(samples, setpoint, vmax);
    print_metrics(m, setpoint, duration);
    if (!csv_path.empty()) {
        write_csv(csv_path, samples);
        std::printf("CSV ditulis: %s (%zu sampel)\n", csv_path.c_str(), samples.size());
    }
    if (do_plot) print_ascii_plot(samples, setpoint);
    return 0;
}
