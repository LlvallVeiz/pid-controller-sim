#!/usr/bin/env python3
"""plot_examples.py — Plot perbandingan respons step dari CSV contoh.

Membaca CSV yang dihasilkan examples/run_examples.sh dan menyimpan
examples/comparison.png (respons P-only / PI / PID / Z-N, saturasi +/-12 V)
dan examples/nosat_comparison.png (efek tiap suku tanpa saturasi).

Membutuhkan matplotlib (sudah tersedia di VM). Jalankan dari root repo:
    python3 examples/plot_examples.py
"""
import csv
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))


def load_csv(name):
    t, w = [], []
    with open(os.path.join(HERE, name), newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row["t"]))
            w.append(float(row["omega_rad_s"]))
    return t, w


def plot(series, title, out_name, setpoint=1.0):
    plt.figure(figsize=(8, 4.5))
    for label, fname in series:
        t, w = load_csv(fname)
        plt.plot(t, w, label=label)
    plt.axhline(setpoint, color="black", linestyle="--", linewidth=1,
                label=f"setpoint {setpoint} rad/s")
    plt.xlabel("waktu (s)")
    plt.ylabel("kecepatan (rad/s)")
    plt.title(title)
    plt.legend(loc="lower right")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    out = os.path.join(HERE, out_name)
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"ditulis: {out}")


if __name__ == "__main__":
    plot(
        [
            ("P-only (Kp=100)", "p_only.csv"),
            ("PI (Kp=100, Ki=200)", "pi.csv"),
            ("PID default (Kp=100, Ki=200, Kd=10)", "default.csv"),
            ("Ziegler-Nichols", "zn.csv"),
        ],
        "Respons step motor DC — saturasi aktuator +/-12 V",
        "comparison.png",
    )
    plot(
        [
            ("P-only (Kp=100)", "p_only_nosat.csv"),
            ("PI (Kp=100, Ki=200)", "pi_nosat.csv"),
            ("PID (Kp=100, Ki=200, Kd=10)", "pid_nosat.csv"),
        ],
        "Efek suku P/I/D — tanpa saturasi (perilaku buku teks)",
        "nosat_comparison.png",
    )
