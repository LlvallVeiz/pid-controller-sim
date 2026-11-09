#!/usr/bin/env bash
# run_examples.sh — Menjalankan semua contoh pid-controller-sim dan
# menyimpan output ASLI ke examples/*.txt / *.csv. Jalankan dari root repo:
#   make examples   atau   bash examples/run_examples.sh
# Skrip ini idempotent: output lama ditimpa dengan hasil run yang baru.
set -euo pipefail
cd "$(dirname "$0")/.."
BIN=./pid-sim
EX=examples

# 1. Default: preset CTMS Kp=100, Ki=200, Kd=10, saturasi +/-12 V.
"$BIN" --preset default --csv "$EX/default.csv" --plot > "$EX/default_output.txt"
# 2. P-only: tanpa I, steady-state error 9.1% tetap ada.
"$BIN" --preset p-only --csv "$EX/p_only.csv" > "$EX/p_only_output.txt"
# 3. PI: I menghilangkan ess, tapi tanpa D overshoot membesar
#    (lihat contoh 4 untuk versi tanpa saturasi).
"$BIN" --preset pi --csv "$EX/pi.csv" > "$EX/pi_output.txt"
# 4. Efek tiap suku, tanpa saturasi (perilaku buku teks murni):
#    P-only vs PI vs PID pada aktuator tak terbatas.
"$BIN" --preset p-only --vmax 0 --csv "$EX/p_only_nosat.csv" > "$EX/p_only_nosat_output.txt"
"$BIN" --preset pi --vmax 0 --csv "$EX/pi_nosat.csv" > "$EX/pi_nosat_output.txt"
"$BIN" --preset default --vmax 0 --csv "$EX/pid_nosat.csv" > "$EX/pid_nosat_output.txt"
# 5. Ziegler-Nichols dari identifikasi kurva reaksi (auto-tune).
"$BIN" --preset zn --csv "$EX/zn.csv" --plot > "$EX/zn_output.txt"
# 6. Saturasi aktuator: setpoint 5 rad/s butuh ~50 V, aktuator mentok 12 V.
"$BIN" --setpoint 5 --csv "$EX/saturation.csv" > "$EX/saturation_output.txt"
# 7. Setpoint negatif: respons simetris.
"$BIN" --setpoint -1 --csv "$EX/negative.csv" > "$EX/negative_output.txt"
# 8. Kasus batas Kp=0: kontrol mati, motor diam.
"$BIN" --kp 0 --ki 0 --kd 0 > "$EX/zero_gain_output.txt"
# 9. Identifikasi kurva reaksi saja.
"$BIN" --identify > "$EX/identify_output.txt"

echo "Contoh selesai. Output di $EX/:"
ls "$EX"/*.txt
