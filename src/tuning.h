// tuning.h — Identifikasi plant + tuning Ziegler-Nichols (metode kurva
// reaksi / open-loop, Ziegler & Nichols 1942).
//
// Metode closed-loop Z-N (ultimate gain) dan relay-feedback
// (Astrom-Hagglund 1984) TIDAK dapat dipakai pada plant ini: fungsi
// alih motor orde-2 ini punya dua kutub real, sehingga beda fasenya
// tidak pernah mencapai -180 derajat dan tidak ada gain proporsional
// yang menimbulkan osilasi berkelanjutan. Karena itu auto-tune di sini
// memakai metode kurva reaksi: step open-loop, garis singgung di titik
// belok, lalu tabel Z-N (lihat docs/RESEARCH.md).
#pragma once

#include "motor.h"
#include "pid.h"

struct ReactionCurve {
    double K = 0.0;   // gain DC plant (rad/s per volt), dari step tunak
    double L = 0.0;   // waktu mati semu / lag (s), dari garis singgung
    double T = 0.0;   // konstanta waktu semu (s), dari garis singgung
    double t_inflect = 0.0;  // waktu titik belok (s)
    bool ok = false;  // false bila identifikasi gagal (mis. L <= 0)
};

// Identifikasi kurva reaksi: beri step tegangan `step_voltage` pada motor
// diam, simulasikan selama `duration` detik dengan langkah `dt`, cari
// titik belok (kemiringan d(w)/dt maksimum) dan garis singgungnya.
ReactionCurve identify_reaction_curve(const MotorParams& params, double step_voltage,
                                      double duration, double dt);

// Tabel Ziegler-Nichols kurva reaksi (PID): Kp = 1.2*T/(K*L),
// Ti = 2L, Td = 0.5L, dengan Ki = Kp/Ti dan Kd = Kp*Td.
PidGains ziegler_nichols_pid(const ReactionCurve& rc);

// Varian PI tabel yang sama: Kp = 0.9*T/(K*L), Ti = L/0.3.
PidGains ziegler_nichols_pi(const ReactionCurve& rc);
