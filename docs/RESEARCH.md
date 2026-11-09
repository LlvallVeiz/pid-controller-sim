# Research notes — pid-controller-sim

Catatan riset untuk simulasi PID motor DC. Semua klaim teori di bawah
dirujuk ke sumber yang dicek langsung; angka hasil simulasi dikutip dari
run program (`examples/`), bukan dari teori.

## 1. Kontroler PID

Kontroler PID bentuk paralel (ISA standard form):

    u(t) = Kp·e(t) + Ki·∫e(τ)dτ + Kd·de(t)/dt,   e(t) = r(t) − y(t)

Peran tiap suku (Åström & Hägglund 1995, bab 3; CTMS "Introduction: PID
Controller Design"):

- **P** — aksi sebanding error; menaikkan Kp mempercepat respons dan
  mengecilkan steady-state error, tapi menaikkan overshoot.
- **I** — menghilangkan steady-state error untuk referensi step (plant
  tipe-0 menjadi tipe-1); harga yang dibayar: overshoot dan settling
  lebih besar bila Ki terlalu besar.
- **D** — aksi antisipatif terhadap laju perubahan error; meredam
  overshoot dan memperbaiki stabilitas. D murni pada error menimbulkan
  *derivative kick* saat setpoint berubah mendadak, sehingga implementasi
  digital lazimnya memakai turunan pada pengukuran (lihat §5).

## 2. Model motor DC

Motor DC magnet permanen yang dikontrol jangkarnya (armature-controlled)
dimodelkan oleh dua persamaan (CTMS, "DC Motor Speed: System Modeling"):

    L·di/dt = V − R·i − Ke·ω        (Kirchhoff, rangkaian jangkar)
    J·dω/dt = Kt·i − B·ω           (Newton, gerak rotasi)

dengan back-EMF eb = Ke·ω dan torsi Tm = Kt·i. Fungsi alih kecepatan
terhadap tegangan:

    Ω(s)/V(s) = Kt / ((L·s + R)(J·s + B) + Kt·Ke)

Dalam satuan SI, Kt = Ke secara numerik. Bila L diabaikan, plant menjadi
orde-1 dengan gain Km = Kt/(R·B + Kt·Ke) dan konstanta waktu
τm = R·J/(R·B + Kt·Ke) (CTMS; IIT Kharagpur Virtual Labs).

### Parameter yang dipakai

Parameter default program ini adalah contoh motor CTMS (University of
Michigan), yang menurut CTMS "diturunkan dari eksperimen pada motor
nyata di laboratorium kontrol sarjana Carnegie Mellon":

| Parameter | Nilai | Satuan |
| --- | --- | --- |
| J (inersia rotor) | 0,01 | kg·m² |
| B (gesekan viscous) | 0,1 | N·m·s/rad |
| Ke = Kt | 0,01 | V·s/rad = N·m/A |
| R (resistansi jangkar) | 1 | Ω |
| L (induktansi jangkar) | 0,5 | H |

Konsekuensi yang dapat dicek: gain DC plant K = 0,01/(1·0,1 + 0,01²) =
0,0999 rad/s per volt — motor hanya mencapai 0,1 rad/s per 1 V (CTMS:
"when 1 V is applied, the motor can only achieve 0.1 rad/sec"). Kutub
plant: s = −2,00 dan s = −9,99 (dua kutub real; hitung dari
0,005s² + 0,06s + 0,1001 = 0).

## 3. Tuning Ziegler–Nichols

Ziegler & Nichols (1942) mengusulkan dua metode (paper asli:
"Optimum Settings for Automatic Controllers", *Transactions of the
ASME* 64:759–768, doi:10.1115/1.2899060):

**a. Metode closed-loop (ultimate gain)** — naikkan Kp (I = D = 0)
sampai terjadi osilasi berkelanjutan; catat gain ultimit Ku dan periode
ultimit Pu. Lalu PID: Kp = 0,6·Ku, Ti = Pu/2, Td = Pu/8
(Ki = Kp/Ti, Kd = Kp·Td). Target desainnya adalah *quarter-amplitude
decay*.

**b. Metode kurva reaksi (open-loop)** — beri step pada plant loop
terbuka, tarik garis singgung di titik belok, baca lag semu L dan
konstanta waktu semu T, serta gain K = Δy/Δu. Lalu PID:
Kp = 1,2·T/(K·L), Ti = 2L, Td = 0,5L; PI: Kp = 0,9·T/(K·L), Ti = L/0,3.

### Mengapa program ini memakai (b), bukan (a)

Metode (a) mensyaratkan plant *dapat* dibuat berosilasi oleh kontrol
P-only, yaitu beda fase loop terbukanya harus mencapai −180°.
Plant motor ini hanya punya dua kutub real, sehingga beda fasenya
mendekati −180° secara asimtotik tapi tidak pernah mencapainya: tidak
ada Kp yang menghasilkan osilasi berkelanjutan (persamaan karakteristik
0,005s² + 0,06s + 0,1001 + 0,01·Kp = 0 selalu berakar real/kompleks
dengan bagian real negatif). Hal yang sama berlaku untuk relay-feedback
Åström–Hägglund (1984): tidak ada limit cycle pada plant ini. Karena
itu auto-tune di program ini memakai metode kurva reaksi (b), yang
tetap terdefinisi untuk plant orde-2 semacam ini.

Hasil identifikasi program ini (step open-loop 5 V, `examples/identify_output.txt`):
K = 0,0999 rad/s per V (cocok dengan gain DC teori), L = 0,0535 s,
T = 0,7470 s, titik belok di t = 0,201 s. Nilai L dan T ini juga cocok
dengan hitung tangan dari kutub plant (τ1 = 0,5 s, τ2 = 0,1 s):
t_belok = ln(5)/8 = 0,201 s; L = 0,0534 s; T = 0,748 s.

Gain Z-N yang dihasilkan: PID Kp = 167,74, Ki = 1567,88, Kd = 4,49;
PI Kp = 125,81, Ki = 705,54. Z-N memang agresif (dirancang untuk
quarter-decay): tanpa saturasi overshoot-nya 57,5% — lihat §6.

## 4. Relay-feedback Åström–Hägglund (1984)

Åström & Hägglund ("Automatic tuning of simple regulators with
specifications on phase and amplitude margins", *Automatica* 20(5):
645–651, 1984, doi:10.1016/0005-1098(84)90014-1) mengganti uji
osilasi Z-N dengan relay on/off di loop: relay amplitudo d
menghasilkan osilasi dengan amplitudo a dan periode Pu, lalu
Ku = 4d/(π·a) dari analisis describing function, dan gain dihitung
dengan rumus Z-N. Kelebihannya: uji berlangsung loop tertutup dan
amplitudonya terkendali (tidak ada risiko runaway seperti menaikkan
Kp sampai berosilasi). Metode ini menjadi dasar banyak auto-tuner
industri (Hang, Åström & Wang 2002, *J. Process Control* 12:143–162).
Seperti dijelaskan di §3, metode ini tidak diterapkan di program ini
karena plant dua-kutub-real ini tidak menghasilkan limit cycle —
bukan karena metodenya buruk, melainkan karena syarat fasenya tidak
terpenuhi.

## 5. Dua keputusan implementasi digital

**Turunan pada pengukuran.** Program memakai −Kd·dω/dt sebagai suku D
(default), bukan Kd·de/dt. Untuk setpoint konstan keduanya identik
(de/dt = −dω/dt); bedanya hanya pada saat setpoint melompat: turunan
pada error menghasilkan lonjakan tegangan sesaat (*derivative kick*).
Praktik ini standar pada kontroler digital (mis. pustaka PID Arduino).

**Anti-windup (conditional integration).** Saat keluaran jenuh
(|u| = vmax) dan error mendorong lebih jauh ke arah jenuh, integrator
dibekukan; integrasi dilanjutkan saat error berlawanan arah. Tanpa ini,
integrator membesar tanpa batas selama saturasi dan menimbulkan
overshoot besar saat setpoint akhirnya tercapai (Åström & Hägglund
1995, bab 3.5).

## 6. Validasi silang terhadap model kontinu (SciPy)

Simulator diskret ini (RK4, dt = 1 ms) divalidasi terhadap model
fungsi-alih kontinu yang dihitung independen dengan SciPy:

| Kasus | Simulator ini | Model kontinu (SciPy) |
| --- | --- | --- |
| P-only, tanpa saturasi: ess | 0,0910 rad/s | 0,0910 rad/s |
| P-only, tanpa saturasi: puncak | 1,1387 rad/s | 1,1355 rad/s |
| Motor open-loop, 1 V: w_ss | 0,0999 rad/s | 0,0999 rad/s (teori) |

Selisih puncak 0,3% adalah error diskretisasi (dt = 1 ms), bukan bug.

Satu perbedaan yang disengaja: PID kontinu (turunan pada error)
memberi overshoot 1,03%, sedangkan simulator ini memberi 11,59%
tanpa saturasi. Penyebabnya adalah *derivative kick* versi kontinu:
fungsi alih loop tertutupnya punya nol di s = −2,76 dan −7,24, dan
kemiringan awal responsnya y′(0+) = 20 rad/s² — setara lonjakan
tegangan impulsif sesaat yang tidak dapat dihasilkan driver nyata
(dan yang memang dihilangkan oleh turunan-pada-pengukuran). Dengan
saturasi realistis ±12 V, overshoot default program ini 0% dan semua
kriteria CTMS terpenuhi (lihat §7). Perbedaan ini didokumentasikan
jujur di README, bukan disembunyikan.

## 7. Hasil run (diringkas dari `examples/`)

Setpoint 1 rad/s, durasi 4 s, dt 1 ms, saturasi ±12 V:

| Preset | Overshoot | Rise time | Settling (2%) | ess |
| --- | --- | --- | --- | --- |
| P-only (Kp=100) | 0% | 0,694 s | tidak menetap* | 9,10% |
| PI (Kp=100, Ki=200) | 0% | 0,693 s | 1,601 s | 0,02% |
| PID default (100/200/10) | 0% | 0,713 s | 1,243 s | 0,00% |
| Ziegler–Nichols | 0,59% | 0,692 s | 0,962 s | 0,00% |

\* P-only tidak pernah masuk pita 2% karena ess-nya 9,1% > 2%.

Tanpa saturasi (vmax = 0), perilaku buku teks muncul murni:
P-only overshoot 13,87%, PI 30,91% (I menambah overshoot), PID
11,59% (D meredam overshoot). Kriteria desain CTMS (Ts < 2 s,
overshoot < 5%, ess < 1%) terpenuhi oleh preset default dengan
saturasi realistis.

## References

1. Ziegler, J. G. & Nichols, N. B. (1942). Optimum settings for
   automatic controllers. *Transactions of the ASME*, 64, 759–768.
   doi:10.1115/1.2899060 —
   https://doi.org/10.1115/1.2899060
2. Åström, K. J. & Hägglund, T. (1984). Automatic tuning of simple
   regulators with specifications on phase and amplitude margins.
   *Automatica*, 20(5), 645–651. doi:10.1016/0005-1098(84)90014-1 —
   https://doi.org/10.1016/0005-1098(84)90014-1
3. Åström, K. J. & Hägglund, T. (1995). *PID Controllers: Theory,
   Design, and Tuning*. 2nd ed., Instrument Society of America.
4. Control Tutorials for MATLAB and Simulink (CTMS), University of
   Michigan. "DC Motor Speed: System Modeling" —
   https://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=SystemModeling
5. Control Tutorials for MATLAB and Simulink (CTMS). "Motor Speed:
   PID Controller Design" —
   https://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=ControlPID
6. Hang, C. C., Åström, K. J. & Wang, Q. G. (2002). Relay feedback
   auto-tuning of process controllers — a tutorial review. *Journal
   of Process Control*, 12(1), 143–162.
   doi:10.1016/S0959-1524(01)00025-7 —
   https://www.sciencedirect.com/science/article/pii/S0959152401000257
7. IIT Kharagpur Virtual Labs. "Study of DC Motor" (model motor DC
   orde-1, Km dan τm) —
   http://vlabs.iitkgp.ac.in/gps/ctrl/Exp3/theory.html
