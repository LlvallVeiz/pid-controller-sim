// motor.h — Model motor DC magnet permanen (armature-controlled).
//
// Persamaan keadaan (lihat docs/RESEARCH.md):
//   L di/dt = V - R i - Ke w
//   J dw/dt = Kt i - B w
// Parameter default diambil dari contoh motor DC CTMS (University of
// Michigan), yang diturunkan dari pengukuran motor nyata di lab kontrol
// sarjana Carnegie Mellon:
//   J = 0.01 kg.m^2, B = 0.1 N.m.s, Ke = Kt = 0.01, R = 1 ohm, L = 0.5 H
// Dalam satuan SI, Kt == Ke secara numerik.
#pragma once

struct MotorParams {
    double J  = 0.01;  // momen inersia rotor + beban (kg.m^2)
    double B  = 0.1;   // koefisien gesekan viscous (N.m.s/rad)
    double Ke = 0.01;  // konstanta back-EMF (V.s/rad)
    double Kt = 0.01;  // konstanta torsi (N.m/A)
    double R  = 1.0;   // resistansi jangkar (ohm)
    double L  = 0.5;   // induktansi jangkar (H)
};

// Kecepatan tunak (steady state) untuk tegangan konstan V:
//   w_ss = Kt * V / (R*B + Kt*Ke)
double motor_steady_state_speed(const MotorParams& p, double voltage);

// Gain DC plant (rad/s per volt): w_ss / V.
double motor_dc_gain(const MotorParams& p);

class DcMotor {
public:
    explicit DcMotor(const MotorParams& params = MotorParams());

    // Maju satu langkah integrasi RK4 dengan tegangan jangkar `voltage`
    // (dianggap konstan selama langkah) dan langkah waktu `dt` (detik).
    void step(double voltage, double dt);

    double current() const { return i_; }  // arus jangkar (A)
    double speed() const { return w_; }    // kecepatan sudut (rad/s)
    void reset();

private:
    // Turunan keadaan [di/dt, dw/dt] pada keadaan (i, w) dan tegangan v.
    void derivatives(double i, double w, double v, double& di, double& dw) const;

    MotorParams p_;
    double i_ = 0.0;  // arus jangkar (A)
    double w_ = 0.0;  // kecepatan sudut (rad/s)
};
