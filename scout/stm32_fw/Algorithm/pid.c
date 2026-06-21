#include "pid.h"
static float clamp(float v, float lo, float hi) {
    return v > hi ? hi : v < lo ? lo : v;
}
void PID_Init(PID_t *p, float kp, float ki, float kd, float ilim, float olim) {
    p->kp=kp; p->ki=ki; p->kd=kd;
    p->integral_limit=ilim; p->output_limit=olim;
    PID_Reset(p);
}
float PID_Calc(PID_t *p, float sp, float meas, float dt) {
    float err = sp - meas;
    p->integral = clamp(p->integral + err*dt, -p->integral_limit, p->integral_limit);
    float d = (dt > 1e-6f) ? (err - p->prev_error) / dt : 0.0f;
    p->prev_error = err;
    return clamp(p->kp*err + p->ki*p->integral + p->kd*d,
                 -p->output_limit, p->output_limit);
}
void PID_Reset(PID_t *p) { p->integral = p->prev_error = 0.0f; }
