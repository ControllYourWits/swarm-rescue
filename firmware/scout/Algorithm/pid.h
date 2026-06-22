#pragma once
typedef struct {
    float kp, ki, kd;
    float integral_limit, output_limit;
    float integral, prev_error;
} PID_t;
void  PID_Init (PID_t *p, float kp, float ki, float kd, float ilim, float olim);
float PID_Calc (PID_t *p, float sp, float meas, float dt);
void  PID_Reset(PID_t *p);
