#pragma once
typedef struct {
    float q[4];
    float roll, pitch, yaw;
    float iFBx, iFBy, iFBz;
} Mahony_t;
void Mahony_Init  (Mahony_t *m);
void Mahony_Update(Mahony_t *m, float gx, float gy, float gz,
                   float ax, float ay, float az, float dt);
