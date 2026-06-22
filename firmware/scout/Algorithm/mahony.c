#include "mahony.h"
#include <math.h>
#define TWO_KP  1.0f
#define TWO_KI  0.0f

static float inv_sqrt(float x) {
    float h = 0.5f * x; int32_t i;
    __builtin_memcpy(&i, &x, 4);
    i = 0x5f3759df - (i >> 1);
    __builtin_memcpy(&x, &i, 4);
    return x * (1.5f - h * x * x);
}

void Mahony_Init(Mahony_t *m) {
    m->q[0]=1; m->q[1]=m->q[2]=m->q[3]=0;
    m->roll=m->pitch=m->yaw=0;
    m->iFBx=m->iFBy=m->iFBz=0;
}

void Mahony_Update(Mahony_t *m, float gx, float gy, float gz,
                   float ax, float ay, float az, float dt) {
    float q0=m->q[0], q1=m->q[1], q2=m->q[2], q3=m->q[3];
    float rn = inv_sqrt(ax*ax+ay*ay+az*az);
    ax*=rn; ay*=rn; az*=rn;
    float hx=q1*q3-q0*q2, hy=q0*q1+q2*q3, hz=q0*q0-0.5f+q3*q3;
    float ex=ay*hz-az*hy, ey=az*hx-ax*hz, ez=ax*hy-ay*hx;
    if(TWO_KI>0){m->iFBx+=TWO_KI*ex*dt; m->iFBy+=TWO_KI*ey*dt; m->iFBz+=TWO_KI*ez*dt;
        gx+=m->iFBx; gy+=m->iFBy; gz+=m->iFBz;}
    gx+=TWO_KP*ex; gy+=TWO_KP*ey; gz+=TWO_KP*ez;
    float d=0.5f*dt; gx*=d; gy*=d; gz*=d;
    float qa=q0,qb=q1,qc=q2;
    q0+=(-qb*gx-qc*gy-q3*gz); q1+=(qa*gx+qc*gz-q3*gy);
    q2+=(qa*gy-qb*gz+q3*gx);  q3+=(qa*gz+qb*gy-qc*gx);
    rn=inv_sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
    m->q[0]=q0*rn; m->q[1]=q1*rn; m->q[2]=q2*rn; m->q[3]=q3*rn;
    m->roll =atan2f(2*(m->q[0]*m->q[1]+m->q[2]*m->q[3]),
                    1-2*(m->q[1]*m->q[1]+m->q[2]*m->q[2]));
    m->pitch=asinf(2*(m->q[0]*m->q[2]-m->q[3]*m->q[1]));
    m->yaw  =atan2f(2*(m->q[0]*m->q[3]+m->q[1]*m->q[2]),
                    1-2*(m->q[2]*m->q[2]+m->q[3]*m->q[3]));
}
