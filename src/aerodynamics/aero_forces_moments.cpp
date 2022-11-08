#include "common.h"
#include <string.h>
#include <math.h>
#include "aerodynamics/aero_forces_moments.h"

void aero_forces_moments(float u_r, float v_r, float w_r, float p,
  float q, float r, float rho, float cbar, float b,
  float S, float u[3], float CD, float CL, float Cm, float CYb, float Cnb, float Clb,
  float CLq, float Cmq, float CLad, float Cmad, float Clp, float CYp,
  float Cnp, float Cnr, float Clr, float DCL, float DCm, float DCD, float DCl,
  float DCn, float *Fx, float *Fy, float *Fz, float *Mx, float *My, float
  *Mz, float *L, float *D)
{
  float Va;
  float alpha;
  float beta;
  float qbarS;
  float p_s;
  float q_s;
  float r_s;
  float l;
  float n;

  /*  Derive conditions */
  Va = sqrt(u_r * u_r + v_r * v_r + w_r * w_r);
  alpha = atan2(w_r, u_r);
  beta = asin(v_r / Va);
  qbarS = 0.5 * rho * S * Va * Va;

  /*  Rotate Body Rates p, q, r, from Body Frame to Stability Frame */
  p_s = cos(alpha) * p * 180.0 / M_PI + sin(alpha) * r * 180.0 / M_PI;
  q_s = q * 180.0 / M_PI;
  r_s = -sin(alpha) * p * 180.0 / M_PI + cos(alpha) * r * 180.0 / M_PI;

  /*  Forces and Moments in Stability Frame */
  *D = -qbarS * (CD + DCD);

  *Fy = qbarS * (CYb * beta * 180.0 / M_PI + CYp * p_s * b / (2.0 * Va) + 0.1*u[2] * M_PI/180);

  *L = -qbarS * (CL + CLq * q_s * cbar / (2.0 * Va) + DCL);

  l = qbarS * b * (Clb * beta * 180.0 / M_PI + Clp * p_s * b / (2.0 * Va) + DCl + 0.0012*u[2] * M_PI/180);

  *My = qbarS * cbar * (Cm + Cmq * q_s * cbar / (2.0 * Va) + DCm);

  n = qbarS * b * (Cnb * beta * 180.0 / M_PI +
      (Cnp * p_s + Cnr * r_s) * b / (2.0 * Va) + DCn + -0.05*u[2] * M_PI/180);

  /*  Transform Forces and Moments from Stability Frame to Body Frame */
  *Fx = cos(alpha) * *D - sin(alpha) * *L;
  *Fz = sin(alpha) * *D + cos(alpha) * *L;
  *Mx = cos(alpha) * l - sin(alpha) * n;
  *Mz = sin(alpha) * l + cos(alpha) * n;
}
