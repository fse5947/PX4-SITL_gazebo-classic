#ifndef AERO_FORCES_MOMENTS_H
#define AERO_FORCES_MOMENTS_H

/* Function Declarations */
void aero_forces_moments(float u_r, float v_r, float w_r, float p,
  float q, float r, float rho, float cbar, float b,
  float S, float u[3], float CD, float CL, float Cm, float CYb, float Cnb, float Clb,
  float CLq, float Cmq, float CLad, float Cmad, float Clp, float CYp,
  float Cnp, float Cnr, float Clr, float DCL, float DCm, float DCD, float DCl,
  float DCn, float *Fx, float *Fy, float *Fz, float *Mx, float *My, float
  *Mz, float *L, float *D);

#endif
