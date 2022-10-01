#include "aerodynamics/aero_lookup.h"
#include "aerodynamics/interp2.h"
#include "aerodynamics/interp3.h"


void aero_lookup(float alpha, float Va, std::vector<float> uu, std::vector<float>
                 ALPHA, std::vector<float> MACH, std::vector<float> XC,
                 std::vector<float> cd, std::vector<float> cl, std::vector<float> cm,
                 std::vector<float> cyb, std::vector<float> cnb, std::vector<float>
                 clb, std::vector<float> clq, std::vector<float> cmq,
                 std::vector<float> clad, std::vector<float> cmad, std::vector<float>
                 clp, std::vector<float> cyp, std::vector<float> cnp,
                 std::vector<float> cnr, std::vector<float> clr, std::vector<float>
                 dcl, std::vector<float> dcm, std::vector<float> dcdi,
                 std::vector<float> clroll, std::vector<float> cn_asy, float *CD,
                 float *CL, float *Cm, float *CYb, float *Cnb, float *Clb,
                 float *CLq, float *Cmq, float *CLad, float *Cmad, float
                 *Clp, float *CYp, float *Cnp, float *Cnr, float *Clr,
                 float *DCL, float *DCm, float *DCD, float *DCl, float *DCn,
                 int len_a, int len_m, int len_xc)
{
  float Mach;
  int idx;
  int k;
  float Alpha;
  float d;

  /*  r2d = 180/pi; % Radian to Degree Conversion */
  /*  Speed of Sound in m/s */
  Mach = Va / 343.0;

  if (alpha > *std::max_element(ALPHA.begin(), ALPHA.end())) {
    Alpha = *std::max_element(ALPHA.begin(), ALPHA.end());
  } else if (alpha < *std::min_element(ALPHA.begin(), ALPHA.end())) {
    Alpha = *std::min_element(ALPHA.begin(), ALPHA.end());
  } else {
    Alpha = alpha;
  }

  // /*  Mach Number */
  // /*  Define x values */
  // /*  Define y values */
  // /*  Define x values for Surface Deflections */
  // /*  Check if Angle of Attack is Out of Range, and if so, Taper the */
  // /*  Coefficients Values at Final data.datapoint */
  // b = rtIsNaN(ALPHA[0]);
  // if (!b) {
  //   idx = 1;
  // } else {
  //   idx = 0;
  //   k = 2;
  //   exitg1 = false;
  //   while ((!exitg1) && (k < 186)) {
  //     if (!rtIsNaN(ALPHA[k - 1])) {
  //       idx = k;
  //       exitg1 = true;
  //     } else {
  //       k++;
  //     }
  //   }
  // }

  // if (idx == 0) {
  //   Alpha = ALPHA[0];
  // } else {
  //   Alpha = ALPHA[idx - 1];
  //   idx++;
  //   for (k = idx; k < 186; k++) {
  //     d = ALPHA[k - 1];
  //     if (Alpha > d) {
  //       Alpha = d;
  //     }
  //   }
  // }

  // if (!(alpha < Alpha)) {
  //   if (!b) {
  //     idx = 1;
  //   } else {
  //     idx = 0;
  //     k = 2;
  //     exitg1 = false;
  //     while ((!exitg1) && (k < 186)) {
  //       if (!rtIsNaN(ALPHA[k - 1])) {
  //         idx = k;
  //         exitg1 = true;
  //       } else {
  //         k++;
  //       }
  //     }
  //   }

  //   if (idx == 0) {
  //     Alpha = ALPHA[0];
  //   } else {
  //     Alpha = ALPHA[idx - 1];
  //     idx++;
  //     for (k = idx; k < 186; k++) {
  //       d = ALPHA[k - 1];
  //       if (Alpha < d) {
  //         Alpha = d;
  //       }
  //     }
  //   }

  //   if (!(alpha > Alpha)) {
  //     /*  Alpha = alpha*r2d; % Angle of Attack in Degrees */
  //     Alpha = alpha;
  //   }
  // }

  /*  Interpolate Using Spline to Extrapolate Values Outside data.data Range */
  /*  Static Coefficients */
  *CD = interp2(MACH, ALPHA, cd, Mach, Alpha, len_m, len_a);
  *CL = interp2(MACH, ALPHA, cl, Mach, Alpha, len_m, len_a);
  *Cm = interp2(MACH, ALPHA, cm, Mach, Alpha, len_m, len_a);

  /*  Coeff.Cma = interp2(Y,X,data.data.cma,Mach,Alpha,'spline'); */
  *CYb = interp2(MACH, ALPHA, cyb, Mach, Alpha, len_m, len_a);
  *Cnb = interp2(MACH, ALPHA, cnb, Mach, Alpha, len_m, len_a);
  *Clb = interp2(MACH, ALPHA, clb, Mach, Alpha, len_m, len_a);

  /*  Dynamic Coefficients */
  *CLq = interp2(MACH, ALPHA, clq, Mach, Alpha, len_m, len_a);
  *Cmq = interp2(MACH, ALPHA, cmq, Mach, Alpha, len_m, len_a);
  *CLad = interp2(MACH, ALPHA, clad, Mach, Alpha, len_m, len_a);
  *Cmad = interp2(MACH, ALPHA, cmad, Mach, Alpha, len_m, len_a);
  *Clp = interp2(MACH, ALPHA, clp, Mach, Alpha, len_m, len_a);
  *CYp = interp2(MACH, ALPHA, cyp, Mach, Alpha, len_m, len_a);
  *Cnp = interp2(MACH, ALPHA, cnp, Mach, Alpha, len_m, len_a);
  *Cnr = interp2(MACH, ALPHA, cnr, Mach, Alpha, len_m, len_a);
  *Clr = interp2(MACH, ALPHA, clr, Mach, Alpha, len_m, len_a);

  /*  Control Coefficients */
  /*  Elevator Control Coefficients */
  *DCL = interp2(MACH, XC, dcl, Mach, uu[1], len_m, len_xc);
  *DCm = interp2(MACH, XC, dcm, Mach, uu[1], len_m, len_xc);
  *DCD = interp3(XC, ALPHA, MACH, dcdi, uu[1], Alpha, Mach, len_xc, len_a, len_m);

  /*  Aileron Control Coefficients */
  *DCl = interp2(MACH, XC, clroll, Mach, uu[0], len_m, len_xc);
  *DCn = interp3(XC, ALPHA, MACH, cn_asy, uu[0], Alpha, Mach, len_xc, len_a, len_m);
}

/* End of code generation (aero_lookup.cpp) */
