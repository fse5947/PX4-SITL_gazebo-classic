#include "aerodynamics/aero_lookup.h"


void aero_lookup(float alpha, std::vector<float> uu, std::vector<float>
                 ALPHA, std::vector<float> XC, 
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
                 int len_a, int len_xc)
{
  int idx;
  int k;
  float Alpha;
  float d;

  if (alpha > *std::max_element(ALPHA.begin(), ALPHA.end())) {
    Alpha = *std::max_element(ALPHA.begin(), ALPHA.end());
  } else if (alpha < *std::min_element(ALPHA.begin(), ALPHA.end())) {
    Alpha = *std::min_element(ALPHA.begin(), ALPHA.end());
  } else {
    Alpha = alpha;
  }

  /*  Interpolate Using Spline to Extrapolate Values Outside data.data Range */
  /*  Static Coefficients */
  *CD = interp1(ALPHA, cd, Alpha, len_a);
  *CL = interp1(ALPHA, cl, Alpha, len_a);
  *Cm = interp1(ALPHA, cm, Alpha, len_a);

  *CYb = interp1(ALPHA, cyb, Alpha, len_a);
  *Cnb = interp1(ALPHA, cnb, Alpha, len_a);
  *Clb = interp1(ALPHA, clb, Alpha, len_a);

  /*  Dynamic Coefficients */
  *CLq = interp1(ALPHA, clq, Alpha, len_a);
  *Cmq = interp1(ALPHA, cmq, Alpha, len_a);
  *CLad = interp1(ALPHA, clad, Alpha, len_a);
  *Cmad = interp1(ALPHA, cmad, Alpha, len_a);
  *Clp = interp1(ALPHA, clp, Alpha, len_a);
  *CYp = interp1(ALPHA, cyp, Alpha, len_a);
  *Cnp = interp1(ALPHA, cnp, Alpha, len_a);
  *Cnr = interp1(ALPHA, cnr, Alpha, len_a);
  *Clr = interp1(ALPHA, clr, Alpha, len_a);

  /*  Control Coefficients */
  /*  Elevator Control Coefficients */
  *DCL = interp1(XC, dcl, uu[1], len_xc);
  *DCm = interp1(XC, dcm, uu[1], len_xc);
  *DCD = interp2(XC, ALPHA, dcdi, uu[1], Alpha, len_xc, len_a);

  /*  Aileron Control Coefficients */
  *DCl = interp1(XC, clroll, uu[0], len_xc);
  *DCn = interp2(XC, ALPHA, cn_asy, uu[0], Alpha, len_xc, len_a);
}

/* End of code generation (aero_lookup.cpp) */
