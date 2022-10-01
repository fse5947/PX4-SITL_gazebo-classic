#ifndef AERO_LOOKUP_H
#define AERO_LOOKUP_H

#include <cstddef>
#include <cstdlib>
#include <math.h>
#include <vector>
#include <algorithm>


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
                 int len_a, int len_m, int len_xc);
#endif
