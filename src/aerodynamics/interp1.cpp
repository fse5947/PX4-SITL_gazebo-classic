#include "aerodynamics/interp1.h"
#include "aerodynamics/aero_lookup.h"

extern float interp1(std::vector<float> varargin_1, std::vector<float> varargin_2,
                      float varargin_3, int len_1)
{
  float Vq;
  int low_i;
  int exitg1;
  double xtmp;
  int low_ip1;
  int high_i;
  int mid_i;
  low_i = 0;
  do {
    exitg1 = 0;
    if (low_i < len_1) {
      if (isnan(varargin_1[low_i])) {
        exitg1 = 1;
      } else {
        low_i++;
      }
    } else {
      if (varargin_1[1] < varargin_1[0]) {
        for (low_i = 0; low_i < (int)(len_1-1)/2; low_i++) {
          xtmp = varargin_1[low_i];
          varargin_1[low_i] = varargin_1[(len_1-1) - low_i];
          varargin_1[(len_1-1) - low_i] = xtmp;
          xtmp = varargin_2[low_i];
          varargin_2[low_i] = varargin_2[(len_1-1) - low_i];
          varargin_2[(len_1-1) - low_i] = xtmp;
        }
      }

      Vq = nan("");
      if ((!isnan(varargin_3)) && (!(varargin_3 > varargin_1[(len_1-1)])) && (!(varargin_3 <
            varargin_1[0]))) {
        low_i = 1;
        low_ip1 = 2;
        high_i = len_1;
        while (high_i > low_ip1) {
          mid_i = (low_i + high_i) >> 1;
          if (varargin_3 >= varargin_1[mid_i - 1]) {
            low_i = mid_i;
            low_ip1 = mid_i + 1;
          } else {
            high_i = mid_i;
          }
        }

        xtmp = varargin_1[low_i - 1];
        xtmp = (varargin_3 - xtmp) / (varargin_1[low_i] - xtmp);
        if (xtmp == 0.0) {
          Vq = varargin_2[low_i - 1];
        } else if (xtmp == 1.0) {
          Vq = varargin_2[low_i];
        } else if (varargin_2[low_i - 1] == varargin_2[low_i]) {
          Vq = varargin_2[low_i - 1];
        } else {
          Vq = (1.0 - xtmp) * varargin_2[low_i - 1] + xtmp * varargin_2[low_i];
        }
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return Vq;
}

/* End of code generation (interp1.cpp) */
