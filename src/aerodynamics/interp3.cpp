#include "aerodynamics/interp3.h"
#include "aerodynamics/aero_lookup.h"


float interp3(std::vector<float> varargin_1, std::vector<float> varargin_2,
               std::vector<float> varargin_3, std::vector<float> varargin_4, float
               varargin_5, float varargin_6, float varargin_7,
               int len_1, int len_2, int len_3)
{
  float Vq;
  int low_i;
  int low_ip1;
  int high_i;
  int b_low_i;
  int mid_i;
  int c_low_i;
  float r;
  float v11;
  float v21;
  float v12;
  if ((varargin_5 >= varargin_1[0]) && (varargin_5 <= varargin_1[(len_1 - 1)]) &&
      (varargin_6 >= varargin_2[0]) && (varargin_6 <= varargin_2[(len_2 - 1)]) &&
      (varargin_7 >= varargin_3[0]) && (varargin_7 <= varargin_3[(len_3 - 1)])) {
    low_i = 0;
    low_ip1 = 2;
    high_i = len_1;
    while (high_i > low_ip1) {
      mid_i = ((low_i + high_i) + 1) >> 1;
      if (varargin_5 >= varargin_1[mid_i - 1]) {
        low_i = mid_i - 1;
        low_ip1 = mid_i + 1;
      } else {
        high_i = mid_i;
      }
    }

    b_low_i = 1;
    low_ip1 = 2;
    high_i = len_2;
    while (high_i > low_ip1) {
      mid_i = (b_low_i + high_i) >> 1;
      if (varargin_6 >= varargin_2[mid_i - 1]) {
        b_low_i = mid_i;
        low_ip1 = mid_i + 1;
      } else {
        high_i = mid_i;
      }
    }

    c_low_i = 1;
    low_ip1 = 2;
    high_i = len_3;
    while (high_i > low_ip1) {
      mid_i = (c_low_i + high_i) >> 1;
      if (varargin_7 >= varargin_3[mid_i - 1]) {
        c_low_i = mid_i;
        low_ip1 = mid_i + 1;
      } else {
        high_i = mid_i;
      }
    }

    if (varargin_5 == varargin_1[low_i]) {
      low_i = b_low_i + len_2 * low_i;
      low_ip1 = low_i + (len_2*len_1) * (c_low_i - 1);
      v11 = varargin_4[low_ip1 - 1];
      v21 = varargin_4[low_ip1];
      low_i += (len_2*len_1) * c_low_i;
      v12 = varargin_4[low_i - 1];
      Vq = varargin_4[low_i];
    } else {
      r = varargin_1[low_i + 1];
      if (varargin_5 == r) {
        low_i = b_low_i + len_2 * (low_i + 1);
        low_ip1 = low_i + (len_2*len_1) * (c_low_i - 1);
        v11 = varargin_4[low_ip1 - 1];
        v21 = varargin_4[low_ip1];
        low_i += (len_2*len_1) * c_low_i;
        v12 = varargin_4[low_i - 1];
        Vq = varargin_4[low_i];
      } else {
        r = (varargin_5 - varargin_1[low_i]) / (r - varargin_1[low_i]);
        if (varargin_4[((b_low_i + len_2 * low_i) + (len_2*len_1) * (c_low_i - 1)) - 1] ==
            varargin_4[((b_low_i + len_2 * (low_i + 1)) + (len_2*len_1) * (c_low_i - 1)) -
            1]) {
          v11 = varargin_4[((b_low_i + len_2 * low_i) + (len_2*len_1) * (c_low_i - 1)) - 1];
        } else {
          v11 = (1.0 - r) * varargin_4[((b_low_i + len_2 * low_i) + (len_2*len_1) *
            (c_low_i - 1)) - 1] + r * varargin_4[((b_low_i + len_2 * (low_i + 1))
            + (len_2*len_1) * (c_low_i - 1)) - 1];
        }

        if (varargin_4[(b_low_i + len_2 * low_i) + (len_2*len_1) * (c_low_i - 1)] ==
            varargin_4[(b_low_i + len_2 * (low_i + 1)) + (len_2*len_1) * (c_low_i - 1)]) {
          v21 = varargin_4[(b_low_i + len_2 * low_i) + (len_2*len_1) * (c_low_i - 1)];
        } else {
          v21 = (1.0 - r) * varargin_4[(b_low_i + len_2 * low_i) + (len_2*len_1) * (c_low_i
            - 1)] + r * varargin_4[(b_low_i + len_2 * (low_i + 1)) + (len_2*len_1) *
            (c_low_i - 1)];
        }

        if (varargin_4[((b_low_i + len_2 * low_i) + (len_2*len_1) * c_low_i) - 1] ==
            varargin_4[((b_low_i + len_2 * (low_i + 1)) + (len_2*len_1) * c_low_i) - 1]) {
          v12 = varargin_4[((b_low_i + len_2 * low_i) + (len_2*len_1) * c_low_i) - 1];
        } else {
          v12 = (1.0 - r) * varargin_4[((b_low_i + len_2 * low_i) + (len_2*len_1) * c_low_i)
            - 1] + r * varargin_4[((b_low_i + len_2 * (low_i + 1)) + (len_2*len_1) *
            c_low_i) - 1];
        }

        if (varargin_4[(b_low_i + len_2 * low_i) + (len_2*len_1) * c_low_i] == varargin_4
            [(b_low_i + len_2 * (low_i + 1)) + (len_2*len_1) * c_low_i]) {
          Vq = varargin_4[(b_low_i + len_2 * low_i) + (len_2*len_1) * c_low_i];
        } else {
          Vq = (1.0 - r) * varargin_4[(b_low_i + len_2 * low_i) + (len_2*len_1) * c_low_i]
            + r * varargin_4[(b_low_i + len_2 * (low_i + 1)) + (len_2*len_1) * c_low_i];
        }
      }
    }

    r = varargin_2[b_low_i - 1];
    if (varargin_6 == r) {
      v21 = v11;
      Vq = v12;
    } else {
      if (!(varargin_6 == varargin_2[b_low_i])) {
        r = (varargin_6 - r) / (varargin_2[b_low_i] - r);
        if (v11 == v21) {
          v21 = v11;
        } else {
          v21 = (1.0 - r) * v11 + r * v21;
        }

        if (v12 == Vq) {
          Vq = v12;
        } else {
          Vq = (1.0 - r) * v12 + r * Vq;
        }
      }
    }

    r = varargin_3[c_low_i - 1];
    if ((varargin_7 == r) || (v21 == Vq)) {
      Vq = v21;
    } else {
      if (!(varargin_7 == varargin_3[c_low_i])) {
        r = (varargin_7 - r) / (varargin_3[c_low_i] - r);
        Vq = (1.0 - r) * v21 + r * Vq;
      }
    }
  } else {
    Vq = nan("");
  }

  return Vq;
}
