#include "aerodynamics/interp2.h"
#include "aerodynamics/aero_lookup.h"

float interp2(std::vector<float> varargin_1, std::vector<float> varargin_2,
              std::vector<float> varargin_3, float varargin_4, float varargin_5, int len_1, int len_2)
{
  float Vq;
  int low_i;
  int low_ip1;
  int high_i;
  int b_low_i;
  int mid_i;
  float rx;
  float qx1;
  if ((varargin_4 >= varargin_1[0]) && (varargin_4 <= varargin_1[(len_1 - 1)]) &&
      (varargin_5 >= varargin_2[0]) && (varargin_5 <= varargin_2[(len_2 - 1)])) {
    low_i = 0;
    low_ip1 = 2;
    high_i = len_1;
    while (high_i > low_ip1) {
      mid_i = ((low_i + high_i) + 1) >> 1;
      if (varargin_4 >= varargin_1[mid_i - 1]) {
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
      if (varargin_5 >= varargin_2[mid_i - 1]) {
        b_low_i = mid_i;
        low_ip1 = mid_i + 1;
      } else {
        high_i = mid_i;
      }
    }

    if (varargin_4 == varargin_1[low_i]) {
      low_i = b_low_i + len_2 * low_i;
      qx1 = varargin_3[low_i - 1];
      Vq = varargin_3[low_i];
    } else {
      rx = varargin_1[low_i + 1];
      if (varargin_4 == rx) {
        low_i = b_low_i + len_2 * (low_i + 1);
        qx1 = varargin_3[low_i - 1];
        Vq = varargin_3[low_i];
      } else {
        rx = (varargin_4 - varargin_1[low_i]) / (rx - varargin_1[low_i]);
        if (varargin_3[(b_low_i + len_2 * low_i) - 1] == varargin_3[(b_low_i + len_2
             * (low_i + 1)) - 1]) {
          qx1 = varargin_3[(b_low_i + len_2 * low_i) - 1];
        } else {
          qx1 = (1.0 - rx) * varargin_3[(b_low_i + len_2 * low_i) - 1] + rx *
            varargin_3[(b_low_i + len_2 * (low_i + 1)) - 1];
        }

        if (varargin_3[b_low_i + len_2 * low_i] == varargin_3[b_low_i + len_2 *
            (low_i + 1)]) {
          Vq = varargin_3[b_low_i + len_2 * low_i];
        } else {
          Vq = (1.0 - rx) * varargin_3[b_low_i + len_2 * low_i] + rx *
            varargin_3[b_low_i + len_2 * (low_i + 1)];
        }
      }
    }

    rx = varargin_2[b_low_i - 1];
    if ((varargin_5 == rx) || (qx1 == Vq)) {
      Vq = qx1;
    } else {
      if (!(varargin_5 == varargin_2[b_low_i])) {
        rx = (varargin_5 - rx) / (varargin_2[b_low_i] - rx);
        Vq = (1.0 - rx) * qx1 + rx * Vq;
      }
    }
  } else {
    Vq = nan("");
  }

  return Vq;
}
