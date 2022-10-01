#ifndef INTERP3_H
#define INTERP3_H

#include <cstddef>
#include <cstdlib>
#include <math.h>
#include <vector>


extern float interp3(std::vector<float> varargin_1, std::vector<float> varargin_2,
                      std::vector<float> varargin_3, std::vector<float> varargin_4,
                      float varargin_5, float varargin_6, float varargin_7,
                      int len_1, int len_2, int len_3);

#endif