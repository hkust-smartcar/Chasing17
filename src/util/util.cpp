/*
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening)
 */

#include "util/util.h"

#include <string>

using std::bitset;
using std::string;

namespace util {
void CopyByteArray(const Byte &src, Byte *dest, const size_t size) {
  for (size_t i = 0; i < size; ++i) {
    dest[i] = (&src)[i];
  }
}

void ByteTo1DBitArray(const Byte &src, bool *dest, const size_t size) {
  for (size_t i = 0; i < size / 8; ++i) {
    for (uint8_t j = 0; j < 8; ++j) {
      dest[i * 8 + j] = ((&src)[i] >> (7 - j)) & 1;
    }
  }
}

bool GetBitValue(const Byte &a, const size_t pos) {
  return ((&a)[pos / 8] >> (7 - pos % 8)) & 1;
}

bool GetBitValue(const Byte &a, const size_t x_size, const size_t x, const size_t y) {
  if (x >= x_size) {
    return false;
  }
  size_t pos = y * x_size + x;
  return ((&a)[pos / 8] >> (7 - pos % 8)) & 1;
}

float CalcLinearRegressionSlope(const int *x, const int *y, size_t size) {
  int lhs_matrix[2][2]{};
  int rhs_matrix[2]{};

  // least squares method approximation
  for (unsigned int i = 0; i < size; ++i) {
    lhs_matrix[0][0] += (x[i] * x[i]);
    lhs_matrix[0][1] += x[i];
    lhs_matrix[1][0] += x[i];
    lhs_matrix[1][1] += 1;
    rhs_matrix[0] += (x[i] * y[i]);
    rhs_matrix[1] += y[i];
  }

  // cramer's rule
  float det = lhs_matrix[0][0] * lhs_matrix[1][1] - lhs_matrix[1][0] * lhs_matrix[0][1];
  float m = (rhs_matrix[0] * lhs_matrix[1][1] - lhs_matrix[0][1] * rhs_matrix[1]) / det;

  return m;
}

float CalcLinearRegressionSlope(const std::vector<int> &x, const std::vector<int> &y) {
  if (x.size() != y.size()) {
    return std::numeric_limits<float>::infinity();
  }
  std::array<std::array<int, 2>, 2> lhs_matrix{};
  std::array<int, 2> rhs_matrix{};

  // least squares method approximation
  for (unsigned int i = 0; i < x.size(); ++i) {
    lhs_matrix.at(0).at(0) += (x.at(i) * x.at(i));
    lhs_matrix.at(0).at(1) += x.at(i);
    lhs_matrix.at(1).at(0) += x.at(i);
    lhs_matrix.at(1).at(1) += 1;
    rhs_matrix.at(0) += (x.at(i) * y.at(i));
    rhs_matrix.at(1) += y.at(i);
  }

  // cramer's rule
  float det = (lhs_matrix.at(0).at(0) * lhs_matrix.at(1).at(1)) - (lhs_matrix.at(1).at(0) * lhs_matrix.at(0).at(1));
  float m = ((rhs_matrix.at(0) * lhs_matrix.at(1).at(1)) - (lhs_matrix.at(0).at(1) * rhs_matrix.at(1))) / det;

  return m;
}

namespace distortion {
void GetUndistortCoord(std::vector<std::array<int, 2>> *v) {
  for (std::array<int, 2> a : *v) {
    a.at(0) =
        static_cast<int>(DistortionConstants.f_x * a.at(0) + DistortionConstants.s * a.at(1) + DistortionConstants.c_x);
    a.at(1) = static_cast<int>(DistortionConstants.f_y * a.at(1) + DistortionConstants.c_y);
  }
}

void GetUndistortCoord(std::vector<int> *x, std::vector<int> *y) {
  if (x->size() != y->size()) {
    return;
  }
  for (size_t i = 0; i < x->size(); ++i) {
    x->at(i) =
        static_cast<int>(DistortionConstants.f_x * x->at(i) + DistortionConstants.s * y->at(i) + DistortionConstants.c_x);
    y->at(i) = static_cast<int>(DistortionConstants.f_y * y->at(i) + DistortionConstants.c_y);
  }
}
}  // namespace distortion
}  // namespace util
