/*
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening)
 */

#pragma once

#include <algorithm>
#include <array>
#include <bitset>
#include <iterator>
#include <string>
#include <type_traits>
#include <vector>

#include "libbase/misc_types.h"

namespace util {
/**
 * Copies an array to another one
 *
 * @param src Source (C-style) array
 * @param dest Destination (C-style) array
 * @param size Size (i.e. number of elements) of the arrays
 */
[[deprecated("use std::array version instead")]]
void CopyByteArray(const Byte &src, Byte *dest, const size_t size);
/**
 * Copies a byte array from its pointer to a C++11-style byte array
 *
 * @tparam size Size (i.e. number of elements) of the arrays
 * @param src Source (C++11-style) array
 * @param dest Destination (C++11-style) array
 */
template<size_t size>
void CopyByteArray(const Byte &src, std::array<Byte, size> *dest);

/**
 * Converts a byte array to a C-style 1D bit array
 *
 * @param byte_arr Source byte (C-style) array
 * @param bit_arr Destination 1D bit (C-style) array
 * @param size Size (i.e. number of elements) of @c src array
 */
[[deprecated("use std::array to std::array version instead")]]
void ByteTo1DBitArray(const Byte &src, bool *dest, const size_t size);
/**
 * Converts a byte array to a C++11-style 1D bit array
 *
 * @tparam size Size of @c src array
 * @param src Source byte (C-style) array
 * @param dest Destination 1D bit (C++11-style) array
 */
template<size_t size>
[[deprecated("use std::array to std::array version instead")]]
void ByteTo1DBitArray(const Byte &src, std::array<bool, size> *dest);
/**
 * Converts a byte array to a C++11-style 1D bit array
 *
 * @tparam size Size of @c dest array
 * @param src Source byte (C++11-style) array
 * @param dest Destination 1D bit (C++11-style) array
 */
template<size_t size>
void ByteTo1DBitArray(const std::array<Byte, size / 8> &src, std::array<bool, size> *dest);

/**
 * Converts a byte array to a C++11-style 2D bit array
 *
 * @tparam width Width of the array (size of the interior array)
 * @tparam height Height of the array (size of the exterior array)
 * @param src Source byte (C-style) array
 * @param dest Destination bit (C++11-style) array
 */
template<size_t width, size_t height>
[[deprecated("use std::array to std::array version instead")]]
void ByteTo2DBitArray(const Byte &src, std::array<std::array<bool, width>, height> *dest);
/**
 * Converts a byte array to a C++11-style 2D bit array
 *
 * @tparam width Width of the array (size of the interior array)
 * @tparam height Height of the array (size of the exterior array)
 * @param src Source byte (C++11-style) array
 * @param dest Destination bit (C++11-style) array
 */
template<size_t width, size_t height>
void ByteTo2DBitArray(const std::array<Byte, width * height / 8> &src,
                      std::array<std::array<bool, width>, height> *dest);

/**
 * Retrieves the bit value from a byte array, given a 1D coordinate.
 *
 * @note No bounds checking is provided.
 *
 * @param byte_arr Source byte (C-style) array
 * @param pos Index of the bit
 * @return Bit value
 */
[[deprecated("use std::array version instead")]]
bool GetBitValue(const Byte &byte_arr, const size_t pos);
/**
 * Retrieves the bit value from a byte array, given a 1D coordinate.
 *
 * @tparam size Size of the byte array
 * @param byte_arr Source byte (C++11-style) array
 * @param pos Index of the bit
 * @return False if index out of bounds. Otherwise the bit value.
 */
template<size_t size>
bool GetBitValue(const std::array<Byte, size> &byte_arr, const size_t pos);

/**
 * Retrieves the bit value from a byte array, given a 2D coordinate.
 *
 * @param byte_arr Source byte (C-style) array
 * @param x_size Length of the row
 * @param x Column index
 * @param y Row index
 * @return False if index out of bounds. Otherwise the bit value.
 */
[[deprecated("use std::array version instead")]]
bool GetBitValue(const Byte &byte_arr, const size_t x_size, const size_t x, const size_t y);
/**
 * Retrieves the bit value from a byte array, given a 2D coordinate.
 *
 * @tparam size Size of the byte array
 * @param byte_arr Source byte (C++11-style) array
 * @param x_size Length of the row
 * @param x Column index
 * @param y Row index
 * @return False if index out of bounds. Otherwise the bit value.
 */
template<size_t size>
bool GetBitValue(const std::array<Byte, size> &byte_arr, const size_t x_size, const size_t x, const size_t y);

/**
 * Applies median filter to a C++11-style 2D bit array
 *
 * @tparam width Width of the array (size of the interior array)
 * @tparam height Height of the array (size of the exterior array)
 * @param src Source bit (C++11-style) array
 * @param dest Destination bit (C++11-style) array
 */
template<size_t width, size_t height>
void MedianFilter(const std::array<std::array<bool, width>, height> &src,
                  std::array<std::array<bool, width>, height> *dest);
/**
 * Applies median filter to a C++11-style 2D bit array
 *
 * @tparam width Width of the array (size of the interior array)
 * @tparam height Height of the array (size of the exterior array)
 * @param arr Bit array (C++11-style) to apply filter to
 */
template<size_t width, size_t height>
void MedianFilter(std::array<std::array<bool, width>, height> *arr);

/**
 * Calculates the slope of the linear regression line from a given set of points.
 *
 * @param x Array of x values
 * @param y Array of y values
 * @param size Size of the arrays
 * @return Slope of regression line
 */
[[deprecated("use std::vector version instead")]]
float CalcLinearRegressionSlope(const int *x, const int *y, size_t size);
/**
 * Calculates the slope of the linear regression line from a given set of points.
 *
 * @note If vector sizes do not match, function will return @c inf.
 *
 * @param x Vector of x values
 * @param y Vector of y values
 * @return Slope of regression line
 */
float CalcLinearRegressionSlope(const std::vector<int> &x, const std::vector<int> &y);

namespace distortion {
/**
 * Distortion constants of the intrinsic matrix used in @p GetUndistortCoord.
 *
 * @note These vaules should be obtained using some camera calibration software,
 * for example OpenCV and MATLAB.
 *
 * In matrix form, the constants are arranged as follows:
 * [f_x] [ s ] [c_x]
 * [ 0 ] [f_y] [c_y]
 * [ 0 ] [ 0 ] [ 1 ]
 */
struct {
  float f_x;  // x-axis focal length (px)
  float f_y;  // y-axis focal length (px)
  float s;  // axis skew
  float c_x;  // x-axis principal offset
  float c_y;  // y-axis principal offset
} DistortionConstants;

/**
 * Calculates the undistorted coordinates using the @c DistortionConstants
 *
 * @param coords Vector of arrays storing the x and y coordinates.
 * The first and second elements in the array should be the x and y
 * coordinates respectively.
 */
void GetUndistortCoord(std::vector<std::array<int, 2>> coords);
/**
 * Calculates the undistorted coordinates using the @c DistortionConstants
 *
 * @note If vector sizes do not match, function will return without altering
 * the vectors.
 *
 * @param x Vector of x values
 * @param y Vector of y values
 */
void GetUndistortCoord(std::vector<int> *x, std::vector<int> *y);
}  // namespace distortion

#if __cplusplus < 201402L
/**
 * Backport @c std::enable_if_t from C++14 if we're not compiling with it
 */
template<bool B, class T = void>
using enable_if_t = typename std::enable_if<B, T>::type;
#else
using std::enable_if_t;
#endif

/**
 * Finds a specified element in an integer-typed C-style array. Searches indices @c [start,end].
 *
 * @note @p start and @p end can be flipped to iterate the array backwards.
 * @note Function does not include bounds checking.
 *
 * @tparam T An integer primitive type
 * @param arr Data array
 * @param first Starting index
 * @param last Ending index
 * @param value The value to find
 * @param return_last If @c true, returns last element if value is not found. Otherwise, returns -1.
 * @return Index of first matching element if found. Otherwise dependent on @p return_last.
 */
template<class T, typename = enable_if_t<std::is_integral<T>::value>>
[[deprecated("use std::array version instead")]]
int FindElement(const T &arr, int first, int last, T value, bool return_last = true);

/**
 * Finds a specified element in an integer-typed C++ standard array. Searches indices @c [start,end].
 *
 * @note @p start and @p end can be flipped to iterate the array backwards.
 * @note Function does not include bounds checking.
 *
 * @tparam T An integer primitive type
 * @param arr Data array
 * @param first Starting index
 * @param last Ending index
 * @param value The value to find
 * @param return_last If @c true, returns last element if value is not found. Otherwise, returns -1.
 * @return Index of first matching element if found. Otherwise dependent on @p return_last.
 */
template<class T, typename = enable_if_t<std::is_integral<T>::value>, std::size_t size>
int FindElement(const std::array<T, size> &arr, int first, int last, T value, bool return_last = true);
}  // namespace util

#include "util/util.tcc"
