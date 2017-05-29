/*
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening)
 *
 * Utility functions to avoid repetition of code.
 *
 */

#ifndef CHASING17_UTIL_UTIL_H_
#define CHASING17_UTIL_UTIL_H_

#include <algorithm>
#include <array>
#include <bitset>
#include <iterator>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "libbase/misc_types.h"

namespace util {
/**
 * Copies a byte array from its pointer to a C++11-style byte array
 *
 * @tparam size Size (i.e. number of elements) of the arrays
 * @param src Source (C++11-style) array
 * @param dest Destination (C++11-style) array
 */
template<size_t size>
void CopyByteArray(const Byte& src, std::array<Byte, size>* dest);

/**
 * Converts a byte array to a C++11-style 1D bit array
 *
 * @tparam size Size of @c dest array
 * @param src Source byte (C++11-style) array
 * @param dest Destination 1D bit (C++11-style) array
 */
template<size_t size>
void ByteTo1DBitArray(const std::array<Byte, size / 8>& src, std::array<bool, size>* dest);

/**
 * Converts a byte array to a C++11-style 2D bit array
 *
 * @tparam width Width of the array (size of the interior array)
 * @tparam height Height of the array (size of the exterior array)
 * @param src Source byte (C++11-style) array
 * @param dest Destination bit (C++11-style) array
 */
template<size_t width, size_t height>
void ByteTo2DBitArray(const std::array<Byte, width * height / 8>& src,
                      std::array<std::array<bool, width>, height>* dest);

/**
 * Retrieves the bit value from a byte array, given a 1D coordinate.
 *
 * @tparam size Size of the byte array
 * @param byte_arr Source byte (C++11-style) array
 * @param pos Index of the bit
 * @return False if index out of bounds. Otherwise the bit value.
 */
template<size_t size>
bool GetBitValue(const std::array<Byte, size>& byte_arr, const size_t pos);

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
bool GetBitValue(const std::array<Byte, size>& byte_arr, const size_t x_size, const size_t x, const size_t y);

/**
 * Applies median filter to a C++11-style 2D bit array
 *
 * @tparam width Width of the array (size of the interior array)
 * @tparam height Height of the array (size of the exterior array)
 * @param src Source bit (C++11-style) array
 * @param dest Destination bit (C++11-style) array
 */
template<size_t width, size_t height>
void MedianFilter(const std::array<std::array<bool, width>, height>& src,
                  std::array<std::array<bool, width>, height>* dest);
/**
 * Applies median filter to a C++11-style 2D bit array
 *
 * @tparam width Width of the array (size of the interior array)
 * @tparam height Height of the array (size of the exterior array)
 * @param arr Bit array (C++11-style) to apply filter to
 */
template<size_t width, size_t height>
void MedianFilter(std::array<std::array<bool, width>, height>* arr);

/**
 * Calculates the slope of the linear regression line from a given set of points.
 *
 * @note If vector sizes do not match, function will return @c inf.
 *
 * @param x Vector of x values
 * @param y Vector of y values
 * @return Slope of regression line
 */
float CalcLinearRegressionSlope(const std::vector<int>& x, const std::vector<int>& y);

/**
 * Converts an uint16_t to an array with 2 bytes.
 *
 * @param num The uint16_t number
 * @param bytes Destination byte (C++11-style) array
 */
void Int16To2ByteArray(const uint16_t num, std::array<Byte, 2>& bytes);

/**
 * Sends an image from the camera via Bluetooth every 5 seconds.
 *
 * @note The function must be called after @code System::Init() @endcode
 */
void BtSendImage();

#if __cplusplus < 201402L
/**
 * Backport @c std::enable_if_t from C++14 if we're not compiling with it
 */
template<bool B, class T = void>
using enable_if_t = typename std::enable_if<B, T>::type;

/**
 * Backport of std::make_unique from C++14
 */
template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&& ... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}
#else
using std::enable_if_t;
using std::make_unique;
#endif

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
int FindElement(const std::array<T, size>& arr, int first, int last, T value, bool return_last = true);
}  // namespace util

#include "util/util.tcc"

#endif
