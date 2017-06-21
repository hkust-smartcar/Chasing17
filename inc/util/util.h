/*
 * util.h
 *
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
#include "libsc/lcd_console.h"

namespace util {
/**
 * Copies a byte array from its pointer to a C++11-style byte array
 *
 * @tparam size Size (i.e. number of elements) of the arrays
 * @param src Source (C++11-style) array
 * @param dest Destination (C++11-style) array
 */
template<size_t size>
void CopyByteArray(const Byte* const src, std::array<Byte, size>& dest);

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

/**
 * Extension function for LcdConsole::WriteString for std::string.
 *
 * @param console Pointer to console object
 * @param s String to be sent
 */
void ConsoleWriteString(libsc::LcdConsole* const console, const std::string& s);

/**
 * Extension function for LcdConsole::ClearRow for one row only
 *
 * @param console Pointer to console object
 * @param row Row of text to clear
 */
void ConsoleClearRow(libsc::LcdConsole* const console, const uint8_t row);

std::string to_string(int val);
std::string to_string(unsigned val);
std::string to_string(long val);
std::string to_string(unsigned long val);
std::string to_string(long long val);
std::string to_string(unsigned long long val);
std::string to_string(float val);
std::string to_string(double val);
std::string to_string(long double val);

#if __cplusplus > 201103L
using std::make_unique;
#else

/**
 * Backport of std::make_unique from C++14
 */
template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&& ... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

#endif  // __cplusplus > 201103L (check for C++14 support)

#if __cplusplus > 201402L
using std::clamp;
#else

/**
 * Backport of std::clamp from C++17
 *
 * @tparam T Any value type with @c operator<
 * @param v Value to be clamped
 * @param lo Lower limit of the value
 * @param hi Higher limit of the value
 * @return Clamped value
 */
template<class T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi) {
  return (v < lo) ? lo : (hi < v) ? hi : v;
}

#endif  // __cplusplus > 201402L (check for C++17 support)

}  // namespace util

#include "util/util.tcc"

#endif
