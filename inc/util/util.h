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
