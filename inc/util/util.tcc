/*
 * util.tcc
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening)
 *
 * Template implementations for util.h
 *
 */

#include "util/util.h"

#include <cstring>

namespace util {
template<std::size_t size>
void CopyByteArray(const Byte* const src, std::array<Byte, size>& dest) {
  std::memcpy(dest.data(), src, size);
}

template<std::size_t size>
bool GetBitValue(const std::array<Byte, size>& a, const std::size_t pos) {
  if (pos >= size * 8) {
    return false;
  }
  return (a.at(pos / 8) >> (7 - pos % 8)) & 1;
}

template<std::size_t size>
bool GetBitValue(const std::array<Byte, size>& a, const std::size_t x_size, const std::size_t x, const std::size_t y) {
  if (x >= x_size) {
    return false;
  }
  std::size_t pos = y * x_size + x;
  return (a.at(pos / 8) >> (7 - pos % 8)) & 1;
}

template<class T, typename = enable_if_t <std::is_integral<T>::value>, std::size_t size>
int FindElement(const std::array<T, size>& arr, int first, int last, T value, bool return_last) {
  if (last > first) {
    for (; first <= last; ++first) {
      if (arr.at(first) == value) {
        return first;
      }
    }
  } else if (first > last) {
    for (; first >= last; --first) {
      if (arr.at(first) == value) {
        return first;
      }
    }
  } else if (first == last) {
    if (arr.at(first) == value) {
      return first;
    }
    return return_last ? last : -1;
  }
  return return_last ? last : -1;
}
}  // namespace util
