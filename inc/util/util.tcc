/*
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening)
 */

#include "util/util.h"

namespace util {
template<std::size_t size>
void CopyByteArray(const Byte &src, std::array<Byte, size> *dest) {
  for (std::size_t i = 0; i < size; ++i) {
    dest->at(i) = (&src)[i];
  }
}

template<std::size_t size>
void ByteTo1DBitArray(const std::array<Byte, size / 8> &src, std::array<bool, size> *dest) {
  for (std::size_t i = 0; i < src.size(); ++i) {
    for (uint8_t j = 0; j < 8; ++j) {
      dest->at(i * 8 + j) = (src.at(i) >> (7 - j)) & 1;
    }
  }
}

template<std::size_t width, std::size_t height>
void ByteTo2DBitArray(const std::array<Byte, width * height / 8> &src,
                      std::array<std::array<bool, width>, height> *dest) {
  for (std::size_t i = 0; i < src.size(); ++i) {
    for (int j = 0; j < 8; ++j) {
      dest->at(i * 8 / width).at((i * 8 % width) + (7 - j)) = (src.at(i) >> (7 - j)) & 1;
    }
  }
}

template<std::size_t size>
bool GetBitValue(const std::array<Byte, size> &a, const std::size_t pos) {
  if (pos >= size * 8) {
    return false;
  }
  return (a.at(pos / 8) >> (7 - pos % 8)) & 1;
}

template<std::size_t size>
bool GetBitValue(const std::array<Byte, size> &a, const std::size_t x_size, const std::size_t x, const std::size_t y) {
  if (x >= x_size) {
    return false;
  }
  std::size_t pos = y * x_size + x;
  return (a.at(pos / 8) >> (7 - pos % 8)) & 1;
}

template<std::size_t width, std::size_t height>
void MedianFilter(const std::array<std::array<bool, width>, height> &src,
                  std::array<std::array<bool, width>, height> *dest) {
  for (Uint i = 0; i < height; ++i) {
    for (Uint j = 0; j < width; ++j) {
      if (i == 0) {
        if (j == 0) {
          dest->at(i).at(j) = static_cast<bool>((
              src.at(i).at(j) +
                  src.at(i).at(j + 1) +
                  src.at(i + 1).at(j) +
                  src.at(i + 1).at(j + 1))
              / 3);
        } else if (j == width - 1) {
          dest->at(i).at(j) = static_cast<bool>((
              src.at(i).at(j - 1) +
                  src.at(i).at(j) +
                  src.at(i + 1).at(j - 1) +
                  src.at(i + 1).at(j))
              / 3);
        } else {
          dest->at(i).at(j) = static_cast<bool>((
              src.at(i).at(j - 1) +
                  src.at(i).at(j) +
                  src.at(i).at(j + 1) +
                  src.at(i + 1).at(j - 1) +
                  src.at(i + 1).at(j) +
                  src.at(i + 1).at(j + 1))
              / 4);
        }
      } else if (i == height - 1) {
        if (j == 0) {
          dest->at(i).at(j) = static_cast<bool>((
              src.at(i - 1).at(j) +
                  src.at(i - 1).at(j + 1) +
                  src.at(i).at(j) +
                  src.at(i).at(j + 1))
              / 3);
        } else if (j == width - 1) {
          dest->at(i).at(j) = static_cast<bool>((
              src.at(i - 1).at(j - 1) +
                  src.at(i - 1).at(j) +
                  src.at(i).at(j - 1) +
                  src.at(i).at(j))
              / 3);
        } else {
          dest->at(i).at(j) = static_cast<bool>((
              src.at(i - 1).at(j - 1) +
                  src.at(i - 1).at(j) +
                  src.at(i - 1).at(j + 1) +
                  src.at(i).at(j - 1) +
                  src.at(i).at(j) +
                  src.at(i).at(j + 1))
              / 4);
        }
      } else {
        if (j == 0) {
          dest->at(i).at(j) = static_cast<bool>((
              src.at(i - 1).at(j) +
                  src.at(i - 1).at(j + 1) +
                  src.at(i).at(j) +
                  src.at(i).at(j + 1) +
                  src.at(i + 1).at(j) +
                  src.at(i + 1).at(j + 1))
              / 4);
        } else if (j == width - 1) {
          dest->at(i).at(j) = static_cast<bool>((
              src.at(i - 1).at(j - 1) +
                  src.at(i - 1).at(j) +
                  src.at(i).at(j - 1) +
                  src.at(i).at(j) +
                  src.at(i + 1).at(j - 1) +
                  src.at(i + 1).at(j))
              / 4);
        } else {
          dest->at(i).at(j) = static_cast<bool>((
              src.at(i - 1).at(j - 1) +
                  src.at(i - 1).at(j) +
                  src.at(i - 1).at(j + 1) +
                  src.at(i).at(j - 1) +
                  src.at(i).at(j) +
                  src.at(i).at(j + 1) +
                  src.at(i + 1).at(j - 1) +
                  src.at(i + 1).at(j) +
                  src.at(i + 1).at(j + 1))
              / 5);
        }
      }
    }
  }
}

template<std::size_t width, std::size_t height>
void MedianFilter(std::array<std::array<bool, width>, height> *arr) {
  std::array<std::array<bool, width>, height> tmp{};
  for (Uint i = 0; i < height; ++i) {
    for (Uint j = 0; j < width; ++j) {
      if (i == 0) {
        if (j == 0) {
          tmp.at(i).at(j) = static_cast<bool>((
              arr->at(i).at(j) +
                  arr->at(i).at(j + 1) +
                  arr->at(i + 1).at(j) +
                  arr->at(i + 1).at(j + 1))
              / 3);
        } else if (j == width - 1) {
          tmp.at(i).at(j) = static_cast<bool>((
              arr->at(i).at(j - 1) +
                  arr->at(i).at(j) +
                  arr->at(i + 1).at(j - 1) +
                  arr->at(i + 1).at(j))
              / 3);
        } else {
          tmp.at(i).at(j) = static_cast<bool>((
              arr->at(i).at(j - 1) +
                  arr->at(i).at(j) +
                  arr->at(i).at(j + 1) +
                  arr->at(i + 1).at(j - 1) +
                  arr->at(i + 1).at(j) +
                  arr->at(i + 1).at(j + 1))
              / 4);
        }
      } else if (i == height - 1) {
        if (j == 0) {
          tmp.at(i).at(j) = static_cast<bool>((
              arr->at(i - 1).at(j) +
                  arr->at(i - 1).at(j + 1) +
                  arr->at(i).at(j) +
                  arr->at(i).at(j + 1))
              / 3);
        } else if (j == width - 1) {
          tmp.at(i).at(j) = static_cast<bool>((
              arr->at(i - 1).at(j - 1) +
                  arr->at(i - 1).at(j) +
                  arr->at(i).at(j - 1) +
                  arr->at(i).at(j))
              / 3);
        } else {
          tmp.at(i).at(j) = static_cast<bool>((
              arr->at(i - 1).at(j - 1) +
                  arr->at(i - 1).at(j) +
                  arr->at(i - 1).at(j + 1) +
                  arr->at(i).at(j - 1) +
                  arr->at(i).at(j) +
                  arr->at(i).at(j + 1))
              / 4);
        }
      } else {
        if (j == 0) {
          tmp.at(i).at(j) = static_cast<bool>((
              arr->at(i - 1).at(j) +
                  arr->at(i - 1).at(j + 1) +
                  arr->at(i).at(j) +
                  arr->at(i).at(j + 1) +
                  arr->at(i + 1).at(j) +
                  arr->at(i + 1).at(j + 1))
              / 4);
        } else if (j == width - 1) {
          tmp.at(i).at(j) = static_cast<bool>((
              arr->at(i - 1).at(j - 1) +
                  arr->at(i - 1).at(j) +
                  arr->at(i).at(j - 1) +
                  arr->at(i).at(j) +
                  arr->at(i + 1).at(j - 1) +
                  arr->at(i + 1).at(j))
              / 4);
        } else {
          tmp.at(i).at(j) = static_cast<bool>((
              arr->at(i - 1).at(j - 1) +
                  arr->at(i - 1).at(j) +
                  arr->at(i - 1).at(j + 1) +
                  arr->at(i).at(j - 1) +
                  arr->at(i).at(j) +
                  arr->at(i).at(j + 1) +
                  arr->at(i + 1).at(j - 1) +
                  arr->at(i + 1).at(j) +
                  arr->at(i + 1).at(j + 1))
              / 5);
        }
      }
    }
  }
  *arr = tmp;
}

template<class T, typename = enable_if_t <std::is_integral<T>::value>, std::size_t size>
int FindElement(const std::array<T, size> &arr, int first, int last, T value, bool return_last) {
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
