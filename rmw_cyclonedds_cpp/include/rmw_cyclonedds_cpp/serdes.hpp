// Copyright 2018 to 2019 ADLINK Technology
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RMW_CYCLONEDDS_CPP__SERDES_HPP_
#define RMW_CYCLONEDDS_CPP__SERDES_HPP_

#include <inttypes.h>
#include <stdarg.h>
#include <string.h>

#include <algorithm>
#include <array>
#include <cassert>
#include <string>
#include <type_traits>
#include <vector>

#include "rmw_cyclonedds_cpp/deserialization_exception.hpp"

using rmw_cyclonedds_cpp::DeserializationException;

class cycser
{
public:
  explicit cycser(std::vector<unsigned char> & dst_);
  cycser() = delete;

  template <typename T>
  cycser & operator<<(const T & x)
  {
    serialize(x);
    return *this;
  }

  template <typename T>
  void serialize(T x)
  {
    if ((off % sizeof(T)) != 0) {
      off += sizeof(T) - (off % sizeof(T));
    }
    resize(off + sizeof(T));
    *(reinterpret_cast<T *>(data() + off)) = x;
    off += sizeof(T);
  }

  inline void serialize(bool x) { serialize(static_cast<unsigned char>(x)); }

  inline void serialize(const std::string & x) { serialize_list(x.c_str(), x.size() + 1); }

  inline void serialize(const std::wstring & x)
  {
    serialize_list(
      x.c_str(),
      x.size());  // todo: is the size right or do we need to include the null terminator?
  }

  template <typename T>
  void serialize_many(const T * x, size_t cnt)
  {
    if (cnt > 0) {
      if ((off % sizeof(T)) != 0) {
        off += sizeof(T) - (off % sizeof(T));
      }
      resize(off + cnt * sizeof(T));
      memcpy(data() + off, reinterpret_cast<const void *>(x), cnt * sizeof(T));
      off += cnt * sizeof(T);
    }
  }

  template <class T>
  inline void serialize(const std::vector<T> & x)
  {
    serialize_list(x.data(), x.size());
  }
  inline void serialize(const std::vector<bool> & x)
  {
    serialize(static_cast<uint32_t>(x.size()));
    for (bool i : x) {
      serialize(i);
    }
  }

  template <class T>
  inline void serialize_list(const T * x, size_t cnt)
  {
    serialize(static_cast<uint32_t>(cnt));
    serialize_many(x, cnt);
  }

private:
  inline void resize(size_t n) { dst.resize(n + 4); }
  inline unsigned char * data() { return dst.data() + 4; }

  std::vector<unsigned char> & dst;
  size_t off;
};

class cycdeserbase
{
public:
  explicit cycdeserbase(const char * data_, size_t lim_);
  cycdeserbase() = delete;

protected:
  template <typename T>
  T maybe_bswap(T value)
  {
      auto bytes = reinterpret_cast<unsigned char *>(value);
      for (auto i = 0; i < sizeof(T) / 2; i++) {
          std::swap(bytes[i], bytes[sizeof(T) - 1 - i]);
      }
      return value;
  }
  inline void align(size_t a)
  {
    if ((pos % a) != 0) {
      pos += a - (pos % a);
      if (pos > lim) {
        throw DeserializationException("invalid data size");
      }
    }
  }
  inline void validate_size(size_t count, size_t sz)
  {
    assert(sz == 1 || sz == 2 || sz == 4 || sz == 8);
    if (count > (lim - pos) / sz) {
      throw DeserializationException("invalid data size");
    }
  }
  inline void validate_str(size_t sz)
  {
    if (sz > 0 && data[pos + sz - 1] != '\0') {
      throw DeserializationException("string data is not null-terminated");
    }
  }

  const char * data;
  size_t pos;
  size_t lim;
  bool swap_bytes;
};

class cycdeser : cycdeserbase
{

public:
  cycdeser(const void * data, size_t size);
  cycdeser() = delete;

  template <typename T>
  inline cycdeser & operator>>(T && x)
  {
    deserialize(x);
    return *this;
  }

  template <typename T>
  T deserialize()
  {
      static_assert(std::is_integral<T>::value, "Should only be used with integral types");
    align(sizeof(T));
    validate_size(1, sizeof(T));
    T x = maybe_bswap(*reinterpret_cast<const T *>(data + pos));
    pos += sizeof(T);
    return x;
  }

  template <>
  bool deserialize<bool>()
  {
    return (deserialize<unsigned char>() != 0);
  }

  inline size_t deserialize_len(size_t el_sz)
  {
    auto sz = deserialize<uint32_t>();
    validate_size(sz, el_sz);
    return sz;
  }

  template <>
  std::string deserialize<std::string>()
  {
    std::string x;
    const size_t sz = deserialize_len(sizeof(char));
    if (sz == 0) {
      return std::string("");
    } else {
      std::string x(data + pos);
      deserialize_many(std::begin(x), std::end(x));

      // read, validate, and discard the null terminator
      if (deserialize<char>() != '\0') {
        throw DeserializationException("string data is not null-terminated");
      }

      return x;
    }
  }

  template <>
  std::wstring deserialize()
  {
    // wstring is not null-terminated in cdr
    auto vec = deserialize<std::vector<wchar_t>>();
    return std::wstring(vec.begin(), vec.end());
  }

  template <typename OutputIterator>
  void deserialize_many(OutputIterator begin, OutputIterator end)
  {
    while (begin != end) {
      *(begin++) = deserialize<typename OutputIterator::value_type>();
    }
  }

  template<typename T>
  std::vector<T> deserialize_list(){
      std::vector<T> x(deserialize_len(sizeof(T)));
      deserialize_many(x.begin(), x.end());
      return x;
  }

  template <typename T, size_t S>
  std::array<T, S> deserialize_array()
  {
      // todo: does this need a count?
      std::array<T, S> x;
      deserialize_many(x.begin(), x.end());
      return x;
  }
};

class cycprint : cycdeserbase
{
public:
  cycprint(char * buf, size_t bufsize, const void * data, size_t size);
  cycprint() = delete;

  void print_constant(const char * x) { prtf(&buf, &bufsize, "%s", x); }

  inline cycprint & operator>>(bool & x)
  {
    print(x);
    return *this;
  }
  inline cycprint & operator>>(char & x)
  {
    print(x);
    return *this;
  }
  inline cycprint & operator>>(int8_t & x)
  {
    print(x);
    return *this;
  }
  inline cycprint & operator>>(uint8_t & x)
  {
    print(x);
    return *this;
  }
  inline cycprint & operator>>(int16_t & x)
  {
    print(x);
    return *this;
  }
  inline cycprint & operator>>(uint16_t & x)
  {
    print(x);
    return *this;
  }
  inline cycprint & operator>>(int32_t & x)
  {
    print(x);
    return *this;
  }
  inline cycprint & operator>>(uint32_t & x)
  {
    print(x);
    return *this;
  }
  inline cycprint & operator>>(int64_t & x)
  {
    print(x);
    return *this;
  }
  inline cycprint & operator>>(uint64_t & x)
  {
    print(x);
    return *this;
  }
  inline cycprint & operator>>(float & x)
  {
    print(x);
    return *this;
  }
  inline cycprint & operator>>(double & x)
  {
    print(x);
    return *this;
  }
  inline cycprint & operator>>(char *& x)
  {
    print(x);
    return *this;
  }
  inline cycprint & operator>>(std::string & x)
  {
    print(x);
    return *this;
  }
  inline cycprint & operator>>(std::wstring & x)
  {
    print(x);
    return *this;
  }
  template <class T>
  inline cycprint & operator>>(std::vector<T> & x)
  {
    print(x);
    return *this;
  }
  template <class T, size_t S>
  inline cycprint & operator>>(std::array<T, S> & x)
  {
    print(x);
    return *this;
  }

  template <typename T>
  inline void print(T & x)
  {
    align(sizeof(x));
    validate_size(1, sizeof(x));
    x = *reinterpret_cast<const T *>(data + pos);
    x = maybe_bswap(x);

    prtf(&buf, &bufsize, '%', x);
    pos += sizeof(x);
  }

  inline void print(bool & x)
  {
    static_cast<void>(x);
    unsigned char z;
    print(z);
  }
  inline void print(float & x)
  {
    union {
      uint32_t u;
      float f;
    } tmp;
    align(sizeof(x));
    validate_size(1, sizeof(x));
    tmp.u = *reinterpret_cast<const uint32_t *>(data + pos);
    tmp.u = maybe_bswap(tmp.u);
    static_cast<void>(tmp.u);
    prtf(&buf, &bufsize, "%f", tmp.f);
    pos += sizeof(x);
  }
  inline void print(double & x)
  {
    union {
      uint64_t u;
      double f;
    } tmp;
    align(sizeof(x));
    validate_size(1, sizeof(x));
    tmp.u = *reinterpret_cast<const uint64_t *>(data + pos);
    tmp.u = maybe_bswap(tmp.u);
    static_cast<void>(tmp.u);
    prtf(&buf, &bufsize, "%f", tmp.f);
    pos += sizeof(x);
  }
  inline uint32_t get_len(size_t el_sz)
  {
    uint32_t sz;
    align(sizeof(sz));
    validate_size(1, sizeof(sz));
    sz = *reinterpret_cast<const uint32_t *>(data + pos);
    sz = maybe_bswap(sz);

    pos += sizeof(sz);
    validate_size(sz, el_sz);
    return sz;
  }
  inline void print(char *& x)
  {
    const uint32_t sz = get_len(sizeof(char));
    validate_str(sz);
    const int len = (sz == 0) ? 0 : (sz > INT32_MAX) ? INT32_MAX : static_cast<int>(sz - 1);
    static_cast<void>(x);
    prtf(&buf, &bufsize, "\"%*.*s\"", len, len, static_cast<const char *>(data + pos));
    pos += sz;
  }
  inline void print(std::string & x)
  {
    const uint32_t sz = get_len(sizeof(char));
    validate_str(sz);
    const int len = (sz == 0) ? 0 : (sz > INT32_MAX) ? INT32_MAX : static_cast<int>(sz - 1);
    static_cast<void>(x);
    prtf(&buf, &bufsize, "\"%*.*s\"", len, len, static_cast<const char *>(data + pos));
    pos += sz;
  }
  inline void print(std::wstring & x)
  {
    const uint32_t sz = get_len(sizeof(wchar_t));
    // wstring is not null-terminated in cdr
    x = std::wstring(reinterpret_cast<const wchar_t *>(data + pos), sz);
    prtf(&buf, &bufsize, "\"%ls\"", x.c_str());
    pos += sz * sizeof(wchar_t);
  }

  template <class T>
  inline void printA(T * x, size_t cnt)
  {
    prtf(&buf, &bufsize, "{");
    for (size_t i = 0; i < cnt; i++) {
      if (i != 0) {
        prtf(&buf, &bufsize, ",");
      }
      print(*x);
    }
    prtf(&buf, &bufsize, "}");
  }

  template <class T>
  inline void print(std::vector<T> & x)
  {
    const uint32_t sz = get_len(1);
    printA(x.data(), sz);
  }
  template <class T, size_t S>
  inline void print(std::array<T, S> & x)
  {
    printA(x.data(), x.size());
  }

private:
  static bool prtf(char * __restrict * buf, size_t * __restrict bufsize, const char * fmt, ...)
  {
    va_list ap;
    if (*bufsize == 0) {
      return false;
    }
    va_start(ap, fmt);
    int n = vsnprintf(*buf, *bufsize, fmt, ap);
    va_end(ap);
    if (n < 0) {
      **buf = 0;
      return false;
    } else if ((size_t)n <= *bufsize) {
      *buf += (size_t)n;
      *bufsize -= (size_t)n;
      return *bufsize > 0;
    } else {
      *buf += *bufsize;
      *bufsize = 0;
      return false;
    }
  }

  char * buf;
  size_t bufsize;
};

#endif  // RMW_CYCLONEDDS_CPP__SERDES_HPP_
