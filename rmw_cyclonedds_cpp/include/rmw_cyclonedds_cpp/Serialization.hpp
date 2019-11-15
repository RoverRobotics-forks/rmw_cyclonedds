//
// Created by dan on 11/13/19.
//

#ifndef ROS2_MASTER_SERIALIZATION_HPP
#define ROS2_MASTER_SERIALIZATION_HPP
#include "TypeSupport2.hpp"
#include "dds/ddsrt/endian.h"

namespace rmw_cyclonedds_cpp
{
struct SizeAccumulator
{
  size_t _size;
  size_t _capacity;

  SizeAccumulator(size_t capacity = std::numeric_limits<size_t>::max(), size_t size = 0)
  : _size(size), _capacity(capacity)
  {
  }
  auto size() { return _size; }

  void put_bytes(const void * s, size_t n)
  {
    (void)s;

    if (_size + n > _capacity) throw std::length_error("Data exceeds buffer size");
    _size += n;
  }
};

struct DataAccumulator
{
  void * _data;
  size_t _size;
  size_t _capacity;

  DataAccumulator() = delete;
  DataAccumulator(void * data, size_t capacity, size_t size = 0)
  : _data(data), _size(size), _capacity(capacity)
  {
    assert(data);
    assert(size <= capacity);
  };

  auto size() { return _size; }

  void put_bytes(const void * s, size_t n)
  {
    if (_size + n > _capacity) throw std::length_error("Data exceeds buffer size");

    memcpy((byte *)_data + _size, s, n);
    _size += n;
  }
};

template <typename Accumulator>
class CDRWriter
{
protected:
  Accumulator & dst;
  const size_t max_align = 4;

  void put_bytes(const void * bytes, size_t size) { dst.put_bytes(bytes, size); }

  template <typename ByteCollection>
  void put_bytes(ByteCollection t)
  {
    put_bytes(t.data(), t.size());
  }

  void align(size_t n_bytes)
  {
    auto align_to = std::min(max_align, n_bytes);
    auto current_alignment = dst.size() % align_to;
    if (current_alignment != 0) {
      for (size_t i = 0; i < align_to - current_alignment; i++) {
        auto fill = '\0';
        dst.put_bytes(&fill, 1);
      }
    }
  }

public:
  CDRWriter() = delete;
  explicit CDRWriter(Accumulator & dst) : dst(dst) {}

  template <typename... Args>
  void serialize_top_level(Args... args)
  {
    put_encapsulation_header();
    (void)std::initializer_list<int>{(serialize(args), 0)...};
  }

protected:
  void put_encapsulation_header()
  {
    // beginning of message
    std::array<uint8_t, 4> prefix = {0x00,  // 0x00 = plain cdr
                                     (DDSRT_ENDIAN == DDSRT_LITTLE_ENDIAN) ? 0x01 : 0x00,
                                     /* options: defaults to 0x0000 */
                                     0, 0};
    put_bytes(prefix);
  };

  template <typename T, std::enable_if_t<std::is_arithmetic<T>::value, bool> = true>
  void serialize(const T & data)
  {
    align(sizeof(data));
    put_bytes(static_cast<const void *>(&data), sizeof(data));
  }

  template <typename T, typename char_type = typename T::traits_type::char_type>
  void serialize(const T & value)
  {
    if (sizeof(char_type) == 1) {
      serialize_u32(value.size() + 1);
    } else {
      serialize_u32(value.size() * sizeof(char_type));
    }
    for (char_type s : value) {
      serialize(s);
    }
    if (sizeof(char_type) == 1) {
      serialize('\0');
    }
  }

  template <typename MetaMessage>
  void serialize(const MessageRef<MetaMessage> & message)
  {
    for (size_t i = 0; i < message.size(); i++) {
      auto member = message.at(i);
      switch (member.get_container_type()) {
        case MemberContainerType::SingleValue:
          member.with_single_value([&](auto m) { serialize(m); });
          break;
        case MemberContainerType::Array:
          member.with_array([&](auto m) {
            for (size_t i = 0; i < m.size(); i++) {
              serialize(m[i]);
            }
          });
          break;
        case MemberContainerType::Sequence:
          member.with_sequence([&](auto m) {
            serialize_u32(m.size());
            for (size_t i = 0; i < m.size(); i++) {
              serialize(m[i]);
            }
          });
      }
    }
  }

  void serialize_u32(size_t s)
  {
    assert(s < UINT32_MAX);
    serialize(static_cast<uint32_t>(s));
  }
};  // namespace rmw_cyclonedds_cpp

template <typename... Args>
size_t get_serialized_size(Args &&... args)
{
  SizeAccumulator accum;
  CDRWriter<SizeAccumulator> writer{accum};

  writer.serialize_top_level(std::forward<Args>(args)...);
  return accum.size();
}

template <typename... Args>
void serialize(uint8_t * dest, size_t dest_size, Args &&... args)
{
  DataAccumulator accum{dest, dest_size};
  CDRWriter<DataAccumulator> writer{accum};

  writer.serialize_top_level(std::forward<Args>(args)...);
}

template <typename... Args>
void serialize(std::vector<uint8_t> & dest, Args &&... args)
{
  size_t start_at = dest.size();

  auto n_new_bytes = get_serialized_size(std::forward<Args>(args)...);
  dest.resize(start_at + n_new_bytes);

  DataAccumulator data_accum(dest.data() + start_at, n_new_bytes);
  CDRWriter<DataAccumulator> writer{data_accum};
  writer.serialize_top_level(std::forward<Args>(args)...);
}

}  // namespace rmw_cyclonedds_cpp

#endif  //ROS2_MASTER_SERIALIZATION_HPP
