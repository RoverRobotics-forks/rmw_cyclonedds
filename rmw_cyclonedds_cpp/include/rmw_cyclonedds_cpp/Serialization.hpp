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
  uint8_t * _data;
  size_t _size;
  size_t _capacity;

  DataAccumulator() = delete;
  DataAccumulator(uint8_t * data, size_t capacity, size_t = 0)
  : _data(data), _size(0), _capacity(capacity){};

  auto size() { return _size; }

  void put_bytes(const void * s, size_t n)
  {
    if (_size + n > _capacity) throw std::length_error("Data exceeds buffer size");

    memcpy(_data + _size, s, n);
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

  void align_and_put(void * data, size_t n_bytes)
  {
    align(n_bytes);
    dst.put_bytes(data, n_bytes);
  }

public:
  CDRWriter() = delete;
  explicit CDRWriter(Accumulator & dst) : dst(dst) {}

  void serialize_top_level(void * message_data, const rosidl_message_type_support_t * type_support)
  {
    put_encapsulation_header();
    with_typesupport(type_support, [&](auto ts) { serialize(make_message_ref(ts, message_data)); });
  }

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
  void serialize(T data)
  {
    align(sizeof(data));
    put_bytes(static_cast<void *>(&data), sizeof(data));
  }

  template <typename T, typename char_type = typename T::traits_type::char_type>
  void serialize(T && value)
  {
    if (sizeof(char_type) == 1) {
      serialize_u32(value.size() + 1);
    } else {
      serialize_u32(value.size() * sizeof(char_type));
    }
    for (char & s : value) {
      serialize(s);
    }
    if (sizeof(char_type) == 1) {
      serialize('\0');
    }
  }

  template <typename MetaMessage>
  void serialize(MessageRef<MetaMessage> message)
  {
    for (size_t i = 0; i < message.size(); i++) {
      auto member = message.at(i);

      if (member.is_sequence()) {
        serialize_u32(member.get_sequence_size());
      }

      member.for_each_value([&](auto s) { serialize(s); });
    }
  }

  void serialize_u32(size_t s)
  {
    assert(s < UINT32_MAX);
    serialize(static_cast<uint32_t>(s));
  }
};

size_t get_serialized_size(void * message_data, const rosidl_message_type_support_t * type_support)
{
  SizeAccumulator accum;
  CDRWriter<SizeAccumulator> writer{accum};
  writer.serialize_top_level(message_data, type_support);
  return accum.size();
}

void serialize_entire_ros_message(
  uint8_t * dest, size_t dest_size, void * message_data,
  const rosidl_message_type_support_t * type_support)
{
  DataAccumulator accum{dest, dest_size};

  CDRWriter<DataAccumulator> writer{accum};
  writer.serialize_top_level(message_data, type_support);
}

template <typename... Args>
void serialize(std::vector<uint8_t> & dest, Args &&... args)
{
  SizeAccumulator accum{dest.max_size(), dest.size()};

  CDRWriter<SizeAccumulator> writer{accum};
  (void)std::tuple<int>{writer.serialize(args)...};

  DataAccumulator data_accum(&dest.back(), accum.size());
  dest.resize(accum.size());
  CDRWriter<DataAccumulator> real_writer{data_accum};
}

}  // namespace rmw_cyclonedds_cpp

#endif  //ROS2_MASTER_SERIALIZATION_HPP
