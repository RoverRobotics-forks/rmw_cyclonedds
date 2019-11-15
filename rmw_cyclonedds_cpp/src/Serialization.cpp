//
// Created by dan on 11/14/19.
//
#include "Serialization.hpp"
#include "TypeSupport2.hpp"
#include "dds/ddsrt/endian.h"
#include <array>

namespace rmw_cyclonedds_cpp {

std::pair<rosidl_message_type_support_t, rosidl_message_type_support_t>
get_svc_request_response_typesupports(const rosidl_service_type_support_t * svc)
{
  return with_typesupport(svc, [&](auto svc_ts) {
    return std::make_pair(
        rosidl_message_type_support_t{
            svc->typesupport_identifier,
            svc_ts.request_members_,
            get_message_typesupport_handle_function,
        },
        rosidl_message_type_support_t{
            svc->typesupport_identifier,
            svc_ts.response_members_,
            get_message_typesupport_handle_function,
        });
  });
}

struct SizeAccumulator {
  size_t _size;
  size_t _capacity;

  explicit SizeAccumulator(size_t capacity = std::numeric_limits<size_t>::max(),
                           size_t size = 0)
      : _size(size), _capacity(capacity) {}
  auto size() const { return _size; }

  void put_bytes(const void *s, size_t n) {
    (void)s;

    if (_size + n > _capacity)
      throw std::length_error("Data exceeds buffer size");
    _size += n;
  }
};

struct DataAccumulator {
  void *_data;
  size_t _size;
  size_t _capacity;

  DataAccumulator() = delete;
  DataAccumulator(void *data, size_t capacity, size_t size = 0)
      : _data(data), _size(size), _capacity(capacity) {
    assert(data);
    assert(size <= capacity);
  };

  auto size() const { return _size; }

  void put_bytes(const void *s, size_t n) {
    if (_size + n > _capacity)
      throw std::length_error("Data exceeds buffer size");

    memcpy((byte *)_data + _size, s, n);
    _size += n;
  }
};

template <typename Accumulator> class CDRWriter {
protected:
  Accumulator &dst;
  const size_t max_align = 4;

  void put_bytes(const void *bytes, size_t size) { dst.put_bytes(bytes, size); }

  template <typename ByteCollection> void put_bytes(ByteCollection t) {
    put_bytes(t.data(), t.size());
  }

  void align(size_t n_bytes) {
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
  explicit CDRWriter(Accumulator &dst) : dst(dst) {}

  template <typename MetaMessage>
  void serialize_top_level(MessageRef<MetaMessage> msg) {
    put_encapsulation_header();
    serialize(msg);
  }

  template <typename MetaMessage>
  void serialize_top_level(cdds_request_header_t request_header,
                           MessageRef<MetaMessage> msg) {
    put_encapsulation_header();
    serialize(request_header.guid);
    serialize(request_header.seq);
    serialize(msg);
  }

protected:
  void put_encapsulation_header() {
    // beginning of message
    std::array<uint8_t, 4> prefix = {
        0x00, // 0x00 = plain cdr
        (DDSRT_ENDIAN == DDSRT_LITTLE_ENDIAN) ? 0x01 : 0x00,
        /* options: defaults to 0x0000 */
        0, 0};
    put_bytes(prefix);
  };

  template <typename T,
            std::enable_if_t<std::is_arithmetic<T>::value, bool> = true>
  void serialize(const T &data) {
    align(sizeof(data));
    put_bytes(static_cast<const void *>(&data), sizeof(data));
  }

  template <typename T, typename char_type = typename T::traits_type::char_type>
  void serialize(const T &value) {
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
  void serialize(const MessageRef<MetaMessage> &message) {
    for (size_t i = 0; i < message.size(); i++) {
      auto member = message.at(i);
      switch (member.get_container_type()) {
      case MemberContainerType::SingleValue:
        member.with_single_value([&](auto m) { serialize(m.get()); });
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
  void serialize_u32(size_t s) {
    assert(s < UINT32_MAX);
    serialize(static_cast<uint32_t>(s));
  }
};

void serialize(std::vector<uint8_t> &dest,
               const cdds_request_wrapper_t &request,
               const rosidl_message_type_support_t &ts) {
  with_message(&ts, request.data, [&](auto msg) {
    size_t size;
    {
      SizeAccumulator accum;
      CDRWriter<SizeAccumulator> writer{accum};
      writer.serialize_top_level(request.header, msg);
      size = accum.size();
    }
    {
      dest.resize(size);
      DataAccumulator accum{dest.data(), size};
      CDRWriter<DataAccumulator> writer{accum};
      writer.serialize_top_level(request.header, msg);
    }
  });
}

void serialize(std::vector<uint8_t> &dest, const void *data,
               const rosidl_message_type_support_t &ts) {
  assert(dest.empty());
  auto size = get_serialized_size(data, ts);
  dest.resize(size);
  serialize(dest.data(), size, data, ts);
}

void serialize(uint8_t *dest, size_t dest_size, const void *data,
               const rosidl_message_type_support_t &ts) {

  DataAccumulator accum{dest, dest_size};
  CDRWriter<DataAccumulator> writer{accum};
  return with_message(&ts, data,
                      [&](auto msg) { writer.serialize_top_level(msg); });
}

size_t get_serialized_size(const void *data,
                           const rosidl_message_type_support_t &ts) {
  SizeAccumulator accum;
  CDRWriter<SizeAccumulator> writer{accum};
  with_message(&ts, data, [&](auto msg) { writer.serialize_top_level(msg); });
  return accum.size();
}

} // namespace rmw_cyclonedds_cpp