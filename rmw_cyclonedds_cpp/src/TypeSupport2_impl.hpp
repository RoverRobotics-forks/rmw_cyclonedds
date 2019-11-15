#ifndef ROS2_MASTER_TYPESUPPORT2_IMPL_HPP
#define ROS2_MASTER_TYPESUPPORT2_IMPL_HPP
#include "TypeSupport2.hpp"

namespace rmw_cyclonedds_cpp {
template <typename T> struct typeval { using type = T; };

template <typename UnaryFunction, typename Result>
Result with_type(ValueType value_type, UnaryFunction f) {
  switch (value_type) {
  case ValueType::FLOAT:
    return f(typeval<float>());
  case ValueType::DOUBLE:
    return f(typeval<double>());
  case ValueType::LONG_DOUBLE:
    return f(typeval<long double>());
  case ValueType::WCHAR:
    return f(typeval<char16_t>());
  case ValueType::CHAR:
    return f(typeval<char>());
  case ValueType::BOOLEAN:
    return f(typeval<bool>());
  case ValueType::OCTET:
    return f(typeval<unsigned char>());
  case ValueType::UINT8:
    return f(typeval<uint8_t>());
  case ValueType::INT8:
    return f(typeval<int8_t>());
  case ValueType::UINT16:
    return f(typeval<uint16_t>());
  case ValueType::INT16:
    return f(typeval<int16_t>());
  case ValueType::UINT32:
    return f(typeval<uint32_t>());
  case ValueType::INT32:
    return f(typeval<int32_t>());
  case ValueType::UINT64:
    return f(typeval<uint64_t>());
  case ValueType::INT64:
    return f(typeval<int64_t>());
  default:
    throw std::invalid_argument("not a primitive value");
  }
}

template <typename T> struct NativeValueHelper {
  using reference_type = T &;
  static size_t sizeof_value() { return sizeof(T); }
  static reference_type cast_value(void *ptr) { return *static_cast<T *>(ptr); }
  static reference_type cast_value(const void *ptr) {
    return *static_cast<const T *>(ptr);
  }
};

template <typename MetaMessage> struct MessageValueHelper {
  MetaMessage value_members;
  using reference_type = MessageRef<MetaMessage>;
  size_t sizeof_value() const { return value_members.size_of_; }
  reference_type cast_value(void *ptr) {
    return make_message_ref(value_members, ptr);
  }
  reference_type cast_value(const void *ptr) {
    return make_message_ref(value_members, ptr);
  }
};

template <>
template <typename UnaryFunction, typename Result>
Result MemberRef<RTI_Cpp::MetaMember>::with_value_helper(UnaryFunction f) {
  auto vt = ValueType(meta_member.type_id_);
  switch (vt) {
  case ValueType::MESSAGE:
    return with_typesupport(meta_member.members_, [&](auto submessage_members) {
      return f(
          MessageValueHelper<decltype(submessage_members)>{submessage_members});
    });
  case ValueType::STRING:
    return f(NativeValueHelper<std::string>());
  case ValueType::WSTRING:
    return f(NativeValueHelper<std::wstring>());
  default:
    return with_type(vt, [&](auto t) {
      return f(NativeValueHelper<typename decltype(t)::type>());
    });
  }
}

template <>
template <typename UnaryFunction, typename Result>
Result MemberRef<RTI_C::MetaMember>::with_value_helper(UnaryFunction f) {
  auto vt = ValueType(meta_member.type_id_);
  switch (vt) {
  case ValueType::MESSAGE:
    return with_typesupport(meta_member.members_, [&](auto submessage_members) {
      return f(
          MessageValueHelper<decltype(submessage_members)>{submessage_members});
    });
  case ValueType::STRING:
    return f(NativeValueHelper<RTI_C::String>());
  case ValueType::WSTRING:
    return f(NativeValueHelper<RTI_C::WString>());
  default:
    return with_type(vt, [&](auto t) {
      return f(NativeValueHelper<typename decltype(t)::type>());
    });
  }
}
template <typename MetaMember, typename ValueHelper>
struct SingleValueMemberRef : MemberRef<MetaMember> {
  SingleValueMemberRef(ValueHelper h, MemberRef<MetaMember> m)
      : MemberRef<MetaMember>{m}, value_helper{h} {};
  ValueHelper value_helper;
  auto get() { return value_helper.cast_value(this->data); }
  auto get() const { return value_helper.cast_value(this->data); }
};

template <typename MetaMember>
template <typename UnaryFunction, typename Result>
Result MemberRef<MetaMember>::with_single_value(UnaryFunction f) {
  assert(get_container_type() == MemberContainerType::SingleValue);
  return with_value_helper([&](auto helper) {
    return f(SingleValueMemberRef<MetaMember, decltype(helper)>{helper, *this});
  });
}

template <typename MetaMember, typename ValueHelper>
struct ArrayMemberRef : MemberRef<MetaMember> {
  ArrayMemberRef(ValueHelper h, MemberRef<MetaMember> m)
      : MemberRef<MetaMember>{m}, value_helper{h} {};
  ValueHelper value_helper;
  auto operator[](size_t index) {
    return value_helper.cast_value((byte *)this->data +
                                   value_helper.sizeof_value() * index);
  }
  auto operator[](size_t index) const {
    return value_helper.cast_value((const byte *)this->data +
                                   value_helper.sizeof_value() * index);
  }
  size_t size() const { return this->meta_member.array_size_; }
};

template <typename MetaMember>
template <typename UnaryFunction, typename Result>
Result MemberRef<MetaMember>::with_array(UnaryFunction f) {
  assert(get_container_type() == MemberContainerType::Array);
  return with_value_helper([&](auto helper) {
    return f(ArrayMemberRef<MetaMember, decltype(helper)>(helper, *this));
  });
}

template <typename MetaMember, typename ValueHelper>
struct SequenceMemberRef : MemberRef<MetaMember> {
  SequenceMemberRef(ValueHelper h, MemberRef<MetaMember> m)
      : MemberRef<MetaMember>{m}, value_helper{h} {
    assert(this->meta_member.get_function);
    assert(this->meta_member.get_const_function);
    assert(this->meta_member.size_function);
    assert(this->meta_member.resize_function);
  };

  ValueHelper value_helper;
  auto operator[](size_t index) {
    return value_helper.cast_value(
        this->meta_member.get_function(this->data, index));
  }
  auto operator[](size_t index) const {
    return value_helper.cast_value(
        this->meta_member.get_const_function(this->data, index));
  };
  size_t size() const { return this->meta_member.size_function(this->data); };
};

template <typename MetaMember>
template <typename UnaryFunction, typename Result>
Result MemberRef<MetaMember>::with_sequence(UnaryFunction f) {
  assert(get_container_type() == MemberContainerType::Sequence);
  return with_value_helper([&](auto helper) {
    return f(SequenceMemberRef<MetaMember, decltype(helper)>(helper, *this));
  });
}

#endif
}