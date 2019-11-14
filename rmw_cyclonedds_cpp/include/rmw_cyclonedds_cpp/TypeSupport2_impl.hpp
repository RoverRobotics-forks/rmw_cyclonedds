#include "TypeSupport2.hpp"

#ifndef ROS2_MASTER_TYPESUPPORT2_IMPL_HPP
#define ROS2_MASTER_TYPESUPPORT2_IMPL_HPP
namespace rmw_cyclonedds_cpp
{
template <typename UnaryFunction>
constexpr auto apply_to_primitive_value(UnaryFunction fn, ValueType value_type, void * data)
{
  switch (value_type) {
    case ValueType::FLOAT:
      return fn(*static_cast<float *>(data));
    case ValueType::DOUBLE:
      return fn(*static_cast<double *>(data));
    case ValueType::LONG_DOUBLE:
      return fn(*static_cast<long double *>(data));
    case ValueType::WCHAR:
      return fn(*static_cast<char16_t *>(data));
    case ValueType::CHAR:
      return fn(*static_cast<char *>(data));
    case ValueType::BOOLEAN:
      return fn(*static_cast<bool *>(data));
    case ValueType::OCTET:
      return fn(*static_cast<unsigned char *>(data));
    case ValueType::UINT8:
      return fn(*static_cast<uint8_t *>(data));
    case ValueType::INT8:
      return fn(*static_cast<int8_t *>(data));
    case ValueType::UINT16:
      return fn(*static_cast<uint16_t *>(data));
    case ValueType::INT16:
      return fn(*static_cast<int16_t *>(data));
    case ValueType::UINT32:
      return fn(*static_cast<uint32_t *>(data));
    case ValueType::INT32:
      return fn(*static_cast<int32_t *>(data));
    case ValueType::UINT64:
      return fn(*static_cast<uint64_t *>(data));
    case ValueType::INT64:
      return fn(*static_cast<int64_t *>(data));
    default:
      throw std::invalid_argument("not a primitive value");
  }
}

template <typename UnaryFunction>
auto apply_to_typed_value(
  UnaryFunction fn, const void * value_data, const RTI_C::MetaMember & meta_member)
{
  auto vt = (ValueType)meta_member.type_id_;
  switch (vt) {
    case ValueType::MESSAGE:
      return fn(make_message_ref(meta_member.members_, value_data));
    case ValueType::STRING:
      return fn(*static_cast<const typename RTI_C::String *>(value_data));
    case ValueType::WSTRING:
      return fn(*static_cast<const typename RTI_C::WString *>(value_data));
    default:
      return apply_to_primitive_value(fn, vt, value_data);
  }
}

template <typename UnaryFunction>
auto apply_to_typed_value(
  UnaryFunction fn, const void * value_data, const RTI_Cpp::MetaMember & meta_member)
{
  auto vt = (ValueType)meta_member.type_id_;
  switch (vt) {
    case ValueType::MESSAGE:
      return fn(make_message_ref(meta_member.members_, value_data));
    case ValueType::STRING:
      return fn(*static_cast<const std::string *>(value_data));
    case ValueType::WSTRING:
      return fn(*static_cast<const std::string *>(value_data));
    default:
      return apply_to_primitive_value(fn, vt, value_data);
  }
}

template <typename MetaMember>
template <typename UnaryFunction>
void MemberRef<MetaMember>::for_each_value(UnaryFunction f)
{
  if (is_single_value()) {
    apply_to_typed_value(f, data, meta_member);
  } else if (is_array()) {
    size_t stride = get_array_stride();
    for (size_t i = 0; i < get_array_size(); i++) {
      apply_to_typed_value(f, (byte *)data + i * stride, meta_member);
    }
  } else {
    assert(is_sequence());
    auto size = get_sequence_size();
    for (size_t i = 0; i < size; i++) {
      apply_to_typed_value(f, meta_member.get_function(data, i), meta_member);
    }
  }
}

}  // namespace rmw_cyclonedds_cpp

#endif  //ROS2_MASTER_TYPESUPPORT2_IMPL_HPP
