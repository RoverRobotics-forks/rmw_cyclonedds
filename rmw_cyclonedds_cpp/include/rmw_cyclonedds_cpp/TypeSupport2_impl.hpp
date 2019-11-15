#ifndef ROS2_MASTER_TYPESUPPORT2_IMPL_HPP
#define ROS2_MASTER_TYPESUPPORT2_IMPL_HPP
#include "TypeSupport2.hpp"

namespace rmw_cyclonedds_cpp
{
template <typename T>
struct declval
{
  using type = T;
};

template <typename UnaryFunction>
constexpr auto with_type(ValueType value_type, UnaryFunction f)
{
  switch (value_type) {
    case ValueType::FLOAT:
      return f(declval<float>());
    case ValueType::DOUBLE:
      return f(declval<double>());
    case ValueType::LONG_DOUBLE:
      return f(declval<long double>());
    case ValueType::WCHAR:
      return f(declval<char16_t>());
    case ValueType::CHAR:
      return f(declval<char>());
    case ValueType::BOOLEAN:
      return f(declval<bool>());
    case ValueType::OCTET:
      return f(declval<unsigned char>());
    case ValueType::UINT8:
      return f(declval<uint8_t>());
    case ValueType::INT8:
      return f(declval<int8_t>());
    case ValueType::UINT16:
      return f(declval<uint16_t>());
    case ValueType::INT16:
      return f(declval<int16_t>());
    case ValueType::UINT32:
      return f(declval<uint32_t>());
    case ValueType::INT32:
      return f(declval<int32_t>());
    case ValueType::UINT64:
      return f(declval<uint64_t>());
    case ValueType::INT64:
      return f(declval<int64_t>());
    default:
      throw std::invalid_argument("not a primitive value");
  }
}

template <typename UnaryFunction>
constexpr auto with_type(const RTI_C::MetaMember & meta_member, UnaryFunction f)
{
  auto vt = (ValueType)meta_member.type_id_;
  switch (vt) {
    case ValueType::MESSAGE:
      throw std::invalid_argument("this is a message type");
    case ValueType::STRING:
      return f(declval<RTI_C::String>());
    case ValueType::WSTRING:
      return f(declval<RTI_C::WString>());
    default:
      return with_type(vt, f);
  }
}

template <typename UnaryFunction>
constexpr auto with_type(const RTI_Cpp::MetaMember & meta_member, UnaryFunction f)
{
  auto vt = (ValueType)meta_member.type_id_;
  switch (vt) {
    case ValueType::MESSAGE:
      throw std::invalid_argument("this is a message type");
    case ValueType::STRING:
      return f(declval<std::string>());
    case ValueType::WSTRING:
      return f(declval<std::wstring>());
    default:
      return with_type(vt, f);
  }
}

template <typename UnaryFunction, typename MetaMemberOrValueType>
constexpr auto with_typed_ptr(void * ptr, MetaMemberOrValueType vt, UnaryFunction f)
{
  return with_type(vt, [&](auto x) {
    using T = typename decltype(x)::type;
    return f(static_cast<T *>(ptr));
  });

  //  static_cast<std::add_pointer_t <std::remove_reference_t<decltype(x)>> >(ptr));
}
//
//template <typename UnaryFunction, typename MetaMemberOrValueType>
//constexpr auto with_typed_ptr(const void * ptr, MetaMemberOrValueType vt, UnaryFunction f)
//{
//  return with_type(
//    vt, [&](auto x) { return f(static_cast<const std::remove_reference_t<decltype(x)> *>(ptr)); });
//}

template <typename MetaMessage>
MessageRef<MetaMessage> ArrayOfMessageRef<MetaMessage>::operator[](size_t index)
{
  assert(index < n);
  return make_message_ref(meta_message, (byte *)data + index * meta_message.size_of_);
}

template <typename MetaMember>
template <typename UnaryFunction, typename Result>
Result MemberRef<MetaMember>::with_array(UnaryFunction f)
{
  assert(get_container_type() == MemberContainerType::Array);

  auto size = meta_member.array_size_;
  if (!is_submessage_type()) {
    return with_typed_ptr(data, meta_member, [&](auto x) {
      return f(ArrayRef<std::decay_t<decltype(*x)>>{x, size});
    });
  } else {
    return with_submessage_typesupport([&](auto message_members) {
      return f(ArrayOfMessageRef<decltype(message_members)>{data, size, message_members});
    });
  }
}

template <typename MetaMember>
template <typename UnaryFunction, typename Result>
Result MemberRef<MetaMember>::with_single_value(UnaryFunction f)
{
  assert(get_container_type() == MemberContainerType::SingleValue);
  if (is_submessage_type()) {
    return with_message(this->meta_member.members_, data, f);
  } else {
    return with_typed_ptr(data, meta_member, [&](auto * x) { return f(*x); });
  }
}

template <typename T, typename Deref>
struct CommonObjectSequence
{
  void * object;
  Deref deref;

  size_t (*size_function)(const void *);
  const void * (*get_const_function)(const void *, size_t index);
  void * (*get_function)(void *, size_t index);

  size_t size() const { return size_function(object); }
  T & get(size_t index) const { return deref(get_const_function(object, index)); }
  T & get(size_t index) { return deref(get_function(object, index)); }
};

template <typename T, typename Deref>
struct CObjectSequence {
  bool (* resize_function)(void *, size_t size);

  void resize(size_t size){
    assert (resize_function(size));
  }
};

template <typename T, typename Deref>
struct CppObjectSequence {
  void (* resize_function)(void *, size_t size);

  void resize(size_t size){
    resize_function(size);
  }
};

template <>
template <typename UnaryFunction, typename Result>
Result MemberRef<RTI_Cpp::MetaMember>::with_sequence(UnaryFunction f)
{
  assert(!meta_member.size_function);
  assert(false);
}

template <>
template <typename UnaryFunction, typename Result>
Result MemberRef<RTI_C::MetaMember>::with_sequence(UnaryFunction f)
{
  assert(!meta_member.size_function);
  assert(!meta_member.get_function);
  assert(!meta_member.get_const_function);

  return with_type(meta_member, [&](auto t) {
    using T = typename decltype(t)::type;
    return f(*static_cast<RTI_C::Sequence<T> *>(data));
  });
}

}  // namespace rmw_cyclonedds_cpp

#endif  //ROS2_MASTER_TYPESUPPORT2_IMPL_HPP
