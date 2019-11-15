#ifndef ROS2_MASTER_TYPESUPPORT2_IMPL_HPP
#define ROS2_MASTER_TYPESUPPORT2_IMPL_HPP
#include "TypeSupport2.hpp"

namespace rmw_cyclonedds_cpp
{
template <typename T>
struct typeval
{
  using type = T;
};

template <typename UnaryFunction>
constexpr auto with_type(ValueType value_type, UnaryFunction f)
{
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

template <typename UnaryFunction>
constexpr auto with_type(const RTI_C::MetaMember & meta_member, UnaryFunction f)
{
  auto vt = (ValueType)meta_member.type_id_;
  switch (vt) {
    case ValueType::MESSAGE:
      throw std::invalid_argument("this is a message type");
    case ValueType::STRING:
      return f(typeval<RTI_C::String>());
    case ValueType::WSTRING:
      return f(typeval<RTI_C::WString>());
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
      return f(typeval<std::string>());
    case ValueType::WSTRING:
      return f(typeval<std::wstring>());
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

template <typename MetaMember>
struct ObjectSequence : MemberRef<MetaMember>
{
  void resize(size_t size);
  auto size() const { return this->meta_member.size_function(this->data); }
};

template <>
void ObjectSequence<RTI_C::MetaMember>::resize(size_t size)
{
  if (!meta_member.resize_function(this->data, size)) {
    throw std::runtime_error("Could not resize sequence");
  }
}
template <>
void ObjectSequence<RTI_Cpp::MetaMember>::resize(size_t size)
{
  return meta_member.resize_function(this->data, size);
}
template <typename MetaMember, typename T>
struct ObjectSequenceOfValue : ObjectSequence<MetaMember>
{
  T & operator[](size_t index)
  {
    return *static_cast<T *>(this->meta_member.get_function(this->data, index));
  }
  const T & operator[](size_t index) const
  {
    return *static_cast<const T *>(this->meta_member.get_const_function(this->data, index));
  }
};
template <typename MetaMember, typename MetaMessage>
struct ObjectSequenceOfMessage : ObjectSequence<MetaMember>
{
  const MetaMessage & submessage_meta()
  {
    return *static_cast<const MetaMessage *>(this->meta_member.members_->data);
  }
  MessageRef<MetaMessage> operator[](size_t index)
  {
    return make_message_ref(submessage_meta(), this->meta_member.get_function(this->data, index));
  }
  MessageRef<MetaMessage> operator[](size_t index) const
  {
    return make_message_ref(
      submessage_meta(), this->meta_member.get_const_function(this->data, index));
  }
};

template <>
template <typename UnaryFunction, typename Result>
Result MemberRef<RTI_Cpp::MetaMember>::with_sequence(UnaryFunction f)
{
  assert(!meta_member.size_function);
  assert(false);
}

template <typename MetaMember>
template <typename UnaryFunction, typename Result>
Result MemberRef<MetaMember>::with_sequence(UnaryFunction f)
{
  if (meta_member.size_function && meta_member.get_function && meta_member.get_const_function) {
    if (is_submessage_type()) {
      return with_submessage_typesupport([&](auto ts) {
        return f(*static_cast<ObjectSequenceOfMessage<MetaMember, decltype(ts)> *>(this));
      });
    } else {
      return with_type(meta_member, [&](auto tp) {
        return f(
          *static_cast<ObjectSequenceOfValue<MetaMember, typename decltype(tp)::type> *>(this));
      });
    }
  } else if (std::is_same<MetaMember, RTI_C::MetaMember>::value) {
    return with_type(meta_member, [&](auto tp) {
      return f(*static_cast<RTI_C::Sequence<typename decltype(tp)::type> *>(data));
    });
  }
  assert(false);
}

#endif  //ROS2_MASTER_TYPESUPPORT2_IMPL_HPP
}