#include "sertopic.hpp"
#include "serdata.hpp"

//
//#include <rmw/allocators.h>
//
//#include <cstring>
//#include <memory>
//#include <regex>
//#include <sstream>
//#include <string>
//#include <utility>
//
//#include "Serialization.hpp"
//#include "TypeSupport2.hpp"
//#include "bytewise.hpp"
//#include "dds/ddsi/q_radmin.h"
//#include "rmw/error_handling.h"
//#include "rmw_cyclonedds_cpp/MessageTypeSupport.hpp"
//#include "rmw_cyclonedds_cpp/ServiceTypeSupport.hpp"
//#include "rmw_cyclonedds_cpp/serdes.hpp"
using namespace rmw_cyclonedds_cpp;



sertopic_rmw::sertopic_rmw(
  const char * topicname, const char * type_support_identifier,
  void * type_support, bool is_request_header,
  std::__1::unique_ptr<rmw_cyclonedds_cpp::StructValueType> message_type)
{
#if DDSI_SERTOPIC_HAS_TOPICKIND_NO_KEY
  std::string type_name = get_type_name(type_support_identifier, type_support);
  ddsi_sertopic_init(
    static_cast<struct ddsi_sertopic *>(this), topicname,
    type_name.c_str(), &sertopic_rmw::ddsi_ops, &serdata_rmw_ops, true);
#else
  cpp_name = std::string(topicname);
  cpp_type_name = get_type_name(type_support_identifier, type_support);
  cpp_name_type_name = st->cpp_name + std::string(";") + std::string(st->cpp_type_name);
  ops = &sertopic_rmw_ops;
  serdata_ops = &serdata_rmw_ops;
  serdata_basehash = ddsi_sertopic_compute_serdata_basehash(st->serdata_ops);
  name_type_name = const_cast<char *>(st->cpp_name_type_name.c_str());
  name = const_cast<char *>(st->cpp_name.c_str());
  type_name = const_cast<char *>(st->cpp_type_name.c_str());
  iid = ddsi_iid_gen();
  ddsrt_atomic_st32(&st->refc, 1);
#endif
  type_support.typesupport_identifier_ = type_support_identifier;
  type_support.type_support_ = type_support;
  is_request_header = is_request_header;
  cdr_writer = rmw_cyclonedds_cpp::make_cdr_writer(std::move(message_type));
}

static void sertopic_rmw_zero_samples(const struct ddsi_sertopic * d, void * samples, size_t count)
{
  static_cast<void>(d);
  static_cast<void>(samples);
  static_cast<void>(count);
  /* Not using code paths that rely on the samples getting zero'd out */
}
static void sertopic_rmw_realloc_samples(
  void ** ptrs, const struct ddsi_sertopic * d, void * old,
  size_t oldcount, size_t count)
{
  static_cast<void>(ptrs);
  static_cast<void>(d);
  static_cast<void>(old);
  static_cast<void>(oldcount);
  static_cast<void>(count);
  /* Not using code paths that rely on this (loans, dispose, unregister with instance handle,
     content filters) */
  abort();
}


bool sertopic_rmw_equal(
  const struct ddsi_sertopic * acmn, const struct ddsi_sertopic * bcmn)
{
  /* A bit of a guess: topics with the same name & type name are really the same if they have
     the same type support identifier as well */
  const struct rmw_cyclonedds_cpp::sertopic_rmw * a = static_cast<const struct rmw_cyclonedds_cpp::sertopic_rmw *>(acmn);
  const struct rmw_cyclonedds_cpp::sertopic_rmw * b = static_cast<const struct rmw_cyclonedds_cpp::sertopic_rmw *>(bcmn);
  if (a->is_request_header != b->is_request_header) {
    return false;
  }
  if (strcmp(
      a->type_support.typesupport_identifier_,
      b->type_support.typesupport_identifier_) != 0)
  {
    return false;
  }
  return true;
}

uint32_t sertopic_rmw_hash(const struct ddsi_sertopic * tpcmn)
{
  const struct rmw_cyclonedds_cpp::sertopic_rmw * tp = static_cast<const struct rmw_cyclonedds_cpp::sertopic_rmw *>(tpcmn);
  uint32_t h2 = static_cast<uint32_t>(std::hash<bool>{} (tp->is_request_header));
  uint32_t h1 =
    static_cast<uint32_t>(
    std::__1::hash<std::string>{} (std::string(
      tp->type_support.typesupport_identifier_)));
  return h1 ^ h2;
}

//struct ddsi_sertopic_ops {
//  ddsi_sertopic_free_t free;
//  ddsi_sertopic_zero_samples_t zero_samples;
//  ddsi_sertopic_realloc_samples_t realloc_samples;
//  ddsi_sertopic_free_samples_t free_samples;
//  ddsi_sertopic_equal_t equal;
//  ddsi_sertopic_hash_t hash;
//};


const struct ddsi_sertopic_ops sertopic_rmw::ddsi_ops {
  (ddsi_sertopic_free_t) [](struct ddsi_sertopic * d){
    delete static_cast<rmw_cyclonedds_cpp::sertopic_rmw *>(d);
  },
  (ddsi_sertopic_zero_samples_t) [](const struct ddsi_sertopic * d, void * samples, size_t count) {
    throw std::logic_error("ddsi_sertopic_zero_samples not yet implemented");
  },
  (ddsi_sertopic_realloc_samples_t) [] (
    void ** ptrs, const struct ddsi_sertopic * d, void * old,
    size_t oldcount, size_t count){
    throw std::logic_error("ddsi_sertopic_realloc_samples not yet implemented");
  },
  (ddsi_sertopic_free_samples_t) [](
      const struct ddsi_sertopic * d, void ** ptrs, size_t count,
      dds_free_op_t op)
  {
    throw std::logic_error("ddsi_sertopic_free_samples not yet implemented");
  },
#if DDSI_SERTOPIC_HAS_EQUAL_AND_HASH

  (ddsi_sertopic_equal_t) [](const struct ddsi_sertopic * a, const struct ddsi_sertopic * b) -> bool{
    auto ac = static_cast<const rmw_cyclonedds_cpp::sertopic_rmw *>(a);
    auto bc = static_cast<const rmw_cyclonedds_cpp::sertopic_rmw *>(b);
    return *ac == *bc;
  },

  (ddsi_sertopic_hash_t)[](const struct ddsi_sertopic * d){
    return static_cast<const rmw_cyclonedds_cpp::sertopic_rmw *>(d)->hash();
  },
#endif
};

sertopic_rmw::~sertopic_rmw() {
  #if DDSI_SERTOPIC_HAS_TOPICKIND_NO_KEY
    ddsi_sertopic_fini(this);
  #endif
}

bool sertopic_rmw::operator==(const sertopic_rmw & other) const {
    /* A bit of a guess: topics with the same name & type name are really the same if they have
       the same type support identifier as well */
    const struct sertopic_rmw * a = static_cast<const struct sertopic_rmw *>(acmn);
    const struct sertopic_rmw * b = static_cast<const struct sertopic_rmw *>(bcmn);
    if (a->is_request_header != b->is_request_header) {
      return false;
    }
    if (strcmp(
      a->type_support.typesupport_identifier_,
      b->type_support.typesupport_identifier_) != 0)
    {
      return false;
    }
    return true;
}
