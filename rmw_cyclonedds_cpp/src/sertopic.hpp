//
// Created by Dan Rose on 2020-04-18.
//

#ifndef ROS2_MASTER_SERTOPIC_HPP
#define ROS2_MASTER_SERTOPIC_HPP

#include "dds/ddsi/ddsi_sertopic.h"
#include "TypeSupport2.hpp"
#include "serdata.hpp"

namespace rmw_cyclonedds_cpp
{

class sertopic_rmw : public ddsi_sertopic
{
public:
  CddsTypeSupport type_support;
  bool is_request_header;
#if !DDSI_SERTOPIC_HAS_TOPICKIND_NO_KEY
  std::string cpp_name;
  std::string cpp_type_name;
  std::string cpp_name_type_name;
#endif
  std::unique_ptr<const BaseCDRWriter> cdr_writer;

  sertopic_rmw(
    const char * topicname, const char * type_support_identifier, void * type_support,
    bool is_request_header,
    std::__1::unique_ptr<rmw_cyclonedds_cpp::StructValueType> message_type_support);

  ~sertopic_rmw();

  uint32_t hash() const;

  bool operator==(const sertopic_rmw & other) const;

  static const struct ddsi_sertopic_ops ddsi_ops;
};

}  // namespace rmw_cyclonedds_cpp
#endif  //ROS2_MASTER_SERTOPIC_HPP
