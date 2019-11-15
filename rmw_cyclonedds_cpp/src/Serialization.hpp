//
// Created by dan on 11/13/19.
//

#ifndef ROS2_MASTER_SERIALIZATION_HPP
#define ROS2_MASTER_SERIALIZATION_HPP

#include "rmw_cyclonedds_cpp/serdata.hpp"
#include <rosidl_generator_c/service_type_support_struct.h>
#include <utility>

namespace rmw_cyclonedds_cpp {

std::pair<rosidl_message_type_support_t, rosidl_message_type_support_t>
get_svc_request_response_typesupports(const rosidl_service_type_support_t *svc);

size_t get_serialized_size(const void *data,
                           const rosidl_message_type_support_t &ts);

void serialize(uint8_t *dest, size_t dest_size, const void *data,
               const rosidl_message_type_support_t &ts);

void serialize(std::vector<uint8_t> &dest, const void *data,
               const rosidl_message_type_support_t &ts);

void serialize(std::vector<uint8_t> &dest,
               const cdds_request_wrapper_t &request,
               const rosidl_message_type_support_t &ts);

} // namespace rmw_cyclonedds_cpp

#endif // ROS2_MASTER_SERIALIZATION_HPP
