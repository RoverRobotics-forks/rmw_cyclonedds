//
// Created by dan on 11/14/19.
//

#include "rmw_cyclonedds_cpp/TypeSupport2.hpp"

namespace rmw_cyclonedds_cpp
{
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
}  // namespace rmw_cyclonedds_cpp