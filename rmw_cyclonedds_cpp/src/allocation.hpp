//
// Created by dan on 12/23/19.
//

#ifndef ROS2_MASTER_ALLOCATION_HPP
#define ROS2_MASTER_ALLOCATION_HPP

#include <rcl/types.h>

#include <boost/pool/object_pool.hpp>

#include "serdata.hpp"

class PublisherAllocationImpl
{
protected:
  boost::object_pool<serdata_rmw> serdata_rmw_pool;
  boost::pool<> raw_data_pool;

public:
  static PublisherAllocationImpl * from_rmw(rmw_publisher_allocation_t * allocation)
  {
    assert(allocation->implementation_identifier == eclipse_cyclonedds_identifier);
    return static_cast<PublisherAllocationImpl *>(allocation->data);
  }

  PublisherAllocationImpl() : serdata_rmw_pool{}, raw_data_pool(sizeof(max_align_t)) {}

  void * alloc_serdata_rmw() { return serdata_rmw_pool.malloc(); }
  void destroy_serdata_rmw(serdata_rmw * obj) { return serdata_rmw_pool.destroy(obj); }

  void * alloc_raw_data(size_t n_bytes)
  {
    return raw_data_pool.ordered_malloc(n_bytes / raw_data_pool.get_requested_size());
  }
  void free_raw_data(void * buffer, size_t n_bytes)
  {
    return raw_data_pool.free(buffer, n_bytes / raw_data_pool.get_requested_size());
  }
};

#endif  //ROS2_MASTER_ALLOCATION_HPP
