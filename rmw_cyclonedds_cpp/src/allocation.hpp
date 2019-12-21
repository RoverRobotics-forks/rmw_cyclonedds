//
// Created by Dan Rose on 2019-12-20.
//
#ifndef ROS2_MASTER_ALLOCATION_HPP
#define ROS2_MASTER_ALLOCATION_HPP

#include <boost/pool/singleton_pool.hpp>
#include <functional>
#include <memory>
#include <boost/pool/object_pool.hpp>

#include "bytewise.hpp"

using byte_pool_alloc = boost::singleton_pool<boost::pool_allocator_tag, sizeof(byte)>;

static std::unique_ptr<void, std::function<void(void *)>> pool_alloc(size_t size)
{
  auto result = byte_pool_alloc::ordered_malloc(size);
  if (!result) throw std::bad_alloc();
  return {result, [size](void * ptr) { byte_pool_alloc::free(ptr, size); }};
}

template <typename T, typename... Args>
static std::unique_ptr<T, std::function<void(T *)>> pool_make_unique(Args&&... args)
{
  static boost::object_pool<T> pool {};
  auto ptr = pool.malloc();
  if (!ptr) {
    throw std::bad_alloc();
  }
  try {
    new (ptr) T(std::forward<Args>(args)...);
  } catch (...) {
    pool.free(ptr);
    throw;
  }
  return {ptr, [](T * x) {
            x->~T();
            pool.free(x);
          }};
};
  //
  //#include <array>
  //#include <cstddef>
  //#include <deque>
  //#include <vector>
  //
  //#include "bytewise.hpp"
  //
  //class CachingMemoryPool {
  //  static const size_t MIN_SIZE = sizeof(int);
  //  static const size_t NUM_BINS = 32;
  //
  //  std::array<std::deque<void *>, NUM_BINS> storage;
  //
  //  static size_t index_for_size(size_t n_bytes) {
  //    size_t result = 0;
  //    while (n_bytes > sizeof(int)){
  //      result ++;
  //      n_bytes = n_bytes / 2 ;
  //    }
  //  }
  //};
  //
  //
  //template <typename T>
  //class CachingObjectPool {
  //  CachingObjectPool(size_t slack){}
  //  std::vector<T *> objs;
  //  thread_local static size_t hint;
  //
  //
  //  T * get(){
  //    static thread_local size_t x = 0;
  //    objs[x];
  //
  //  }
  //};
  //
  ///// An extremely limited allocation strategy.
  ///// Assumptions:
  ///// 1. One object at a time is needed; no arrays.
  ///// 2. The number of objects at a time is less than capacity (with a little bit of extra margin).
  ///// 3. No object lasts very long.
  //template <typename T>
  //class ObjectPool
  //{
  //  const size_t capacity;
  //  const std::vector<std::atomic_bool> is_allocated;
  //  std::atomic_size_t next_object;
  //  const T * storage;
  //
  //public:
  //  ObjectPool(size_t capacity) : capacity(capacity),
  //    is_allocated(capacity),
  //    next_object{0},
  //    storage{std::malloc(capacity * sizeof(T))} {};
  //
  //  template <typename... Args>
  //  T * construct(Args &&... args)
  //  {
  //    auto index = next_object++;
  //    index %= capacity;
  //    if (!is_allocated[index].compare_exchange_strong(false, true)) {
  //      throw std::bad_alloc();
  //    }
  //    return new (storage + index) T(std::forward<Args>(args)...);
  //  }
  //
  //  void destroy(T * obj)
  //  {
  //    assert(data < obj);
  //    assert(obj < data + capacity);
  //    auto index = obj - storage;
  //    try {
  //      delete obj;
  //    } catch (...) {
  //      is_allocated[index] = false;
  //      throw;
  //    }
  //  }
  //
  //  template <typename... Args>
  //  auto make_unique(Args &&... args)
  //  {
  //    auto deleter = [this](T * obj){destroy(obj);};
  //    const std::unique_ptr<T, decltype(deleter)> result{construct(std::forward<Args>(args)...), deleter};
  //    return std::move(result);
  //  }
  //};

#endif  //ROS2_MASTER_ALLOCATION_HPP
