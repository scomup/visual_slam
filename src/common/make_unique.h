#ifndef VISUAL_SLAM_COMMON_MAKE_UNIQUE_H_
#define VISUAL_SLAM_COMMON_MAKE_UNIQUE_H_

#include <cstddef>
#include <memory>
#include <type_traits>
#include <utility>

namespace visual_slam {
namespace common {

// Implementation of c++14's std::make_unique, taken from
// https://isocpp.org/files/papers/N3656.txt
template <class T>
struct _Unique_if {
  typedef std::unique_ptr<T> _Single_object;
};

template <class T>
struct _Unique_if<T[]> {
  typedef std::unique_ptr<T[]> _Unknown_bound;
};

template <class T, size_t N>
struct _Unique_if<T[N]> {
  typedef void _Known_bound;
};

template <class T, class... Args>
typename _Unique_if<T>::_Single_object make_unique(Args&&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

template <class T>
typename _Unique_if<T>::_Unknown_bound make_unique(size_t n) {
  typedef typename std::remove_extent<T>::type U;
  return std::unique_ptr<T>(new U[n]());
}

template <class T, class... Args>
typename _Unique_if<T>::_Known_bound make_unique(Args&&...) = delete;

}  // namespace common
}  // namespace visual_slam

#endif  // VISUAL_SLAM_COMMON_MAKE_UNIQUE_H_
