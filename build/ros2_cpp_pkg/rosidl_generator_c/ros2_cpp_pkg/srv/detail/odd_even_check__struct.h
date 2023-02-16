// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2_cpp_pkg:srv/OddEvenCheck.idl
// generated code does not contain a copyright notice

#ifndef ROS2_CPP_PKG__SRV__DETAIL__ODD_EVEN_CHECK__STRUCT_H_
#define ROS2_CPP_PKG__SRV__DETAIL__ODD_EVEN_CHECK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/OddEvenCheck in the package ros2_cpp_pkg.
typedef struct ros2_cpp_pkg__srv__OddEvenCheck_Request
{
  /// check if odd or even
  int64_t number;
} ros2_cpp_pkg__srv__OddEvenCheck_Request;

// Struct for a sequence of ros2_cpp_pkg__srv__OddEvenCheck_Request.
typedef struct ros2_cpp_pkg__srv__OddEvenCheck_Request__Sequence
{
  ros2_cpp_pkg__srv__OddEvenCheck_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_cpp_pkg__srv__OddEvenCheck_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'decision'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/OddEvenCheck in the package ros2_cpp_pkg.
typedef struct ros2_cpp_pkg__srv__OddEvenCheck_Response
{
  rosidl_runtime_c__String decision;
} ros2_cpp_pkg__srv__OddEvenCheck_Response;

// Struct for a sequence of ros2_cpp_pkg__srv__OddEvenCheck_Response.
typedef struct ros2_cpp_pkg__srv__OddEvenCheck_Response__Sequence
{
  ros2_cpp_pkg__srv__OddEvenCheck_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_cpp_pkg__srv__OddEvenCheck_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2_CPP_PKG__SRV__DETAIL__ODD_EVEN_CHECK__STRUCT_H_
