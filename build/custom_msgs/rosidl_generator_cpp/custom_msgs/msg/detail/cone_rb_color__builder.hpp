// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/ConeRBColor.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__CONE_RB_COLOR__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__CONE_RB_COLOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/cone_rb_color__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_ConeRBColor_color
{
public:
  explicit Init_ConeRBColor_color(::custom_msgs::msg::ConeRBColor & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::ConeRBColor color(::custom_msgs::msg::ConeRBColor::_color_type arg)
  {
    msg_.color = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::ConeRBColor msg_;
};

class Init_ConeRBColor_bearing
{
public:
  explicit Init_ConeRBColor_bearing(::custom_msgs::msg::ConeRBColor & msg)
  : msg_(msg)
  {}
  Init_ConeRBColor_color bearing(::custom_msgs::msg::ConeRBColor::_bearing_type arg)
  {
    msg_.bearing = std::move(arg);
    return Init_ConeRBColor_color(msg_);
  }

private:
  ::custom_msgs::msg::ConeRBColor msg_;
};

class Init_ConeRBColor_range
{
public:
  Init_ConeRBColor_range()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ConeRBColor_bearing range(::custom_msgs::msg::ConeRBColor::_range_type arg)
  {
    msg_.range = std::move(arg);
    return Init_ConeRBColor_bearing(msg_);
  }

private:
  ::custom_msgs::msg::ConeRBColor msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::ConeRBColor>()
{
  return custom_msgs::msg::builder::Init_ConeRBColor_range();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__CONE_RB_COLOR__BUILDER_HPP_
