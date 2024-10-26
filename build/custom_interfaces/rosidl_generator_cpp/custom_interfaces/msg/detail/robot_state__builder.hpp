// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:msg/RobotState.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/msg/detail/robot_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace msg
{

namespace builder
{

class Init_RobotState_state
{
public:
  explicit Init_RobotState_state(::custom_interfaces::msg::RobotState & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::msg::RobotState state(::custom_interfaces::msg::RobotState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::msg::RobotState msg_;
};

class Init_RobotState_robot_id
{
public:
  Init_RobotState_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotState_state robot_id(::custom_interfaces::msg::RobotState::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_RobotState_state(msg_);
  }

private:
  ::custom_interfaces::msg::RobotState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::msg::RobotState>()
{
  return custom_interfaces::msg::builder::Init_RobotState_robot_id();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
