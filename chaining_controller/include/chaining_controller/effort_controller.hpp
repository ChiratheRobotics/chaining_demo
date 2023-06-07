// Copyright (c) 2020-2022 Indian Institute of Science
// Copyright (c) 2023 Chirathe Robotics
// All rights reserved.
/// @file foot_state_estimator.hpp
/// @author Aditya Sagi
/// @date 21 Apr, 2022
/// @brief Description: Library to create foot estimator controller

#ifndef EFFORT_CONTROLLER_HPP
#define EFFORT_CONTROLLER_HPP

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <string>

#include "chaining_controller/visibility_control.h"

namespace chaining_controller
{

class EffortController : public controller_interface::ControllerInterface
{
public:
  CHAINING_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  CHAINING_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CHAINING_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CHAINING_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

private:
  void registerJointFeedback(const std::vector<std::string> &);

  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
};
}  // namespace chaining_controller
#endif