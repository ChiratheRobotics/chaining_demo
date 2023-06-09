#ifndef EFFORT_CONTROLLER_HPP
#define EFFORT_CONTROLLER_HPP

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
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
  controller_interface::CallbackReturn on_init() override;

  CHAINING_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  CHAINING_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CHAINING_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void registerJointFeedback(const std::vector<std::string> &);
  void registerJointCommand(const std::vector<std::string> &, const std::vector<double> &);

  int n_joints_;
  std::vector<std::string> joint_names_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
};
}  // namespace chaining_controller
#endif
