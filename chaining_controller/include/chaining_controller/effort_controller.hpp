#ifndef EFFORT_CONTROLLER_HPP
#define EFFORT_CONTROLLER_HPP

// Libraries
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <string>

#include "chaining_controller/visibility_control.h"

namespace chaining_controller
{
/// @brief Effort Controller (Higher Level Controller) to set reference interfaces received from Chainable Controller
class EffortController : public controller_interface::ControllerInterface
{
public:
  /// @brief Documentation Inherited
  CHAINING_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  /// @brief Documentation Inherited
  CHAINING_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /// @brief Documentation Inherited
  CHAINING_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  /// @brief Documentation Inherited
  CHAINING_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// @brief Handler to get state interfaces
  /// @param effort Efforts to get in state interfaces
  void registerJointFeedback(const std::vector<std::string> &);

  /// @brief Handler to set Command Interfaces
  /// @param Vector of Joint Names
  /// @param  Vector of Joint Efforts to set
  void registerJointCommand(const std::vector<std::string> &, const std::vector<double> &);

  /// @brief Method to set Random Values to joint effort
  /// @param  Vector of joint effort
  void jointEffortTester(std::vector<double> &);

  /// @brief Number of joints
  int n_joints_;

  /// @brief Vector of joint names
  std::vector<std::string> joint_names_;

  /// @brief Vector of joint position received from State Interfaces
  std::vector<double> joint_position_;

  /// @brief Vector of joint velocity received from State Interfaces
  std::vector<double> joint_velocity_;

  /// @brief Vector of joint effort received from State Interfaces
  std::vector<double> joint_effort_;
};
}  // namespace chaining_controller
#endif
