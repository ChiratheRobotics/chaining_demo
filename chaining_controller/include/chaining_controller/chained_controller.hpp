#ifndef CHAINED_CONTROLLER_HPP
#define CHAINED_CONTROLLER_HPP

// Libraries
#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include "chaining_controller/visibility_control.h"
#include "pluginlib/class_list_macros.hpp"

namespace chaining_controller
{

/// @brief Chained Controller class to send Reference Interfaces to Higher Level Controller
class ChainedController : public controller_interface::ChainableControllerInterface
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

protected:
  /// @brief Export reference_interfaces_ to Higher level controller
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  /// @brief Setting Chained Mode for Controller Chaining
  /// @param chained_mode True if Chaining Mode Activated, else False
  /// @return Bool of Chaining Mode
  bool on_set_chained_mode(bool chained_mode) override;

  /// @brief Update Interfaces from subscribers. This should be using a realtime subscriber if Chaining mode is false
  /// @return Controller Interface Success
  controller_interface::return_type update_reference_from_subscribers() override;

  /// @brief Update Interface from update of High Level Controller. Chaining Mode is true
  /// @param time Current Time
  /// @param period Current Period
  /// @return Controller Interface Success
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// @brief Update method for both the methods for
  /// @return If Successful then True, else false
  bool update();

private:
  /// @brief Max Effort to be clipped
  double EFFORT_MAX = 100;

  /// @brief Min Effort to be clipped
  double EFFORT_MIN = -100;

  /// @brief Register Joint Commands in Command Interfaces
  /// @param effort Efforts to set in command interfaces
  void registerJointCommand(const std::vector<double> & effort);

  /// @brief Vector of Joint Effort Commands
  std::vector<double> joint_effort_command_;

  /// @brief Number of joints
  size_t n_joints_;

  /// @brief Vector of Joint Names
  std::vector<std::string> joint_names_;
};

/// @brief Clipping the effort into upper and lower limits
/// @tparam T is type of input. double in our case
/// @param n Value that has to be clipped
/// @param lower Lower limit
/// @param upper Upper Limit
/// @return Clipped Value
template <typename T>
T clip(const T & n, const T & lower, const T & upper)
{
  return std::max(lower, std::min(n, upper));
}

}  // namespace chaining_controller

#endif
