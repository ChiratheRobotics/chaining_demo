#ifndef CHAINED_CONTROLLER_HPP
#define CHAINED_CONTROLLER_HPP

#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include "chaining_controller/visibility_control.h"
#include "pluginlib/class_list_macros.hpp"

namespace chaining_controller
{

class ChainedController : public controller_interface::ChainableControllerInterface
{
public:
  CHAINING_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  CHAINING_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  CHAINING_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  bool on_set_chained_mode(bool chained_mode) override;

  controller_interface::return_type update_reference_from_subscribers() override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  bool update();

private:
  double EFFORT_MAX = 1000;
  double EFFORT_MIN = -1000;

  void registerJointCommand(const std::vector<double> & effort);

  std::vector<double> joint_effort_command_;

  size_t n_joints_;
  std::vector<std::string> joint_names_;
};

template <typename T>
T clip(const T & n, const T & lower, const T & upper)
{
  return std::max(lower, std::min(n, upper));
}

}  // namespace chaining_controller

#endif
