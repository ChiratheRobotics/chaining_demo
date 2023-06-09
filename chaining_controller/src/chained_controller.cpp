#include "chaining_controller/chained_controller.hpp"

controller_interface::CallbackReturn chaining_controller::ChainedController::on_init()
{
  std::string param_name = "joints";

  // Receiving Joint Names from Config file
  if (!get_node()->get_parameter(param_name, joint_names_))
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Failed to get parameter: " << param_name);
    return controller_interface::CallbackReturn::ERROR;
  }

  // Joint size
  n_joints_ = joint_names_.size();

  if (n_joints_ == 0)
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "List of joint names is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Resize this value depending on reference interfaces to be sent
  reference_interfaces_.resize(n_joints_, std::numeric_limits<double>::quiet_NaN());
  joint_effort_command_.resize(n_joints_, std::numeric_limits<double>::quiet_NaN());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
chaining_controller::ChainedController::command_interface_configuration() const
{
  RCLCPP_INFO(
    get_node()->get_logger(),
    "Command Interface ChainedController. Command Interface Required: Effort");

  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Sending Command Interface value to Resource manager after Clipping it
  std::for_each(
    joint_names_.begin(), joint_names_.end(),
    [&command_interfaces_config](auto & joint_name)
    {
      command_interfaces_config.names.push_back(
        joint_name + "/" + hardware_interface::HW_IF_EFFORT);
    });

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
chaining_controller::ChainedController::state_interface_configuration() const
{
  RCLCPP_INFO(get_node()->get_logger(), "State Interface ChainedController.");

  // No State Interfaces
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::NONE;

  return state_interfaces_config;
}

std::vector<hardware_interface::CommandInterface>
chaining_controller::ChainedController::on_export_reference_interfaces()
{
  RCLCPP_INFO(get_node()->get_logger(), "export_reference_interfaces");

  std::vector<hardware_interface::CommandInterface> reference_interfaces;

  std::string reference_interface_name = "effort";

  // Exporting reference interfaces to Higher Level Controller
  // Note: Name (or prefix name) of the controller should be name of the controller itself.
  // In our case it's "chaining_controller"
  for (size_t i = 0; i < n_joints_; i++)
  {
    reference_interfaces.push_back(hardware_interface::CommandInterface(
      std::string(get_node()->get_name()), reference_interface_name, &reference_interfaces_[i]));
  }

  return reference_interfaces;
}

bool chaining_controller::ChainedController::on_set_chained_mode(bool chained_mode)
{
  RCLCPP_INFO(get_node()->get_logger(), "CHAINED MODE ACTIVE");

  chained_mode = true;
  return chained_mode;
}

controller_interface::return_type
chaining_controller::ChainedController::update_reference_from_subscribers()
{
  RCLCPP_INFO_ONCE(get_node()->get_logger(), "update_reference_from_subscribers");
  return update() ? controller_interface::return_type::OK
                  : controller_interface::return_type::ERROR;
}

controller_interface::return_type chaining_controller::ChainedController::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_INFO_ONCE(get_node()->get_logger(), "update_and_write_commands");
  return update() ? controller_interface::return_type::OK
                  : controller_interface::return_type::ERROR;
}

bool chaining_controller::ChainedController::update()
{
  // Clipping Value
  for (size_t i = 0; i < n_joints_; i++)
  {
    reference_interfaces_[i] = clip(reference_interfaces_[i], EFFORT_MIN, EFFORT_MAX);
  }
  // Registering the Command Interfaces
  registerJointCommand(reference_interfaces_);
  return true;
}
void chaining_controller::ChainedController::registerJointCommand(
  const std::vector<double> & effort)
{
  for (size_t i = 0; i < n_joints_; i++)
  {
    std::string joint_name(joint_names_[i]);

    auto effort_command = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&joint_name](const hardware_interface::LoanedCommandInterface & interface)
      {
        return interface.get_prefix_name() == joint_name &&
               interface.get_interface_name() == hardware_interface::HW_IF_EFFORT;
      });

    effort_command->set_value(effort[i]);
  }
}

PLUGINLIB_EXPORT_CLASS(
  chaining_controller::ChainedController, controller_interface::ChainableControllerInterface)
