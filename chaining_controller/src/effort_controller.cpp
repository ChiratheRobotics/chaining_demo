#include "chaining_controller/effort_controller.hpp"

controller_interface::CallbackReturn chaining_controller::EffortController::on_init()
{
  std::string param_name = "joints";

  if (!get_node()->get_parameter(param_name, joint_names_))
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Failed to get parameter: " << param_name);
    return controller_interface::CallbackReturn::ERROR;
  }

  n_joints_ = joint_names_.size();

  if (n_joints_ == 0)
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "List of joint names is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  joint_position_.resize(n_joints_, std::numeric_limits<double>::quiet_NaN());
  joint_velocity_.resize(n_joints_, std::numeric_limits<double>::quiet_NaN());
  joint_effort_.resize(n_joints_, std::numeric_limits<double>::quiet_NaN());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
chaining_controller::EffortController::command_interface_configuration() const
{
  RCLCPP_INFO(
    get_node()->get_logger(), "Command Interface EffortController. No Command Interface Required");

  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.push_back(
    std::string("chained_controller/") + hardware_interface::HW_IF_EFFORT);
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
chaining_controller::EffortController::state_interface_configuration() const
{
  RCLCPP_INFO(get_node()->get_logger(), "State Interface EffortController.");

  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  std::for_each(
    joint_names_.begin(), joint_names_.end(),
    [&state_interfaces_config](auto & joint_name)
    {
      state_interfaces_config.names.push_back(
        joint_name + "/" + hardware_interface::HW_IF_POSITION);
      state_interfaces_config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
      state_interfaces_config.names.push_back(
        joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    });
  return state_interfaces_config;
}

void chaining_controller::EffortController::jointEffortTester(std::vector<double> & joint_efforts)
{
  joint_efforts[0] = -50 + (rand() % 160);
  RCLCPP_DEBUG_STREAM(
    get_node()->get_logger(),
    "Joint Effort Value From Higher Level Controller:" << joint_efforts[0]);
}
controller_interface::return_type chaining_controller::EffortController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  registerJointFeedback(joint_names_);
  jointEffortTester(joint_effort_);
  registerJointCommand(joint_names_, joint_effort_);
  return controller_interface::return_type::OK;
}

void chaining_controller::EffortController::registerJointFeedback(
  const std::vector<std::string> & joint_names_)
{
  for (int i = 0; i < n_joints_; i++)
  {
    std::string joint_name(joint_names_[i]);
    auto position_state = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(),
      [&joint_name](const hardware_interface::LoanedStateInterface & interface)
      {
        return interface.get_prefix_name() == joint_name &&
               interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
      });
    joint_position_[i] = position_state->get_value();

    auto velocity_state = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(),
      [&joint_name](const hardware_interface::LoanedStateInterface & interface)
      {
        return interface.get_prefix_name() == joint_name &&
               interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
      });
    joint_velocity_[i] = velocity_state->get_value();

    auto effort_state = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(),
      [&joint_name](const hardware_interface::LoanedStateInterface & interface)
      {
        return interface.get_prefix_name() == joint_name &&
               interface.get_interface_name() == hardware_interface::HW_IF_EFFORT;
      });
    joint_effort_[i] = effort_state->get_value();
  }
}

void chaining_controller::EffortController::registerJointCommand(
  const std::vector<std::string> & joint_names, const std::vector<double> & joint_efforts)
{
  for (int i = 0; i < n_joints_; i++)
  {
    std::string joint_name(joint_names[i]);

    auto effort_command = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&joint_name](const hardware_interface::LoanedCommandInterface & interface)
      {
        return interface.get_prefix_name() == "chained_controller" &&
               interface.get_interface_name() == hardware_interface::HW_IF_EFFORT;
      });

    effort_command->set_value(joint_efforts[i]);
  }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  chaining_controller::EffortController, controller_interface::ControllerInterface)
