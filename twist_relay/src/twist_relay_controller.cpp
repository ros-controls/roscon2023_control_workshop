// Copyright (c) 2023, ros2_control development team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "twist_relay/twist_relay_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace
{
const auto DEFAULT_COMMAND_TOPIC = "/cmd_vel";
using controller_interface::interface_configuration_type;
}  // namespace

namespace twist_relay_controller
{
RelayController::RelayController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn RelayController::on_init()
{
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RelayController::command_interface_configuration()
  const
{
  std::vector<std::string> conf_names = {
    params_.linear_velocity_cmd_if, params_.angular_velocity_cmd_if, params_.steering_angle_cmd_if};
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::InterfaceConfiguration RelayController::state_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn RelayController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();
  RCLCPP_INFO(get_node()->get_logger(), "Parameters were updated");

  twist_subscriber_ = get_node()->create_subscription<Twist>(
    DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<Twist> msg) -> void
    {
      if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
      {
        RCLCPP_WARN_ONCE(
          get_node()->get_logger(),
          "Received TwistStamped with zero timestamp, setting it to current "
          "time, this message will only be shown once");
        msg->header.stamp = get_node()->get_clock()->now();
      }
      last_msg_ptr_.set(std::move(msg));
    });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type RelayController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Trying to update...");
  std::shared_ptr<Twist> last_command_msg;
  last_msg_ptr_.get(last_command_msg);
  if (last_command_msg != nullptr)
  {
      RCLCPP_INFO(get_node()->get_logger(), "UPDATED");

    command_interfaces_[0].set_value(last_command_msg->twist.linear.x);
    command_interfaces_[1].set_value(last_command_msg->twist.angular.z);
    double fake_steering_angle = last_command_msg->twist.angular.z * params_.yaw_multiplier;
    command_interfaces_[2].set_value(fake_steering_angle);
  }
  return controller_interface::return_type::OK;
}

}  // namespace twist_relay_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  twist_relay_controller::RelayController, controller_interface::ControllerInterface)
