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
  // TODO
  // interface_configuration_type::ALL -- for claiming all available state interfaces
  // interface_configuration_type::INDIVIDUAL -- for claiming individual interfaces
  // interface_configuration_type::NONE -- for not claiming anything at all

  // return {FLAG, VECTOR_IF_NAMES}
  return {};
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
        msg->header.stamp = get_node()->get_clock()->now();
      }
      last_msg_ptr_.set(std::move(msg));
    });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type RelayController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  std::shared_ptr<Twist> last_command_msg;
  last_msg_ptr_.get(last_command_msg);
  if (last_command_msg != nullptr)
  {
    RCLCPP_INFO(get_node()->get_logger(), "UPDATED");

    //TODO wink-wink last_command_msg->twist
    // command_interfaces_[0].set_value();
    // command_interfaces_[1].set_value();
    // double fake_steering_angle = ??? * params_.yaw_multiplier;
    // command_interfaces_[2].set_value(fake_steering_angle);
  }
  return controller_interface::return_type::OK;
}

}  // namespace twist_relay_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  twist_relay_controller::RelayController, controller_interface::ControllerInterface)
