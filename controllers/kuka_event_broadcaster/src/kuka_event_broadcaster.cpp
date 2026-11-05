// Copyright 2024 Aron Svastits
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

#include "kuka_drivers_core/hardware_interface_types.hpp"

#include "kuka_event_broadcaster/kuka_event_broadcaster.hpp"

namespace kuka_controllers
{
controller_interface::CallbackReturn EventBroadcaster::on_init()
{
  // Per-arm interface prefix ("left_"); see InterfacePrefixFromIoComponentName.
  prefix_ = auto_declare<std::string>("prefix", "");
  event_publisher_ = get_node()->create_publisher<std_msgs::msg::UInt8>(
    "~/hardware_event", rclcpp::SystemDefaultsQoS());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration EventBroadcaster::command_interface_configuration()
  const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration EventBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.emplace_back(
    prefix_ + hardware_interface::STATE_PREFIX + "/" + hardware_interface::SERVER_STATE);
  return config;
}

controller_interface::CallbackReturn EventBroadcaster::on_configure(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EventBroadcaster::on_activate(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EventBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type EventBroadcaster::update(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  // Publish on every change, and also republish the current event at a fixed interval:
  // a subscriber that joins between changes (a status viewer, `ros2 topic echo`) would
  // otherwise see nothing until the next fault, and the steady stream doubles as a
  // heartbeat -- its absence means the controller, or the driver host, is gone.
  since_last_publish_ = since_last_publish_ + period;
  auto current_event = static_cast<uint8_t>(state_interfaces_[0].get_value());
  if (current_event != last_event_ || since_last_publish_ >= REPUBLISH_INTERVAL)
  {
    event_msg_.data = current_event;
    event_publisher_->publish(event_msg_);
    last_event_ = current_event;
    since_last_publish_ = rclcpp::Duration(0, 0);
  }
  return controller_interface::return_type::OK;
}
}  // namespace kuka_controllers

PLUGINLIB_EXPORT_CLASS(
  kuka_controllers::EventBroadcaster, controller_interface::ControllerInterface)
