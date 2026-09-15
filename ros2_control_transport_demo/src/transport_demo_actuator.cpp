// Copyright 2026 ros2_control Development Team
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

#include <atomic>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "transport_interface/can_transport.hpp"

namespace transport_demo
{

/**
 * @brief Demo CAN motor actuator consuming a shared CanTransport.
 *
 * Resolves its transport by name in on_init() via
 * HardwareComponentParams::transport_provider (the single downcast), registers a
 * frame callback for its arbitration ID, and in write() sends the velocity
 * command as a CAN frame. With the MockCanTransport the command loops back and
 * appears as the position state — no hardware required.
 */
class TransportDemoActuator : public hardware_interface::ActuatorInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    if (ActuatorInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!params.transport_provider)
    {
      RCLCPP_ERROR(
        get_logger(), "No transport provider available for component '%s'",
        params.hardware_info.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const auto & hw_params = params.hardware_info.hardware_parameters;
    if (hw_params.count("transport") == 0)
    {
      RCLCPP_ERROR(get_logger(), "Missing 'transport' parameter in <hardware> block");
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_logger(), "Resolving transport '%s'", hw_params.at("transport").c_str());
    can_ = params.transport_provider->get_transport<transport_interface::CanTransport>(
      hw_params.at("transport"));
    if (!can_)
    {
      RCLCPP_ERROR(
        get_logger(), "Transport '%s' not found or not a CanTransport",
        hw_params.at("transport").c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_logger(), "Transport '%s' successfully resolved", hw_params.at("transport").c_str());

    arb_id_ = static_cast<uint32_t>(std::stoul(hw_params.at("arbitration_id"), nullptr, 0));
    char key[16];
    std::snprintf(key, sizeof(key), "0x%X", arb_id_);
    can_->register_frame_callback(key, [this](const transport_interface::CanFrame & f) {
      float v = 0.0f;
      std::memcpy(&v, f.data, sizeof(v));
      feedback_.store(static_cast<double>(v));
    });

    RCLCPP_INFO(
      get_logger(), "Component '%s' bound to transport '%s' on arb ID 0x%X",
      params.hardware_info.name.c_str(), hw_params.at("transport").c_str(), arb_id_);
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface::ConstSharedPtr>
  on_export_state_interfaces() override
  {
    position_state_interface_ = std::make_shared<hardware_interface::StateInterface>(
      get_hardware_info().joints[0].name, hardware_interface::HW_IF_POSITION);
    return {position_state_interface_};
  }

  std::vector<hardware_interface::CommandInterface::SharedPtr>
  on_export_command_interfaces() override
  {
    velocity_command_interface_ = std::make_shared<hardware_interface::CommandInterface>(
      get_hardware_info().joints[0].name, hardware_interface::HW_IF_VELOCITY);
    return {velocity_command_interface_};
  }

  hardware_interface::return_type read(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    std::ignore = position_state_interface_->set_value(feedback_.load(), true);
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    double v_cmd = 0.0;
    std::ignore = velocity_command_interface_->get_value(v_cmd, true);
    transport_interface::CanFrame frame;
    frame.id = arb_id_;
    frame.dlc = sizeof(float);
    const float v = static_cast<float>(v_cmd);
    std::memcpy(frame.data, &v, sizeof(v));
    if (!can_->send(frame, false))
    {
      RCLCPP_ERROR(get_logger(), "Failed to send frame on arb ID 0x%X", arb_id_);
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }

private:
  std::shared_ptr<transport_interface::CanTransport> can_;
  uint32_t arb_id_ = 0;
  std::atomic<double> feedback_{0.0};
  hardware_interface::StateInterface::SharedPtr position_state_interface_;
  hardware_interface::CommandInterface::SharedPtr velocity_command_interface_;
};

}  // namespace transport_demo

PLUGINLIB_EXPORT_CLASS(
  transport_demo::TransportDemoActuator, hardware_interface::ActuatorInterface)
