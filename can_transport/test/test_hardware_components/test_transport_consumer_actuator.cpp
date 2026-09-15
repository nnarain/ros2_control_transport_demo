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

#include "can_transport/can_transport.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace test_hardware_components
{

class TestTransportConsumerActuator : public hardware_interface::ActuatorInterface
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
      RCLCPP_ERROR(get_logger(), "No transport provider available for component '%s'",
                   params.hardware_info.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const auto & hw_params = params.hardware_info.hardware_parameters;
    can_ = params.transport_provider->get_transport<can_transport::CanTransport>(
      hw_params.at("transport"));
    if (!can_)
    {
      RCLCPP_ERROR(get_logger(), "Transport '%s' not found or not a CanTransport",
                   hw_params.at("transport").c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    arb_id_ = static_cast<uint32_t>(std::stoul(hw_params.at("arbitration_id"), nullptr, 0));
    char key[16];
    std::snprintf(key, sizeof(key), "0x%X", arb_id_);
    can_->register_frame_callback(key, [this](const can_transport::CanFrame & frame) {
      float value = 0.0f;
      std::memcpy(&value, frame.data, sizeof(value));
      feedback_.store(static_cast<double>(value));
    });

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
    double command = 0.0;
    std::ignore = velocity_command_interface_->get_value(command, true);
    can_transport::CanFrame frame;
    frame.id = arb_id_;
    frame.dlc = sizeof(float);
    const float value = static_cast<float>(command);
    std::memcpy(frame.data, &value, sizeof(value));
    return can_->send(frame, false) ? hardware_interface::return_type::OK
                                    : hardware_interface::return_type::ERROR;
  }

private:
  std::shared_ptr<can_transport::CanTransport> can_;
  uint32_t arb_id_ = 0;
  std::atomic<double> feedback_{0.0};
  hardware_interface::StateInterface::SharedPtr position_state_interface_;
  hardware_interface::CommandInterface::SharedPtr velocity_command_interface_;
};

}  // namespace test_hardware_components

PLUGINLIB_EXPORT_CLASS(
  test_hardware_components::TestTransportConsumerActuator, hardware_interface::ActuatorInterface)