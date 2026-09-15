//
// socketcan_transport.cpp
//
// @author Natesh Narain <nnaraindev@gmail.com>
// @date Sep 14 2026
//

#include "socketcan_transport/socketcan_transport.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <utility>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "can_transport/can_transport.hpp"
#include "socketcan_adapter/socketcan_adapter.hpp"

namespace socketcan_transport
{

namespace
{
constexpr const char * LOGGER_NAME = "socketcan_transport";

std::string frame_key(const polymath::socketcan::CanFrame & frame)
{
  char key_buf[16];
  std::snprintf(key_buf, sizeof(key_buf), "0x%X", frame.get_id());
  return key_buf;
}

can_transport::CanFrame to_transport_frame(const polymath::socketcan::CanFrame & frame)
{
  can_transport::CanFrame result;
  result.id = frame.get_id();
  result.dlc = std::min<uint8_t>(frame.get_len(), CAN_MAX_DLC);
  result.is_extended = frame.get_id_type() == polymath::socketcan::IdType::EXTENDED;

  const auto data = frame.get_data();
  std::copy_n(data.begin(), result.dlc, result.data);
  return result;
}

polymath::socketcan::CanFrame to_socketcan_frame(const can_transport::CanFrame & frame)
{
  std::array<unsigned char, CAN_MAX_DLC> data{};
  const auto dlc = std::min<uint8_t>(frame.dlc, CAN_MAX_DLC);
  std::copy_n(frame.data, dlc, data.begin());

  polymath::socketcan::CanFrame result(
    frame.id, data, 0U, dlc);
  if (frame.is_extended)
  {
    result.set_id_as_extended();
  }
  return result;
}
}  // namespace

SocketCanTransport::~SocketCanTransport() = default;

transport_interface::return_type SocketCanTransport::on_init(
  const transport_interface::TransportInfo & info)
{
  info_ = info;
  const auto it = info.parameters.find("interface");
  if (it == info.parameters.end())
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Missing 'interface' parameter");
    return transport_interface::return_type::ERROR;
  }
  iface_ = it->second;
  return transport_interface::return_type::OK;
}

transport_interface::return_type SocketCanTransport::on_configure()
{
  adapter_ = std::make_unique<polymath::socketcan::SocketcanAdapter>(iface_);
  if (!adapter_->openSocket())
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Failed to open CAN socket on '%s'", iface_.c_str());
    adapter_.reset();
    return transport_interface::return_type::ERROR;
  }
  return transport_interface::return_type::OK;
}

transport_interface::return_type SocketCanTransport::on_activate()
{
  if (!adapter_)
  {
    return transport_interface::return_type::ERROR;
  }

  adapter_->setOnReceiveCallback(
    [this](std::unique_ptr<const polymath::socketcan::CanFrame> frame) {
      const auto transport_frame = to_transport_frame(*frame);
      frames_in_.fetch_add(1);
      link_up_ = true;

      can_transport::FrameCallback callback;
      {
        std::lock_guard<std::mutex> lock(cb_mutex_);
        const auto it = callbacks_.find(frame_key(*frame));
        if (it != callbacks_.end())
        {
          callback = it->second;
        }
      }
      if (callback)
      {
        callback(transport_frame);
      }
    });
  adapter_->setOnErrorCallback(
    [this](const std::string & error) {
      if (error.find("timed out") == std::string::npos)
      {
        errors_.fetch_add(1);
        link_up_ = false;
      }
    });

  if (!adapter_->startReceptionThread())
  {
    return transport_interface::return_type::ERROR;
  }
  running_ = true;
  link_up_ = true;
  return transport_interface::return_type::OK;
}

void SocketCanTransport::register_frame_callback(
  const std::string & arbitration_id, can_transport::FrameCallback cb)
{
  std::lock_guard<std::mutex> lock(cb_mutex_);
  callbacks_[arbitration_id] = std::move(cb);
}

bool SocketCanTransport::send(const can_transport::CanFrame & frame, bool /*wait_for_lock*/)
{
  if (!adapter_ || adapter_->get_socket_state() != polymath::socketcan::SocketState::OPEN)
  {
    errors_.fetch_add(1);
    return false;
  }

  if (adapter_->send(to_socketcan_frame(frame)))
  {
    errors_.fetch_add(1);
    return false;
  }

  frames_out_.fetch_add(1);
  return true;
}

transport_interface::return_type SocketCanTransport::on_deactivate()
{
  if (adapter_ && !adapter_->joinReceptionThread())
  {
    return transport_interface::return_type::ERROR;
  }
  running_ = false;
  link_up_ = false;
  return transport_interface::return_type::OK;
}

transport_interface::return_type SocketCanTransport::on_shutdown()
{
  if (adapter_)
  {
    if (running_ && !adapter_->joinReceptionThread())
    {
      return transport_interface::return_type::ERROR;
    }
    adapter_->closeSocket();
    adapter_.reset();
    running_ = false;
  }
  return transport_interface::return_type::OK;
}

transport_interface::TransportStatus SocketCanTransport::get_status() const
{
  transport_interface::TransportStatus status;
  status.state =
    running_.load() ? transport_interface::TransportStatus::State::ACTIVE
                    : transport_interface::TransportStatus::State::INACTIVE;
  status.link_up = link_up_.load();
  status.frames_in = frames_in_.load();
  status.frames_out = frames_out_.load();
  status.errors = errors_.load();
  return status;
}

}  // namespace socketcan_transport

PLUGINLIB_EXPORT_CLASS(
  socketcan_transport::SocketCanTransport, transport_interface::TransportInterface)
