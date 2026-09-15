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

#include "socketcan_transport/socketcan_transport.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <utility>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace socketcan_transport
{

transport_interface::return_type SocketCanTransport::on_init(
  const transport_interface::TransportInfo & info)
{
  info_ = info;
  const auto it = info.parameters.find("interface");
  if (it == info.parameters.end())
  {
    RCLCPP_ERROR(rclcpp::get_logger("socketcan_transport"), "Missing 'interface' parameter");
    return transport_interface::return_type::ERROR;
  }
  iface_ = it->second;
  return transport_interface::return_type::OK;
}

transport_interface::return_type SocketCanTransport::on_configure()
{
  sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock_ < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("socketcan_transport"), "Failed to open CAN socket");
    return transport_interface::return_type::ERROR;
  }

  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, iface_.c_str(), IFNAMSIZ - 1);
  if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("socketcan_transport"), "Failed to get interface index for '%s'",
      iface_.c_str());
    close(sock_);
    sock_ = -1;
    return transport_interface::return_type::ERROR;
  }

  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(sock_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("socketcan_transport"), "Failed to bind CAN socket to '%s'",
      iface_.c_str());
    close(sock_);
    sock_ = -1;
    return transport_interface::return_type::ERROR;
  }

  return transport_interface::return_type::OK;
}

transport_interface::return_type SocketCanTransport::on_activate()
{
  running_ = true;
  rx_thread_ = std::thread(&SocketCanTransport::receive_loop, this);  // async by design
  return transport_interface::return_type::OK;
}

void SocketCanTransport::receive_loop()
{
  struct can_frame frame;
  while (running_)
  {
    const ssize_t n = read(sock_, &frame, sizeof(frame));
    if (n < 0)
    {
      if (errno == EINTR)
      {
        continue;
      }
      errors_.fetch_add(1);  // bus error / bus-off — visible in get_status()
      link_up_ = false;
      continue;
    }
    frames_in_.fetch_add(1);
    link_up_ = true;

    char key_buf[16];
    std::snprintf(key_buf, sizeof(key_buf), "0x%X", frame.can_id);
    const std::string key = key_buf;

    std::lock_guard<std::mutex> lock(cb_mutex_);
    const auto it = callbacks_.find(key);
    if (it != callbacks_.end())
    {
      transport_interface::CanFrame f;
      f.id = frame.can_id;
      f.dlc = frame.can_dlc;
      std::memcpy(f.data, frame.data, frame.can_dlc);
      it->second(f);  // component callback snapshots into atomics — no locking there
    }
  }
}

void SocketCanTransport::register_frame_callback(
  const std::string & arbitration_id, transport_interface::FrameCallback cb)
{
  std::lock_guard<std::mutex> lock(cb_mutex_);
  callbacks_[arbitration_id] = std::move(cb);
}

bool SocketCanTransport::send(const transport_interface::CanFrame & frame, bool /*wait_for_lock*/)
{
  struct can_frame f {};
  f.can_id = frame.id;
  f.can_dlc = frame.dlc;
  std::memcpy(f.data, frame.data, frame.dlc);
  const ssize_t n = write(sock_, &f, sizeof(f));
  if (n < 0)
  {
    errors_.fetch_add(1);
    return false;
  }
  frames_out_.fetch_add(1);
  return true;
}

transport_interface::return_type SocketCanTransport::on_deactivate()
{
  running_ = false;
  if (rx_thread_.joinable())
  {
    rx_thread_.join();
  }
  return transport_interface::return_type::OK;
}

transport_interface::return_type SocketCanTransport::on_shutdown()
{
  if (sock_ >= 0)
  {
    close(sock_);
    sock_ = -1;
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
