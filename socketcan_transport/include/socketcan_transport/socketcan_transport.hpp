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

#ifndef SOCKETCAN_TRANSPORT__SOCKETCAN_TRANSPORT_HPP_
#define SOCKETCAN_TRANSPORT__SOCKETCAN_TRANSPORT_HPP_

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "transport_interface/can_transport.hpp"

namespace socketcan_transport
{

/**
 * @brief SocketCAN transport plugin: a raw CAN socket with an async receive
 * thread dispatching frames to per-arbitration-ID callbacks.
 *
 * This is the "real" counterpart of transport_interface::MockCanTransport and
 * mirrors the production Connection base class + interface-plugin pattern:
 * the receive thread pushes callbacks; send() writes frames to the socket.
 * Declared once per bus in the URDF as <ros2_control type="transport"> and
 * shared by any number of hardware components.
 */
class SocketCanTransport : public transport_interface::CanTransport
{
public:
  // --- lifecycle ---
  transport_interface::return_type on_init(const transport_interface::TransportInfo & info) override;
  transport_interface::return_type on_configure() override;
  transport_interface::return_type on_activate() override;
  transport_interface::return_type on_deactivate() override;
  transport_interface::return_type on_shutdown() override;
  transport_interface::TransportStatus get_status() const override;

  // --- CanTransport ---
  void register_frame_callback(
    const std::string & arbitration_id, transport_interface::FrameCallback cb) override;
  bool send(const transport_interface::CanFrame & frame, bool wait_for_lock) override;

private:
  void receive_loop();

  int sock_ = -1;
  std::string iface_;
  std::thread rx_thread_;
  std::atomic<bool> running_{false};

  std::mutex cb_mutex_;
  std::unordered_map<std::string, transport_interface::FrameCallback> callbacks_;

  // Status counters (read by get_status(), written by the rx thread / send())
  std::atomic<bool> link_up_{false};
  std::atomic<uint64_t> frames_in_{0};
  std::atomic<uint64_t> frames_out_{0};
  std::atomic<uint64_t> errors_{0};
};

}  // namespace socketcan_transport

#endif  // SOCKETCAN_TRANSPORT__SOCKETCAN_TRANSPORT_HPP_
