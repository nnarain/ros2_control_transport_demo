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

#ifndef CAN_TRANSPORT__CAN_TRANSPORT_HPP_
#define CAN_TRANSPORT__CAN_TRANSPORT_HPP_

#include <cstdint>
#include <functional>
#include <string>

#include "transport_interface/transport_interface.hpp"

namespace can_transport
{

/// A CAN frame as seen by a transport consumer.
struct CanFrame
{
  uint32_t id = 0;    // arbitration ID
  uint8_t dlc = 0;
  uint8_t data[8] = {};
  bool is_extended = false;
};

/// Async receive: the transport's own thread (or mock loopback) dispatches
/// frames to callbacks by arbitration ID.
using FrameCallback = std::function<void(const CanFrame & frame)>;

/**
 * @brief The generic "CAN transport" interface.
 *
 * SocketCAN implements it today; CAN-over-IP could implement it tomorrow.
 * Hardware components never see the concrete driver — they hold a typed
 * shared_ptr<CanTransport> obtained from a TransportProvider.
 */
class CanTransport : public transport_interface::TransportInterface
{
public:
  /// Register a callback for frames arriving on the given arbitration ID.
  virtual void register_frame_callback(
    const std::string & arbitration_id, FrameCallback cb) = 0;

  /// Non-blocking write into the transport's TX path.
  virtual bool send(const CanFrame & frame, bool wait_for_lock = false) = 0;
};

}  // namespace can_transport

#endif  // CAN_TRANSPORT__CAN_TRANSPORT_HPP_
