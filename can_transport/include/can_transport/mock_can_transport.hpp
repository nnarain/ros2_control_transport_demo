//
// mock_can_transport.hpp
//
// @author Natesh Narain <nnaraindev@gmail.com>
// @date Sep 15 2026
//

#ifndef CAN_TRANSPORT__MOCK_CAN_TRANSPORT_HPP_
#define CAN_TRANSPORT__MOCK_CAN_TRANSPORT_HPP_

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>

#include "can_transport/can_transport.hpp"

namespace can_transport
{

/**
 * @brief In-memory loopback CAN transport for tests and demos.
 *
 * send() delivers the frame synchronously to the callback registered for that
 * arbitration ID. Deterministic, thread-safe, no hardware required.
 */
class MockCanTransport : public CanTransport
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
  void register_frame_callback(const std::string & arbitration_id, FrameCallback cb) override;
  bool send(const CanFrame & frame, bool wait_for_lock) override;

private:
  std::mutex cb_mutex_;
  std::unordered_map<std::string, FrameCallback> callbacks_;

  mutable std::atomic<bool> link_up_{false};
  mutable std::atomic<uint64_t> frames_in_{0};
  mutable std::atomic<uint64_t> frames_out_{0};
  mutable std::atomic<uint64_t> errors_{0};
};

}  // namespace can_transport

#endif  // CAN_TRANSPORT__MOCK_CAN_TRANSPORT_HPP_
