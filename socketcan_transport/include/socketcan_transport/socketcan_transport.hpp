//
// socketcan_transport.hpp
//
// @author Natesh Narain <nnaraindev@gmail.com>
// @date Sep 14 2026
//

#ifndef SOCKETCAN_TRANSPORT__SOCKETCAN_TRANSPORT_HPP_
#define SOCKETCAN_TRANSPORT__SOCKETCAN_TRANSPORT_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "can_transport/can_transport.hpp"

namespace polymath::socketcan
{
class SocketcanAdapter;
}

namespace socketcan_transport
{

/**
 * @brief SocketCAN transport plugin, using a base library by polymath robotics.
 */
class SocketCanTransport : public can_transport::CanTransport
{
public:
  ~SocketCanTransport() override;

  // --- lifecycle ---
  transport_interface::return_type on_init(const transport_interface::TransportInfo & info) override;
  transport_interface::return_type on_configure() override;
  transport_interface::return_type on_activate() override;
  transport_interface::return_type on_deactivate() override;
  transport_interface::return_type on_shutdown() override;
  transport_interface::TransportStatus get_status() const override;

  // --- CanTransport ---
  void register_frame_callback(
    const std::string & arbitration_id, can_transport::FrameCallback cb) override;
  bool send(const can_transport::CanFrame & frame, bool wait_for_lock) override;

private:
  std::string iface_;
  std::unique_ptr<polymath::socketcan::SocketcanAdapter> adapter_;
  std::atomic<bool> running_{false};

  std::mutex cb_mutex_;
  std::unordered_map<std::string, can_transport::FrameCallback> callbacks_;

  // Status counters (read by get_status(), written by the rx thread / send())
  std::atomic<bool> link_up_{false};
  std::atomic<uint64_t> frames_in_{0};
  std::atomic<uint64_t> frames_out_{0};
  std::atomic<uint64_t> errors_{0};
};

}  // namespace socketcan_transport

#endif  // SOCKETCAN_TRANSPORT__SOCKETCAN_TRANSPORT_HPP_
