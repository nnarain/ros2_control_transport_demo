//
// mock_can_transport.cpp
//
// @author Natesh Narain <nnaraindev@gmail.com>
// @date Sep 15 2026
//

#include "can_transport/mock_can_transport.hpp"

#include <cstring>
#include <cstdio>
#include <utility>
#include <iostream>

#include "pluginlib/class_list_macros.hpp"

namespace can_transport
{

transport_interface::return_type MockCanTransport::on_init(const transport_interface::TransportInfo & info)
{
  info_ = info;
  return transport_interface::return_type::OK;
}

transport_interface::return_type MockCanTransport::on_configure()
{
  return transport_interface::return_type::OK;
}

transport_interface::return_type MockCanTransport::on_activate()
{
  link_up_ = true;
  return transport_interface::return_type::OK;
}

transport_interface::return_type MockCanTransport::on_deactivate()
{
  link_up_ = false;
  return transport_interface::return_type::OK;
}

transport_interface::return_type MockCanTransport::on_shutdown()
{
  return transport_interface::return_type::OK;
}

transport_interface::TransportStatus MockCanTransport::get_status() const
{
  transport_interface::TransportStatus status;
  status.state = link_up_.load() ? transport_interface::TransportStatus::State::ACTIVE
                                 : transport_interface::TransportStatus::State::INACTIVE;
  status.link_up = link_up_.load();
  status.frames_in = frames_in_.load();
  status.frames_out = frames_out_.load();
  status.errors = errors_.load();
  return status;
}

void MockCanTransport::register_frame_callback(
  const std::string & arbitration_id, FrameCallback cb)
{
  std::cout << "Registering frame callback for arb ID '" << arbitration_id << "'" << std::endl;
  std::lock_guard<std::mutex> lock(cb_mutex_);
  callbacks_[arbitration_id] = std::move(cb);
}

bool MockCanTransport::send(const CanFrame & frame, bool /*wait_for_lock*/)
{
  std::cout << "Sending frame on arb ID 0x" << std::hex << frame.id << std::dec << std::endl;
  char key_buf[16];
  std::snprintf(key_buf, sizeof(key_buf), "0x%X", frame.id);
  const std::string key = key_buf;

  FrameCallback cb;
  {
    std::lock_guard<std::mutex> lock(cb_mutex_);
    const auto it = callbacks_.find(key);
    if (it != callbacks_.end())
    {
      cb = it->second;
    }
  }

  if (cb)
  {
    cb(frame);          // synchronous loopback
    frames_in_.fetch_add(1);
    frames_out_.fetch_add(1);
    return true;
  }

  errors_.fetch_add(1);  // no consumer for this arbitration ID
  return false;
}

}  // namespace can_transport

PLUGINLIB_EXPORT_CLASS(
  can_transport::MockCanTransport, transport_interface::TransportInterface)
