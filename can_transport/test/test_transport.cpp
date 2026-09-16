//
// test_transport.cpp
//
// @author Natesh Narain <nnaraindev@gmail.com>
// @date Sep 15 2026
//
#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "pluginlib/class_loader.hpp"
#include "can_transport/can_transport.hpp"
#include "can_transport/mock_can_transport.hpp"
#include "transport_interface/modbus_transport.hpp"
#include "transport_interface/transport_provider.hpp"

TEST(TestTransportPlugin, LoadMockTransportViaPluginlib)
{
  pluginlib::ClassLoader<transport_interface::TransportInterface> loader(
    "transport_interface", "transport_interface::TransportInterface");

  auto transport = loader.createSharedInstance("can_transport/MockCanTransport");
  ASSERT_NE(transport, nullptr);

  // The mock is a CanTransport — the single downcast a consumer would do
  auto can = std::dynamic_pointer_cast<can_transport::CanTransport>(transport);
  ASSERT_NE(can, nullptr);

  transport_interface::TransportInfo info;
  info.name = "can0";
  info.type = "transport";
  info.plugin_name = "can_transport/MockCanTransport";
  EXPECT_EQ(can->on_init(info), transport_interface::return_type::OK);
  EXPECT_EQ(can->get_name(), "can0");
}

TEST(TestTransportPlugin, MockLoopbackDeliversToRegisteredCallback)
{
  auto mock = std::make_shared<can_transport::MockCanTransport>();
  transport_interface::TransportInfo info;
  info.name = "can0";
  mock->on_init(info);
  mock->on_configure();
  mock->on_activate();

  can_transport::CanFrame received;
  bool got_frame = false;

  mock->register_frame_callback("0x201", [&](const can_transport::CanFrame & frame) {
    received = frame;
    got_frame = true;
  });

  can_transport::CanFrame out;
  out.id = 0x201;
  out.dlc = 2;
  out.data[0] = 0xAB;
  out.data[1] = 0xCD;

  EXPECT_TRUE(mock->send(out, false));
  EXPECT_TRUE(got_frame);
  EXPECT_EQ(received.id, 0x201u);
  EXPECT_EQ(received.dlc, 2u);
  EXPECT_EQ(received.data[0], 0xAB);
  EXPECT_EQ(received.data[1], 0xCD);

  const auto status = mock->get_status();
  EXPECT_EQ(status.state, transport_interface::TransportStatus::State::ACTIVE);
  EXPECT_TRUE(status.link_up);
  EXPECT_GE(status.frames_in, 1u);
  EXPECT_GE(status.frames_out, 1u);

  // Sending to an arbitration ID with no consumer is an error
  can_transport::CanFrame unhandled;
  unhandled.id = 0x999;
  EXPECT_FALSE(mock->send(unhandled, false));
  EXPECT_GE(mock->get_status().errors, 1u);
}

class FakeProvider : public transport_interface::TransportProvider
{
public:
  explicit FakeProvider(std::shared_ptr<transport_interface::TransportInterface> t)
  : transport_(std::move(t))
  {
  }

  // Bring the base class's templated get_transport<T>() into scope; otherwise
  // this override would hide it (name hiding), breaking get_transport<Type>(...) calls.
  using transport_interface::TransportProvider::get_transport;

  std::shared_ptr<transport_interface::TransportInterface> get_transport(
    const std::string & name) const override
  {
    if (name == "can0")
    {
      return transport_;
    }
    return nullptr;
  }

private:
  std::shared_ptr<transport_interface::TransportInterface> transport_;
};

TEST(TestTransportProvider, TypedAccessDowncastsOnceAtInit)
{
  auto mock = std::make_shared<can_transport::MockCanTransport>();
  FakeProvider provider(mock);

  // Correct type: resolves
  auto can = provider.get_transport<can_transport::CanTransport>("can0");
  ASSERT_NE(can, nullptr);

  // Wrong type: single dynamic_pointer_cast yields nullptr (no exception)
  auto modbus = provider.get_transport<transport_interface::ModbusTransport>("can0");
  EXPECT_EQ(modbus, nullptr);

  // Unknown name: nullptr
  auto missing = provider.get_transport<can_transport::CanTransport>("nope");
  EXPECT_EQ(missing, nullptr);
}
