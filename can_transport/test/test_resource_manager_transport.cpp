//
// test_resource_manager_transport.cpp
//
// @author Natesh Narain <nnaraindev@gmail.com>
// @date Sep 15 2026
//

#include <gmock/gmock.h>

#include <memory>
#include <string>

#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "transport_interface/transport_interface.hpp"

constexpr const char * kTransportTestUrdf = R"(
<?xml version="1.0"?>
<robot name="transport_test_robot">
  <link name="base_link"/>
  <link name="left_wheel"/>
  <link name="right_wheel"/>

  <joint name="left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <axis xyz="0 1 0"/>
  </joint>
  <joint name="right_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- One CAN transport declaration, shared by both actuators -->
  <ros2_control name="can0" type="transport">
    <plugin>can_transport/MockCanTransport</plugin>
    <param name="loopback">true</param>
  </ros2_control>

  <ros2_control name="left_motor" type="actuator">
    <hardware>
      <plugin>test_hardware_components/TestTransportConsumerActuator</plugin>
      <param name="transport">can0</param>
      <param name="arbitration_id">0x201</param>
    </hardware>
    <joint name="left_joint" type="continuous">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>

  <ros2_control name="right_motor" type="actuator">
    <hardware>
      <plugin>test_hardware_components/TestTransportConsumerActuator</plugin>
      <param name="transport">can0</param>
      <param name="arbitration_id">0x202</param>
    </hardware>
    <joint name="right_joint" type="continuous">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>
</robot>
 )";

class TestResourceManagerTransports : public ::testing::Test
{
protected:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  static void TearDownTestCase() { rclcpp::shutdown(); }
};

TEST_F(TestResourceManagerTransports, TransportIsLoadedAndShared)
{
  auto node = std::make_shared<rclcpp::Node>("test_transport_node");
  try
  {
    hardware_interface::ResourceManager rm(
      kTransportTestUrdf, node->get_clock(), node->get_logger(), true, 100);

    auto transport = rm.get_transport("can0");
    ASSERT_NE(transport, nullptr);
    EXPECT_EQ(rm.transport_names().size(), 1u);
    EXPECT_EQ(rm.transport_names().at(0), "can0");
    EXPECT_NE(transport->get_name(), std::string());

    EXPECT_TRUE(rm.command_interface_exists("left_joint/velocity"));
    EXPECT_TRUE(rm.state_interface_exists("left_joint/position"));
    EXPECT_TRUE(rm.command_interface_exists("right_joint/velocity"));
    EXPECT_TRUE(rm.state_interface_exists("right_joint/position"));

    auto cmd = rm.claim_command_interface("left_joint/velocity");
    const double kCmd = 1.5;
    EXPECT_TRUE(cmd.set_value(kCmd));

    rclcpp::Time time(0);
    rclcpp::Duration period(0, 1000000);
    EXPECT_EQ(rm.write(time, period).result, hardware_interface::return_type::OK);
    EXPECT_EQ(rm.read(time, period).result, hardware_interface::return_type::OK);

    auto state = rm.claim_state_interface("left_joint/position");
    ASSERT_TRUE(state.get_optional<double>().has_value());
    EXPECT_NEAR(state.get_optional<double>().value(), kCmd, 1e-6);
  }
  catch (const std::exception & e)
  {
    FAIL() << "Exception in test: " << e.what();
  }
}