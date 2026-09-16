# ros2_control transport demo

This is a demo for `transport` plugins in ROS 2.

## What is a transport plugin?

The idea is to make a transport or communication bus a first class concept in ros2_control. For example, define CAN bus, MODBUS, EtherCAT transports independent of the hardware components that use them.

## What does it look like?

Basically you can define a `transport` using a ros2_control tag in the URDF. The resource manager loads the transport plugin and makes it available to hardware components.

```xml
  <!-- ============================================================
       Define a transport for the CAN bus.
       ============================================================ -->
  <ros2_control name="can0" type="transport">
    <plugin>socketcan_transport/SocketCanTransport</plugin>
    <param name="interface">vcan0</param>
  </ros2_control>

  <!-- ============================================================
       Referenced int the hardware component as `transport`.
       ============================================================ -->
  <ros2_control name="left_motor" type="actuator">
    <hardware>
      <plugin>ros2_control_transport_demo/TransportDemoActuator</plugin>
      <param name="transport">can0</param>
      <param name="arbitration_id">0x201</param>
    </hardware>
    <joint name="left_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>

  <ros2_control name="right_motor" type="actuator">
    <hardware>
      <plugin>ros2_control_transport_demo/TransportDemoActuator</plugin>
      <param name="transport">can0</param>
      <param name="arbitration_id">0x202</param>
    </hardware>
    <joint name="right_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>

```

## Ok, but why?

* Testability - The hardware component can be decouple from the hardware bus making it testable in CI
* Flexibility - Because the hardware component is decoupled from the hardware, where the data comes from is configurable (CAN data can be socketcan, or CAN-over-IP)
* Better representation of communication interfaces - is the communication link the issue or is the hardware component the issue
* Community owned interfaces - standard the drivers and interfaces the community uses for integrating common devices, making it easier for people to get started
* Common drivers for the community - more run time on the same drivers builds more robust systems
