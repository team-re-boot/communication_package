# communication_package

This is a ROS 2 package responsible for communication between robots in the RoboCup Humanoid League.

## Overview of the communication system

In the kid-size game, there are four robots playing soccer on the field, with two replacement robots waiting off the field.
A computer with a wired connection is used by the team members to monitor the robots.

The referee ([GameController](https://github.com/RoboCup-Humanoid-TC/GameController)) sends messages using UDP.
All robots and a monitoring computer can receive the packets.
The `gc_receiver` node of this package translates the packet into the ROS 2 msg defined in [communication_interfaces](https://github.com/team-re-boot/communication_interfaces) and publishes it to the `/game_controller_receiver/gc_data` topic.

The `com_sender` and `com_receiver` nodes (to be defined in this package) are used for communication between robots.
These nodes are compatible with the protocols defined in [RoboCup-Humanoid-TC/mitecom](https://github.com/RoboCup-Humanoid-TC/mitecom) and [RoboCup-Humanoid-TC/RobocupProtocol](https://github.com/RoboCup-Humanoid-TC/RobocupProtocol).
Our own protocol may be defined in the future.

The total amount of data that robots in a team can send is limited to 1 Mbits/s.
Robots playing soccer send UDP packets containing their state.
Substitute robots don't send any packets.
Monitoring PC is not allowed to send any information.
Switching between sending packets or not is done automatically.
All packets are received by all robots and the monitor PC.

On the monitoring computer, all data from the robots and the referee is translated into ROS 2 messages by `com_receiver` and `gc_receiver`.
The [communication_rviz_plugin](https://github.com/team-re-boot/communication_rviz_plugin) is developed to help team members monitor the data.
