# mecanum controller

ROS2 mecanum controller node for ABU R0bocon 2024. Specifically designed for Linux Host.

# Required internal dependencies

- geometry_msgs
- nav_msgs
- rclcpp
- std_msgs
- std_srvs
- tf2
- tf2_ros

# Hardware requirement 
- Linux based PC woth ROS2 (Such as Orange Pi)
- ESP32 
- Motors plus encoders
- 

sub
=

- /cmd_vel

pub
=

- /odom
- tf2 transform odom->base_link

# Known issue(s)
- min/max speed is yet to implement
- you tell me