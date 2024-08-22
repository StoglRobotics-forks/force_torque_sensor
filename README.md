# force_torque_sensor

[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

ROS2 script for calibrating force-torque sensors. Uses `WrenchStamped` topic as input.
Outputs `CoG` (Center of Gravity), `Fg` (Force of Gravity) and sensor offset values.