roscon2023_control_workshop ![Level](https://img.shields.io/badge/Level-Intermediate-red.svg)
================================================================================================

Support files for the ros2_control workshop at ROSCon 2023 in New Orleans, Louisiana, USA.

![Licence](https://img.shields.io/badge/License-Apache_2.0-blue.svg)


# Installation

Create a new workspace for ROS 2 rolling and clone this repository.
Then add all the dependencies from source using `vcs` command:
```
<enter your workspace's src folder>
vcs import --input roscon2023_control_workshop/roscon2023_control_workshop.ci.repos .
````

Then compile your workspace. In this case is is recommended to use `--symlink-install` flag for faster testing of changes.


# Workshop Tasks and Examples

## x.Complex Hardware - Hercules

1. Try starting part of the Hercules with two URe5 arms and grippers:
   ```
   ros2 launch hercules_description dual_ur_sim_control.launch.py
   ```
   And execute example movements in another terminal using:
   ```
   ros2 launch hercules_description test_dual_ur_controllers.launch.py
   ```

1. Start the full Hercules setup and test movements executing the following command in separate terminals:
   ```
   ros2 launch hercules_description hercules_sim_control.launch.py
   ```
   ```
   ros2 launch hercules_description test_hercules_controllers.launch.py
   ```
