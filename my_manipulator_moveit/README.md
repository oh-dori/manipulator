# my_manipulator_moveit

MoveIt 2 configuration package for `my_manipulator`.

This package is intentionally separate from `my_manipulator` so the robot model,
ros2_control configuration, Arduino hardware interface, and firmware stay in the
base package while MoveIt-specific files live here.

Expected generated files include:

- `config/*.srdf`
- `config/kinematics.yaml`
- `config/joint_limits.yaml`
- `config/moveit_controllers.yaml`
- `launch/*moveit*.launch.py`
- `rviz/*.rviz`

The MoveIt Setup Assistant should load the robot model from:

```bash
~/ros2_ws/src/manipulator/my_manipulator/urdf/manipulator.xacro
```
