# Unitree G1 Drummer

Run this project in Ubuntu 24.04 with ROS 2 Jazzy and RViz2.

## Build and launch

```bash
cd /mnt/hgfs/VM/packages/drummer
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to unitree_drummer
source install/setup.bash
ros2 launch unitree_drummer drummer.launch.py
```

If your checkout is elsewhere, use its project root in the first command. RViz opens with the seated G1, both drumsticks, and the downloaded drum kit.

After editing the kit URDF or RViz config, stop the launch with **Ctrl+C** and rerun the last command. To adjust the kit height, launch with `kit_z:=-0.65` and change the value as needed.

The simulated G1 is 10% larger by default, including its drumsticks. Pass `g1_scale:=1.0` to use the original size. Both RViz and the kinematics node use the scaled joint offsets.

See the [package README](unitree_drummer/README.md) for chart and kinematics details.
