# Unitree drummer starter

Python starter for Ubuntu 24.04, ROS 2 Jazzy, and RViz2. Like the HW7 node, it calls an update function on a 100 Hz ROS timer and publishes joint states. At startup the model holds an approximate seated pose. The drum chart and both stick-tip frames are ready for the kinematics implementation.

## Build and run

Run from the project checkout containing `unitree_drummer`, `g1_description`, and `wide_drum_kit.urdf`:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to unitree_drummer
source install/setup.bash
ros2 launch unitree_drummer drummer.launch.py
```

In RViz, set the fixed frame to `pelvis`. Add two **RobotModel** displays with **Description Source** set to **Topic**. Use `/robot_description` for the G1 and `/drum_kit/robot_description` for the kit. The launch file starts both publishers. Stop older copies of those publishers or close your previous demo launch before starting this one.

From another directory, pass the full path as `kit_urdf:=/path/to/wide_drum_kit.urdf`. Use `kit_z:=...` to change the kit's vertical offset relative to the pelvis and `rviz:=false` to run without the GUI. Pass a custom chart with `chart:=/path/to/song.json`. The example chart is [test_groove.json](config/test_groove.json).

## Where to code

Open [kinematics.py](unitree_drummer/kinematics.py). Build chains from `pelvis` to `left_drumstick_tip` and `right_drumstick_tip`. The actual URDF text and active joint names are passed into the class. Model waist joints as shared variables and keep both legs at the seated pose.

`DrummerKinematics.update(time_s, hits, previous)` receives time since song start, chart events, and the previous upper-body joint positions in radians. Return a mapping with every active joint. The placeholder returns the previous pose. Each chart event gives a time, a drumhead frame, and a priority. Events at the same time are simultaneous. Implement FK, target generation, and IK here. For a first pass, try one hit before coordinating both arms and applying the priority rule. See [Kinematics.md](../Kinematics.md) for the planned solver and tests.

The first chart hit has a two-second pre-roll after the joint-state publisher connects. To use TF in the solver, add a listener to the node or use the kit URDF and the launch offset to resolve the head frames. A sample, once the launch is running:

```bash
ros2 run tf2_ros tf2_echo pelvis left_drumstick_tip
ros2 run tf2_ros tf2_echo pelvis snare_head
```

This launch is for RViz visualization. Hit detection, audio playback, and hardware control are future implementation work.
