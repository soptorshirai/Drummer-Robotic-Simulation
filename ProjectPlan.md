# 133a project plan — Unitree drum-set player

## Objective

Make a seated Unitree humanoid play the drum part for a selected song excerpt. Hold the pelvis and legs in a seated pose and coordinate both arms with the available waist and torso joints. Generate joint trajectories from drum target points and hit times. The 133a contribution is solving for natural, coordinated upper-body motion and choosing which hits to play when the kit layout or timing makes the full part infeasible.

Use one arm and one pad to verify the model and hit detector, then demonstrate both arms and torso with two short song excerpts. Start with a compact two-drum layout and use a wider layout to demonstrate a reach conflict. The song library uses authored drum charts.

## Setup and scope

The baseline runs on **Ubuntu 24.04 with ROS 2 Jazzy, RViz2, and Python (`rclpy`)**. Use RViz2 to display the seated robot, sticks, and drums. Run the scheduler, trajectory playback, and geometric hit detection in Python so timing does not depend on the RViz refresh rate.

| Decision | Working plan |
|---|---|
| Robot | Confirm the available Unitree model, joint configuration, control interface, and matching URDF. |
| Seated base | Fix the pelvis transform and lower-body joints. Enable the model's available upper-body joints for coordinated motion. Store the seated pose and seat-to-kit transform with the demo configuration. |
| Physical seat | The team's initial idea was to glue the lower body to a seat. For hardware, use a lab-approved, removable support that carries the robot and stays stable when power is cut. Confirm that the supported pose and upper-body controller mode are compatible before powered tests. |
| Sticks and targets | Model each stick as a rigid hand-to-tip transform. Set the pads within comfortable arm reach and record their target positions, surface normals, and clearances. |
| Demo | Make the simulation and recorded replay the baseline. Add low-speed hardware trials on a compliant pad if the robot, mount, and control mode are available. |

Unitree publishes robot descriptions by model and variant. Match the model to the available robot before using its limits. [Unitree robot descriptions](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/README.md)

## Song selection and optional controls

Select a song by ID at launch. Each song has a drum chart with timestamps or beat indices and a tempo, drum IDs, and optional musical priorities. Store drum target points in the kit configuration so the same chart works with different layouts. Compute and validate the motion for the selected song and layout before playback. If using a backing track, align it to the same playback start time as the chart.

A song-picker UI, BPM slider, editable beat grid, tap tempo, live pads, and interactive pad placement are stretch goals.

## Implementation

Publish the complete joint state, including animated torso and arm positions, on `/joint_states`. Load the URDF into `robot_state_publisher` to generate TF for RViz's RobotModel display. Use the seated base as the RViz Fixed Frame and publish drumheads, targets, and contact indicators as markers.

Model each drumhead as a circular plane in the seated base frame. On a detected hit, log the time and play the matching preloaded hi-hat or snare sample from Python. Support overlapping samples for simultaneous hits. Add fixed-volume playback first so audio reveals late or missed contacts during the demo.

1. Load the model, set the seated pose, and visualize the sticks and pads. Use forward kinematics to confirm the tip frames and solve static hit and hover poses.
2. Build a timed strike and recovery for one arm. Log commanded hit time, observed or simulated hit time, and tip position.
3. Load a song chart and solve the timed targets with a shared torso and two arms. Use wrist orientation and posture preferences to choose smooth, comfortable strokes. Check clearance and joint limits along the connecting motion.
4. Try feasible arm assignments for simultaneous hits. If the pair is infeasible, keep the higher-priority feasible hit at its original timestamp and log the skipped hit and reason. Use snare backbeats over hi-hat subdivisions as the initial default, with per-song overrides.
5. Play both excerpts and compare the compact and wider layouts. Log detected hits, planner skips, unexpected misses, timing and tip errors, and peak joint speeds.

Divide ownership among robot setup and calibration, motion and timing, and evaluation and demo. Combine the last two as needed for a two-person team. See [suggested kinematics](Kinematics.md) for the model and test procedure.

## Course checkpoints

The dates and submission expectations below come from the course brief. The milestone content is the team's proposed work. The brief calls for one concise team submission for the idea and preliminary update and names Gradescope for submissions.

| Course checkpoint | Proposed milestone |
|---|---|
| Nov 11–13 · Idea discussion | Seated Unitree drummer concept and preliminary goals. |
| Nov 22, 23, or 25 · Preliminary update | Identify the model, show the seated geometry and reachable targets, and report the status of the first simulated stroke. |
| Dec 2–4 · Major update | Show the first song excerpt with two-arm and torso motion, plus an initial wide-kit reach test. |
| Dec 9–11 · Presentation | Present the model, motion design, results, and demo video. The brief reserves 30 minutes for presentation and Q&A. |
| Dec 11, 11:59 p.m. · Final submission | Submit the final report and video. |

## Proposed demo criteria

- Select and play either of two short song excerpts in RViz. Show both arms playing on the shared timeline, with visible, continuous torso and wrist motion where the targets call for it.
- Demonstrate a simultaneous hit on the compact kit and a wide-kit case where the planner keeps the higher-priority feasible hit on time.
- Count detected hits, planner skips, and unexpected misses separately. For accepted hits, aim for zero unexpected misses or extra hits, a maximum contact timing error of 25 ms, and tip contact within 15 mm of its target. Revisit those thresholds once the model, logging resolution, and pad size are known.
- Stay within model joint limits and avoid unintended collisions throughout the sequence.
