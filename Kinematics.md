# Suggested kinematics for a seated Unitree drummer

The main task variable is the **stick tip**, since that is the point that must reach the drumhead at a scheduled time. Use the same robot and pad configuration for the workspace checks, trajectory generation, and playback. See the [project plan](ProjectPlan.md) for scope and milestones.

## 1. Model the seat, arms, sticks, and drums

Set the fixed base frame `B` at the seated pelvis. Let `q_T` contain the available waist and torso joints, and `q_i` contain the shoulder, elbow, and wrist joints for arm `i`. Use the URDF to compute each moving shoulder frame. For each arm, compose

`T_B_tip_i(q_T, q_i) = T_B_shoulder_i(q_T) · T_shoulder_hand_i(q_i) · T_hand_tip_i`

Record each drumhead center, unit surface normal `n` pointing out of the head, usable radius, and target point `p_hit` in frame `B`. Measure the grip-to-tip transform with the stick in its actual playing orientation. A quick calibration check is whether a predicted tip location matches a measured tip location at a known arm pose.

For the simulated 29-DOF G1, use `left_drumstick_tip` and `right_drumstick_tip` in [the URDF](g1_description/urdf/g1_29dof.urdf) as the IK and hit-detection endpoints. Each tip frame's +X axis points from the grip toward the striking end. The fixed hand-to-tip transform is already part of the model.

```text
Top view, schematic

          [hi-hat / pad]          [snare / pad]
                 *                      *       * = target
                 |                      |
             left stick            right stick
                 |                      |
             left arm               right arm
                  \                    /
                   [torso and seated base]
```

## 2. Check reach and solve inverse kinematics

Use a single configuration `q = [q_T, q_L, q_R]` so both arms share the same torso pose. Stack the active tip position errors for a simultaneous hit and solve them together. Seed the first solve with a neutral seated posture and subsequent waypoints with the previous solution. The position-only damped update below is a starting point for the combined solve.

For natural motion, prefer moderate wrist bend and twist, elbows clear of the torso and each other, and small changes from the previous pose. Give torso motion a modest cost so the waist helps extend reach without swinging on every hit. Near contact, add a soft stick-axis preference based on the drumhead normal, leaving roll free where possible. Use the full stick geometry to check that the tip contacts first. Enforce joint limits and required clearance separately from posture preferences.

Use the translational Jacobian at the **tip**. If you start with a geometric Jacobian at the hand, each column's tip linear velocity is `v_hand + omega × r_hand_to_tip`, with all terms expressed in the same frame. A position-only damped least-squares update is

`Δq = J_tipᵀ (J_tip J_tipᵀ + λ² I)⁻¹ (p_des − p_tip)`

Bound step sizes and enforce joint limits in the solve. Prefer solutions near a comfortable seated posture. Track the smallest singular value of the position Jacobian where it has full task rank, and inspect the commanded joint rates as the workspace becomes poorly conditioned. With added orientation terms, scale the translation and rotation residuals explicitly. [Modern Robotics, numerical inverse kinematics](https://modernrobotics.northwestern.edu/nu-gm-book-resource/6-2-numerical-inverse-kinematics-part-1-of-2/)

Before placing the kit, sample candidate target locations and solve both the hit pose and a hover pose above each target. Keep locations where the connecting motion clears the drum rim and robot body. For simultaneous strikes, evaluate both arms together over time, including the full lengths of their sticks.

### When both targets cannot be reached together

1. Try both left/right assignments for the pair. Compare feasible trajectories by posture cost and travel, checking the approach, contact, recovery, and transition to the next hit.
2. If neither assignment passes, solve each hit separately with whichever arm can reach it. Keep the highest-priority feasible hit on the original beat. For equal priorities, prefer the solution with lower motion cost. Skip both if neither passes.
3. Log skipped beats and the limiting constraint, such as shared torso reach, clearance, or travel time. Track these separately from hits accepted by the planner that subsequently miss.

## 3. Plan the stroke around its contact time

Define hover as `p_hover = p_hit + h n`, where `h` is the lift height. Start with a near-normal approach so the tip travels toward the pad, then retract along the outward normal. Use smooth trajectories between waypoints with explicit boundary velocities. A hovering waypoint can have zero speed. If modeling a physical hit, specify a small inward velocity immediately before contact and a separate outward recovery after contact. Use the drumhead as the contact boundary and model any rebound separately.

```text
Tip height along pad normal (qualitative)

 h | * hover                       * hover
   |   \                         /
   |     \                     /
 0 +-------*------------------------------> time
           scheduled contact
       approach      recovery
```

For the first pass, solve IK at a set of Cartesian samples and interpolate a continuous joint trajectory. Check the interpolated path using forward kinematics to catch tip deviations and collisions between samples.

In ROS 2 Jazzy, evaluate the tip from the same joint samples the Python playback node publishes for RViz. Detect a hit when the tip moves inward from above the head and reaches or crosses the plane. Interpolate the contact position and timestamp between samples and check that the contact point is inside the playable radius. Rearm once the tip clears a small height above the head to prevent duplicate hits. Use the same clock for scheduled and detected timestamps. Trigger the drum sample from the contact event and use the event timestamp for timing metrics, since audio playback can add output latency.

RViz displays the resulting poses and markers. The crossing test is a geometric estimate based on the published trajectory. Physical hit timing can be measured with a pad trigger or synchronized video or audio.

## 4. Translate BPM into a motion budget

Read the hit times from the selected song chart. Generate hover and recovery points from each drum target and its surface normal, then solve for joint trajectories before playback. Use explicit timestamps when the song has a varying tempo. The grid below is a constant-tempo test fixture for checking playback and simultaneous hits.

For a beat index `b` measured from the start of the pattern, schedule contact at `t_hit = t_start + b × 60 / BPM`. Eighth-note positions increment `b` by 0.5. At 60 BPM, a one-bar schedule is:

```text
Count      1    &    2    &    3    &    4    &
Time (s)   0   0.5   1   1.5   2   2.5   3   3.5
Hi-hat     X    X    X    X    X    X    X    X
Snare      .    .    X    .    .    .    X    .
```

Use a pre-roll so the first downstroke can start before the first scheduled hit. Use one clock for both arms and allow enough time between strikes for recovery, the next approach, and any travel between targets.

```text
Time available between consecutive eighth-note hits
(each block is approximately 50 ms)

 60 BPM  ██████████  500 ms
 90 BPM  ███████     333 ms
120 BPM  █████       250 ms
150 BPM  ████        200 ms
180 BPM  ███         167 ms
```

This is a timing graph calculated from `30 / BPM` seconds. For a fixed path, halving the duration doubles peak joint speed and quadruples peak joint acceleration. Recompute both after changes to lift height or kit position, since those changes alter the path itself.

## 5. Use measured plots to decide what to change

| Plot | What to record and compare |
|---|---|
| Tip height vs. time | Overlay desired and observed height for both arms and mark each scheduled and detected hit. |
| Timing error vs. BPM | Plot median and maximum absolute timing error, and report misses and extra hits separately at each tempo. |
| Peak joint speed vs. BPM | Plot the highest fraction of the corresponding joint speed limit across both arms. Note the joint that limits playback. |
| Target locations in the seat frame | Mark feasible and infeasible hit-and-hover pairs and color feasible points by the minimum clearance or smallest Jacobian singular value over the stroke. |
| Joint angles vs. time | Show torso and wrist angles through single and simultaneous hits to check for abrupt changes and poses near joint limits. |

If a target is hard to reach even at low speed, move the pad or change the seated posture. If a well-conditioned stroke only fails at higher speeds, shorten the lift or reduce the tempo and rerun the sweep. Setup loads and impact behavior require hardware measurements or a dynamics model.
