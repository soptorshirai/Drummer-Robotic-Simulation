"""Course project workspace for FK, Jacobians, IK, and strike planning."""
import numpy as np
from dataclasses import dataclass

from .KinematicChain import KinematicChain
from .TrajectoryUtils import goto

BASE_FRAME = 'pelvis'
TIP_FRAMES = ('left_drumstick_tip', 'right_drumstick_tip')


@dataclass(frozen=True)
class Hit:
    time_s: float
    target_frame: str
    priority: int


class DrummerKinematics:
    def __init__(self, robot_description: str, active_joints: tuple[str, ...]):
        self.robot_description = robot_description
        self.active_joints = active_joints
        self.joint_indices = {
            name: index for index, name in enumerate(self.active_joints)
        }
        # Setting up the kinematic chains for the left and right drumsticks. 
        # The base frame is the pelvis, and the tip frames are the left and right drumstick tips. 
        self.left_chain = KinematicChain(
            None, BASE_FRAME, TIP_FRAMES[0],
            robot_description=self.robot_description, joint_indices=self.joint_indices,
        )
        self.right_chain = KinematicChain(
            None, BASE_FRAME, TIP_FRAMES[1],
            robot_description=self.robot_description, joint_indices=self.joint_indices,
        )
        self.left_joints = [step.name for step in self.left_chain.chain]
        self.right_joints = [step.name for step in self.right_chain.chain]

    def update(
        self, time_s: float, hits: tuple[Hit, ...], previous: dict[str, float],
    ) -> dict[str, float]:
        """Return waist/arm positions in radians, keyed by URDF joint name.

        time_s is relative to song start (negative during pre-roll). Hits
        contain the whole chart, including simultaneous hits. Target frames
        are defined by the kit URDF. Use the previous pose as the IK seed.
        """

        # Example for one snare hit at song time 1.0 s.
        snare_contact = (0.4455, 0.345, 0.000)
        snare_hover = (0.4455, 0.345, 0.050)

        snare_waypoints = (
            ( 1.0, snare_hover),    # Before the hit
            ( 1.5, snare_contact),  # On the beat
            ( 2.0, snare_hover),    # After the hit
        )

        (t_hover, hover), (t_contact, contact), (t_recover, _) = snare_waypoints
        hover = np.asarray(hover, dtype=float)
        contact = np.asarray(contact, dtype=float)

        # 1. Desired tip position and velocity in the pelvis frame.
        if time_s < t_hover and time_s > 0.5:
            pd = hover
            vd = np.zeros(3)
        elif time_s < t_contact:
            pd, vd = goto(time_s - t_hover, t_contact - t_hover, hover, contact)
        elif time_s <= t_recover:
            pd, vd = goto(time_s - t_contact, t_recover - t_contact, contact, hover)
        else:
            return previous.copy()

        # 2. Get the actual tip position and position Jacobian from FK.
        qc = np.array([previous[name] for name in self.active_joints])
        pc, _, Jv, _ = self.right_chain.fkin(qc)
        pc = np.asarray(pc).reshape(3)

        # 3. Use only the left-arm columns so the waist and right arm hold.
        right_joines = [name for name in self.active_joints if name.startswith("right_")]
        columns = [self.joint_indices[name] for name in right_joines]
        J = Jv[:, columns]

        # 4. Follow the desired velocity and correct position error, like HW7.
        lam = 10.0
        gamma = 0.05
        vc = vd + lam * (pd - pc)
        Jdinv = J.T @ np.linalg.pinv(J @ J.T + gamma**2 * np.eye(3))
        qdot = Jdinv @ vc

        # 5. Integrate one tick at the trajectory node's 100 Hz rate.
        dt = 0.01
        q = previous.copy()
        for name, vel in zip(right_joines, qdot):
            q[name] += dt * vel
        return q



        pd = previous
        # TODO: Resolve drumhead frames into pelvis coordinates with TF or
        # kit geometry, then generate approach, contact, and recovery targets.
        # TODO: Solve shared-waist, two-arm IK using the stick-tip frames.
        # Include joint limits, clearance, and wrist/posture preferences.
        # TODO: Resolve infeasible hit pairs by priority and record skips.
        return previous.copy()
