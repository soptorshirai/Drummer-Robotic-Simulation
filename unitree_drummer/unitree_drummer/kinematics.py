"""Course project workspace for FK, Jacobians, IK, and strike planning."""

from dataclasses import dataclass

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
        # TODO: Build both pelvis-to-tip chains with shared waist coordinates.

    def update(
        self, time_s: float, hits: tuple[Hit, ...], previous: dict[str, float],
    ) -> dict[str, float]:
        """Return waist/arm positions in radians, keyed by URDF joint name.

        time_s is relative to song start (negative during pre-roll). Hits
        contain the whole chart, including simultaneous hits. Target frames
        are defined by the kit URDF. Use the previous pose as the IK seed.
        """
        # TODO: Resolve drumhead frames into pelvis coordinates with TF or
        # kit geometry, then generate approach, contact, and recovery targets.
        # TODO: Solve shared-waist, two-arm IK using the stick-tip frames.
        # Include joint limits, clearance, and wrist/posture preferences.
        # TODO: Resolve infeasible hit pairs by priority and record skips.
        return previous.copy()
