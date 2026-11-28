'''KinematicChain.py

   This is a skeleton for Kinematic Chains (HW5 Problem 5/6).
   PLEASE EDIT (SEARCH FOR FIXME)!

   chain = KinematicChain(node, basefame, tipframe, expectedjointnames)

      Initialize the kinematic chain, reading from the URDF message on
      the topic '/robot_description', sent by the robot_state_publisher.
      Determine the kinematic steps walking from the baseframe to the
      tipframe.  This expects the active joints to match the given names.

   chain.report()

      Simply report the content of the kinematic chain (all steps)

   (ptip, Rtip, Jv, Jw) = chain.fkin(q)

      Compute the forward kinematics and return the results.


   Node:        As called
   Subscribe:   /robot_description      std_msgs/String

'''

import enum
import rclpy
import numpy as np

from rclpy.node                 import Node
from rclpy.qos                  import QoSProfile, DurabilityPolicy
from std_msgs.msg               import String
from urdf_parser_py.urdf        import Robot

# Grab the Utilities
from .TransformHelpers          import (
    R_from_T, Reye, Rotn, T_from_Rp, T_from_URDF_origin, Teye,
    cross, n_from_URDF_axis, p_from_T, pzero,
)


######################################################################
#################  HELPER CODE - FEEL FREE TO SKIP  ##################
######################################################################
#
#   Helper Functions: Report info and trigger an error
#
#   This uses the given Node's logger to post information (print) or
#   trigger an error.
#
def info(node, string):
    if node is not None:
        node.get_logger().info("KinematicChain: " + string)

def error(node, string):
    if node is not None:
        node.get_logger().error("KinematicChain: " + string)
    raise ValueError(string)


#
#   Helper Function: Read URDF's HTML from /robot_description
#
#   This uses the given Node to read the URDF's HTML desctiption from
#   the standard /robot_description topic.
#
def read_HTML(node):
    # Report the action.
    info(node, "Waiting for the URDF to be published...")

    # Define the variable to receive the html and a callback function
    # to place the message content there.
    html = None
    def callback(msg):
        nonlocal html
        html = msg.data

    # Create a temporary subscriber to receive the URDF.  We use the
    # TRANSIENT_LOCAL durability, so that we see the last message
    # already published (if any).
    topic   = '/robot_description'
    quality = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
    sub     = node.create_subscription(String, topic, callback, quality)

    # Wait for the message.
    while html is None:
        rclpy.spin_once(node)

    # Destroy the subscriber and return the html!
    node.destroy_subscription(sub)
    return html


#
#   Helper Function: Parse HTML into the kinematic chain steps
#
#   Parse the given HTML to find the kinematic chain steps from
#   baseframe to tipframe.  It uses the given node to report problems
#   and errors.
#
def parse_HTML(node, html, baseframe, tipframe):
    # Convert the URDF string into a Robot object and report.
    robot = Robot.from_xml_string(html)
    info(node, f"Proccessing URDF for robot '{robot.name}'")
    info(node, f"Building chain from '{baseframe}' to '{tipframe}'")

    # Prepare the empty chain (list of kinematic steps).
    chain = []

    # Parse the Robot object into a list of URDF steps from the base
    # frame to the tip frame.  Search backwards, from the tip to the
    # base, as the robot could be a tree structure: while a parent may
    # have multiple children, every child has only one parent.  The
    # resulting chain of steps is unique.
    frame = tipframe
    while (frame != baseframe):
        # Look for the URDF joint to the parent frame.
        joints = [j for j in robot.joints if j.child == frame]
        if len(joints) == 0:
            error(node, f"Unable find joint connecting to '{frame}'")
        elif len(joints) != 1:
            error(node, f"Unable find unique joint connecting to '{frame}'")
        joint = joints[0]

        # Find the parent frame.
        if (joint.parent == frame):
            error(node, f"Joint '{joint.name}' connects '{frame}' to itself")
        frame = joint.parent

        # Check the type (use the above enumeration)
        if  (joint.type == 'revolute' or
             joint.type == 'continuous'):  type = JointType.REVOLUTE
        elif joint.type == 'prismatic':    type = JointType.LINEAR
        elif joint.type == 'fixed':        type = JointType.FIXED
        else:
            error(node, f"Joint '{joint.name}' has unknown type '{joint.type}'")

        # Check that the axis is normalized (of axis is expected).
        if type is JointType.FIXED:
            nlocal = None
        else:
            nlocal = n_from_URDF_axis(joint.axis)
            mag = np.sqrt(np.inner(nlocal, nlocal))
            if abs(mag - 1) > 1e-6:
                info(node, f"WARNING Joint '{joint.name}' axis normalized")
            nlocal = nlocal / mag

        # Grab the shift (or identity if none specified).
        if joint.origin is None:
            info(node, f"WARNING Joint '{joint.name}' has no <origin>")
            Tshift = Teye()
        else:
            Tshift = T_from_URDF_origin(joint.origin)

        # Convert the collected information into a single URDF step.
        # Note the axis (nlocal) is meaningless for a fixed joint.
        chain.insert(0, URDFStep(
            name=joint.name, type=type, Tshift=Tshift, nlocal=nlocal))

    # Return the chain.
    return chain


######################################################################
############################  MAIN CODE  #############################
######################################################################
#
#   Single URDF Step
#
#   This captures a single step from one frame to the next.  It be of type:
#
#     FIXED     Just a fixed T-matrix shift, nothing moving, not an active DOF.
#     REVOLUTE  A fixed T-matrix shift, followed by a rotation about an axis.
#     LINEAR    A fixed T-matrix shift, followed by a transation along an axis.
#
#   It contains several pieces of permanent data (coming from the URDF):
#
#     name      String showing the name
#     type      One of the above
#     Tshift    Fixed shift: Transform of this frame w.r.t. previous frame
#     nlocal    Joint axis (if applicable) in this local frame
#
#   We also add information how this relates to the active joints:
#
#     dof       If an active dof (not FIXED), the dof number
#
#   And this provides repr() and str() methods to display this information.
#

# Define the joint types.
class JointType(enum.Enum):
    FIXED    = 0
    REVOLUTE = 1
    LINEAR   = 2

# Define a single step in the URDF (kinematic chain).
class URDFStep():
    def __init__(self, name, type, Tshift, nlocal):
        # Store the permanent/fixed/URDF data.
        self.name   = name      # Joint name
        self.type   = type      # Joint type (per above enumeration)
        self.Tshift = Tshift    # Fixed transform w.r.t. previous frame
        self.nlocal = nlocal    # Joint axis in this local frame

        # Match against the joint numbers
        self.dof    = None      # Joint DOF number (or None if FIXED)

    # Return a compact representation
    def __repr__(self):
        return ((f"{self.type.name:8} ") +
                (" "*6 if self.dof is None else f"DOF #{self.dof}") +
                (f" '{self.name}'"))

    # Return a complete multi-line description
    def __str__(self, prefix="step."):
        space = " " * (len(prefix) + 8)
        transform = str(self.Tshift).replace("\n", "\n" + space)
        return (
            "-" * (len(prefix) + 31) + "\n" +
            f"{prefix}name    '{self.name}'\n" +
            f"{prefix}type    {self.type.name}\n" +
            f"{prefix}Tshift  {transform}\n" +
            f"{prefix}nlocal  {str(self.nlocal)}\n" +
            f"{prefix}dof     {str(self.dof)}\n")


#
#   Kinematic Chain Object
#
#   This stores the information provided by the URDF in the form of
#   kinematic chain (list of steps).  In particular, see the fkin()
#   function, as it walks up the chain to determine the transforms.
#

# Define the full kinematic chain
class KinematicChain():
    # Initialization - load the URDF and set up the chain.
    def __init__(
        self, node, baseframe, tipframe, expectedjointnames=None, *,
        robot_description=None, joint_indices=None,
    ):
        # Store the node (so we can properly report errors later).
        self.node = node

        # Read the URDF's HTML description.
        html = read_HTML(node) if robot_description is None else robot_description

        # Parse the HTML to find the kinematic chain steps.
        self.chain = parse_HTML(node, html, baseframe, tipframe)
        self.steps = len(self.chain)

        # Set/count the active DOF numbers, walking up the chain.
        moving_steps = [step for step in self.chain if step.type != JointType.FIXED]
        jointnames = [step.name for step in moving_steps]
        if joint_indices is None:
            joint_indices = {name: index for index, name in enumerate(jointnames)}
        for step in moving_steps:
            if step.name not in joint_indices:
                error(node, f"Joint '{step.name}' is missing from the active joints")
            step.dof = joint_indices[step.name]
        self.dofs = len(joint_indices)
        info(node, "URDF has %d steps, %d active DOFs:" %
             (self.steps, len(moving_steps)))

        # Report the steps we found.
        for (i, step) in enumerate(self.chain):
            info(node, "Step #%d %s" % (i, repr(step)))

        # Confirm the active joint names matches the expectation.
        if expectedjointnames is not None and jointnames != list(expectedjointnames):
            error(node, "Chain does not match the expected names: " +
                  str(expectedjointnames))


    # Report the details of the chain (of each step).
    def report(self):
        info(self.node, ("The chain steps are:\n" +
                         "".join(str(step) for step in self.chain)))


    # Compute the forward kinematics!
    def fkin(self, q):
        # Check the number of joints
        if (len(q) != self.dofs):
            error(self.node, "Given %d joint angles, expected %d" %
                  (len(q), self.dofs))

        ### INITIALIZE ###
        # We will build up three lists.  For each DOF (non-fixed, active
        # step) collect the type, position (pi), axis (ni) w.r.t. the base.
        type = []
        p    = []
        n    = []
        columns = []

        # Initialize the T matrix to walk up the chain, w.r.t. the base frame!
        T = Teye()

        ### PHASE 1: WALK UP THE CHAIN ###
        # We walk the chain, one URDF step at a time, adjusting T as we
        # go.  Each step could be a fixed or active URDF joint.
        for step in self.chain:
            # Take action based on the joint type: Move the transform T
            # up the kinematic chain (remaining w.r.t. the base frame).
            if step.type is JointType.REVOLUTE:
                # Revolute is a shift followed by a rotation:
                rot = Rotn(step.nlocal, q[step.dof])
                T = T @ step.Tshift @ T_from_Rp(rot, pzero())

            elif step.type is JointType.LINEAR:
                # Linear is a shift followed by a translation:
                dis = step.nlocal * q[step.dof]
                T = T @ step.Tshift @ T_from_Rp(Reye(), dis)

            else:
                # Fixed is only a shift.
                T = T @ step.Tshift


            # For active joints (our DOFs), store the type, positon (pi),
            # and axis (ni) info, w.r.t. the base frame.
            if step.type != JointType.FIXED:
                type.append(step.type)
                p.append(p_from_T(T))
                n.append(R_from_T(T) @ step.nlocal)
                columns.append(step.dof)

        # Collect the tip information.
        ptip = p_from_T(T)
        Rtip = R_from_T(T)

        ### PHASE 2: USE ABOVE INFOMATION TO BUILD THE JACOBIAN ###
        # Collect the Jacobian for each active joint.
        Jv = np.zeros((3,self.dofs))
        Jw = np.zeros((3,self.dofs))
        for i, column in enumerate(columns):
            # Fill in the appropriate Jacobian column based on the
            # type.  The Jacobian (like the data) is w.r.t. the base.
            if type[i] is JointType.REVOLUTE:
                # Revolute is a rotation:
                Jv[:,column] = cross(n[i], ptip - p[i])
                Jw[:,column] = n[i]

            elif type[i] is JointType.LINEAR:
                # Linear is a translation:
                Jv[:,column] = n[i]


        # Return the info
        return (ptip, Rtip, Jv, Jw)
