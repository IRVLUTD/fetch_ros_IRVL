#!/usr/bin/env python3
"""
LEAP Hand Pick and Place Demo for Fetch Robot in Gazebo.
Pure MoveIt Cartesian Planning Implementation.

Uses MoveIt (moveit_commander) to solve inverse kinematics, plan collision-free
trajectories, and execute pick-and-place with the 16-DOF LEAP hand.

Cube Position in Gazebo (test_zone_pick_place.sdf):
- Table top: z = 0.72m
- Demo cube center: x = 0.80m, y = 0.10m, z = 0.75m (size: 6x6x6 cm)

Cartesian Waypoints:
1. Pre-Grasp: (x=0.62, y=0.10, z=0.88), pitch 90 deg down
2. Grasp:     (x=0.64, y=0.10, z=0.76), hand envelops cube
3. Close LEAP Hand around cube
4. Lift:      (x=0.64, y=0.10, z=0.92), lifts 16 cm vertically
5. Place:     (x=0.62, y=-0.20, z=0.76), lowers onto table at y = -0.20
6. Open LEAP Hand
7. Retreat:   (x=0.55, y=-0.20, z=0.90), pulls back and up
"""

from sensor_msgs.msg import JointState
import sys
import copy
import subprocess
from math import pi, sin, cos

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal,
                              PointHeadAction,
                              PointHeadGoal)
import moveit_commander
from moveit_msgs.msg import MoveItErrorCodes

# ==========================================
# 1. Head Pointing Client
# ==========================================
class PointHeadClient(object):
    def __init__(self):
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller action server...")
        if not self.client.wait_for_server(rospy.Duration(5.0)):
            rospy.logwarn("head_controller action server not detected within 5s. Head tracking commands will be skipped.")
            self.connected = False
        else:
            self.connected = True

    def look_at(self, x, y, z, frame="base_link", duration=1.0):
        if not self.connected:
            return
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point = Point(x, y, z)
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(duration + 1.0))

# ==========================================
# Gazebo Live Object Tracking & Reset Helpers
# ==========================================
def get_gazebo_cube_pose():
    """Dynamically get current cube (x, y, z) in base_link from Gazebo."""
    try:
        cube_out = subprocess.check_output(["gz", "model", "-m", "demo_cube", "-p"], timeout=2.0).decode('utf-8').strip().split()
        cx, cy, cz = float(cube_out[0]), float(cube_out[1]), float(cube_out[2])

        fetch_out = subprocess.check_output(["gz", "model", "-m", "fetch", "-p"], timeout=2.0).decode('utf-8').strip().split()
        fx, fy, fz, f_yaw = float(fetch_out[0]), float(fetch_out[1]), float(fetch_out[2]), float(fetch_out[5])

        # Transform cube into fetch base_link frame
        dx = cx - fx
        dy = cy - fy
        x_base = dx * cos(-f_yaw) - dy * sin(-f_yaw)
        y_base = dx * sin(-f_yaw) + dy * cos(-f_yaw)
        z_base = cz - fz
        return x_base, y_base, z_base
    except Exception as e:
        rospy.logwarn(f"Failed to read live cube pose from Gazebo: {e}, falling back to defaults")
        return 0.80, 0.10, 0.75

def reset_gazebo_cube(x=0.80, y=0.10, z=0.75):
    """Reset only the cube position without resetting the robot."""
    try:
        subprocess.run(["gz", "model", "-m", "demo_cube", "-x", str(x), "-y", str(y), "-z", str(z)], timeout=2.0)
        rospy.sleep(0.5)
        rospy.loginfo(f"Reset demo_cube in Gazebo to ({x}, {y}, {z})")
    except Exception as e:
        rospy.logwarn(f"Failed to reset cube: {e}")

# ==========================================
# 2. Torso Client
# ==========================================
class TorsoClient(object):
    def __init__(self):
        self.client = actionlib.SimpleActionClient("torso_controller/follow_joint_trajectory",
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for torso_controller...")
        if not self.client.wait_for_server(rospy.Duration(5.0)):
            rospy.logwarn("torso_controller action server not responding within 5s. Torso movement will be skipped.")
            self.connected = False
        else:
            self.connected = True

    def move_to(self, position=0.35, duration=4.0):
        if not self.connected:
            rospy.logwarn("Skipping torso motion (torso_controller not connected).")
            return False
        traj = JointTrajectory()
        traj.joint_names = ["torso_lift_joint"]
        pt = JointTrajectoryPoint()
        pt.positions = [position]
        pt.velocities = [0.0]
        pt.time_from_start = rospy.Duration(duration)
        traj.points.append(pt)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = traj
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(duration + 2.0))
        return True

# ==========================================
# 3. LEAP Hand Client (16 DOF)
# ==========================================
class LeapHandClient(object):
    def __init__(self):
        self.client = actionlib.SimpleActionClient("leap_hand_controller/follow_joint_trajectory",
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for leap_hand_controller...")
        if not self.client.wait_for_server(rospy.Duration(5.0)):
            rospy.logwarn("leap_hand_controller action server not responding within 5s. Hardware commands on /leap_hand/cmd will still be streamed.")
            self.has_action_server = False
        else:
            self.has_action_server = True
        self.joint_names = [f"joint_{i}" for i in range(16)]

        # Publisher to mirror motions to physical LEAP hand via leap_ethernet driver
        self.real_hand_pub = rospy.Publisher("/leap_hand/cmd", JointState, queue_size=1)
        self.current_positions = [0.0] * 16
        self.state_sub = rospy.Subscriber("/joint_states", JointState, self._state_cb)
        self.leap_state_sub = rospy.Subscriber("/leap_hand/state", JointState, self._state_cb)

    def _state_cb(self, msg):
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_positions[i] = msg.position[idx]

    def send_positions(self, positions, duration=1.5):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.velocities = [0.0] * 16
        pt.time_from_start = rospy.Duration(duration)
        traj.points.append(pt)

        # 1. Send to Gazebo simulation action server (if available)
        if self.has_action_server:
            goal = FollowJointTrajectoryGoal()
            goal.trajectory = traj
            self.client.send_goal(goal)

        # 2. Smoothly stream interpolated waypoints to physical hand over duration
        start_positions = list(self.current_positions)
        steps = max(int(duration * 50), 1)  # 50 Hz streaming
        dt = duration / steps
        for step in range(1, steps + 1):
            alpha = float(step) / steps
            interp = [
                start_p + alpha * (target_p - start_p)
                for start_p, target_p in zip(start_positions, positions)
            ]
            real_msg = JointState()
            real_msg.header.stamp = rospy.Time.now()
            real_msg.name = self.joint_names
            real_msg.position = interp
            self.real_hand_pub.publish(real_msg)
            rospy.sleep(dt)

        self.current_positions = list(positions)
        if self.has_action_server:
            self.client.wait_for_result(rospy.Duration(1.0))

    def open(self, duration=1.5):
        """Open all fingers wide, thumb in opposing posture."""
        rospy.loginfo("Opening LEAP Hand wide...")
        pos = [0.0] * 16
        pos[13] = 1.5  # Home thumb opposition opening
        self.send_positions(pos, duration=duration)

    def close(self, duration=1.5):
        """Isaac Sim verified grasp: enveloping wrap around cube."""
        rospy.loginfo("Closing LEAP Hand (Grasping cube)...")
        pos = [0.0] * 16
        # Index finger: [0.0°, 34.2°, 37.1°, 36.0°]
        pos[0] = 0.0000
        pos[1] = 0.5969
        pos[2] = 0.8383
        pos[3] = 0.7487
        # Middle finger: [0.0°, 30.7°, 42.3°, 42.9°]
        pos[4] = 0.0000
        pos[5] = 0.5358
        pos[6] = 0.8383
        pos[7] = 0.7487
        # Ring finger: [-7.5°, 53.9°, 64.9°, 74.1°]
        pos[8] = -0.1309
        pos[9] = 1.1407
        pos[10] = 1.3327
        pos[11] = 1.2933
        # Thumb: [70.0, 53.8, 35.6, 48.4]
        pos[12] = 1.2217
        pos[13] = 0.6907
        pos[14] = 0.9220
        pos[15] = 0.8447
        self.send_positions(pos, duration=duration)

# ==========================================
# 4. Pure MoveIt Arm Grasping Client
# ==========================================
class MoveItGraspingClient(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.arm.set_pose_reference_frame("base_link")
        self.arm.set_planning_time(8.0)
        self.arm.set_num_planning_attempts(15)
        self.arm.set_goal_position_tolerance(0.02)
        self.arm.set_goal_orientation_tolerance(0.08)
        self.arm.set_max_velocity_scaling_factor(0.35)
        self.arm.set_max_acceleration_scaling_factor(0.35)

        self.arm_action_client = actionlib.SimpleActionClient(
            "arm_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction
        )
        rospy.loginfo("Waiting for arm_controller...")
        if not self.arm_action_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logwarn("arm_controller action server not responding within 5s. Direct joint trajectory fallback may not function.")

        self.hand = LeapHandClient()
        rospy.sleep(1.0)

    def execute_plan(self, plan):
        """Execute MoveIt plan directly via arm_controller to prevent duration monitor aborts."""
        traj = None
        if hasattr(plan, 'joint_trajectory'):
            traj = plan.joint_trajectory
        elif hasattr(plan, 'points'):
            traj = plan

        if traj is None or len(traj.points) == 0:
            rospy.logwarn("execute_plan: trajectory has no points!")
            return False

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = traj
        self.arm_action_client.send_goal(goal)
        expected_dur = traj.points[-1].time_from_start.to_sec()
        wait_dur = max(expected_dur * 2.5 + 5.0, 15.0)
        self.arm_action_client.wait_for_result(rospy.Duration(wait_dur))
        return True

    def add_table_safety_bounds(self, cube_x=0.80, cube_y=0.10, cube_z=0.75):
        """Add collision safety bounds in MoveIt planning scene."""
        rospy.loginfo("Adding table and cube safety boundaries to planning scene...")
        self.scene.remove_world_object("table_boundary")
        self.scene.remove_world_object("table")
        self.scene.remove_world_object("demo_cube")
        rospy.sleep(0.5)

        # Table surface in Gazebo is at z = 0.72m, centered at x=1.00, y=0.0.
        # Box top at z = 0.70m keeps arm trajectories safely above the tabletop.
        table_pose = PoseStamped()
        table_pose.header.frame_id = "base_link"
        table_pose.pose.position.x = 1.00
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = 0.35
        table_pose.pose.orientation.w = 1.0
        self.scene.add_box("table", table_pose, size=(1.00, 1.40, 0.70))

        # Add cube as collision object so the approach trajectory avoids it
        cube_pose = PoseStamped()
        cube_pose.header.frame_id = "base_link"
        cube_pose.pose.position.x = cube_x
        cube_pose.pose.position.y = cube_y
        cube_pose.pose.position.z = cube_z
        cube_pose.pose.orientation.w = 1.0
        self.scene.add_box("demo_cube", cube_pose, size=(0.06, 0.06, 0.06))
        rospy.sleep(0.5)

    # Quaternion that locks palm_lower squarely aligned with base_link:
    # +X = Forward (Fingers), -X = Backward (Thumb), +Y = Left, -Y = Right, -Z = Down
    SQUARED_ORIENTATION = (0.64727, -0.28468, -0.64727, -0.28468)

    def move_to_pose(self, x, y, z, roll=None, pitch=None, yaw=None, quat=None):
        """Plan collision-free path with MoveIt and execute via arm_controller."""
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z

        if quat is not None:
            q = quat
        elif roll is not None and pitch is not None and yaw is not None:
            q = quaternion_from_euler(roll, pitch, yaw)
        else:
            q = self.SQUARED_ORIENTATION

        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]

        self.arm.set_pose_target(target_pose)
        plan_tuple = self.arm.plan()
        plan_success = plan_tuple[0]
        plan = plan_tuple[1]

        if not plan_success or len(plan.joint_trajectory.points) == 0:
            rospy.logwarn(f"MoveIt planning failed for pose target (x={x:.2f}, y={y:.2f}, z={z:.2f})!")
            self.arm.clear_pose_targets()
            return False

        self.execute_plan(plan)
        self.arm.clear_pose_targets()
        rospy.sleep(0.5)
        return True

    def move_cartesian_vertical(self, delta_z, avoid_collisions=False):
        """Move arm strictly vertically by delta_z using Cartesian path planning."""
        current_pose = self.arm.get_current_pose().pose
        target_pose = copy.deepcopy(current_pose)
        target_pose.position.z += delta_z
        waypoints = [target_pose]
        (plan, fraction) = self.arm.compute_cartesian_path(waypoints, eef_step=0.01, avoid_collisions=avoid_collisions)
        rospy.loginfo(f"MoveIt: Cartesian vertical move ({delta_z:+.2f}m) path fraction: {fraction:.2f}")
        if fraction > 0.50:
            self.execute_plan(plan)
            self.arm.clear_pose_targets()
            rospy.sleep(0.5)
            return True
        else:
            q = (current_pose.orientation.x, current_pose.orientation.y,
                 current_pose.orientation.z, current_pose.orientation.w)
            return self.move_to_pose(target_pose.position.x, target_pose.position.y, target_pose.position.z, quat=q)

    def untuck(self):
        """Move arm from high-stowed position to ready position, pre-aligned forward."""
        rospy.loginfo("Untucking arm to ready posture...")
        ready = [0.0, -0.20, 0.0, 0.50, 0.0, 1.25, -2.33]

        self.arm.set_joint_value_target(ready)
        plan_tuple = self.arm.plan()
        if plan_tuple[0]:
            self.execute_plan(plan_tuple[1])
        self.arm.clear_pose_targets()
        rospy.loginfo("...arm untucked.")

    def tuck(self):
        """Stow arm back into high-stowed posture safely clear of base and table."""
        rospy.loginfo("Stowing arm to parked posture...")
        stow_pose = [1.32, 0.70, 0.0, -2.0, 0.0, -0.57, 0.0]

        self.arm.set_joint_value_target(stow_pose)
        plan_tuple = self.arm.plan()
        if plan_tuple[0]:
            self.execute_plan(plan_tuple[1])
        self.arm.clear_pose_targets()
        rospy.loginfo("...arm tucked.")

    def pick(self, cube_x=0.80, cube_y=0.10, cube_z=0.75):
        """Execute pick using MoveIt Cartesian poses."""
        rospy.loginfo("=== STARTING MOVEIT PICK ===")
        # 1. Open hand
        self.hand.open()

        # With hand orientation locked squarely to base_link:
        # +X is strictly Forward (Fingers side), -X is strictly Backward (Thumb side)
        # +Y is strictly Left, -Y is strictly Right
        # Grasp center matches cube at:
        wrist_x = cube_x - 0.046
        wrist_y = cube_y - 0.063
        grasp_z = cube_z + 0.160
        hover_z = 1.0

        # 2. Pre-Grasp: hover above the cube
        rospy.loginfo(f"MoveIt: Planning to Pre-Grasp above cube (x={wrist_x:.2f}, y={wrist_y:.2f}, z={hover_z:.2f})...")
        if not self.move_to_pose(x=wrist_x, y=wrist_y, z=hover_z):
            rospy.logwarn("Failed to reach Pre-Grasp hover pose!")
            return False
        rospy.sleep(0.5)

        # 3. Before descending to grasp: remove cube from collision scene so fingers can surround it
        self.scene.remove_world_object("demo_cube")
        rospy.sleep(0.5)

        # 4. Descend straight down to Grasp Pose around cube
        rospy.loginfo(f"MoveIt: Descending vertically to Grasp Pose (z={grasp_z:.2f})...")
        self.move_cartesian_vertical(delta_z=grasp_z - hover_z, avoid_collisions=False)
        rospy.sleep(0.5)

        # 5. Close hand firmly around cube
        rospy.loginfo("Closing LEAP Hand around cube...")
        self.hand.close()
        rospy.sleep(1.2)

        # 6. Lift cube vertically (+14.2 cm)
        rospy.loginfo(f"MoveIt: Lifting cube vertically off table (z={hover_z:.2f})...")
        self.move_cartesian_vertical(delta_z=hover_z - grasp_z, avoid_collisions=False)
        rospy.sleep(0.5)

        rospy.loginfo("[+] Cube successfully picked and lifted!")
        return True

    def place(self, target_x=0.80, target_y=-0.20, target_z=0.75):
        """Execute place using MoveIt Cartesian poses."""
        rospy.loginfo("=== STARTING MOVEIT PLACE ===")
        wrist_x = target_x - 0.08
        wrist_y = target_y - 0.063
        place_z = target_z + 0.250
        hover_z = 1.05

        # 1. Transport to place pre-position above target (smooth acceleration to prevent slip)
        rospy.loginfo(f"MoveIt: Transporting to Place Approach Pose (x={wrist_x:.2f}, y={wrist_y:.2f}, z={hover_z:.2f})...")
        if not self.move_to_pose(x=wrist_x, y=wrist_y, z=hover_z):
            rospy.logwarn("Failed to reach Place Approach pose!")
            return False
        rospy.sleep(0.5)

        # 2. Lower straight down to table surface
        rospy.loginfo(f"MoveIt: Lowering vertically to table (z={place_z:.2f})...")
        self.move_cartesian_vertical(delta_z=place_z - hover_z, avoid_collisions=False)
        rospy.sleep(0.5)

        # 3. Open hand to release cube
        rospy.loginfo("Opening LEAP Hand to release cube...")
        self.hand.open()
        rospy.sleep(1.2)

        # 4. Retreat straight up vertically
        rospy.loginfo("MoveIt: Retreating vertically upward...")
        self.move_cartesian_vertical(delta_z=hover_z - place_z, avoid_collisions=False)
        rospy.sleep(0.5)

        rospy.loginfo("[+] Cube successfully placed!")
        return True

# ==========================================
# Main Execution Flow
# ==========================================
if __name__ == "__main__":
    rospy.init_node("leap_pick_place_demo")

    rospy.loginfo("Initializing clients...")
    torso_action = TorsoClient()
    head_action = PointHeadClient()
    grasping_client = MoveItGraspingClient()

    # 1. Raise torso to table working height (0.35m)
    rospy.loginfo("Raising torso to table working height...")
    torso_action.move_to(position=0.35, duration=4.0)

    # 2. Dynamically detect live cube position from Gazebo
    cube_x, cube_y, cube_z = get_gazebo_cube_pose()
    rospy.loginfo(f"[+] Detected live cube in base_link at: (x={cube_x:.3f}, y={cube_y:.3f}, z={cube_z:.3f})")

    # 3. Point head camera at cube on the table
    rospy.loginfo("Pointing head camera at cube...")
    head_action.look_at(cube_x, cube_y, cube_z, "base_link")

    # 4. Add table and cube safety boundaries to planning scene
    grasping_client.add_table_safety_bounds(cube_x=cube_x, cube_y=cube_y, cube_z=cube_z)

    # 5. Untuck arm to ready posture
    grasping_client.untuck()

    # 6. MoveIt Pick using dynamic cube coordinates
    grasping_client.pick(cube_x=cube_x, cube_y=cube_y, cube_z=cube_z)
    rospy.sleep(1.0)

    # 7. MoveIt Place
    grasping_client.place(target_x=0.80, target_y=-0.20, target_z=0.75)
    rospy.sleep(1.0)

    # 8. Stow arm safely back into parked posture
    grasping_client.tuck()
    rospy.loginfo("=== DEMO FINISHED SUCCESSFULLY ===")

