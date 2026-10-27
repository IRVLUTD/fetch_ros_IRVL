#!/usr/bin/env python3
"""
LEAP Hand Gripper Action Server Bridge.

Hosts a standard `control_msgs/GripperCommandAction` on 'gripper_controller/gripper_action'.
Translates 1-DOF gripper commands from Fetch joystick teleop or high-level scripts into
coordinated 16-DOF grasp trajectories and sends them to `leap_trajectory_action_server`.

Teleop moves slowly and smoothly over `teleop_duration` (default 1.5s) for safe, controlled
interaction with objects.
"""

import sys
import rospy
import actionlib
from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandFeedback,
    GripperCommandResult,
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


class LeapGripperActionServer:
    def __init__(self):
        rospy.init_node("leap_gripper_action_server")

        self.joint_names = [f"joint_{i}" for i in range(16)]

        # Fetch gripper standard physical travel limits (meters)
        self.gripper_closed_pos = float(rospy.get_param("~gripper_closed_pos", 0.0))
        self.gripper_open_pos = float(rospy.get_param("~gripper_open_pos", 0.115))
        self.teleop_duration = float(rospy.get_param("~teleop_duration", 1.5))

        # Default Open Posture: all fingers open, thumb opposition open to 1.5 rad
        self.open_positions = [0.0] * 16
        self.open_positions[13] = 1.5

        # Default Close Posture: Isaac Sim verified enveloping grasp
        self.close_positions = [0.0] * 16
        self.close_positions[0] = 0.0000
        self.close_positions[1] = 0.5969
        self.close_positions[2] = 0.8383
        self.close_positions[3] = 0.7487
        self.close_positions[4] = 0.0000
        self.close_positions[5] = 0.5358
        self.close_positions[6] = 0.8383
        self.close_positions[7] = 0.7487
        self.close_positions[8] = -0.1309
        self.close_positions[9] = 1.1407
        self.close_positions[10] = 1.3327
        self.close_positions[11] = 1.2933
        self.close_positions[12] = 1.2217
        self.close_positions[13] = 0.6907
        self.close_positions[14] = 0.9220
        self.close_positions[15] = 0.8447

        # Internal state tracking
        self.current_joint_positions = list(self.open_positions)
        self.current_gripper_pos = self.gripper_open_pos

        # Track live joint positions from robot or hand telemetry
        self.state_sub = rospy.Subscriber("/joint_states", JointState, self._joint_state_cb, queue_size=1)
        self.leap_state_sub = rospy.Subscriber("/leap_hand/state", JointState, self._joint_state_cb, queue_size=1)

        # Connect to the underlying 16-DOF trajectory action server
        traj_action_name = rospy.get_param("~trajectory_action_name", "leap_hand_controller/follow_joint_trajectory")
        self.traj_client = actionlib.SimpleActionClient(traj_action_name, FollowJointTrajectoryAction)
        rospy.loginfo(f"Waiting for trajectory action server '{traj_action_name}'...")
        if self.traj_client.wait_for_server(rospy.Duration(5.0)):
            rospy.loginfo(f"Connected to trajectory controller '{traj_action_name}'.")
        else:
            rospy.logwarn(f"Trajectory server '{traj_action_name}' not detected within 5s. Ensure leap_trajectory_action_server is running.")

        # Standard Fetch GripperCommandAction server
        action_name = rospy.get_param("~action_name", "gripper_controller/gripper_action")
        self.server = actionlib.SimpleActionServer(
            action_name,
            GripperCommandAction,
            execute_cb=self.execute_callback,
            auto_start=False,
        )
        self.server.start()
        rospy.loginfo(f"LEAP Gripper Action Bridge online at '{action_name}' (teleop speed: {self.teleop_duration}s).")

    def _joint_state_cb(self, msg: JointState):
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                if idx < len(msg.position):
                    self.current_joint_positions[i] = msg.position[idx]

    def _interpolate_target(self, target_pos_m: float):
        """Map linear gripper command (meters, [0.0, 0.115]) to 16 joint angles."""
        clamped_pos = max(self.gripper_closed_pos, min(self.gripper_open_pos, target_pos_m))
        total_span = self.gripper_open_pos - self.gripper_closed_pos
        open_ratio = (clamped_pos - self.gripper_closed_pos) / total_span if total_span > 0 else 1.0

        target_joints = [
            c_val + open_ratio * (o_val - c_val)
            for c_val, o_val in zip(self.close_positions, self.open_positions)
        ]
        return target_joints, clamped_pos

    def execute_callback(self, goal):
        req_position = goal.command.position
        max_effort = goal.command.max_effort
        target_joints, clamped_pos = self._interpolate_target(req_position)
        rospy.loginfo(f"Received GripperCommandGoal: position={req_position:.4f}m -> target: {clamped_pos:.4f}m over {self.teleop_duration:.1f}s")

        # Build smooth timed trajectory for leap_trajectory_action_server
        traj_goal = FollowJointTrajectoryGoal()
        traj_goal.trajectory.joint_names = self.joint_names

        # Start waypoint (current positions at t=0)
        pt_start = JointTrajectoryPoint()
        pt_start.positions = list(self.current_joint_positions)
        pt_start.velocities = [0.0] * 16
        pt_start.time_from_start = rospy.Duration(0.0)

        # Target waypoint at t=teleop_duration
        pt_end = JointTrajectoryPoint()
        pt_end.positions = target_joints
        pt_end.velocities = [0.0] * 16
        pt_end.time_from_start = rospy.Duration(self.teleop_duration)

        traj_goal.trajectory.points = [pt_start, pt_end]

        # Send trajectory goal to leap_trajectory_action_server
        self.traj_client.send_goal(traj_goal)

        # Monitor progress and publish feedback
        start_time = rospy.Time.now()
        start_gripper_pos = self.current_gripper_pos
        rate = rospy.Rate(20)  # 20 Hz feedback

        while not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                rospy.logwarn("Gripper action preempted by caller.")
                self.traj_client.cancel_goal()
                self.server.set_preempted()
                return

            elapsed = (rospy.Time.now() - start_time).to_sec()
            progress = min(1.0, elapsed / self.teleop_duration) if self.teleop_duration > 0 else 1.0

            feedback = GripperCommandFeedback()
            feedback.position = start_gripper_pos + progress * (clamped_pos - start_gripper_pos)
            feedback.effort = max_effort
            feedback.stalled = False
            feedback.reached_goal = (progress >= 1.0)
            self.server.publish_feedback(feedback)

            # Check if trajectory action finished
            if self.traj_client.wait_for_result(rospy.Duration(0.01)) or progress >= 1.0:
                break

            rate.sleep()

        self.current_joint_positions = list(target_joints)
        self.current_gripper_pos = clamped_pos

        result = GripperCommandResult()
        result.position = self.current_gripper_pos
        result.effort = max_effort
        result.stalled = False
        result.reached_goal = True
        self.server.set_succeeded(result)
        rospy.loginfo(f"Gripper grasp completed at {self.current_gripper_pos:.4f}m.")


if __name__ == "__main__":
    try:
        node = LeapGripperActionServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
