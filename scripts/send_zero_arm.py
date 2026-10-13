#!/usr/bin/env python3
import sys
from math import pi
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

"""
Diagnostic & Homing Script for Fetch Arm & LEAP Hand.
Commands the 7-DOF Fetch arm straight forward (all zeros) and
opens all 16 joints of the LEAP hand. Works with both Gazebo
simulation and the physical OpenRB-150 Ethernet bridge.
"""

from sensor_msgs.msg import JointState

def send_arm_zeros():
    print("[1/4] Connecting to arm_controller...")
    client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    if not client.wait_for_server(rospy.Duration(5.0)):
        print("[-] WARNING: arm_controller action server is not responding.")
        return False

    print("[2/4] Sending all-zeros command to 7 arm joints (arm straight forward)...")
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "upperarm_roll_joint",
        "elbow_flex_joint",
        "forearm_roll_joint",
        "wrist_flex_joint",
        "wrist_roll_joint",
    ]

    point = JointTrajectoryPoint()
    point.positions = [0.0] * 7
    point.velocities = [0.0] * 7
    point.time_from_start = rospy.Duration(3.0)
    goal.trajectory.points.append(point)

    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(5.0))
    print(f"[+] Arm command state: {client.get_state()} (3 = SUCCEEDED)")
    return True

def send_hand_open():
    print("[3/4] Connecting to leap_hand_controller...")
    client = actionlib.SimpleActionClient("leap_hand_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    has_action = client.wait_for_server(rospy.Duration(2.0))
    if not has_action:
        print("[-] NOTE: leap_hand_controller action server not responding. Mirroring directly via /leap_hand/cmd.")

    print("[4/4] Sending open command to 16 LEAP hand joints...")
    joint_names = [f"joint_{i}" for i in range(16)]
    # 0.0 for all fingers, 1.5 rad for thumb opposition open posture
    positions = [0.0] * 16
    positions[13] = 1.5

    # 1. Action goal if available
    if has_action:
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * 16
        point.time_from_start = rospy.Duration(2.0)
        goal.trajectory.points.append(point)
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration(4.0))
        print(f"[+] Hand action state: {client.get_state()} (3 = SUCCEEDED)")

    # 2. Direct topic publisher for physical hardware
    cmd_pub = rospy.Publisher("/leap_hand/cmd", JointState, queue_size=1)
    rospy.sleep(0.2)
    state_msg = JointState()
    state_msg.header.stamp = rospy.Time.now()
    state_msg.name = joint_names
    state_msg.position = positions
    cmd_pub.publish(state_msg)
    print("[+] Published open command to /leap_hand/cmd.")
    return True

def main():
    rospy.init_node("send_zero_robot")
    print("=== COMMANDING FETCH ARM & LEAP HAND ===")
    arm_ok = send_arm_zeros()
    hand_ok = send_hand_open()
    print("=== COMPLETED SUCCESSFULLY ===")

if __name__ == "__main__":
    main()
