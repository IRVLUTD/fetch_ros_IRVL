#!/usr/bin/env python3
"""
Test client to send open/close goals to gripper_controller/gripper_action.
Usage:
    python3 scripts/test_gripper_goal.py 0.0     # Close
    python3 scripts/test_gripper_goal.py 0.115   # Open
"""
import sys
import rospy
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

def main():
    rospy.init_node("test_gripper_goal_client")
    client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
    rospy.loginfo("Waiting for gripper_controller/gripper_action server...")
    if not client.wait_for_server(rospy.Duration(5.0)):
        rospy.logerr("Could not connect to gripper action server within 5 seconds.")
        sys.exit(1)

    target_pos = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    rospy.loginfo(f"Sending GripperCommandGoal position: {target_pos:.4f} m")

    goal = GripperCommandGoal()
    goal.command.position = target_pos
    goal.command.max_effort = 100.0

    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    rospy.loginfo(f"Goal completed! reached_goal={result.reached_goal}, position={result.position:.4f} m")

if __name__ == "__main__":
    main()
