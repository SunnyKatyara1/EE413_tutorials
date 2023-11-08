#!/usr/bin/env python3
import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

# Initialize the moveit_commander and rospy nodes
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur5_move_incrementally', anonymous=True)

# Instantiate a RobotCommander object, which provides information about the robot's kinematic model and the robot's current joint states
robot = moveit_commander.RobotCommander()

# Instantiate a PlanningSceneInterface object, which provides a remote interface for getting, setting, and updating the robot's internal understanding of the surrounding world
scene = moveit_commander.PlanningSceneInterface()

# Instantiate a MoveGroupCommander object, which is an interface to a planning group (group of joints). This interface can be used to plan and execute motions on the UR5
group_name = "ur5_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

# Get the current joint values
current_joints = move_group.get_current_joint_values()

# Update the desired joint by increment. In this example, we are incrementing the first joint by 0.1 radians
increment = 0.1
new_joints = [x + increment for x in current_joints]

# Apply the new joint values to the target position
move_group.set_joint_value_target(new_joints)

# Plan to the new joint space goal and execute the plan
plan = move_group.plan()
move_group.execute(plan, wait=True)

# When finished shut down moveit_commander
moveit_commander.roscpp_shutdown()

# Exit the script
move_group.stop()
rospy.sleep(1)
rospy.signal_shutdown('End of the script')

