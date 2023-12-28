#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from visualization_msgs.msg import Marker
from trajectory_functions import logo_trajectory, i_trajectory, m_trajectory, r_trajectory

def execute_trajectory(waypoints):
    (plan, fraction) = group.compute_cartesian_path(
                            waypoints,
                            0.01,  # eef_step
                            0.0)   # jump_threshold
    if fraction == 1.0:
        print("Full trajectory computed!")
        group.execute(plan, wait=True)
    else:
        print("Could not compute full trajectory. Only " + str(fraction*100) + "% achieved.")


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('draw_shapes_and_letters', anonymous=True)

    group_name = "ur5_arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    marker = Marker()
    marker.header.frame_id = "/base_link"
    marker.type = marker.POINTS
    marker.action = marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.01
    marker.scale.y = 0.01
    marker.color.a = 1.0
    marker.color.r = 1.0
    
    marker_publisher = rospy.Publisher('trajectory_marker', Marker, queue_size=10)


    current_pose = group.get_current_pose().pose.position

    waypoints_logo = logo_trajectory(current_pose, group, marker)
    marker_publisher.publish(marker)
    execute_trajectory(waypoints_logo)

    current_pose.y += 0.15
    waypoints_i = i_trajectory(0.1, [current_pose.x, current_pose.y, current_pose.z], group, marker)
    marker_publisher.publish(marker)
    execute_trajectory(waypoints_i)

    current_pose.y += 0.05
    waypoints_m = m_trajectory(0.1, 0.1, [current_pose.x, current_pose.y, current_pose.z], group, marker)
    marker_publisher.publish(marker)
    execute_trajectory(waypoints_m)

    current_pose.y += 0.05
    waypoints_r = r_trajectory(0.1, 0.05, [current_pose.x, current_pose.y, current_pose.z], group, marker)
    marker_publisher.publish(marker)
    execute_trajectory(waypoints_r)

    rospy.sleep(5)

