#!/usr/bin/env python3
from trajectory_functions import *


# Main Execution
if __name__ == '__main__':
    current_pose = group.get_current_pose().pose.position
    current_pose.y += 0.05
    r_height = 0.1  # Change this to the desired height of the 'R'
    r_width = 0.05  # Change this to the desired width of the 'R'
    marker = create_marker(3)
    while not rospy.is_shutdown():
        waypoints = a_trajectory(r_height, r_width, [current_pose.x, current_pose.y, current_pose.z], marker)
        execute_trajectory(waypoints)
        marker_publisher.publish(marker)
        break  # Exit the loop after drawing the 'R'
        rospy.sleep(5)  # Wait for some time after drawing


