#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import math
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

# Initialization
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur5_u_trajectory', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("ur5_arm")

# Marker Initialization and Publisher
marker_publisher = rospy.Publisher('trajectory_marker', Marker, queue_size=10000, latch=True)
marker = Marker()
marker.header.frame_id = "base_link"
marker.id = 3
marker.type = Marker.LINE_STRIP
marker.action = Marker.ADD
marker.pose.orientation.w = 1.0
marker.scale.x = 0.01  # Line width
marker.color.r = 1.0
marker.color.g = 1.0
marker.color.b = 1.0
marker.color.a = 1.0
marker.points = []

# Single create_pose function to avoid redundancy
def create_pose(x, y, z):
    pose = group.get_current_pose().pose
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    return pose

def u_trajectory(height, width, center):
    waypoints = []

    # Move vertically up to the top left of the 'U'
    top_left = (center[0], center[1], center[2]+height/2)
    bottom_left = (center[0], center[1] , center[2] - height / 2)
    waypoints.append(create_pose(*top_left))
    waypoints.append(create_pose(*bottom_left))

    # Semi-circle for the bottom of the 'U', facing downwards
    radius = width / 2
    center_of_semicircle = (center[0], center[1]+width/2, center[2] - height / 2)

    # Create the semicircle from left to right, with the correct orientation
    for angle in range(180, -1, -10):  # angles from 180 to 360 degrees
        rad = math.radians(angle)
        x = center_of_semicircle[0]
        y = center_of_semicircle[1] + radius * math.cos(rad)
        z = center_of_semicircle[2] - radius * math.sin(rad)  # Subtract the sin value to flip the semicircle
        waypoints.append(create_pose(x, y, z))

    # Straight line up (right side of 'U')
    top_right = (center[0], center[1] + width, center[2] + height / 2)
    waypoints.append(create_pose(*top_right))

    # Add marker points for RViz, adjust the offset if required
    for waypoint in waypoints:
        marker.points.append(Point(waypoint.position.x+0.2, waypoint.position.y, waypoint.position.z-1.02))

    # Return the waypoints to be executed by the robot
    return waypoints




# Command the UR5 to follow the trajectory
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

# Main Execution
if __name__ == '__main__':
    current_pose = group.get_current_pose().pose.position
    current_pose.y += 0.05
    current_pose.z += 0.07
    u_height = 0.05  # Change this to the desired height of the 'U'
    u_width = 0.1   # Change this to the desired width of the 'U'
    while not rospy.is_shutdown():
        waypoints = u_trajectory(u_height, u_width, [current_pose.x, current_pose.y, current_pose.z])    
        execute_trajectory(waypoints)
        marker_publisher.publish(marker)
        rospy.sleep(5)  # Wait for some time after drawing
        break  # Exit the loop after drawing the 'U'


