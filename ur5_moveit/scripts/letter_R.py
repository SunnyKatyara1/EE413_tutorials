#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import math
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

# Initialization
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur5_draw_trajectory', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("ur5_arm")

# Marker Initialization and Publisher
marker_publisher = rospy.Publisher('trajectory_marker', Marker, queue_size=10, latch=True)
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

# Define the 'R' trajectory in the YZ plane
def r_trajectory(height, width, center):
    waypoints = []

    # Define the bottom and top of the straight line
    bottom = (center[0], center[1], center[2] - height)
    top = (center[0], center[1], center[2])

    # Straight line for the leg of the 'R'
    waypoints.append(create_pose(bottom))
    waypoints.append(create_pose(top))

    # Half-circle for the top of the 'R'
    radius = width
    center_of_circle = (top[0], top[1], top[2] + radius)

    for angle in range(-90, 91, 10):  # Create the semicircle sideways
        rad = math.radians(angle)
        x = center_of_circle[0]
        y = center_of_circle[1] + radius * math.cos(rad)
        z = center_of_circle[2] - radius * math.sin(rad)  # Minus because we go down from the center
        waypoints.append(create_pose((x, y, z)))

    # Diagonal line for the leg of the 'R'
    diagonal_start = (center_of_circle[0], center_of_circle[1], center_of_circle[2])
    diagonal_end = (center[0], center[1] + width, center[2]-height)
    waypoints.append(create_pose(diagonal_start))
    waypoints.append(create_pose(diagonal_end))

    # Publish the marker points
    for waypoint in waypoints:
        marker.points.append(Point(waypoint.position.x +0.2, waypoint.position.y, waypoint.position.z-1.02))

    # Publish the marker
    return waypoints

# Helper function to create a pose from a point
def create_pose(point):
    pose = group.get_current_pose().pose
    pose.position.x = point[0]
    pose.position.y = point[1]
    pose.position.z = point[2]
    return pose

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
    current_pose.z += 0.1
    r_height = 0.1  # Change this to the desired height of the 'R'
    r_width = 0.05  # Change this to the desired width of the 'R'
    while not rospy.is_shutdown():
        waypoints = r_trajectory(r_height, r_width, [current_pose.x, current_pose.y, current_pose.z])
        execute_trajectory(waypoints)
        marker_publisher.publish(marker)
        break  # Exit the loop after drawing the 'R'
        rospy.sleep(5)  # Wait for some time after drawing


