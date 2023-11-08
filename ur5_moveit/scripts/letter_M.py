#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

# Initialization
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur5_square_trajectory', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("ur5_arm")

# Marker Initialization and Publisher
marker_publisher = rospy.Publisher('trajectory_marker', Marker, queue_size=10000, latch=True)
marker = Marker()
marker.header.frame_id = "base_link"
marker.id = 2
marker.type = Marker.LINE_STRIP
marker.action = Marker.ADD
marker.pose.orientation.w = 1.0
marker.scale.x = 0.01  # Line width
marker.color.r = 1.0
marker.color.g = 1.0
marker.color.b = 1.0
marker.color.a = 1.0
marker.points = []

# Define the 'M' trajectory in YZ plane
def m_trajectory(height, width, center):
    waypoints = []

    half_width = width / 2.0
    bottom_left = (center[0], center[1] - half_width, center[2] - height)
    top_left = (center[0], center[1] - half_width, center[2])
    middle = (center[0], center[1], center[2] - height / 2)
    top_right = (center[0], center[1] + half_width, center[2])
    bottom_right = (center[0], center[1] + half_width, center[2] - height)

    # Define the points for 'M'
    m_points = [
        bottom_left,  # Start at bottom left
        top_left,     # Draw up to the top left
        middle,       # Draw down to the middle
        top_right,    # Draw up to the top right
        bottom_right  # Draw down to the bottom right
    ]

    # Create waypoints for the 'M'
    for point in m_points:
        pose = group.get_current_pose().pose
        pose.position.x = point[0]
        pose.position.y = point[1]
        pose.position.z = point[2]
        waypoints.append(pose)
        # Publish each point as a marker
        marker.points.append(Point(pose.position.x + 0.2, pose.position.y, pose.position.z-1.02))

    # Publish the marker

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
    
    # After executing, stay at the last waypoint
    # Do not command the robot to return to the starting pose

# Main Execution
if __name__ == '__main__':
    current_pose = group.get_current_pose().pose.position
    current_pose.y += 0.15
    current_pose.z += 0.1
    m_height = 0.1  # Change this to the desired height of the 'M'
    m_width = 0.1   # Change this to the desired width of the 'M'
    while not rospy.is_shutdown():
        waypoints = m_trajectory(m_height, m_width, [current_pose.x, current_pose.y, current_pose.z])
        execute_trajectory(waypoints)
        marker_publisher.publish(marker)
        rospy.sleep(5)
        break  # Exit the loop after drawing the 'M'



