#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker

# Initialization
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('mu_logo', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("ur5_arm")

# Marker Initialization and Publisher
marker_publisher = rospy.Publisher('trajectory_marker', Marker, queue_size=10000, latch=True)
def create_marker(marker_id, r, g, b, a=1.0):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.025  # Line width
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.color.a = a  # Alpha is set to 1 for full opacity
    marker.points = []
    return marker

marker_square_1 = create_marker(marker_id=1, r=0.0, g=0.165, b=0.5)  # Blue triangle
marker_square_2 = create_marker(marker_id=2, r=1.0, g=0.75, b=0.0)  # Yellow triangle
marker_square_3 = create_marker(marker_id=3, r=0.1, g=0.455, b=0.54)  # Green square
marker_square_4 = create_marker(marker_id=4, r=0.635, g=0.0, b=0.0)  # Red square


    
def square_trajectory(marker, side_length, center, is_left_triangle=False, is_right_triangle=False):
    waypoints = []

    # Define the points for the square or triangle
    if is_left_triangle:
        # Define the triangle points (assuming a right-angled triangle)
        corners = [
            (center[0], center[1] + side_length / 2, center[2] + side_length / 2),  # Right angle at bottom-left
            (center[0], center[1] - side_length / 2, center[2] + side_length / 2),  # Top-left corner
            (center[0], center[1] + side_length / 2, center[2] - side_length / 2),  # Bottom-right corner
            (center[0], center[1] + side_length / 2, center[2] + side_length / 2),  # Back to right angle to close triangle
        ]
    # Define the points for the square or triangle
    elif is_right_triangle:
        # Define the triangle points (assuming a right-angled triangle)
        corners = [
            (center[0], center[1] - side_length / 2, center[2] + side_length / 2),  # Right angle at bottom-left
            (center[0], center[1] + side_length / 2, center[2] + side_length / 2),  # Top-left corner
            (center[0], center[1] - side_length / 2, center[2] - side_length / 2),  # Bottom-right corner
            (center[0], center[1] - side_length / 2, center[2] + side_length / 2),  # Back to right angle to close triangle
        ]
    else:
        # Define the square points
        corners = [
            (center[0], center[1] - side_length / 2, center[2] - side_length / 2),
            (center[0], center[1] - side_length / 2, center[2] + side_length / 2),
            (center[0], center[1] + side_length / 2, center[2] + side_length / 2),
            (center[0], center[1] + side_length / 2, center[2] - side_length / 2),
            (center[0], center[1] - side_length / 2, center[2] - side_length / 2),  # Back to start to close the square
        ]

    # Generate waypoints for the corners of the shape
    for corner in corners:
        pose = group.get_current_pose().pose
        pose.position.x = corner[0]
        pose.position.y = corner[1]
        pose.position.z = corner[2]
        waypoints.append(pose)
        # Publish each corner as a marker
        marker.points.append(Point(pose.position.x+0.2, pose.position.y, pose.position.z-1.02))
    return waypoints


# Command the UR5 to follow the trajectory
def execute_trajectory(waypoints):
    (plan, fraction) = group.compute_cartesian_path(
                            waypoints,
                            0.01,  # eef_step
                            0.0)   # jump_threshold

    if fraction < 1.0:
        print("Could not compute full trajectory. Only " + str(fraction * 100) + "% achieved.")
        return False  # Indicate that the full trajectory could not be computed

    print("Full trajectory computed!")
    group.execute(plan, wait=True)
    return True  # Indicate successful execution


if __name__ == '__main__':
    # Get the current position as the starting point
    start_pose = group.get_current_pose().pose.position

    side_length = 0.05  # Define the side length for squares and triangles
    vertical_spacing = side_length  # The vertical spacing is equal to the side length

    # Define the centers of the top layer squares
    centers_top_layer1 = [
        [start_pose.x, start_pose.y - side_length / 2, start_pose.z],   # Top-left square center
    ]
    centers_top_layer2 = [
        [start_pose.x, start_pose.y + side_length / 2, start_pose.z],   # Top-right square center
    ]

    # Define the centers of the middle layer squares
    centers_middle_layer1 = [
        [start_pose.x, start_pose.y - side_length / 2, start_pose.z - vertical_spacing],   # Middle-left square center
    ]
    
    centers_middle_layer2 = [
        [start_pose.x, start_pose.y + side_length / 2, start_pose.z - vertical_spacing],   # Middle-right square center
    ]

    # Define the centers of the bottom layer triangles
    centers_bottom_layer1 = [
        [start_pose.x, start_pose.y - side_length / 2, start_pose.z - 2 * vertical_spacing],   # Bottom-left triangle center
    ]
    # Define the centers of the bottom layer triangles
    centers_bottom_layer2 = [
        [start_pose.x, start_pose.y + side_length / 2, start_pose.z - 2 * vertical_spacing],   # Bottom-right triangle center
    ]

    # Draw the top layer squares
    for center in centers_top_layer1:
        waypoints = square_trajectory(marker_square_1, side_length, center)
        success = execute_trajectory(waypoints)
        marker_publisher.publish(marker_square_1)
        if not success:
            print("Stopping execution due to trajectory planning failure for top layer square.")
            break
    # Draw the top layer squares
    for center in centers_top_layer2:
        waypoints = square_trajectory(marker_square_2, side_length, center)
        success = execute_trajectory(waypoints)
        marker_publisher.publish(marker_square_2)
        if not success:
            print("Stopping execution due to trajectory planning failure for top layer square.")
            break

    # Draw the middle layer squares
    for center in centers_middle_layer1:
        waypoints = square_trajectory(marker_square_3, side_length, center)
        success = execute_trajectory(waypoints)
        marker_publisher.publish(marker_square_3)
        if not success:
            print("Stopping execution due to trajectory planning failure for middle layer square.")
            break
    # Draw the middle layer squares
    for center in centers_middle_layer2:
        waypoints = square_trajectory(marker_square_4, side_length, center)
        success = execute_trajectory(waypoints)
        marker_publisher.publish(marker_square_4)
        if not success:
            print("Stopping execution due to trajectory planning failure for middle layer square.")
            break

    # Draw the bottom layer triangles
    for center in centers_bottom_layer1:
        waypoints = square_trajectory(marker_square_3, side_length, center, is_left_triangle=True)
        success = execute_trajectory(waypoints)
        marker_publisher.publish(marker_square_3)
        if not success:
            print("Stopping execution due to trajectory planning failure for bottom layer triangle.")
            break
    # Draw the bottom layer triangles
    for center in centers_bottom_layer2:
        waypoints = square_trajectory(marker_square_4, side_length, center, is_right_triangle=True)
        success = execute_trajectory(waypoints)
        marker_publisher.publish(marker_square_4)
        if not success:
            print("Stopping execution due to trajectory planning failure for bottom layer triangle.")
            break

    rospy.sleep(5)  # Pause to observe the drawing



