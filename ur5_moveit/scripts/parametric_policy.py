import math
import moveit_commander
import sys
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import re

# Initialization
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur5_draw_trajectory', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("ur5_arm")

# Marker Initialization and Publisher
marker_publisher = rospy.Publisher('trajectory_marker', Marker, queue_size=100, latch=True)


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



def create_marker(id=1, color_rgb=(1.0, 1.0, 1.0)):  # Default color is white
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.id = id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.01  # Line width
    marker.color.r, marker.color.g, marker.color.b = color_rgb
    marker.color.a = 1.0  # Alpha value for transparency
    marker.points = []  # Initialize an empty list of points

    return marker

def generate_circle(height, width, center, marker):
    waypoints = []

    # Parameters for the circle
    radius = min(height, width)
    center_of_circle = (center[0], center[1]+radius, center[2]+height)

    # Create the full circle for the 'O'
    for angle in range(-60, 300, 10):  # Full circle
        rad = math.radians(angle)
        x = center_of_circle[0]
        y = center_of_circle[1] + radius * math.cos(rad)
        z = center_of_circle[2] + radius * math.sin(rad)
        waypoints.append(create_pose((x, y, z)))

    # Close the loop by returning to the start point if needed
    waypoints.append(waypoints[0])

    # Publish the marker points
    for waypoint in waypoints:
        marker.points.append(Point(waypoint.position.x +0.2, waypoint.position.y, waypoint.position.z-1.02))

    # Publish the marker
    return waypoints

def logo_trajectory():
    outer_radius = 0.1
    inner_radius = outer_radius / 2.0
    separation = outer_radius - inner_radius

    current_pose = group.get_current_pose().pose.position
    outer_center = [current_pose.x, current_pose.y, current_pose.z]
    inner_centers = [
        [current_pose.x, current_pose.y, current_pose.z + separation],
        [current_pose.x, current_pose.y + separation * math.sin(math.radians(60)), current_pose.z - separation * math.cos(math.radians(60))],
        [current_pose.x, current_pose.y - separation * math.sin(math.radians(60)), current_pose.z - separation * math.cos(math.radians(60))]
    ]
    concentric_center = outer_center  # This should be the same as outer_center

    # Drawing outer circle
    waypoints = generate_circle(outer_center, outer_radius)
    execute_trajectory(waypoints)
    
    # Drawing three smaller circles
    for center in inner_centers:
        waypoints = generate_circle(center, inner_radius)
        execute_trajectory(waypoints)
    
    # Drawing the concentric circle
    waypoints = generate_circle(concentric_center, inner_radius)
    execute_trajectory(waypoints)
    
    
def a_trajectory(height, width, center, marker):
    waypoints = []

    # Calculate the bottom points of the 'A'
    bottom_left = (center[0], center[1], center[2])
    bottom_right = (center[0], center[1] + width, center[2])

    # Start from the bottom left
    waypoints.append(create_pose(bottom_left))

    # Move to the top (apex of the 'A')
    top = (center[0], center[1]+width/2, center[2]+height)
    waypoints.append(create_pose(top))

    # Move to the bottom right
    waypoints.append(create_pose(bottom_right))

    # Calculate the midpoint at the height for the horizontal line
    mid_height = center[2] + height / 2
    left_mid = (center[0], center[1] + width / 4, mid_height)
    right_mid = (center[0], center[1] + width, mid_height)

    # Draw the horizontal line starting from the bottom right
    waypoints.append(create_pose(right_mid))
    waypoints.append(create_pose(left_mid))

    # Return to the top to complete the trajectory
    waypoints.append(create_pose(top))
    waypoints.append(create_pose(bottom_right))

    # Publish the marker points
    for waypoint in waypoints:
        marker.points.append(Point(waypoint.position.x +0.2, waypoint.position.y, waypoint.position.z-1.02))

    # Publish the marker
    return waypoints
    
def b_trajectory(height, width, center, marker):
    waypoints = []

    # Start from the current position
    start = create_pose((center[0], center[1], center[2]))
    waypoints.append(start)

    # Define the top of the straight line
    top = (center[0], center[1], center[2]+height)

    # Draw the vertical line
    waypoints.append(create_pose(top))

    # Parameters for the semi-circles
    radius = width / 2

    # First semi-circle for the top part of the 'B'
    top_center_of_circle = (top[0], top[1], top[2])
    for angle in range(-90, 91, 5):  # Adjust angle step as needed
        rad = math.radians(angle)
        x = top_center_of_circle[0]
        y = top_center_of_circle[1] + radius * math.cos(rad)
        z = top_center_of_circle[2] - radius * math.sin(rad)
        waypoints.append(create_pose((x, y, z)))

    # Second semi-circle for the bottom part of the 'B'
    bottom_center_of_circle = (top[0], top[1], top[2] - height / 2)
    for angle in range(-90, 91, 5):  # Adjust angle step as needed
        rad = math.radians(angle)
        x = bottom_center_of_circle[0]
        y = bottom_center_of_circle[1] + radius * math.cos(rad)
        z = bottom_center_of_circle[2] - radius * math.sin(rad)
        waypoints.append(create_pose((x, y, z)))

    # No need to return to the bottom
    # The function ends after drawing the second semi-circle

    # Publish the marker points
    for waypoint in waypoints:
        marker.points.append(Point(waypoint.position.x +0.2, waypoint.position.y, waypoint.position.z-1.02))

    # Publish the marker
    return waypoints
    
def c_trajectory(height, width, center, marker):
    waypoints = []

    # Parameters for the semi-circle
    radius = width

    # Center of the circle for the 'C'
    center_of_circle = (center[0], center[1] + radius, center[2] + height / 2)

    # Create the semi-circle for the 'C'
    for angle in range(90, -91, -10):  # Adjust step size as needed
        rad = math.radians(angle)
        x = center_of_circle[0]
        y = center_of_circle[1] - radius * math.cos(rad)  # Adjust Y-coordinate
        z = center_of_circle[2] + radius * math.sin(rad)
        waypoints.append(create_pose((x, y, z)))

    # Publish the marker points
    for waypoint in waypoints:
        marker.points.append(Point(waypoint.position.x +0.2, waypoint.position.y, waypoint.position.z-1.02))

    # Publish the marker
    return waypoints

def d_trajectory(height, width, center, marker):
    waypoints = []

    # Start from the current position
    start = create_pose((center[0], center[1], center[2]))
    waypoints.append(start)

    # Define the top of the straight line
    top = (center[0], center[1], center[2]+height)

    # Draw the vertical line
    waypoints.append(create_pose(top))

    # Parameters for the semi-circles
    radius = width

    # First semi-circle for the top part of the 'B'
    top_center_of_circle = (top[0], top[1] + radius/2, top[2] - height / 2)
    for angle in range(-90, 91, 10):  # Adjust angle step as needed
        rad = math.radians(angle)
        x = top_center_of_circle[0]
        y = top_center_of_circle[1] + radius * math.cos(rad)
        z = top_center_of_circle[2] - radius * math.sin(rad)
        waypoints.append(create_pose((x, y, z)))

    for waypoint in waypoints:
        marker.points.append(Point(waypoint.position.x +0.2, waypoint.position.y, waypoint.position.z-1.02))

    # Publish the marker
    return waypoints
    
def e_trajectory(height, width, center, marker):
    waypoints = []

    # Start from the bottom
    bottom = (center[0], center[1], center[2])
    waypoints.append(create_pose(bottom))

    # Define the top of the vertical line
    top = (center[0], center[1], center[2]+height)
    waypoints.append(create_pose(top))

    # Top horizontal line
    top_horizontal_end = (center[0], center[1] + width, center[2]+height)
    waypoints.append(create_pose(top_horizontal_end))
    waypoints.append(create_pose(top))  # Return to top of vertical line

    # Middle horizontal line
    middle = (center[0], center[1], center[2] + height / 2)
    waypoints.append(create_pose(middle))  # Go to middle
    middle_horizontal_end = (center[0], center[1] + width, center[2] + height / 2)
    waypoints.append(create_pose(middle_horizontal_end))
    waypoints.append(create_pose(middle))  # Return to middle of vertical line

    # Bottom horizontal line
    waypoints.append(create_pose(bottom))  # Go to bottom
    bottom_horizontal_end = (center[0], center[1] + width, center[2])
    waypoints.append(create_pose(bottom_horizontal_end))

    # Publish the marker points
    for waypoint in waypoints:
        marker.points.append(Point(waypoint.position.x +0.2, waypoint.position.y, waypoint.position.z-1.02))

    # Publish the marker
    return waypoints

def f_trajectory(height, width, center, marker):
    waypoints = []

    # Start from the bottom
    bottom = (center[0], center[1], center[2])
    waypoints.append(create_pose(bottom))

    # Define the top of the vertical line
    top = (center[0], center[1], center[2]+height)
    waypoints.append(create_pose(top))

    # Top horizontal line
    top_horizontal_end = (center[0], center[1] + width, center[2]+height)
    waypoints.append(create_pose(top_horizontal_end))
    waypoints.append(create_pose(top))  # Return to top of vertical line

    # Middle horizontal line
    middle = (center[0], center[1], center[2] + height / 2)
    waypoints.append(create_pose(middle))  # Go to middle
    middle_horizontal_end = (center[0], center[1] + width, center[2] + height / 2)
    waypoints.append(create_pose(middle_horizontal_end))
    waypoints.append(create_pose(middle))  # Go to middle
    waypoints.append(create_pose(bottom))

    # No need to return to the bottom, as the 'F' ends with the middle horizontal line

    # Publish the marker points
    for waypoint in waypoints:
        marker.points.append(Point(waypoint.position.x +0.2, waypoint.position.y, waypoint.position.z-1.02))

    # Publish the marker
    return waypoints

def g_trajectory(height, width, center, marker):
    waypoints = []

    # Parameters for the semi-circle (like 'C')
    radius = width

    # Start with a semi-circle
    center_of_semi_circle = (center[0], center[1] + 1.5*radius, center[2] + height/2)
    for angle in range(90, -91, -10):  # Semi-circle
        rad = math.radians(angle)
        x = center_of_semi_circle[0]
        y = center_of_semi_circle[1] - radius * math.cos(rad)
        z = center_of_semi_circle[2] + radius * math.sin(rad)
        waypoints.append(create_pose((x, y, z)))

    vertical_start = (center[0], center[1]+ 1.5*radius, center[2]+0.001)
    vertical_end = (center[0], center[1]+ 1.5*radius, center[2] + height / 2)
    waypoints.append(create_pose(vertical_start))
    waypoints.append(create_pose(vertical_end))

    # Horizontal line starting where the vertical line ends
    # Extend a bit inside the semi-circle
    horizontal_start = (center[0], center[1]+radius, center[2]+height / 2)
    # Extend a bit outside the semi-circle
    horizontal_end = (center[0], center[1] + 1.5*radius, center[2]+height / 2)
    waypoints.append(create_pose(horizontal_start))
    waypoints.append(create_pose(horizontal_end))
    waypoints.append(create_pose(vertical_start))

    # Publish the marker points
    for waypoint in waypoints:
        marker.points.append(Point(waypoint.position.x +0.2, waypoint.position.y, waypoint.position.z-1.02))

    # Publish the marker
    return waypoints
    
def h_trajectory(height, width, center, marker):
    waypoints = []

    # Define the bottom and top points of the first vertical line
    left_bottom = (center[0], center[1] + width / 4, center[2])
    left_top = (center[0], center[1] + width / 4, center[2]+height)

    # First vertical line
    waypoints.append(create_pose(left_bottom))
    waypoints.append(create_pose(left_top))

    # Define the bottom and top points of the second vertical line
    right_bottom = (center[0], center[1] + width, center[2])
    right_top = (center[0], center[1] + width, center[2]+height)

    # Horizontal line in the middle
    middle = (center[0], center[1]+ width / 4, center[2] + height / 2)
    waypoints.append(create_pose(middle))  # Move to the middle of 'H'
    middle_right = (center[0], center[1] + width, center[2] + height / 2)
    waypoints.append(create_pose(middle_right))

    # Second vertical line
    waypoints.append(create_pose(right_top))
    waypoints.append(create_pose(right_bottom))

    # No need to return to the start, as the 'H' ends with the second vertical line

    # Publish the marker points
    for waypoint in waypoints:
        marker.points.append(Point(waypoint.position.x +0.2, waypoint.position.y, waypoint.position.z-1.02))

    # Publish the marker
    return waypoints

def i_trajectory(height, width, center, marker):
    waypoints = []

    # Define the bottom and top of the vertical line
    bottom = (center[0], center[1], center[2])
    top = (center[0], center[1], center[2]+height)

    # Draw the vertical line
    waypoints.append(create_pose(top))
    waypoints.append(create_pose(bottom))

    # No need for additional lines or returning to start, as 'I' is just a straight line

    # Publish the marker points
    for waypoint in waypoints:
        marker.points.append(Point(waypoint.position.x +0.2, waypoint.position.y, waypoint.position.z-1.02))

    # Publish the marker
    return waypoints

def j_trajectory(height, width, center, marker):
    waypoints = []

    # Parameters for the semi-circle at the bottom of 'J'
    radius = width / 2
    bottom_center = (center[0], center[1] + radius, center[2])

    # Create the semi-circle for the bottom of 'J'
    for angle in range(0, 181, 10):  # Half circle
        rad = math.radians(angle)
        x = bottom_center[0]
        y = bottom_center[1] - radius * math.cos(rad)
        z = bottom_center[2] - radius * math.sin(rad)
        waypoints.append(create_pose((x, y, z)))

    # Vertical line for the main part of 'J'
    top = (center[0], center[1] + 2*radius, center[2]+height)
    botom = (center[0], center[1] + 2*radius, center[2])
    waypoints.append(create_pose(top))
    waypoints.append(create_pose(botom))

    # Publish the marker points
    for waypoint in waypoints:
        marker.points.append(Point(waypoint.position.x +0.2, waypoint.position.y, waypoint.position.z-1.02))

    # Publish the marker
    return waypoints

def k_trajectory(height, width, center, marker):
    waypoints = []

    # Define the bottom and top of the vertical line
    bottom = (center[0], center[1], center[2])
    top = (center[0], center[1], center[2]+height)

    # Vertical line for the backbone of the 'K'
    waypoints.append(create_pose(bottom))
    waypoints.append(create_pose(top))

    # The point where the arms of the 'K' meet the vertical line
    middle = (center[0], center[1], center[2] + height / 2)

    # Upper diagonal arm
    upper_diagonal_end = (center[0], center[1] + width / 2, center[2]+height)
    waypoints.append(create_pose(middle))
    waypoints.append(create_pose(upper_diagonal_end))

    # Lower diagonal arm
    lower_diagonal_end = (center[0], center[1] + width / 2, center[2])
    waypoints.append(create_pose(middle))
    waypoints.append(create_pose(lower_diagonal_end))

    # Publish the marker points
    for waypoint in waypoints:
        marker.points.append(Point(waypoint.position.x +0.2, waypoint.position.y, waypoint.position.z-1.02))

    # Publish the marker
    return waypoints

def l_trajectory(height, width, center, marker):
    waypoints = []

    # Define the bottom and top of the vertical line
    bottom = (center[0], center[1], center[2])
    top = (center[0], center[1], center[2]+height)

    # Vertical line for the 'L'
    waypoints.append(create_pose(top))
    waypoints.append(create_pose(bottom))


    # Horizontal line at the bottom
    horizontal_end = (center[0], center[1] + 1.5*width, center[2])
    waypoints.append(create_pose(horizontal_end))

    # Publish the marker points
    for waypoint in waypoints:
        marker.points.append(Point(waypoint.position.x +0.2, waypoint.position.y, waypoint.position.z-1.02))

    # Publish the marker
    return waypoints

def m_trajectory(height, width, center, marker):
    waypoints = []

    half_width = width / 2.0
    bottom_left = (center[0], center[1], center[2])
    top_left = (center[0], center[1], center[2]+height)
    middle = (center[0], center[1]+half_width, center[2] + height / 2)
    top_right = (center[0], center[1] + width, center[2]+height)
    bottom_right = (center[0], center[1] + width, center[2])

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

# Define the 'M' trajectory in YZ plane
def n_trajectory(height, width, center, marker):
    waypoints = []

    # Define the bottom and top points of the first vertical line
    bottom_left = (center[0], center[1], center[2])
    top_left = (center[0], center[1], center[2]+height)

    # First vertical line
    waypoints.append(create_pose(bottom_left))
    waypoints.append(create_pose(top_left))

    # Diagonal line to the bottom right
    bottom_right = (center[0], center[1] + width, center[2])
    waypoints.append(create_pose(bottom_right))

    # Second vertical line
    top_right = (center[0], center[1] + width, center[2]+height)
    waypoints.append(create_pose(top_right))
    waypoints.append(create_pose(bottom_right))

    # Publish the marker points
    for waypoint in waypoints:
        marker.points.append(Point(waypoint.position.x +0.2, waypoint.position.y, waypoint.position.z-1.02))

    # Publish the marker
    return waypoints
    
def o_trajectory(height, width, center, marker):
    waypoints = []

    # Parameters for the circle
    radius = min(height, width)
    center_of_circle = (center[0], center[1]+radius/2, center[2]+height/2)

    # Create the full circle for the 'O'
    for angle in range(-60, 300, 10):  # Full circle
        rad = math.radians(angle)
        x = center_of_circle[0]
        y = center_of_circle[1] + radius * math.cos(rad)
        z = center_of_circle[2] + radius * math.sin(rad)
        waypoints.append(create_pose((x, y, z)))

    # Close the loop by returning to the start point if needed
    waypoints.append(waypoints[0])

    # Publish the marker points
    for waypoint in waypoints:
        marker.points.append(Point(waypoint.position.x +0.2, waypoint.position.y, waypoint.position.z-1.02))

    # Publish the marker
    return waypoints

def p_trajectory(height, width, center, marker):
    waypoints = []

    # Define the bottom and top of the vertical line
    bottom = (center[0], center[1], center[2])
    top = (center[0], center[1], center[2]+height)

    # Straight line for the backbone of the 'P'
    waypoints.append(create_pose(bottom))
    waypoints.append(create_pose(top))

    # Parameters for the semi-circle
    radius = width / 2
    top_center_of_circle = (top[0], top[1], top[2])

    # Semi-circle for the top of the 'P'
    for angle in range(-90, 91, 10):  # Adjust step size as needed
        rad = math.radians(angle)
        x = top_center_of_circle[0]
        y = top_center_of_circle[1] + radius * math.cos(rad)
        z = top_center_of_circle[2] - radius * math.sin(rad)
        waypoints.append(create_pose((x, y, z)))

    # Return to the top to close the loop
    waypoints.append(create_pose(bottom))

    # Publish the marker points
    for waypoint in waypoints:
        marker.points.append(Point(waypoint.position.x +0.2, waypoint.position.y, waypoint.position.z-1.02))

    # Publish the marker
    return waypoints
    
def q_trajectory(height, width, center, marker):
    waypoints = []

    # Parameters for the circle
    radius = min(height, width)
    center_of_circle = (center[0], center[1], center[2]+height/2)

    # Create the full circle for the 'Q'
    for angle in range(0, 360, 10):  # Full circle
        rad = math.radians(angle)
        x = center_of_circle[0]
        y = center_of_circle[1] + radius * math.cos(rad)
        z = center_of_circle[2] + radius * math.sin(rad)
        waypoints.append(create_pose((x, y, z)))

    # Tail of the 'Q'
    tail_start = (center_of_circle[0], center_of_circle[1] + radius / math.sqrt(2), center_of_circle[2] - radius / math.sqrt(2))
    tail_end = (tail_start[0], tail_start[1] + width / 4, tail_start[2] - width / 4)  # Adjust tail length and angle
    waypoints.append(create_pose(tail_start))
    waypoints.append(create_pose(tail_end))

    # Publish the marker points
    for waypoint in waypoints:
        marker.points.append(Point(waypoint.position.x +0.2, waypoint.position.y, waypoint.position.z-1.02))

    # Publish the marker
    return waypoints

def r_trajectory(height, width, center, marker):
    waypoints = []

    # Define the bottom and top of the straight line
    bottom = (center[0], center[1], center[2])
    top = (center[0], center[1], center[2]+height)

    # Straight line for the leg of the 'R'
    waypoints.append(create_pose(bottom))
    waypoints.append(create_pose(top))

    # Semi-circle for the top of the 'R'
    radius = width / 2
    center_of_semi_circle = (top[0], top[1], top[2])

    for angle in range(-90, 91, 10):  # Create the semi-circle
        rad = math.radians(angle)
        x = center_of_semi_circle[0]
        y = center_of_semi_circle[1] + radius * math.cos(rad)
        z = center_of_semi_circle[2] - radius * math.sin(rad)
        waypoints.append(create_pose((x, y, z)))

    # Diagonal line for the leg of the 'R'
    diagonal_end = (center[0], center[1] + width, center[2])
    waypoints.append(create_pose(diagonal_end))

    # Publish the marker points
    for waypoint in waypoints:
        marker.points.append(Point(waypoint.position.x +0.2, waypoint.position.y, waypoint.position.z-1.02))

    # Publish the marker
    return waypoints
    
def s_trajectory(height, width, center, marker):
    waypoints = []

    # Parameters for the first 'C' curve (top half of 'S')
    top_radius = width
    top_center_of_curve = (center[0], center[1], center[2]+height)

    # Create the first 'C' curve (top half of 'S')
    for angle in range(120, -91, -10):  # Semi-circle
        rad = math.radians(angle)
        x = top_center_of_curve[0]
        y = top_center_of_curve[1] - top_radius * math.cos(rad)
        z = top_center_of_curve[2] + top_radius * math.sin(rad)
        waypoints.append(create_pose((x, y, z)))

    # Parameters for the second 'C' curve (bottom half of 'S')
    bottom_radius = width
    bottom_center_of_curve = (center[0], center[1], top_center_of_curve[2]-height/2)

    # Create the second 'C' curve (bottom half of 'S')
    for angle in range(-70, 140, 10):  # Semi-circle
        rad = math.radians(angle)
        x = bottom_center_of_curve[0]
        y = bottom_center_of_curve[1] + bottom_radius * math.cos(rad)
        z = bottom_center_of_curve[2] - bottom_radius * math.sin(rad)
        waypoints.append(create_pose((x, y, z)))

    # Publish the marker points
    for waypoint in waypoints:
        marker.points.append(Point(waypoint.position.x +0.2, waypoint.position.y, waypoint.position.z-1.02))

    # Publish the marker
    return waypoints


def t_trajectory(height, width, center, marker):
    waypoints = []

    # Define the top middle point and the ends of the horizontal line
    top_middle = (center[0], center[1]+width/2, center[2]+height)
    left_end = (center[0], center[1], center[2]+height)
    right_end = (center[0], center[1] + width, center[2]+height)

    # Horizontal line at the top
    waypoints.append(create_pose(left_end))
    waypoints.append(create_pose(right_end))

    # Vertical line in the middle
    bottom_middle = (center[0], center[1]+width/2, center[2])
    waypoints.append(create_pose(top_middle))
    waypoints.append(create_pose(bottom_middle))

    # Publish the marker points
    for waypoint in waypoints:
        marker.points.append(Point(waypoint.position.x +0.2, waypoint.position.y, waypoint.position.z-1.02))

    # Publish the marker
    return waypoints 
    
def u_trajectory(height, width, center, marker):
    waypoints = []

    # Define the top points and the bottom center of the 'U'
    top_left = (center[0], center[1], center[2]+height)
    bottom_left = (center[0], center[1], center[2]-0.001)
    top_right = (center[0], center[1] + width, center[2]+height)
    bottom_right = (center[0], center[1] + width, center[2])
    bottom_center = (center[0], center[1]+width/2, center[2])


    waypoints.append(create_pose(top_left))
    waypoints.append(create_pose(bottom_left))

    # Bottom semi-circle
    radius = width / 2
    for angle in range(180, 360, 10):  # Semi-circle
        rad = math.radians(angle)
        x = bottom_center[0]
        y = bottom_center[1] + radius * math.cos(rad)
        z = bottom_center[2] + radius * math.sin(rad)
        waypoints.append(create_pose((x, y, z)))

    # Right vertical line
    waypoints.append(create_pose(bottom_right))
    waypoints.append(create_pose(top_right))
    waypoints.append(create_pose(bottom_right))

    # Publish the marker points
    for waypoint in waypoints:
        marker.points.append(Point(waypoint.position.x + 0.2, waypoint.position.y, waypoint.position.z-1.02))

    # Publish the marker
    return waypoints


def v_trajectory(height, width, center, marker):
    waypoints = []

    # Define the top points and the bottom point of the 'V'
    top_left = (center[0], center[1], center[2]+height)
    top_right = (center[0], center[1] + width, center[2]+height)
    bottom = (center[0], center[1]+width/2, center[2])

    # Diagonal line from top left to bottom
    waypoints.append(create_pose(top_left))
    waypoints.append(create_pose(bottom))

    # Diagonal line from bottom to top right
    waypoints.append(create_pose(top_right))
    waypoints.append(create_pose(bottom))

    # No need to return to the start, as the 'V' ends at the top right

    # Publish the marker points
    for waypoint in waypoints:
        marker.points.append(Point(waypoint.position.x +0.2, waypoint.position.y, waypoint.position.z-1.02))

    # Publish the marker
    return waypoints
    
def w_trajectory(height, width, center, marker):
    waypoints = []

    # Define the points of the 'W'
    top_left = (center[0], center[1] , center[2]+height)
    bottom_left = (center[0], center[1], center[2])
    middle_top = (center[0], center[1]+width / 2, center[2]+height/2)
    bottom_right = (center[0], center[1] + width, center[2])
    top_right = (center[0], center[1] + width, center[2]+height)

    # Diagonal line from top left to bottom left
    waypoints.append(create_pose(top_left))
    waypoints.append(create_pose(bottom_left))

    # Diagonal line from bottom left to middle top
    waypoints.append(create_pose(middle_top))

    # Diagonal line from middle top to bottom right
    waypoints.append(create_pose(bottom_right))

    # Diagonal line from bottom right to top right
    waypoints.append(create_pose(top_right))
    waypoints.append(create_pose(bottom_right))

    # Publish the marker points
    for waypoint in waypoints:
        marker.points.append(Point(waypoint.position.x +0.2, waypoint.position.y, waypoint.position.z-1.02))

    # Publish the marker
    return waypoints
    
def x_trajectory(height, width, center, marker):
    waypoints = []

    # Define the points for the 'X'
    top_right = (center[0], center[1]+width, center[2]+height)
    bottom_right = (center[0], center[1] + width, center[2])
    middle = (center[0], center[1]+width/2, center[2]+height/2)
    left_right = (center[0], center[1], center[2]+height)
    bottom_left = (center[0], center[1], center[2])
    
    waypoints.append(create_pose(bottom_left))
    waypoints.append(create_pose(top_right))
    waypoints.append(create_pose(middle))
    waypoints.append(create_pose(left_right))
    waypoints.append(create_pose(bottom_right))

    # Publish the marker points
    for waypoint in waypoints:
        marker.points.append(Point(waypoint.position.x +0.2, waypoint.position.y, waypoint.position.z-1.02))

    # Publish the marker
    return waypoints
    
def y_trajectory(height, width, center, marker):
    waypoints = []

    # Define the top points and the middle point of the 'Y'
    top_left = (center[0], center[1], center[2]+height)
    top_right = (center[0], center[1] + width, center[2]+height)
    middle = (center[0], center[1]+width/2, center[2] + height / 2)

    # Diagonal line from top left to middle
    waypoints.append(create_pose(top_left))
    waypoints.append(create_pose(middle))

    # Diagonal line from top right to middle
    waypoints.append(create_pose(top_right))
    waypoints.append(create_pose(middle))

    # Vertical line from middle downwards
    bottom = (center[0], center[1]+width/2, center[2])
    waypoints.append(create_pose(bottom))

    # Publish the marker points
    for waypoint in waypoints:
        marker.points.append(Point(waypoint.position.x +0.2, waypoint.position.y, waypoint.position.z-1.02))

    # Publish the marker
    return waypoints

def z_trajectory(height, width, center, marker):
    waypoints = []

    # Define the points for the 'Z'
    top_left = (center[0], center[1], center[2]+height)
    top_right = (center[0], center[1] + width, center[2]+height)
    bottom_left = (center[0], center[1], center[2])
    bottom_right = (center[0], center[1] + width, center[2])

    # Top horizontal line
    waypoints.append(create_pose(top_left))
    waypoints.append(create_pose(top_right))

    # Diagonal line to bottom left
    waypoints.append(create_pose(bottom_left))

    # Bottom horizontal line
    waypoints.append(create_pose(bottom_right))

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


def extract_name(input_text):
    # Updated pattern to include different entities and possessive pronouns
    pattern = r"(write|draw|show|try|create|sketch|depict) (his |her |my |this |our )?(brand's |company's |organization's |name's |brand |company |organization |name )?(\w+)"
    match = re.search(pattern, input_text, re.IGNORECASE)

    if match:
        return match.group(4)  # The name or brand should be in the fourth group
    else:
        return None

