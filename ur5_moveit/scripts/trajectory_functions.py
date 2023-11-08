import math
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

def create_pose(point, group):
    pose = group.get_current_pose().pose
    pose.position.x = point[0]
    pose.position.y = point[1]
    pose.position.z = point[2]
    return pose

def generate_circle(center, radius, num_points=100, marker=None, group=None):
    waypoints = []
    angle_step = 2 * math.pi / num_points
    for i in range(num_points):
        angle = angle_step * i
        x = center[0]
        y = center[1] + radius * math.cos(angle)
        z = center[2] + radius * math.sin(angle)
        pose = group.get_current_pose().pose
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        waypoints.append(pose)
        
        # Add to marker points if marker is provided
        if marker:
            marker.points.append(Point(x+0.2, y, z-1.02))
    return waypoints

def logo_trajectory(current_pose, group, marker):
    outer_radius = 0.1
    inner_radius = outer_radius / 2.0

    separation = outer_radius - inner_radius  # Distance from the center of outer circle to the center of inner circles
    outer_center = [current_pose.x, current_pose.y, current_pose.z]
    inner_center1 = [current_pose.x, current_pose.y, current_pose.z + separation]
    inner_center2 = [current_pose.x, current_pose.y + separation * math.sin(math.radians(60)), current_pose.z - separation * math.cos(math.radians(60))]
    inner_center3 = [current_pose.x, current_pose.y - separation * math.sin(math.radians(60)), current_pose.z - separation * math.cos(math.radians(60))]
    concentric_center = [current_pose.x, current_pose.y, current_pose.z]  # This should be the same as outer_center

    all_waypoints = []

    # Outer circle waypoints
    all_waypoints.extend(generate_circle(outer_center, outer_radius, marker=marker, group=group))
    
    # Three smaller circles waypoints
    for center in [inner_center1, inner_center2, inner_center3]:
        all_waypoints.extend(generate_circle(center, inner_radius, marker=marker, group=group))

    # Concentric circle waypoints
    all_waypoints.extend(generate_circle(concentric_center, inner_radius, marker=marker, group=group))

    return all_waypoints
    
def i_trajectory(height, center, group, marker):
    waypoints = []

    # Define the bottom and top of the straight line
    bottom = (center[0], center[1], center[2] - height / 2)
    top = (center[0], center[1], center[2] + height / 2)

    # Straight line for the body of the 'I'
    waypoints.append(create_pose(bottom, group))
    waypoints.append(create_pose(top, group))

    # Publish the marker points
    for waypoint in waypoints:
        marker.points.append(Point(waypoint.position.x+0.2, waypoint.position.y, waypoint.position.z-1.02))

    return waypoints


    
def m_trajectory(height, width, center, group, marker):
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

    return waypoints


def r_trajectory(height, width, center, group, marker):
    waypoints = []

    # Define the bottom and top of the straight line
    bottom = (center[0], center[1], center[2] - height)
    top = (center[0], center[1], center[2])

    # Straight line for the leg of the 'R'
    waypoints.append(create_pose(bottom, group))
    waypoints.append(create_pose(top, group))

    # Half-circle for the top of the 'R'
    radius = width
    center_of_circle = (top[0], top[1], top[2] + radius)

    for angle in range(-90, 91, 10):  # Create the semicircle sideways
        rad = math.radians(angle)
        x = center_of_circle[0]
        y = center_of_circle[1] + radius * math.cos(rad)
        z = center_of_circle[2] - radius * math.sin(rad)
        waypoints.append(create_pose((x, y, z), group))

    # Diagonal line for the leg of the 'R'
    diagonal_start = (center_of_circle[0], center_of_circle[1], center_of_circle[2])
    diagonal_end = (center[0], center[1] + width, center[2]-height)
    waypoints.append(create_pose(diagonal_start, group))
    waypoints.append(create_pose(diagonal_end, group))

    # Publish the marker points
    for waypoint in waypoints:
        marker.points.append(Point(waypoint.position.x +0.2, waypoint.position.y, waypoint.position.z-1.02))

    return waypoints
    
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
