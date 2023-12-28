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
marker.id = 0
marker.type = Marker.LINE_STRIP
marker.action = Marker.ADD
marker.pose.orientation.w = 1.0
marker.scale.x = 0.01  # Line width
marker.color.r = 0.93
marker.color.g = 0.51
marker.color.b = 0.93
marker.color.a = 1.0
marker.points = []

def generate_circle(center, radius, num_points=100):
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
        
        # Add to marker points
        marker.points.append(Point(x+0.2, y, z-1.02))
    return waypoints

if __name__ == '__main__':
    outer_radius = 0.1
    inner_radius = outer_radius / 2.0

    separation = outer_radius - inner_radius  # Distance from the center of outer circle to the center of inner circles
    current_pose = group.get_current_pose().pose.position

    outer_center = [current_pose.x, current_pose.y, current_pose.z]
    inner_center1 = [current_pose.x, current_pose.y, current_pose.z + separation]
    inner_center2 = [current_pose.x, current_pose.y + separation * math.sin(math.radians(60)), current_pose.z - separation * math.cos(math.radians(60))]
    inner_center3 = [current_pose.x, current_pose.y - separation * math.sin(math.radians(60)), current_pose.z - separation * math.cos(math.radians(60))]
    concentric_center = [current_pose.x, current_pose.y, current_pose.z]  # This should be the same as outer_center

    drawn = False

    while not rospy.is_shutdown():
        if not drawn:
            # Drawing outer circle
            waypoints = generate_circle(outer_center, outer_radius)
            execute_trajectory(waypoints)
            
            # Drawing three smaller circles
            for center in [inner_center1, inner_center2, inner_center3]:
                waypoints = generate_circle(center, inner_radius)
                execute_trajectory(waypoints)
            
            # Drawing the concentric circle
            waypoints = generate_circle(concentric_center, inner_radius)
            execute_trajectory(waypoints)

            marker_publisher.publish(marker)
            drawn = True
            break

        rospy.sleep(5)

