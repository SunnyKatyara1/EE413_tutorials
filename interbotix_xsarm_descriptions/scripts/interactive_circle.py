#!/usr/bin/env python3

import rospy
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from math import pi, cos, sin
import tf


class InteractiveCircle:
    def __init__(self):
        rospy.init_node("interactive_circle")
        
        # Create an interactive marker server on the topic namespace simple_marker
        self.server = InteractiveMarkerServer("simple_marker")
        
        # Create an interactive marker for our server
        self.int_marker = InteractiveMarker()
        self.int_marker.header.frame_id = "px100/base_link"
        self.int_marker.name = "circle_marker"
        self.int_marker.description = "Interactive Circle"
        
        # Set the initial position of the marker (change as required)
        self.int_marker.pose.position.z = 0.25
        # Set the initial position of the marker (change as required)
        self.int_marker.pose.position.x = 0.08
        
        # Set a scaling factor for our marker
        self.int_marker.scale = 1.0
        
        # Create a marker to be placed inside our interactive marker
        marker = Marker()
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.01
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0
        
         # Rotate the circle to align with the X-axis
        q = tf.transformations.quaternion_from_euler(pi/2, 0, pi/2)
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]
        
        # Define the circle's properties
        circle_points = 100
        radius = 0.1
        
        for i in range(circle_points+1):  # +1 to close the circle
            theta = 2 * 3.14159265359 * i / circle_points
            x = radius * cos(theta)
            y = radius * sin(theta)
            marker.points.append(Point(x, y, 0))
        
        # Add the marker to our interactive marker
        self.int_marker.controls.append(InteractiveMarkerControl(interaction_mode=InteractiveMarkerControl.MOVE_3D, always_visible=True, markers=[marker]))
        
        # Add the interactive marker to our server
        self.server.insert(self.int_marker, self.handle_feedback)
        
        self.server.applyChanges()

    def handle_feedback(self, feedback):
        rospy.loginfo(feedback)

if __name__ == "__main__":
    circle = InteractiveCircle()
    rospy.spin()
