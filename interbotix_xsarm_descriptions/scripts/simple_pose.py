#!/usr/bin/env python3

import rospy
import sys
import copy
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown

class WaypointsExecutor:
    def __init__(self):
        # Initialize the move group for the arm
        self.group = MoveGroupCommander("interbotix_arm")

        # Define the reference frame for target poses
        self.reference_frame = "px100/base_link"
        self.group.set_pose_reference_frame(self.reference_frame)

        # Allow replanning to increase the odds of a trajectory being found
        self.group.allow_replanning(True)

    def execute_waypoints_trajectory(self, scale=1.0):
        waypoints = []

        wpose = self.group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        waypoints.append(copy.deepcopy(wpose))



        (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)
        
        # Execute the plan
        self.group.execute(plan, wait=True)

if __name__ == '__main__':
    try:
        roscpp_initialize(sys.argv)
        rospy.init_node('waypoints_trajectory', anonymous=True)

        waypoints_executor = WaypointsExecutor()

        while not rospy.is_shutdown():
            waypoints_executor.execute_waypoints_trajectory()
            rospy.sleep(2.0)  # Sleep for a couple of seconds before repeating

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass

