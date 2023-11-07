#!/usr/bin/env python3

import rospy
import numpy as np
import sys
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown

class LineTrajectoryExecutor:
    def __init__(self):
        # Initialize the move group for the arm
        self.group = MoveGroupCommander("interbotix_arm")

        # Define the reference frame for target poses
        self.reference_frame = "px100/base_link"
        self.group.set_pose_reference_frame(self.reference_frame)

        # Allow replanning to increase the odds of a trajectory being found
        self.group.allow_replanning(True)

        # Define line start and end points
        self.start = (0, 0.24)  # Adjust based on the desired start point in (y, z)
        self.end = (0, 0.28)    # Adjust based on the desired end point in (y, z)

    def plan_and_move_to_start(self):
        # Plan and move to the starting point
        start_pose = self.group.get_current_pose().pose
        start_pose.position.y = self.start[0]
        start_pose.position.z = self.start[1]
        self.group.set_pose_target(start_pose)
        self.group.go(wait=True)
        self.group.clear_pose_targets()

    def execute_line_trajectory(self):
        waypoints = []
        current_pose = self.group.get_current_pose().pose

        # Generate waypoints that trace a line in the YZ plane
        num_points = 50  # Adjust if needed
        for i in range(num_points):
            fraction = i / float(num_points - 1)
            y = self.start[0] + fraction * (self.end[0] - self.start[0])
            z = self.start[1] + fraction * (self.end[1] - self.start[1])
            current_pose.position.y = y
            current_pose.position.z = z
            waypoints.append(current_pose)

        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0         # jump_threshold
        )

        # Execute the plan
        self.group.execute(plan, wait=True)

if __name__ == '__main__':
    try:
        roscpp_initialize(sys.argv)
        rospy.init_node('line_trajectory', anonymous=True)

        line_trajectory_executor = LineTrajectoryExecutor()

        while not rospy.is_shutdown():
            line_trajectory_executor.plan_and_move_to_start()
            line_trajectory_executor.execute_line_trajectory()
            rospy.sleep(2.0)  # Sleep for a couple of seconds before repeating

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass

