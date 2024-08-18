#!/usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped

class TurtleBot3Controller:
    def __init__(self):
        rospy.init_node('turtlebot3_controller', anonymous=True)

        # Action client to communicate with move_base
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

    def move_to_goal(self, x_goal, y_goal):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x_goal
        goal.target_pose.pose.position.y = y_goal
        goal.target_pose.pose.orientation.w = 1.0  # Facing forward

        rospy.loginfo(f"Sending goal: x={x_goal}, y={y_goal}")
        self.move_base_client.send_goal(goal)
        wait = self.move_base_client.wait_for_result()

        if not wait:
            rospy.logwarn("Action server not available!")
        else:
            rospy.loginfo("Goal reached!" if self.move_base_client.get_result() else "Failed to reach goal.")

if __name__ == '__main__':
    try:
        controller = TurtleBot3Controller()
        x_goal = float(input("Enter the x-coordinate : "))
        y_goal = float(input("Enter the y-coordinate : "))
        controller.move_to_goal(x_goal, y_goal)
    except rospy.ROSInterruptException:
        pass
