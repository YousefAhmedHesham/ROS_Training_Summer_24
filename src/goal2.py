#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

# Initialize global variables
current_pose_turtle1 = Pose()
current_pose_turtle2 = Pose()

def pose_callback_turtle1(data):
    global current_pose_turtle1
    current_pose_turtle1 = data

def pose_callback_turtle2(data):
    global current_pose_turtle2
    current_pose_turtle2 = data

def move_to_goal(x_goal_turtle1, y_goal_turtle1, x_goal_turtle2, y_goal_turtle2):
    global current_pose_turtle1, current_pose_turtle2

    rospy.init_node('turtle_move_to_goal', anonymous=True)
    pub_turtle1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    pub_turtle2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
    sub_turtle1 = rospy.Subscriber('/turtle1/pose', Pose, pose_callback_turtle1)
    sub_turtle2 = rospy.Subscriber('/turtle2/pose', Pose, pose_callback_turtle2)
    rate = rospy.Rate(10)

    goal_pose_turtle1 = Pose()
    goal_pose_turtle1.x = x_goal_turtle1
    goal_pose_turtle1.y = y_goal_turtle1

    goal_pose_turtle2 = Pose()
    goal_pose_turtle2.x = x_goal_turtle2
    goal_pose_turtle2.y = y_goal_turtle2

    vel_msg_turtle1 = Twist()
    vel_msg_turtle2 = Twist()

    while not rospy.is_shutdown():
        # Calculate the distance to the goal for Turtle 1
        distance_turtle1 = math.sqrt((goal_pose_turtle1.x - current_pose_turtle1.x)**2 + (goal_pose_turtle1.y - current_pose_turtle1.y)**2)
        
        # Calculate the linear velocity for Turtle 1
        vel_msg_turtle1.linear.x = 2.0 * distance_turtle1

        # Calculate the angle to the goal for Turtle 1
        angle_to_goal_turtle1 = math.atan2(goal_pose_turtle1.y - current_pose_turtle1.y, goal_pose_turtle1.x - current_pose_turtle1.x)
        angle_diff_turtle1 = angle_to_goal_turtle1 - current_pose_turtle1.theta

        # Normalize the angle difference to [-pi, pi] for Turtle 1
        angle_diff_turtle1 = (angle_diff_turtle1 + math.pi) % (2 * math.pi) - math.pi

        # Calculate the angular velocity for Turtle 1
        vel_msg_turtle1.angular.z = 4.0 * angle_diff_turtle1

        # Stop Turtle 1 when it reaches the goal
        if distance_turtle1 < 0.01:
            vel_msg_turtle1.linear.x = 0
            vel_msg_turtle1.angular.z = 0

        # Calculate the distance to the goal for Turtle 2
        distance_turtle2 = math.sqrt((goal_pose_turtle2.x - current_pose_turtle2.x)**2 + (goal_pose_turtle2.y - current_pose_turtle2.y)**2)

        # Calculate the linear velocity for Turtle 2
        vel_msg_turtle2.linear.x = 2.0 * distance_turtle2

        # Calculate the angle to the goal for Turtle 2
        angle_to_goal_turtle2 = math.atan2(goal_pose_turtle2.y - current_pose_turtle2.y, goal_pose_turtle2.x - current_pose_turtle2.x)
        angle_diff_turtle2 = angle_to_goal_turtle2 - current_pose_turtle2.theta

        # Normalize the angle difference to [-pi, pi] for Turtle 2
        angle_diff_turtle2 = (angle_diff_turtle2 + math.pi) % (2 * math.pi) - math.pi

        # Calculate the angular velocity for Turtle 2
        vel_msg_turtle2.angular.z = 4.0 * angle_diff_turtle2

        # Stop Turtle 2 when it reaches the goal
        if distance_turtle2 < 0.01:
            vel_msg_turtle2.linear.x = 0
            vel_msg_turtle2.angular.z = 0

        # Publish the velocity messages
        pub_turtle1.publish(vel_msg_turtle1)
        pub_turtle2.publish(vel_msg_turtle2)
        rate.sleep()

if __name__ == '__main__':
    try:
        x_goal_turtle1 = float(input("Enter the x goal for Turtle 1: "))
        y_goal_turtle1 = float(input("Enter the y goal for Turtle 1: "))
        x_goal_turtle2 = float(input("Enter the x goal for Turtle 2: "))
        y_goal_turtle2 = float(input("Enter the y goal for Turtle 2: "))
        move_to_goal(x_goal_turtle1, y_goal_turtle1, x_goal_turtle2, y_goal_turtle2)
    except rospy.ROSInterruptException:
        pass
