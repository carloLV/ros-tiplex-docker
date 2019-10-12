#!/usr/bin/env python
import roslib
import rospy
import actionlib
from move_base_msgs.msg import *
from actionlib_msgs.msg import GoalStatusArray

def publish_goal_client():

	x_y = get_input()

	client = actionlib.SimpleActionClient('goal_publisher', MoveBaseAction)
	client.wait_for_server()
	rospy.loginfo('Server connected, sending goal ...')

	goal = MoveBaseGoal()
	goal.target_pose.header.stamp=rospy.Time.now()
	goal.target_pose.header.frame_id="/map"
	goal.target_pose.pose.orientation.w=1.0
	if x_y[0] == 454 and x_y[1]==454:
		sys.exit(0)
	goal.target_pose.pose.position.x=x_y[0]
	goal.target_pose.pose.position.y=x_y[1]

	client.send_goal(goal)
	rospy.loginfo('Sent goal, waiting for result ...')
	client.wait_for_result()

	return client.get_result()

def get_input():
	x = input('X position value: ')
	y = input('Y position value: ')
	return (x,y)

if __name__ == '__main__':
	try:
		rospy.init_node('pose_publisher', anonymous=True)
		result = publish_goal_client()
		print result
	except rospy.ROSInterruptException:
		rospy.loginfo("program interrupted before completion")