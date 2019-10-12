#!/usr/bin/env python
import sys
import roslib
import rospy
import actionlib

from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback, MoveBaseActionResult
from actionlib_msgs.msg import GoalStatusArray, GoalStatus

pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

"""
This node serves like a client to test the move_base of a robot. All you do using RVIZ and navigation Tutorial, you can do using this node. Simply takes coordinates and prints feedback in case of SUCCESS or FAILURE.
Insert input following the instructions printed on terminal.
It doesn't use actionLibs.
"""

def publish_goal(x_y):
	"""
	Publishes the @PoseStamped message with value from tuple x_y
	@params: tuple(int, int)
	"""
	msg=PoseStamped()
	msg.header.stamp=rospy.Time.now()
	msg.header.frame_id="odom"
	msg.pose.orientation.w=1.0
	if x_y[0] == 454 and x_y[1]==454:
		sys.exit(0)
		return
	msg.pose.position.x=x_y[0]
	msg.pose.position.y=x_y[1]
	rospy.loginfo("Publishing pose stamped")
	pub.publish(msg)

"""
#We don't need this function, for now
#
def on_feedbeck_callback(data):
	#rospy.loginfo('Received feedback; waiting...')
	goal_status = data.status_list[0]
	##print goal_status.status
	if goal_status.status == 1:
		#rospy.loginfo('processing message ...')
		pass
	elif goal_status.status == 3:
		pass
	else:
		print goal_status +'\n'
		sys.exit(0)
"""		

def on_result_callback(data):
	"""
	Evaluates state oc operation reading the status field and diplays a expressive output
	@params: MoveBaseActionResult message
	"""
	state = data.status.status
	if state == 3:
		rospy.loginfo('Destination Reached !!! Accepting new Goal\n')
		x_y = get_input()
		publish_goal(x_y)
	else:
		rospy.loginfo('Something went wrong. Insert new Goal\n')
		x_y = get_input()
		publish_goal(x_y)

def get_input():
	"""
	Reads input and handles wrong input insertion.
	Note: use of input function is discouraged; use raw_input
	"""
	x_str = raw_input('X position value: ')
	y_str = raw_input('Y position value: ')
	try:
		x = int(x_str)
		y = int(y_str)
	except ValueError:
		rospy.loginfo("Wrong input inserted: please retry!!")
		return get_input()
	return (x,y)


def run_node():
	rospy.init_node('pose_publisher', anonymous=True)
	x_y = get_input()
	publish_goal(x_y)
	
	#This line is commented because we need a feedback only when state of action is determined, not every timeslot
	#rospy.Subscriber('/move_base/status', GoalStatusArray, on_feedbeck_callback)

	rospy.Subscriber('/move_base/result', MoveBaseActionResult, on_result_callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		run_node()
	except rospy.ROSInterruptException:
		rospy.loginfo("program interrupted before completion")