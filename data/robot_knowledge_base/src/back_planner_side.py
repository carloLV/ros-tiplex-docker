#!/usr/bin/env python

from std_msgs.msg import String
from robot_knowledge_base.msg import StringArray
import rospy
import numpy
from rospy.numpy_msg import numpy_msg
import mongodb_interface
import json

class BackPlannerSide:

	"""
	This node is the layer 2 of the planning system.
	Reads values present in mongoDB, publishes those info and waits to receive the modified values and store it in DB.
	@publish: config_plan_side_sender  -> info from mongoDB to the client.
	@subscribe: config_plan_side_receiver -> definitive info to set planner.
	"""

	def __init__(self):
		self._pub = rospy.Publisher('config_plan_side_sender',numpy_msg(StringArray), queue_size=10)
		self._db = mongodb_interface.MongoDBInterface()

	def publish_data2edit(self):
		message = self._db.read_robot_sv("robot_side")
		rate=rospy.Rate(10)
		print 'Publishing... ...'
		while not rospy.is_shutdown():
			self._pub.publish(message[0])
			rate.sleep()

	def on_data_passing(self, data):
		"""
		Callback for the subscribed topic. Receives data from js client and saves the configuration on mongoDB using label @planner_side
			@params: RosMessage of type StringArray
		"""

		#Write to db
		db = mongodb_interface.MongoDBInterface()
		db.write_robot_sv(data, "planner_side") ## Label for reading

	def run_node(self):
		rospy.init_node('back_planner_side',anonymous=True)
		rospy.loginfo('Back planner is up and running')
		rospy.Subscriber('config_plan_side_receiver',numpy_msg(StringArray), self.on_data_passing)
		self.publish_data2edit()
		rospy.spin()

if __name__ == '__main__':
	node = BackPlannerSide()
	try:
		node.run_node()
	except rospy.ROSInterruptException:
		pass