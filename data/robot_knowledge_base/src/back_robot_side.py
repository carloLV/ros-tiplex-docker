#!/usr/bin/env python

from helpers import scouter
from std_msgs.msg import String
from robot_knowledge_base.msg import StringArray
import rospy
import numpy
from rospy.numpy_msg import numpy_msg
import mongodb_interface

class BackRobotSide:

	"""
	This python node is the entry point to set the basics knowledge of the robot. It uses the @Scouter to get lists of msgs and topics, sends them to the html page, then gets back the messages containing the state variables
	@publish: config_robot_side_sender  -> info to client.
	@subscribe: config_robot_side_receiver -> retrieved info to set planner.
	"""

	def __init__(self):
		print 'Scouting system for ROS topics and types...'
		print 'The more you decide to filter, less time the scouting will take.'
		self._scouter=scouter.Scouter()
		self._pub = rospy.Publisher('config_robot_side_sender',numpy_msg(StringArray), queue_size=10)

	def publish(self):
		msgs_topics_list=self._scouter.get_active_msgs_topics()
		rate=rospy.Rate(10)
		stringArr = self.build_string(msgs_topics_list)
		while not rospy.is_shutdown():
			to_publish=numpy.array(stringArr, dtype=numpy.str)
			self._pub.publish(to_publish)
			rate.sleep()

	def on_data_passing(self, data):
		"""
		Callback for the subscribed topic. Receives data from js client and saves the configuration on mongoDB using label @robot_side
			@params: RosMessage of type StringArray
		"""
		#Write to db
		db = mongodb_interface.MongoDBInterface()
		db.write_robot_sv(data, "robot_side") ## Label for reading

	def build_string(self, msg_topic_list):
		"""
		Creates the array of String to be sent to index.js that will be read using ROSlib.js
		@params: the list of list [ [topic 1, type 1], ... ]
		@output: [ ['topic 1-type 1'] ]
		"""
		return_list=[]
		for topic, t_type in msg_topic_list:
			temp_string=topic+'-'+t_type
			return_list.append(temp_string)
		return return_list

	def run_node(self):
		rospy.init_node('kb_admin',anonymous=False)
		rospy.loginfo('Back robot is up and running')
		rospy.Subscriber('config_robot_side_receiver',numpy_msg(StringArray), self.on_data_passing)
		self.publish()
		rospy.spin()
		
if __name__ == '__main__':
	node = BackRobotSide()
	try:
		node.run_node()
	except rospy.ROSInterruptException:
		pass