import feedback_handler_abc as fh
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import rospy
import roslib

class TestConnectivity(fh.FeedbackHandler):
	""" This testing class refers to the turtlebot running in Gazebo. First test is conducted on the topic
		@/cmd_vel_mux/input/teleop and msg type @geometry_msgs/Twist;
		listening for the feedback on topic @/cmd_vel_mux/active with msg type @String
	"""
	def __init__(self,json_string):
		fh.FeedbackHandler.__init__(self, json_string)
		self._pub = rospy.Publisher(self._topic, Twist, queue_size=10)

	def send_message(self):
		msg = Twist()
		msg.linear.x = 5
		msg.linear.y = 0
		msg.linear.z = 0
		msg.angular.x = 0
		msg.angular.y = 0
		msg.angular.z = 0

		self._pub.publish(msg)

	def get_feedback(self):
		for el in self._states_map:
			if el['id'] == 'Moving': # The state we want to get feedback
				rospy.Subscriber(el['feedback'], String, self.callback)

	def callback(self, data):
		if data.data=='Teleoperation': ##parola per movimento
			print 'I am moving'
		if data.data=='idle': ##msg per sta fermo
			print 'I am arrived, send me another message...'
"""
class TestPhoto(fh.FeedbackHandler)
This testing class refers to the turtlebot running in Gazebo. This test is conducted on the topic
		@/camera/rgb/image_raw and msg type @sensor_msgs/Image;
		listening for the feedback??? 
	def __init__(self,json_string):
		fh.FeedbackHandler.__init__(self, json_string)
		self._pub = rospy.Publisher(self._topic, Image, queue_size=10) #Will publish on /camera/rgb/raw
		self._bridge = CvBridge()

	def send_message(self):


	def get_feedback(self):
		for el in self._states_map:
			if el['id'] == 'Take': # The state we want to get feedback
				rospy.Subscriber(el['feedback'], String, self.callback)

	def callback(self, data):
		try:
			cv_image = self._bridge.imgmsg_to_cv2(data, 'bgr8')
		except CvBridgeError as e:
			print e

		(rows,cols,channels) = cv_image.shape
		if cols > 60 and rows > 60:
			cv2.circle(cv_image, (50,50), 10, 255)

		cv2.imshow('Image window', cv_image)
		cv2.waitKey(3)

		try:
			self._pub.publish(self._bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
		except CvBridgeError as e:
			print e
"""