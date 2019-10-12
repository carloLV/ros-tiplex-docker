#!/usr/bin/env python
import rospy
import sys
from sensor_msgs.msg import Image
from std_msgs.msg import Int32 ## 0 = success; 1 = failure
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError

class PhotoHandler:

	def __init__(self):
		self.feedback_pub = rospy.Publisher('camera/photo/feedback', Int32, queue_size=10)
		self.bridge = CvBridge() #the bridge from ros Image to openCV image
		self.want_image = False
		self.taken_image = None
		self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.on_image_callback) # To get the image from camera
		self.saving_pub = rospy.Subscriber('camera/photo/saving', Int32, self.on_saving_callback) #If it has to do and save the photo
		self.filter_pub = rospy.Subscriber('camera/photo/filtering', Int32, self.on_filtering_callback) #If needs to filter
		rospy.sleep(1)
	
	def on_image_callback(self, data):
		"""
		If the user requests the image, then we save it in istance var for later use
		"""
		if self.want_image:
			rospy.loginfo('User wants the image')
			try:
				self.taken_image = self.bridge.imgmsg_to_cv2(data,'bgr8')
			except CvBridgeError as e:
				print e
			self.want_image = False

	def on_saving_callback(self, data):
		"""
		If user send message for saving Image (just a Int32 msg), then we save it on file
		"""
		if data.data == 3: ##the value for saving
			self.want_image = True
			rospy.sleep(1)
			self.save_image(self.taken_image)
		else:
			rospy.loginfo('You don t want to save any image') 

	def on_filtering_callback(self, data):
		"""
		If user send message for filter Image (just a Int32 msg), we filter it
		"""
		if data.data == 3:
			self.filter_image()
		else:
			rospy.loginfo('You don t want to filter any image') 

	def filter_image(self):
		"""
		First reads the image from the file in which we saved it; then filters it in according to some logic 
		"""
		#TODO for now simply retrieves image
		msg = Int32()
		msg.data=0
		self.feedback_pub.publish(msg)

	def save_image(self, image):
		msg = Int32()
		try:
			self.save_to_file('image.txt',image)
			msg.data=0
			self.feedback_pub.publish(msg)
		except Exception:
			msg.data=1
			feedback_pub.publish(msg)

	def save_to_file(self, path, data):
		home_dir = os.path.expanduser('~')
		desktop_dir = os.path.join(home_dir, 'Desktop')
		with open(os.path.join(desktop_dir, path),'w') as f:
			f.write(str(data))

#this class not needs a run_node method
if __name__ == '__main__':
	rospy.init_node('photo_handler', anonymous=False)
	photo = PhotoHandler()
	rospy.loginfo('Node is up and running ...')
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("Shutting down ROS Image Photo Handler")
	except KeyboardInterrupt:
		print "Shutting down ROS Image Photo Handler"