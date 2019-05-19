import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from robot_knowledge_base.msg import StringArray
import StringIO

class MongoDBInterface:

	"""
	This class performs all the action with the DB. It can write and read, using the functions offered by the @mongodb_store ros package.

	"""

	def __init__(self):
		"""
		Initialize the class creating a DB connection thanks to a @MessageStoreProxy
		"""
		self._msg_store = MessageStoreProxy()
		#self._all_state_var = [] #saves all state var name for later usage
		rospy.loginfo('The DB node has been Created') 

	def write_robot_sv(self, data, side):
		"""
		Writes passed data on db.

		@params: the ROS message containing the list of strings (JSON like) of configuration
						 the @side to save this configuration: could be robot_side or planner_side
		"""
		try:
			data_id = self._msg_store.insert_named(side, data)
			print ' Wrote with success'
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def read_robot_sv(self, side):
		"""
		Reads data on db.

		@params: The side to read the configuration: could be robot_side or planner_side
		"""
		try:
			print 'reading'
			return self._msg_store.query_named(side, StringArray._type)
			print 'Read with success'
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e


