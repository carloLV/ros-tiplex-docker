from abc import ABCMeta, abstractmethod#Abstract Base Classes module
import json
import rosmsg

""" This abstract class implements some method and the topology for classes used for communicating
    with the planner. It can send messages o a known type and listen on some topics to handle feedbacks and
    state changing.
"""

class FeedbackHandler:
	__metaclass__ = ABCMeta

	def __init__(self, json_string):
		""" Initialize the handler with info retrieved by the js client, to
		configure planner.
		@params: the json retrieved from roslibjs in string format
		"""
		json_obj = json.loads(json_string)
		self._topic = json_obj['topic']
		self._message = json_obj['message']
		self._states_map = json_obj['states']
		self._links = json_obj['links']

	@abstractmethod
	def send_message(self):
		"""
		Sends message on actual topic. Msg must be in the correct form.
		Use @self.get_msg_info() for getting info.
		"""
		raise NotImplementedError()

	@abstractmethod
	def get_feedback(self):
		"""Contains the subscribers to handle the feedbacks from the robot.
		Each feedback needs its callback function to be handled.
		"""
		raise NotImplementedError()

	def print_my_data(self):
		"""
		This function has debug purposes: prints information on the actual class
		"""
		print 'I write on: '+self._topic
		print 'Msg type '+self._message
		print 'Links: ',self._links
		print	'States info: ',self._states_map

	def get_msg_info(self):
		"""
		@output: The structure defined in file .msg of the actual message.
		"""
		return rosmsg.get_msg_text(self._message)
