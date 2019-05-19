#!/usr/bin/env python

from std_msgs.msg import String
#from robot_knowledge_base.msg import StringArray
import rospy
import numpy
from rospy.numpy_msg import numpy_msg
import mongodb_interface
import json
import sys

_write_directory = './java_execution/'

class ExecutionClassCreator:

	def __init__(self):
		self._db = mongodb_interface.MongoDBInterface()
		self._message = None # The message containing all state var
		self._publicators = {} # key: SV, value: tuples (topic, type, ). corresponds to msgs published by planner for each SV
		self._subscriptors = {} # key: SV, value: tuples (topic, type, ). corresponds to msgs subscribed by planner for each SV
		self.build_java_methods()
	
	def build_java_methods(self):
		complex_message = self._db.read_robot_sv("planner_side")
		self._message = complex_message[0].data
		for sv in self._message:
			self.set_pub_sub(sv)
		#print self._publicators, self._subscriptors

	def set_pub_sub(self, state_var):
		sv = json.loads(state_var)		
		if sv in self._publicators.keys():
			print 'Something wrong: State Var already checked in publicators'
			return 
		if sv in self._subscriptors.keys():
			print 'Something wrong: State Var already checked in subscriptors'
			return
		name = sv['stateVar']
		self._publicators[name]=[]
		self._subscriptors[name]=[]
		for l in sv['links']:
			if l['type']=='sub':
				self._subscriptors[name].append((l['topic'],l['message'][:-1]))
			elif l['type']=='pub':
				self._publicators[name].append((l['topic'],l['message'][:-1]))

	def java_class_creator(self, state_var):
		sv = json.loads(state_var)
		name = sv['stateVar']
		complete_class = ('import ros.*;\n'+
			'import com.fasterxml.jackson.databind.JsonNode;\n'+
			'//Add other imports here. You could need more.\n\n'+
			'public class '+name+'{\n'+
			'private RosBridge bridge;\n')
		complete_class+='\tpublic '+name+'(RosBridge bridge){\n\tthis.bridge=bridge;\n\t//insert your structure here\n}\n\n'
		complete_class += self.add_publishers_methods(name)
		complete_class += self.add_subscribers_methods(name)

		return complete_class +'\n}'

	def add_publishers_methods(self,name):
		publishers='\t/*****this is the basic code for publishers. You can modify it as you want*****/\n' #string containing all publishers code
		elements = self._publicators[name]
		i=0 #init counter to create different name publishers
		for tup in elements:
			publishers+=('\tPublisher pub%d = new Publisher("%s", "%s", this.bridge);\n'+
			'\t//Create your message object, uncomment this part and edit as you wish\n'+
			'\t//pub.publish(new YourMessageType<T>(yourMessageContent));\n\n') % (i, tup[0], tup[1])
			i+=1
		return publishers

	def add_subscribers_methods(self,name):
		subscribers='\t/*****this is the basic code for subscribers, including callback.\n**You can modify it as you want*****/\n' #string containing all publishers code
		elements = self._subscriptors[name]
		for tup in elements:
			subscribers += ('\tthis.bridge.subscribe(SubscriptionRequestMsg.generate("%s")\n'+
			'\t'*3+'.setType("%s")\n'+
			'\t'*3+'.setThrottleRate(1)\n'+
			'\t'*3+'.setQueueLength(1),\n'+
			'\t'*2+'new RosListenDelegate() {\n'+
			'\t'*3+'@Override\n'+
			'\t'*3+'public void receive(JsonNode data, String repr){\n'+
			'\t'*4+'//Insert you message object and data here\n'+
			'\t'*4+'MessageUnpacker<YourMessageObj<Type>> unpacker = new MessageUnpacker<YourMessageObj<Type>>(YourMessageObj.class);\n'+
			'\t'*4+'YourMessageObj<Type> msg = unpacker.unpackRosMessage(data);\n'+
			'\t'*4+'//Your code to handle callback goes here. You can check messages an act basing on it\n'+
			'\t'*3+'}\n'+
			'\t'*2+'}\n'+
			');\n\n') % (tup[0],tup[1])

		return subscribers

	def run_node(self):
		rospy.init_node('java_classes_initiator',anonymous=True)
		all_state_var = "" # all state var to be written on file
		for st_v in self._message:
			sv = json.loads(st_v)
			name = sv['stateVar']
			my_class=self.java_class_creator(st_v)
			try:
				current_java_class = open(_write_directory+name+'.java', 'w')
				current_java_class.write(my_class)
				rospy.loginfo('Wrote on file with SUCCESS, Exiting ...')
			except IOError:
				rospy.loginfo('Unable to open or write in ddl file.')
		#sys.exit(0)
		rospy.spin()

if __name__ == '__main__':
	node = ExecutionClassCreator()
	try:
		node.run_node()
	except rospy.ROSInterruptException:
		pass

