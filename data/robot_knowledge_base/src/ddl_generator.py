#!/usr/bin/env python
from std_msgs.msg import String
from robot_knowledge_base.msg import StringArray
import rospy
import numpy
from rospy.numpy_msg import numpy_msg
import mongodb_interface
import json
import sys

#filename should be passed by command line
if len(sys.argv) != 3:
	print 'Usage: rosrun robot_knowledge_base ddl_generator.py your_file_name your_time_horizon'
	sys.exit(0)
else: 
	filename = sys.argv[1]
	horizon = sys.argv[2]

class DDLGenerator:
	"""
	This node is the layer 3 of the planning system.
	Reads value from mongoDB using the label @planner_side. The extracted info are relative to the State Variables
	the planning expert has reviewd and modified to better suit the EPSL paradigm.
	Then the info are used to create a @ddl file that will be used to configure the domain.
	"""

	def __init__(self):
		self._db = mongodb_interface.MongoDBInterface()
		self._syncronization = []
		self._timelines = []

	def read_data_from_DB(self):
		"""
		Reads data using the mongoDBInterface and return only the portion strictly needed for operations.
		"""
		message = self._db.read_robot_sv("planner_side")
		return message[0].data

	def append_state_var2file(self, state_var):
		"""
		This function gets the state variables prepared from the GUI and builds a string to describe and add them to a file
		@params: a JSON object representing a state variable
		@output: a string containing all the ddl content to append
		"""
		#print state_var
		states = state_var['states']
		links = state_var['links']
		name = state_var['stateVar']
		try:
			syncRules = state_var['syncRule']
			for el in syncRules:
				self._syncronization.append(el)
				self._timelines.append(name)
			#print syncRules
		except AttributeError:
			print 'No Synchronization rule'
		all_states=''
		states_description =[] #This list contains all the state structure to build SV description

		#Note: we save all uncontrollable states to put '_'before the state name
		uncontrollable = []
		for s in states:
			if s['controlTag'] == 'u':
				uncontrollable.append(s['id'])

		for s in states:
			if s['controlTag'] == 'u':
				s_id = '_'+s['id']
			else:
				s_id = s['id']
			all_states += s_id+'(), '
			states_description.append(DDLStateVarStructure(s,links,uncontrollable))
		#all_states = all_states[:-2] is to remove the last space and comma

		## Declaration of State Variable in ddl file
		state_var_declaration = 'COMP_TYPE SingletonStateVariable '+name+' ('+all_states[:-2]+')'

		## appending iteratively all info for each state
		description ='{\n'
		for el in states_description:
			description +='\tVALUE '+el.get_value()+'() '+el.get_durability()+'\n'
			description +='\tMEETS {\n'
			for m in el.get_meets():
				description += '\t\t'+m+'\n'
			description +='\t}\n' 
		return state_var_declaration+description+'\n}\n'	

	def append_sync_rules(self):
		dictionary = {} #key: State Variable, values: all sync rule associated
		all_rules = '' #empty string to return all sync rules
		for sync in self._syncronization:
			elements = sync.split(' ')
			main_sv = elements[0]
			if main_sv in dictionary.keys():
				dictionary[main_sv].append(sync)
			else:
				dictionary[main_sv] = [sync]
		#iterating on dictionary to add all sync rule for each state var
		for key in dictionary.keys():
			all_rules+='SYNCHRONIZE '+key+'\n{'
			for sync in dictionary[key]:
				elements = sync.split(' ')
				main_sv = elements[0]
				main_state = elements[2]
				secondary_sv = elements[4]
				secondary_state = elements[6]
				rule_type = elements[3]
				all_rules+='\n\tVALUE '+main_state+'()\n\t{\n\tcd0 '+secondary_sv+'.'+secondary_state+'();\n\n\t'+rule_type+'[0?,+INF?];\n\t}\n'
			all_rules+='\n}'
		return all_rules+'\n}' 

	def append_timeline(self):
		all_timeline=''
		for t in self._timelines:
			timeline_name = t[:2]+str(len(t))
			all_timeline+='\t\t COMPONENT '+t+' {FLEXIBLE '+timeline_name+'(primitive)}: '+t+'Type;\n'
		return all_timeline

	def run_node(self):
		rospy.init_node('ddl_generator',anonymous=True)
		message = self.read_data_from_DB()
		all_state_var = "" # all state var to be written on file
		for sv in message:
			all_state_var += self.append_state_var2file(json.loads(sv))
		all_sync_rules = self.append_sync_rules()
		all_timeline = self.append_timeline()
		try:
			self._domain_file = open('./'+filename+'.ddl', 'a')
			self._domain_file.write('DOMAIN '+filename.upper()+'_Domain\n{\n\tTEMPORAL_MODULE temporal_module=[0,'+horizon+'],'+horizon+';\n')
			self._domain_file.write(all_state_var)
			self._domain_file.write(all_timeline)
			self._domain_file.write(all_sync_rules)
			#self._domain_file.write('\n\t}\n}')
			rospy.loginfo('Wrote on file with SUCCESS, Exiting ...')
			sys.exit(0)
		except IOError:
			rospy.loginfo('Unable to open or write in ddl file.')
		rospy.spin()


class DDLStateVarStructure:
	"""
	This class contains the object to store all data in a convenient way to perform writes of .ddl file
	"""
	def __init__(self,state,links,uncontrollable):
		self._value = self.set_value(state)
		self._durability = self.set_durability(state['durability'])
		self._meets = self.set_meets(state,links,uncontrollable)

	def set_durability(self, string_dur):
		"""
		@input: the string of durability in format of type '4-7'
		@output: the string formatted for ddl language
		"""
		bounds = string_dur.split('-')
		start = bounds[0]
		end = bounds[1]
		return '['+start+','+end+']'

	def set_value(self, state):
		"""
		@input: the object representing the state
		@output: the correct name: the same ID if controlTag is 'c'; _ID if controlTag is 'u'
		"""
		if state['controlTag'] == 'c':
			return state['id']
		else:
			return '_'+state['id']

	def set_meets(self, s, links, uncontrollable):
		"""
		@input: the object representing the links
		@output: a String list with all the states name connected to the current one
		"""
		meets = []
		s_id=s['id']
		for l in links:
			if l['source'] == s_id:
				if l['target'] in uncontrollable:
					meets.append('_'+l['target']+'();')
				else:
					meets.append(l['target']+'();')
		return meets

	def get_value(self):
		return self._value
	def get_durability(self):
		return self._durability
	def get_meets(self):
		return self._meets
	def in_printing(self):
		print self._value
		print self._durability
		print self._meets

if __name__ == '__main__':
	node = DDLGenerator()
	try:
		node.run_node()
	except rospy.ROSInterruptException:
		pass