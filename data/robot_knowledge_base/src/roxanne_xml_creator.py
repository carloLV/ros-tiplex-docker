#!/usr/bin/env python

import rospy
import mongodb_interface
import json
import sys

#filename should be passed or default will be used
xmlname = rospy.get_param('/roxanne_xml_creator/xmlname')
INSERT_CODE = 'TODO enter your code here'
STANDARD_STRING_MESSAGE = 'std_msgs/String'
STANDARD_STRING_PUBLISHER = 'it.cnr.istc.pst.ros.roxanne.platform.rosbridge.std.StdStringCommadnFeedbackListener'

# For the command field
OPENING = '<?xml version="1.0" encoding="UTF-8"?>\n\n<ros host="ws://192.168.1.7:9090"> <!-- Default host ROSbridge -->\n'
COMMAND = '<command name="%(svName)s" component="%(state)s">\n%(dispatch)s\n%(feedback)s\n</command>\n'
                                #usage: COMMAND % {"svName": "theName", "state": "theState", "dispatch": DISPATCH_TOPIC, "feedback": FEEDBACK_TOPIC}

#Fields inside command
DISPATCH_TOPIC = '<dispatch-topic\n\tname="%(dispatchName)s"\n\tmsg="%(dispatchMsg)s"\n\tpublisher="%(publisher)s" />'
                    #usage: DISPATCH_TOPIC % {"dispatchName": ""/camera/photo/saving", "dispatchMsg": "/camera/photo/filtering", "publisher" = STANDARD_STRING_PUBLISHER}
FEEDBACK_TOPIC = '<feedback-topic\n\tname="%(feedback)s"\n\tmsg="%(feedbackMsg)s"\n\tdelegate="%(delegate)s" />'
                    #usage: FEEDBACK_TOPIC % {"feedback": ""/camera/photo/saving", "feedbackMsg": "/camera/photo/filtering", "delegate" = STANDARD_STRING_PUBLISHER}

def select_link_for(stateName, links, field):
    """
    Gets the link object that satisfies the stateName for that field
    @params str -> state name
            list -> contains all the links
            str -> the field in the link to use to match the name
    @return: dict -> json obj representing the link
    """
    for l in links:
        if l[field] == stateName:
            return l

def create_command_with_states(svName, stateName, l, f):
    """
    Creates the single command statement with everything inside
    @param: str -> name of SV
            str -> name of state
            dict -> the json object representing the link that is used for the linking
            dict -> the json object representing the feedback
    @return str -> will contain the final commad statement
    """
    which_class_p = ""
    which_class_f = ""
    if l['message'] == STANDARD_STRING_MESSAGE:
        which_class_p = STANDARD_STRING_PUBLISHER
    else:
        which_class_p = INSERT_CODE
    dispatch = DISPATCH_TOPIC % {"dispatchName": l['topic'],
                                "dispatchMsg": l['message'],
                                "publisher" : which_class_p}

    if f['message'] == STANDARD_STRING_MESSAGE:
        which_class_f = STANDARD_STRING_PUBLISHER
    else:
        which_class_f = INSERT_CODE
    feedback = FEEDBACK_TOPIC % {"feedback": f['topic'],
                                "feedbackMsg": f['message'],
                                "delegate" : which_class_f}
    return COMMAND % {"svName": svName,
                    "state": stateName,
                    "dispatch": dispatch,
                    "feedback": feedback}

class RoxanneXmlCreator:
    """
	This node is the layer 3 of the planning system.
	Reads value from mongoDB using the label @planner_side. The extracted info are relative to the State Variables
	the planning expert has reviewed and modified to better suit the EPSL paradigm.
	Then the info are used to create a @xml file that will be used to configure the Roxanne module.
	"""
    def __init__(self):
        self.svs = self.get_state_var()

    def get_state_var(self):
        '''Reads data from JSON file and extracts the state variables only'''
        state_vars = []
        db = mongodb_interface.MongoDBInterface()
        message = db.read_robot_sv("planner_side")
        for sv in message:
            state_vars.append(json.loads(sv))
        return state_vars

    def create_command_field(self, sv):
        """
        Creates field command for the xml
        @param: a json obj representing a state var
        @return: the string representing the command to be added
        """
        command = ""
        states = sv['states']
        links = sv['links']
        name = sv['stateVar']

        for s in states:
            if not s['is_initial']:
                publish = select_link_for(s['id'], links, 'target')
                feedback = select_link_for(s['id'], links, 'source')
                command += create_command_with_states(name, s['id'], publish, feedback)

        # print states
        # print links
        # print name
        return command


    def run_node(self):
        rospy.init_node('xml_roxanne_creator',anonymous=True)
        try:
            xml_file = open('./'+xmlname+'.xml', 'a')
            xml_file.write(OPENING)
            for sv in self.svs:
                xml_file.write(self.create_command_field(sv))
                xml_file.write('</ros>')
            rospy.loginfo('XML has been written on disk with SUCCESS, Exiting ...')
            sys.exit(0)
        except IOError:
            rospy.loginfo('Unable to open or write in ddl file.')

if __name__ == '__main__':
	node = RoxanneXmlCreator()
	try:
		node.run_node()
	except rospy.ROSInterruptException:
		pass
