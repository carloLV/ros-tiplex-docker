#!/usr/bin/env python

""" 
This class extracts all possible couples topic-messages. Filters topics
known to be useless thanks to a @TopicFilter and returns remaining topics.
	@author: Carlo La Viola
 """

import rospy
import itertools
import operator
import topicFilter as TF
from subprocess import check_output

class TopicScouter:
	def __init__(self):
		self.topic_filter=TF.TopicFilter()

	def scout(self):
		"""
		This function searche the active topics, then filters them according to a @topicFilter. Eventually it groups them by type and return the map

		@return A map of topics grouped by type: {type1:[topic1,topicJ], ..., typeN[topic5,topicK]}
		"""
		#topics = rospy.get_published_topics()
		topics_msgs = []
		topics_str = check_output(['rostopic','list'])
		topics = topics_str.split('\n')
		topics.pop() #last element is empty so remove it
		filtered_topics = self.topic_filter.filter_topics(topics)
		for el in filtered_topics:
			msg_type = check_output(['rostopic','type','%s'%el])
			topics_msgs.append([el,msg_type])

		return topics_msgs