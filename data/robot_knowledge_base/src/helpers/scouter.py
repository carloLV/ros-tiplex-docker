#!/usr/bin/env python

""" 
This class is the singleton that implements the logic to extract info 
on topics and messages.
It uses a @TopicScouter to search topics.
	@author: Carlo La Viola
 """

import topicScouter as TS

class Scouter:
	
	def __init__(self):
		self.local_couple_list=[]
		self.topic_scouter = TS.TopicScouter()
		self.start_scouting()

	def start_scouting(self):
		"""Puts couple msg-topic in the var @local_couple_list"""
		self._type_topic_map = self.topic_scouter.scout()

	def get_active_msgs_topics(self):
		return self._type_topic_map;