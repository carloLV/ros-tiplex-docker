#!/usr/bin/env python

""" 
This class filters topics based on a custom logic. You can reimplement the
logic basing on your needs
	@author: Carlo La Viola
 """

#Fill this list with substring of useless topics
KNOWN_TO_BE_USELESS=['diagnostic','gazebo','rosout','imu','navsat','clock','/tf','costmap','/twist']

class TopicFilter:
 	
 	def filter_topics(self, topics):
 		"""
 		This function filters topics based on known string contained by useless topics. The list of useless topics should be filled by hand from who knows the robot

 		@params: List of topic names: [[topic1]...[topicN]] 
 		@output: List of filtered topic names: [[topic1]...[topicN]]
 		"""
 		filtered_topics=[]

 		for t in topics:
 			if not any([sub in t for sub in KNOWN_TO_BE_USELESS]):
 				filtered_topics.append(t)
 		return filtered_topics
