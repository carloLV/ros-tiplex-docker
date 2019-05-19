#Testing purposes
import test_connectivity as tc
# Available connections classes
import planner_connections as pc

class PlannerConnectionsFactory:
	"""
	This class follows the Factory Pattern. Use this factory to get your desired connection object.
	Each class in @planner_connections.py needs to be initialized in dict @__connections_classes. 
	Then you can call that class using the static method @get_connection_object.
	"""
	__connections_classes = {
		'testing': tc.TestConnectivity
	}

	@staticmethod
	def get_connection_object(name, json_string):
		"""
		Returns the class linked with @name in the dict @__connections_classes.
			@params: String
			@output: Class instance
		"""
		connection_class = PlannerConnectionsFactory.__connections_classes.get(name.lower(), None)
		if connection_class:
			return connection_class(json_string)
		raise NotImplementedError('You requested a not implemented class')