import io, json, os

class MongoDBInterface:

	"""
	This class performs all the action with the DB. It can write and read, using the functions offered by the @mongodb_store ros package.

	"""

	def __init__(self):
		"""
		Initialize the class creating a DB connection thanks to a @MessageStoreProxy
		"""
		self._curr_dir = os.getcwd()

	def write_robot_sv(self, data, side):
		"""
		Writes JSON data on a file.

		@params: the ROS message containing the list of strings (JSON like) of configuration
						 the @side to save this configuration: could be robot_side or planner_side
		"""
		file_path = "json_data/" + side + ".txt"
		with io.open(os.path.join(self._curr_dir, file_path), 'w', encoding='utf-8') as f:
			info = json.dumps(data, ensure_ascii=False)
			f.write(unicode(info))

	def read_robot_sv(self, side):
		"""
		Reads data from file.

		@params: The side to read the configuration: could be robot_side or planner_side
		@return: the JSON objects contained in the file
		"""
		file_path = "json_data/" + side + ".txt"
		with io.open(os.path.join(self._curr_dir, file_path), 'r', encoding='utf-8') as f:
			info = json.loads(f.read())
		return info
