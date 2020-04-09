import io, json, os, errno
import numpy as np

class MongoDBInterface:

	"""
	This class performs all the action with the DB. It can write and read, using the functions offered by the @mongodb_store ros package.

	"""

	def __init__(self):
		"""
		Initialize the class creating a folder where to write to
		"""
		self._curr_dir = os.getcwd()
		self._folder = "json_data/"
		try:
			os.makedirs(os.path.join(self._curr_dir, self._folder))
		except OSError as e:
			if e.errno != errno.EEXIST:
				raise

	def write_robot_sv(self, data, side):
		"""
		Writes JSON data on a file.
		The data are on the shape of a numpy StringArray object

		@params: the ROS message containing the list of strings (JSON like) of configuration
						 the @side to save this configuration: could be robot_side or planner_side
		"""
		file_path = self._folder + side + ".txt"
		with io.open(os.path.join(self._curr_dir, file_path), 'w', encoding='utf-8') as f:
			for ob in data:
				f.write(unicode(ob + "\n"))
		print "Data can be accessed at this path:\n " + os.path.join(self._curr_dir, file_path)

	def read_robot_sv(self, side):
		"""
		Reads data from file.

		@params: The side to read the configuration: could be robot_side or planner_side
		@return: the numpy StringArray objects contained in the file
		"""
		file_path = self._folder + side + ".txt"
		info = []
		with io.open(os.path.join(self._curr_dir, file_path), 'r', encoding='utf-8') as f:
			 for line in f:
				 info.append(str(line.strip()))
		return np.array(info)
