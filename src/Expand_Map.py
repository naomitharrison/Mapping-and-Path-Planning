#!/usr/bin/env python

from nav_msgs.msg import OccupancyGrid, MapMetaData
import map_helper as mp
class Expand_Map:

    def __init__(self):
        """
        Use this node to expand the map to ensure that the turtlebot will not enter 
        a space too small for it to enter.
        """
	self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

    def map_callback(self, msg):
        """
            This is a callback for the /map topic
            :param msg: map
            :return: None
        """
	data = my_map
	self.current_map = my_map.data
	self.width = my_map.info.width
	self.height = my_map.info.height
	self.transformed_map = mp.convert_gridcells_to_2d(data,self.width,self.height)
	self.transformed_map = self.expand(self.transformed_map)
	

    def expand(self, map):
	for i in range(self.width):
		for j in range(self.height):
			

        """
            Service call to get map and expand it
            :return:
        """

      

if __name__ == '__main__':

    Expand_Map()
