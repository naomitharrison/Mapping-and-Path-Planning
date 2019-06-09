#!/usr/bin/env python
import sys
import rospy
import math
import numpy as np
import map_helper
from nav_msgs.msg import OccupancyGrid, MapMetaData
from nav_msgs.msg import GridCells, Path, Odometry
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.srv import GetMap, GetPlan
import copy
import tf


class Navigate_Map:

    def __init__(self):
        """
            initialize the class Navigate_Map
        """
        rospy.init_node("navigate_map")  # start node
	self.frontier_show = []
	self.first = True
	self.initialS = None
	self.once = False
        self.frontier = []
        self.centroidValue = None
        self.regions = []
        self.detected = False
	self.resolution = 0.05
    	self.start = None
	self.transformed_map = None
    	self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
    	self.sub = rospy.Subscriber("/map", OccupancyGrid, self.handle_navigate_map)
    	self.robot_path_pub = rospy.Publisher('/robot_path', Path, queue_size = 5)
        self.cent_pub = rospy.Publisher('/cent_set', GridCells, queue_size = 5)
	self.front_pub = rospy.Publisher('/frontier_set', GridCells, queue_size = 5)
	self.goal_pos_pub = rospy.Subscriber('/move_base_simple/goal',PoseStamped ,self.goal_call_back)
	self.tf_listener = tf.TransformListener()




    def odom_callback(self, msg):
	rospy.sleep(0.01)
        #self.start=(msg.pose.pose.position.x,msg.pose.pose.position.y)
	(trans,rot) = self.tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
	self.start = (trans[0],trans[1])
    def goal_call_back(self,msg):
	rospy.loginfo("received in goal")
	x = msg.pose.position.x
	y = msg.pose.position.y
	self.goal =(x,y)
	path = self.run_astar_client(self.start, self.goal)
	self.robot_path_pub.publish(path)


    def handle_navigate_map(self, msg):
	print "handle map"
	if self.first and self.start is not None:
		self.initialS = self.start
		self.first = False

	self.map = msg.data
	self.map_width = msg.info.width
	self.map_height = msg.info.height
	self.resolution = msg.info.resolution
	self.x_offset = msg.info.origin.position.x
	self.y_offset = msg.info.origin.position.y
	self.transformed_map = map_helper.convert_gridcells_to_2d(self.map, self.map_width, self.map_height)

    def run_detect(self):
	if(self.transformed_map is not None):
		self.frontier = []
		self.regions = []
		self.detected = False
	    	self.detect_frontier(self.transformed_map)
		if self.detected:
			rospy.loginfo(len(self.frontier))
			if(len(self.frontier)>3):
				self.goal = self.find_goal()
				if(self.goal is not None):
					self.goal = map_helper.world_to_map(self.goal[0], self.goal[1],self.resolution, self.x_offset,self.y_offset)
					path = self.run_astar_client(self.start, self.goal)
					if( path is not None and not len(path.poses) == 0):
						self.robot_path_pub.publish(path)
				else:
					rospy.loginfo("goal is none")
			else:
				if(not self.once):
					path = self.run_astar_client(self.start, self.initialS)
					if(path is not None and not len(path.poses) == 0):
						self.robot_path_pub.publish(path)
					self.once = True
				print("End")
		else:
		    print "no frontier detected"


    def run_astar_client(self,start,goal):
	rospy.wait_for_service('run_astar')
	try:
		run_astar = rospy.ServiceProxy('run_astar', GetPlan)
		p= GetPlan()
		start_pose = PoseStamped()
		goal_pose = PoseStamped()
		start_pose.pose.position.x = start[0]
		start_pose.pose.position.y = start[1]
		goal_pose.pose.position.x = goal[0]
		goal_pose.pose.position.y = goal[1]
		p.start = start_pose
		p.goal = goal_pose
		resp = run_astar(start_pose,goal_pose,0.5)
		return resp.plan
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e


    def detect_frontier(self, map):
        """
            use map helper function that tests to see if unknown node is next to known and unknown
            changes self.frontier to a list of cells (tuples) that is a frontier
        """
        if self.start:
            queue = []
            start = map_helper.map_to_world(self.start[0], self.start[1], self.resolution, self.x_offset, self.y_offset)
            visited = [start]
            queue = [start]
            while queue:
                current = queue.pop(0)
                if map_helper.is_valid_loc_aStar(current, map):
                    neighbors = map_helper.get_neighbors(current, map)
                    for node in set(neighbors) - set(visited):
                        if map_helper.is_frontier(node, map):
                            self.frontier.append(node)
                            queue.append(node)
                            visited += [node]
                        else:
                            queue.append(node)
                            visited += [node]
            self.detected = True
	    self.frontier_show = copy.deepcopy(self.frontier)
            if len(self.frontier) == 0:
                return True
            return False


    def centroid(self, region_list):
        """
            takes a list of cells in frontier region
            adds all frontier region centroids (cx, cy) to self.regions
        """
        centroid_list = [] # a list of [(distance from robot, centroid)]
        robot = map_helper.map_to_world(self.start[0], self.start[1], self.resolution, self.x_offset, self.y_offset)
	#rospy.loginfo(region_list)
        for region in region_list:
            n = len(region)
            i = math.trunc(n/2)
            centroid = region[i]

            x = abs(centroid[0] - robot[0])
            y = abs(centroid[1] - robot[1])
            dist = math.hypot(x, y)
            centroid_list.append((dist, centroid))
        return self.smallest_centroid(centroid_list)


    def smallest_centroid(self, centroid_list):
        smallest = (100.0, 0)
	#rospy.loginfo(centroid_list)
        for centroid in centroid_list:
            if centroid[0] < smallest[0]:
                smallest = centroid
        return smallest[1]

    def in_bounds(self, centroid):
	#rospy.loginfo(centroid)
	if map_helper.is_valid_for_robot(centroid, self.transformed_map, 2):
        #if map_helper.is_valid_for_robot_aStar(centroid, self.transformed_map, 2):
            return centroid
        else:
		x = centroid[0]
		y = centroid[1]
		x_shape = self.transformed_map.shape[0]
		y_shape = self.transformed_map.shape[1]
		queue = [centroid]
		visited = [centroid]
		size = 3
		while queue:
			current = queue.pop(0)
			if map_helper.is_valid_loc_aStar(current, self.transformed_map):
				 neighbors = map_helper.get_neighbors(current, self.transformed_map)
				 for node in set(neighbors) - set(visited):
					if map_helper.is_valid_for_robot_aStar(node, self.transformed_map,2):
					#if map_helper.is_valid_for_robot_aStar(node, self.transformed_map,2):
						return node
					else:
						queue.append(node)
                            			visited += [node]
                rospy.loginfo("Goal not found")
                return None

    def separate_frontier(self):
        """
            separates self.frontier into self.regions
            self.regions becomes dict with trivial key, list of cells (tuples)
        """
	#print self.frontier
        region = []  # a list of tuples
        region_list = [] # a list of regions
        in_list = False
        region_size = 7
        num_regions = 25
        n = 0
        h = 0
        print "separate frontier"
        while(region_size>0):
            for i in range(len(self.frontier)):
                if (h < num_regions):
                    region = []
                    self.find_region(self.frontier[i], region)
		    #rospy.loginfo(region)
                    in_list = region in region_list
                    if (len(region) > region_size) and (not in_list):
                        region_list.append(region)
                        h += 1
            self.regions = region_list
            region_size -= 1
	#print self.regions

    def find_region(self, location, mlist):
	if location in self.frontier:
		mlist.append(location)
		neighbors = map_helper.get_neighbors(location, self.transformed_map)
		for i in range(len(neighbors)):
			 if neighbors[i] not in mlist:
				self.find_region(neighbors[i], mlist)

    def paint_cells(self, centroid, frontier):
	if centroid is not None and centroid!=0:
		goal_msg = GridCells()
		goal_points = []
		goal_p = Point()
		goal_p.x,goal_p.y= map_helper.world_to_map(centroid[0], centroid[1], self.resolution, self.x_offset,self.y_offset)
		goal_points.append(goal_p)
		goal_msg.header.frame_id = "map"

		goal_msg.cell_width = self.resolution
		goal_msg.cell_height = self.resolution
        	goal_msg.cells = goal_points
		self.cent_pub.publish(goal_msg)
        front_msg = GridCells()
        front_points = []
	i = 0
	while i < len(frontier):
            node = frontier[i]
            location = Point()
            location.x,location.y= map_helper.world_to_map(node[0], node[1], self.resolution, self.x_offset,self.y_offset)
            front_points.append(location)
	    i = i+1
        front_msg.header.frame_id = "map"

        front_msg.cell_width = self.resolution
        front_msg.cell_height = self.resolution
        front_msg.cells = front_points
        self.front_pub.publish(front_msg)

    def find_goal(self):
        """
            returns the centroid with highest ranking
            and removes it from possible regions to explore
            where best region is largest rank
        """

        self.separate_frontier()
        goal = self.centroid(self.regions)
	#rospy.loginfo(goal)
        self.centroidValue = goal
        self.paint_cells(self.centroidValue,self.frontier)
	if goal is not None and goal!=0:
        	final = self.in_bounds(goal)
        	rospy.loginfo("Region Found")
        	return final


if __name__ == '__main__':
	n = Navigate_Map()
	#rate = rospy.Rate(1) # 10hz
	#while not rospy.is_shutdown():
		#n_frontier = copy.deepcopy(n.frontier_show)
		#n.paint_cells(n.centroidValue,n_frontier)
		#rate.sleep()
	while 1:
		n.run_detect()

    	rospy.spin()
