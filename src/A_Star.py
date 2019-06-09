#!/usr/bin/env python
import rospy
from PriorityQueue import PriorityQueue
from map_helper import *
import math
import numpy
from nav_msgs.msg import GridCells, Path
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.srv import GetMap, GetPlan
from nav_msgs.msg import OccupancyGrid, Odometry
import map_helper
import copy
import tf


class A_Star:

    def __init__(self):
        """
            This node handle A star paths requests.
            It is accessed using a service call. It can the publish grid cells
            to show the frontier,closed and path.
        """
	## Sets the initial values
        rospy.init_node("a_star")  # start node
	self.goal = None
	self.start = None
	self.path = None
        self.frontier = PriorityQueue()
        self.came_from = {}
        self.cost_so_far = {}
	self.s = rospy.Service('run_astar', GetPlan, self.handle_a_star)
	## Creates the publisher for showing on rviz
        self.closed_pub = rospy.Publisher('/close_set', GridCells, queue_size = 5)
        #self.front_pub = rospy.Publisher('/frontier_set', GridCells, queue_size = 5)
        self.goal_pub = rospy.Publisher('/path_set', GridCells, queue_size = 5)
        self.path_pub = rospy.Publisher('/path', Path, queue_size = 5)
    	self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.tf_listener = tf.TransformListener()
        self.resolution = 0.05

    def handle_a_star(self, req):

        """
            service call that uses A* to create a path.
            This can be altered to also grab the orentation for better heuristic
            :param req: GetPlan
            :return: Path()
        """
	rospy.loginfo("Handles a star")
	self.dynamic_map_client()
	start = (req.start.pose.position.x,req.start.pose.position.y)
	goal = (req.goal.pose.position.x,req.goal.pose.position.y)
	#start = (start.pose.position.x,start.pose.position.y)
	#goal = (goal.pose.position.x,goal.pose.position.y)
	#rospy.loginfo(start)

	self.start = map_helper.map_to_world(self.start[0],self.start[1], self.resolution, self.x_offset, self.y_offset)
	self.goal = map_helper.map_to_world(goal[0],goal[1], self.resolution, self.x_offset, self.y_offset)
	#rospy.loginfo(start)
	#rospy.loginfo(goal)
	path = self.a_star(self.start,self.goal)
	## This converts the tuple list to Path.msg
	p = Path()
	i = None
	points = []
	if path is not None:
		for i in range(len(path)):
			node = path[i]
			#rospy.loginfo(node)
			location = PoseStamped()
			location.pose.position.x,location.pose.position.y= map_helper.world_to_map(node[0], node[1],self.resolution, self.x_offset,self.y_offset)
			location.pose.position.x = location.pose.position.x
			location.pose.position.y = location.pose.position.y
			points.append(location)
			i += 1
		p.poses = points
	return p

    def odom_callback(self, msg):
        #self.start=(msg.pose.pose.position.x,msg.pose.pose.position.y)
        try:
            (trans,rot) = self.tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            self.start = (trans[0],trans[1])
        except Exception as e:
            print "sad"




    def dynamic_map_client(self):

        """
            Service call to get map and set class variables
            This can be changed to call the expanded map
            :return:
        """
	## This function grabs the map from /static_map service and sets the values in the object
	rospy.wait_for_service('/dynamic_map')
    	try:
		get_map = rospy.ServiceProxy('/dynamic_map', GetMap)
		resp1 = get_map()
		self.map = resp1.map.data
		self.map_width = resp1.map.info.width
		self.map_height = resp1.map.info.height
		self.resolution = resp1.map.info.resolution
		self.yawn = resp1.map.info.origin.orientation.w
		self.x_offset = resp1.map.info.origin.position.x
		self.y_offset = resp1.map.info.origin.position.y
		self.map = map_helper.convert_gridcells_to_2d(self.map, self.map_width, self.map_height)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

        pass
    def a_way_to_get_out(self, start):
		x = start[0]
		y = start[1]
		x_shape = self.map.shape[0]
		y_shape = self.map.shape[1]
		queue = [start]
		visited = [start]
		while queue:
			current = queue.pop(0)
			neighbors = map_helper.get_neighbors_way_out(current, self.map)
			for node in set(neighbors) - set(visited):
				if map_helper.is_valid_for_robot_aStar(node, self.map,2):
						x = node[0] - start[0]
						y = node[1] - start[1]
						if (math.hypot(x, y) >=2):
							return node
				else:
					queue.append(node)
                    			visited += [node]
		return None


    def a_star(self, start, goal):

	#start = map_helper.map_to_world(self.start[0],self.start[1], self.resolution, self.x_offset, self.y_offset)
        """
            A*
            This is where the A* algorithum belongs
            :param start: tuple of start pose
            :param goal: tuple of goal pose
            :return: dict of tuples
        """
	## This is the logic part of the a star
	#self.start = start
	#self.goal = goal
	if(not map_helper.is_valid_for_robot(goal, self.map, 2)):
		rospy.loginfo("not valid")
		return None
	if( not map_helper.is_valid_for_robot(start, self.map, 2)):
		rospy.loginfo("stucked")
		goal = self.a_way_to_get_out(start)
		if(goal is None):
			rospy.loginfo("can not go out")
		else:
			self.came_from[goal] = self.start
			self.path = self.reconstruct_path( self.start, goal, self.came_from)

	else:
		rospy.loginfo("forms astar")
		self.frontier = PriorityQueue()
		self.came_from = {}
		self.cost_so_far = {}
		self.frontier.put(start, 0)
		self.came_from[start] = None
		self.cost_so_far[start] = 0
		while not self.frontier.empty():
		    current = self.frontier.get()
		    if current == goal:
		        break

		    for next in map_helper.get_neighbors_aStar(current, self.map):
		        new_cost = self.cost_so_far[current] + self.move_cost(current, next)
		        #new_cost = self.cost_so_far[current] + 1
		        if next not in self.cost_so_far or new_cost < self.cost_so_far[next]:
		            self.cost_so_far[next] = new_cost
		            priority = new_cost + self.euclidean_heuristic(goal, next)
		            self.frontier.put(next, priority)
		            self.came_from[next] = current
		    #rospy.sleep(0.01)
		    #if(len(self.frontier)>200):
			#break

		## Reconstruct path here
		self.path = self.reconstruct_path(start, goal, self.came_from)
        return self.path

	## The heuristic function for a star
    def euclidean_heuristic(self, point1, point2):
        """
            calculate the dist between two points
            :param point1: tuple of location
            :param point2: tuple of location
            :return: dist between two points
        """
        x = point2[0] - point1[0]
        y = point2[1] - point1[1]
        hyp = math.hypot(x, y)
        return hyp

	## calculates the Manhattan distance
    def move_cost(self, current, Next):
        """
              calculate the dist between two points
              :param current: tuple of location
              :param Next: tuple of location
              :return: dist between two points
        """
        cost = 0
        x = math.fabs(Next[0] - current[0])
        y = math.fabs(Next[1] - current[1])

	came_node = self.came_from[current]

        if (x == 0) and (y > 0):
            cost = 1
        if (x > 0) and (y == 0):
            cost = 1
	if(came_node is not None):
		y_diff = math.fabs(Next[1]- came_node[1])
		x_diff = math.fabs(Next[0]- came_node[0])
		if (x_diff > 0) and (y_diff > 0):
		    cost = 10
        return cost

	## Reconstruct the path from came_from list
    def reconstruct_path(self, start, goal, came_from):
        """
            Rebuild the path from a dictionary
            :param start: starting key
            :param goal: starting value
            :param came_from: dictionary of tuples
            :return: list of tuples
        """
        path = []
        node = goal
        path.append(node)
        while not node == start:
	    if node not in came_from:
		return None
            node = came_from[node]

            path.insert(0, node)
        if len(path) < 3:
            return path
        else:
            return self.optimize_path(path)


    def optimize_path(self, path):
        """
            remove redundant points in the path
            :param path: list of tuples
            :return: reduced list of tuples
        """
        organized = []
        path.append((None, None))
        previous = path[0]
        current = path[1]
        next_node = path[2]
        organized.append(previous)
        i = 0
        for i in range(len(path)-2):
            if (next_node[0] == previous[0]) and (next_node[0] == current[0]):
                current = next_node
                next_node = path[2+i]
            elif (next_node[1] == previous[1]) and (next_node[1] == current[1]):
                current = next_node
                next_node = path[2+i]
            else:
                organized.append(current)
                previous = current
                current = next_node
                next_node = path[2+i]
        organized.append(current)
        return organized

	## This paints the cells by creating 3 GridCells representing expanded nodes, frontier nodes and goal node
    def paint_cells(self, frontier, came_from):
        # type: (list, list) -> None
        """
            published cell of A* to Rviz
            :param frontier: tuples of the point on the frontier set
            :param came_from: tuples of the point on the closed set
            :return:
        """
        front_msg = GridCells()
        closed_msg = GridCells()
        goal_msg = GridCells()
        front_points = []
        closed_points = []
	goal_points = []

        while not frontier.empty():
            node = frontier.get()
            location = Point()
            location.x,location.y= map_helper.world_to_map(node[0], node[1],self.resolution, self.x_offset,self.y_offset)
            front_points.append(location)

	if came_from:
		for key, value in came_from.items():
		    node = came_from[key]
		    if node is not None:
			    location = Point()
            		    location.x,location.y= map_helper.world_to_map(node[0], node[1],self.resolution, self.x_offset,self.y_offset)
			    closed_points.append(location)
	if self.goal is not None:
		goal_p = Point()
		goal_p.x,goal_p.y= map_helper.world_to_map(self.goal[0], self.goal[1],self.resolution, self.x_offset,self.y_offset)
		goal_points.append(goal_p)
		goal_msg.header.frame_id = "map"

		goal_msg.cell_width = self.resolution
		goal_msg.cell_height = self.resolution
        	goal_msg.cells = goal_points
		self.goal_pub.publish(goal_msg)

        front_msg.header.frame_id = "map"
        closed_msg.header.frame_id = "map"

        front_msg.cell_width = self.resolution
        front_msg.cell_height = self.resolution
        closed_msg.cell_width = self.resolution
        closed_msg.cell_height = self.resolution
        front_msg.cells = front_points
        closed_msg.cells = closed_points

        #self.front_pub.publish(front_msg)
        self.closed_pub.publish(closed_msg)


	## This publish a Path to specific topic
    def publish_path(self, points):
        """
            Create a Path() and publishes the cells if Paint is true
            :param points: list of tuples of the path
            :return: Path()
        """
	if self.path is not None:
		path_msg = Path()
		path_points = []
		i = 0
		path_msg.header.frame_id = "map"
		for i in range(len(points)):
		    node = points[i]
		    location = PoseStamped()
		    location.pose.position.x, location.pose.position.y= map_helper.world_to_map(node[0], node[1],self.resolution, self.x_offset,self.y_offset)
		    path_points.append(location)
		    i += 1
		path_msg.poses = path_points
		self.path_pub.publish(path_msg)
	## For sign off of drawing a circle
    def circle_points(self):
        circle = GridCells()
        points = []
        a = Point()
        b = Point()
        c = Point()
        d = Point()
        e = Point()
        a.x = 0
        a.y = 0
        b.x = 0
        b.y = 36
        c.x = 36
        c.y = 36
        d.x = 36
        d.y = 0
        e.x = 0
        e.y = 0
        points.append(a)
        points.append(b)
        points.append(c)
        points.append(d)
        points.append(e)
        circle.cells = points
        return circle

if __name__ == '__main__':

	## Main function that creates an instance of A_Star and paint the cells and path in a specific frequency
	a = A_Star()
    	rate = rospy.Rate(1) # 10hz
	while not rospy.is_shutdown():
		a_frontier = copy.deepcopy(a.frontier)
		a.paint_cells(a_frontier, a.came_from)
		a.publish_path(a.path)
		rate.sleep()
    	rospy.spin()
