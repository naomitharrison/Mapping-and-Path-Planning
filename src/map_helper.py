#!/usr/bin/env python
import sys
import rospy
import numpy
import math
from nav_msgs.msg import OccupancyGrid, GridCells, Path
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseStamped, PoseArray, Pose

## Get the neighbors in the numpy array of sepecific coordinates
def get_neighbors(loc, my_map):
	x = loc[0]
	y = loc[1]
	x_shape = my_map.shape[0]
	y_shape = my_map.shape[1]
	m_list = []
	if(is_valueable((x-1,y),x_shape, y_shape) is True):
		if is_valid_loc((x-1,y), my_map):
			m_list.append((x-1,y))
	if(is_valueable((x+1,y),x_shape, y_shape) is True):
		if is_valid_loc((x+1,y), my_map):
			m_list.append((x+1,y))
	if(is_valueable((x,y+1),x_shape, y_shape) is True):
		if is_valid_loc((x,y+1), my_map):
			m_list.append((x,y+1))
	if(is_valueable((x,y-1),x_shape, y_shape) is True):
		if is_valid_loc((x,y-1), my_map):
			m_list.append((x,y-1))
	return m_list

def get_neighbors_way_out(loc, my_map):
	x = loc[0]
	y = loc[1]
	x_shape = my_map.shape[0]
	y_shape = my_map.shape[1]
	m_list = []
	if(is_valueable((x-1,y),x_shape, y_shape) is True):
		if is_valid_loc_aStar((x-1,y), my_map):
			m_list.append((x-1,y))
	if(is_valueable((x+1,y),x_shape, y_shape) is True):
		if is_valid_loc_aStar((x+1,y), my_map):
			m_list.append((x+1,y))
	if(is_valueable((x,y+1),x_shape, y_shape) is True):
		if is_valid_loc_aStar((x,y+1), my_map):
			m_list.append((x,y+1))
	if(is_valueable((x,y-1),x_shape, y_shape) is True):
		if is_valid_loc_aStar((x,y-1), my_map):
			m_list.append((x,y-1))
	return m_list

def get_neighbors_aStar(loc, my_map):
	x = loc[0]
	y = loc[1]
	x_shape = my_map.shape[0]
	y_shape = my_map.shape[1]
	m_list = []
	actual_list = []
	if(is_valueable((x-1,y),x_shape, y_shape) is True):
		if is_valid_loc_aStar((x-1,y), my_map):
			m_list.append((x-1,y))
	if(is_valueable((x+1,y),x_shape, y_shape) is True):
		if is_valid_loc_aStar((x+1,y), my_map):
			m_list.append((x+1,y))
	if(is_valueable((x,y+1),x_shape, y_shape) is True):
		if is_valid_loc_aStar((x,y+1), my_map):
			m_list.append((x,y+1))
	if(is_valueable((x,y-1),x_shape, y_shape) is True):
		if is_valid_loc_aStar((x,y-1), my_map):
			m_list.append((x,y-1))
	for i in range(len(m_list)):
		if is_valid_for_robot(m_list[i], my_map,2):
			actual_list.append(m_list[i])
	return actual_list

def is_valid_for_robot(loc, my_map, size):
	x = loc[0]
	y = loc[1]
	x_shape = my_map.shape[0]
	y_shape = my_map.shape[1]
	case = True
	for i in range(-size,size+1):
		for j in range(-size,size+1):
			if(i!=0 or j!=0):
				if(is_valueable((x+i,y+j),x_shape, y_shape) is True):
					if not is_valid_loc((x+i,y+j), my_map):
						return False
	return True

def is_valid_for_robot_aStar(loc, my_map, size):
	x = loc[0]
	y = loc[1]
	x_shape = my_map.shape[0]
	y_shape = my_map.shape[1]
	case = True
	for i in range(-size,size+1):
		for j in range(-size,size+1):
			if(i!=0 or j!=0):
				if(is_valueable((x+i,y+j),x_shape, y_shape) is True):
					if not is_valid_loc_aStar((x+i,y+j), my_map):
						return False
	return True

def is_valid_for_robot_aStar_test(loc, my_map, size):
	x = loc[0]
	y = loc[1]
	x_shape = my_map.shape[0]
	y_shape = my_map.shape[1]
	case = True
	for i in range(-size,size+1):
		for j in range(-size,size+1):
			if(i!=0 or j!=0):
				if(is_valueable((x+i,y+j),x_shape, y_shape) is True):
					if not is_valid_loc((x+i,y+j), my_map):
						return False
	return True


## Check if a point is in the map or not
def is_valueable(loc, x_shape, y_shape):
	case = True
	if(loc[0] < 0 or loc[0] >= x_shape):
		case = False
	if(loc[1] < 0 or  loc[1] >= y_shape):
		case = False
	return case

def is_valid_loc_aStar(loc, my_map):
	if my_map[loc] == 100 or my_map[loc] == -1:
		return False
	return True

## Checks the point whether it is a obstacle or not
def is_valid_loc(loc, my_map):
	if my_map[loc] == 100:
		return False
	return True

## Convert the 1d array to the a 2d numpy array
def convert_gridcells_to_2d(gridcells, width, height):
    map = numpy.array(gridcells)
    map = numpy.resize(map,(height, width))
    map = numpy.swapaxes(map, 0 ,1)
    numpy.set_printoptions(threshold=numpy.nan)
    return map

def convert_location(loc, my_map):
    """converts points to the grid"""



## Convert coordinates from the real map coordinates to numpy array
def world_to_map(x, y, resolution, x_offset, y_offset):
	newX =(x*resolution)+x_offset + (0.5 * resolution)

	newY =(y*resolution)+y_offset + (0.5 * resolution)
	return newX,newY
## Convert coordinates from the numpy array coordinates to real map
def map_to_world(x, y, resolution, x_offset, y_offset):
	newX = math.floor((x - x_offset)/resolution)
	newY = math.floor((y - y_offset)/resolution)
	return newX,newY

## Checks the point whether it has been discovered or not
def is_discovered(loc, my_map):
	if my_map[loc] == -1:
		return False
	return True

def is_frontier(location, map):
	neighbors = get_neighbors(location, map)
	if is_valid_loc_aStar(location, map): # if discovered and not an object
		for i in range(len(neighbors)):
			if not is_discovered(neighbors[i],map): # if neighbor is not discovered
				return True # it is on the fronteir
		return False # if location and all neighbors are discovered, it is not on frontier
	return False # if location is undiscovered, it is not on frontier


def to_cells(points, my_map):
    """
        Creates a GridCell() for Rviz distplay
        :param points: list of tuples
        :return: GridCell()
    """


def to_poses(points, my_map):
    """
        Creates a GridCell() for Rviz distplay
        :param points: list of tuples
        :return: GridCell()
    """


def index_to_point(point, my_map):
    """convert a point to a index"""

def point_to_index(location, my_map):
    """convert a index to a point"""
