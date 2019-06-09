#!/usr/bin/env python
import sys
import rospy
from Robot import *
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
import map_helper


## This is the client that will listen for the goal and start in rviz and make service calls
class client:

    	def __init__(self):
		self.start = None
		self.goal = None
	    	rospy.init_node('client', anonymous=True)
		# Subscribers for getting start and goal data
		self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
		# The robot object for driving the robot
		self.start_pub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.start_call_back)
		self.robot = Robot()


    	def odom_callback(self, msg):
		self.px=msg.pose.pose.position.x
		self.py=msg.pose.pose.position.y

	## Use the path to drive the robot
	def drive_robot_to_path(self,path):
		poses = path.poses
		for i in range(0,len(poses)):
			pose = poses[i]
			self.robot.goalX = pose.pose.position.x
			self.robot.goalY = pose.pose.position.y
			self.robot.normal_drive()

	# Make the service call in the client
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
    		print resp.plan
    		return resp.plan
	    except rospy.ServiceException, e:
    		print "Service call failed: %s"%e

    	def run_navigate_map_client(self):
	    rospy.wait_for_service('run_navigate_map')
	    try:
    		run_navigate_map = rospy.ServiceProxy('run_navigate_map', GetPlan)
    		p= GetPlan()
    		start_pose = PoseStamped()
    		start_pose.pose.position.x = self.px
    		start_pose.pose.position.y = self.py
    		resp = run_navigate_map(start_pose,start_pose,0.5)
    		path = resp.plan.poses

    		print path[0]
    		print path[1]
    		return path[0],path[1]
	    except rospy.ServiceException, e:
            	print "Service call failed: %s"%e

	## Call back function that sets the start
	def start_call_back(self,msg):
		rospy.loginfo("received in start")
		x = msg.pose.pose.position.x
		y = msg.pose.pose.position.y
		self.run()
		##self.start =(x,y)

	## Call back function that sets the goal
	def goal_call_back(self,msg):
		rospy.loginfo("received in goal")
		x = msg.pose.position.x
		y = msg.pose.position.y
		self.goal =(x,y)
		if self.start is not None:
			rospy.loginfo("starts a star")
			rospy.loginfo("start"+str(self.start))
			rospy.loginfo("goal"+str(self.goal))
			path = self.run_astar_client(self.start, self.goal)
			self.drive_robot_to_path(path)

	def run(self):
		rospy.loginfo("In running")
		self.start,self.goal = self.run_navigate_map_client()
		start = (self.start.pose.position.x, self.start.pose.position.y)
		goal = (self.goal.pose.position.x, self.goal.pose.position.y)
		rospy.loginfo("Try gettting path")
		path = self.run_astar_client(start, goal)
		self.drive_robot_to_path(path)


if __name__ == "__main__":
	## Creates the client object
	c = client()
	rospy.spin()
