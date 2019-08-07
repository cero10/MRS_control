#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import *
import time
from heapq import nlargest
from heapq import nsmallest


gmap=OccupancyGrid()
coord = {}

class object:
	def __init__(self,posx,posy,block):
		if posx:
			self.pos = Odometry()
			self.map_data = OccupancyGrid()
			self.goal = PoseStamped()
			self.pos.pose.pose.position.x = posx
			self.pos.pose.pose.position.y = posy
			self.map_data.info.origin.position.x = -1.0
			self.map_data.info.origin.position.y = -1.0
			self.map_data.info.resolution = 0.0500000007451
			self.map_data.info.width = 1024
			self.map_data.info.height = 640
			self.goal.header.frame_id = 'map'
			self.goal.pose.position.x = self.pos.pose.pose.position.x
			self.goal.pose.position.y = self.pos.pose.pose.position.y + 2.0
			self.goal.pose.orientation.z = 0.7
			self.goal.pose.orientation.w = 0.7
			self.yaw = pi/2
			self.block = block
			#self.coord = {}
			self.counter = -1
			self.counter0 = 1
			self.counter1 = 0
			self.start = time.time()-5
			#self.lastx = self.pos.pose.pose.position.x
			#self.lasty = self.pos.pose.pose.position.y
			#self.lastyaw = self.yaw
		else:
			self.pos = Odometry()
			self.map_data = OccupancyGrid()
			self.goal = PoseStamped()
			self.pos.pose.pose.position.x = -5.0
			self.pos.pose.pose.position.y = -22.0
			self.map_data.info.origin.position.x = -31.4
			self.map_data.info.origin.position.y = -31.4
			self.map_data.info.resolution = 0.0500000007451
			self.map_data.info.width = 1056
			self.map_data.info.height = 800
			self.goal.header.frame_id = 'map'
			self.goal.pose.position.x = self.pos.pose.pose.position.x
			self.goal.pose.position.y = self.pos.pose.pose.position.y + 2.0
			self.goal.pose.orientation.z = 0.7
			self.goal.pose.orientation.w = 0.7
			self.yaw = pi/2
			self.block = 'X  -5.0Y -22.0'
			#self.coord = {}
			self.counter = -1
			self.counter0 = 1
			self.counter1 = 0
			self.start = time.time()-5
			#self.lastx = self.pos.pose.pose.position.x
			#self.lasty = self.pos.pose.pose.position.y
			#self.lastyaw = self.yaw
	
	def posxres(self,x):
		global gmap
		xx = (abs(gmap.info.origin.position.x - x)/(gmap.info.width*gmap.info.resolution))*gmap.info.width
		return round(xx)
	def posyres(self,y):
		global gmap
		yy = (abs(gmap.info.origin.position.y - y)/(gmap.info.height*gmap.info.resolution))*gmap.info.height
		return round(yy)
	
	def namer(self,x,y):
		x = self.quantify(x)
		y = self.quantify(y)
		string = 'X{:6.1f}Y{:6.1f}'.format(x,y)
		return string


	def quantify(self,x): #round up xy position to the minimum block size
		#print('quantify')
		absx = abs(x)
		a = int(absx/100)
		b = int(absx/10-a*100)
		c = int(absx/1-a*100-b*10)
		d = absx - a*100-b*10-c*1
		if d >= 0.5:
			d = 5
		elif d >= 0 and d < 0.5:
			d = 0
		if x >= 0:
			quant = float('{}{}{}.{}'.format(a,b,c,d)[0:7])
		elif x < -0.4999999:
			quant = float('-{}{}{}.{}'.format(a,b,c,d)[0:7])
		else:
			quant = float('{}{}{}.{}'.format(a,b,c,d)[0:7])
		return(quant)

	def get_dtheta(self,x,y): # get the angle delta between current position and another position
		delta_x = x - self.goal.pose.position.x
		delta_y = y - self.goal.pose.position.y
		theta = atan2(delta_y, delta_x)
		if theta < 0:
		    theta = pi + (pi - abs(theta))
		theta = self.yaw - theta
		if theta < -pi:
		  theta = pi + (pi - abs(theta))
		return (abs(theta))


	def get_distribution(self,x,y): # takes xy coordinates as input and returns a weight based on the distribution of the states of the cells
		#print("get distro")
		global gmap,coord
		sum_occupied = sum_empty = sum_unknown = 0
		#print(x,y)
		x = self.posxres(x)
		y = self.posyres(y)
		#print(x,y)
		for j in range(int(round(y)+5), int(round(y)-5),-1):
			for i in range(int(x-5), int(x+5)):
				if (i > 0) and (i < gmap.info.width) and (j > 0) and (j < gmap.info.height):
					listno = (j*gmap.info.width)+i
					#print(listno,j,i,gmap.info.width)
					if (gmap.data[listno] > 0):
				    		sum_occupied = sum_occupied + 1
					elif (gmap.data[listno] == 0):
				   		sum_empty = sum_empty + 1
					else:
				    		sum_unknown = sum_unknown + 1
				else:
					sum_occupied = sum_occupied + 1
		# max = 100 cells (10x10)
		if (sum_occupied+sum_empty) > 85:
			return (-0.01*(sum_occupied+sum_empty))
		elif sum_unknown < 90 and sum_unknown > 20 and sum_occupied < 3:
			return (1-(sum_occupied*0.1))
		else:
			return 0

	def mark_blocks(self):
		global gmap,coord
		#print("mark_blocks")
		self.counter = self.counter + 1
		self.block = self.namer(self.pos.pose.pose.position.x,self.pos.pose.pose.position.y)
		#print(self.block)
		blockx = float(self.block[1:7])
		blocky = float(self.block[8:14])
		if self.block not in coord:
			x = blockx
			y = blocky
			coord[self.block] = self.get_distribution(x,y)
		if self.yaw > 5.49778714 or self.yaw < 0.78539816:
			for i in range(0,12):
				for j in range((-5-i),(5+i)):
					#if self.pos.pose.pose.position.x > -21 and self.pos.pose.pose.position.x < 40 and self.pos.pose.pose.position.y > -23 and self.pos.pose.pose.position.y < 41:
					x = blockx+(i*0.5)
					y = blocky+(j*0.5)
					if (x > gmap.info.origin.position.x+1) and (x < gmap.info.width*gmap.info.resolution-1) and (y > gmap.info.origin.position.y+1) and (y < gmap.info.height*gmap.info.resolution-1):
						string = self.namer(x,y)
						#print(string)	
						if string not in coord:
							#print(string)
							coord[string] = self.get_distribution(x,y)
		elif self.yaw > 0.78539816 and self.yaw < 2.35619449:
			for i in range(0,12):
				for j in range((-5-i),(5+i)):
					#if self.pos.pose.pose.position.x > -21 and self.pos.pose.pose.position.x < 40 and self.pos.pose.pose.position.y > -23 and self.pos.pose.pose.position.y < 41:
					x = blockx+(i*0.5)
					y = blocky+(j*0.5)
					if (x > gmap.info.origin.position.x+1) and (x < gmap.info.width*gmap.info.resolution-1) and (y > gmap.info.origin.position.y+1) and (y < gmap.info.height*gmap.info.resolution-1):
						string = self.namer(x,y)
						#print(string)
						if string not in coord:
							#print(string)
							coord[string] = self.get_distribution(x,y)
		elif self.yaw > 2.35619449 and self.yaw < 3.92699082:
			for i in range(0,-12,-1):
				for j in range((-5-abs(i)),(5+abs(i))):
					#if self.pos.pose.pose.position.x > -21 and self.pos.pose.pose.position.x < 40 and self.pos.pose.pose.position.y > -23 and self.pos.pose.pose.position.y < 41:
					x = blockx+(i*0.5)
					y = blocky+(j*0.5)
					if (x > gmap.info.origin.position.x+1) and (x < gmap.info.width*gmap.info.resolution-1) and (y > gmap.info.origin.position.y+1) and (y < gmap.info.height*gmap.info.resolution-1):
						string = self.namer(x,y)
						#print(string)
						if string not in coord:
							#print(string)
							coord[string] = self.get_distribution(x,y)
		elif self.yaw > 3.92699082 and self.yaw < 5.49778714:
			for i in range(0,-12,-1):
				for j in range((-5-abs(i)),(5+abs(i))):
					#if self.pos.pose.pose.position.x > -21 and self.pos.pose.pose.position.x < 40 and self.pos.pose.pose.position.y > -23 and self.pos.pose.pose.position.y < 41:
					x = blockx+(i*0.5)
					y = blocky+(j*0.5)
					if (x > gmap.info.origin.position.x+1) and (x < gmap.info.width*gmap.info.resolution-1) and (y > gmap.info.origin.position.y+1) and (y < gmap.info.height*gmap.info.resolution-1):
						string = self.namer(x,y)
						#print(string)
						if string not in coord:
							#print(string)
							coord[string] = self.get_distribution(x,y)

	def reassess_blocks(self):
		global coord
		#print("reassess_blocks")
		self.counter = 0
		self.block = self.namer(self.pos.pose.pose.position.x,self.pos.pose.pose.position.y)
		x = float(self.block[1:7])
		y = float(self.block[8:14])
		for i in range(28,-28,-1):
			for j in range((-28),(28)):
				xx = x+(j*0.5)
				yy = y+(i*0.5)
				string = 'X{:6.1f}Y{:6.1f}'.format(xx,yy)
				if coord.get(string) >= 0:
					coord[string] = (self.get_distribution(xx,yy))

	def frontier_designator(self,drone2,drone3):
		global coord
		self.start = time.time()
		#print("designator")
		mdict = {}
		largest = nlargest(30, coord, key = coord.get)
		for i in range(0,len(largest)):
			if coord.get(largest[i]) > 0:
				x = float(largest[i][1:7])
				y = float(largest[i][8:14])
				theta = self.get_dtheta(x,y)
				a = coord.get(largest[i])
				#print("a",a,type(a),"theta",round(theta,1),type(theta),"x",round(x,1),type(x),"y",round(y,1),type(y),"posx",round(self.pos.pose.pose.position.x,1),type(self.pos.pose.pose.position.x),"posy",round(self.pos.pose.pose.position.y,1),type(self.pos.pose.pose.position.y),"goal2x",round(drone2.goal.pose.position.x,1),type(drone2.goal.pose.position.x),"goal2y",round(drone2.goal.pose.position.y,1),type(drone2.goal.pose.position.y),"goal3x",round(drone3.goal.pose.position.x,1),type(drone3.goal.pose.position.x),"goal3y",round(drone3.goal.pose.position.y,1),type(drone3.goal.pose.position.y))
				#weight = max(0.01,a - min(0.2,(theta /(4*pi))) - min(0.25,(abs(self.pos.pose.pose.position.x - x) + abs(self.pos.pose.pose.position.y - y))/50) - min(0.2,(abs(x - drone2.pos.pose.pose.position.x) + abs(y - drone2.pos.pose.pose.position.y))/80) - min(0.2,(abs(x - drone3.pos.pose.pose.position.x) + abs(y - drone3.pos.pose.pose.position.y))/80))
				weight = max(0.01,a - min(0.15,(theta/(10*pi))) - min(0.20,(abs(self.pos.pose.pose.position.x - x) + abs(self.pos.pose.pose.position.y - y))) - min(0.3,0.015*(20-(abs(x - drone2.goal.pose.position.x) + abs(y - drone2.goal.pose.position.y)))) - min(0.3,0.015*(20-(abs(x - drone3.goal.pose.position.x) + abs(y - drone3.goal.pose.position.y)))))
				mdict[largest[i]] = weight
				#print(type(theta),type(a),type(weight),mdict.get(largest[i]),type(mdict.get(largest[i])))
		if len(mdict) > 1:
			largest = nlargest(2, mdict, key = mdict.get)
		elif len(mdict) == 1:
			if self.goal.pose.position.x != float(largest[0][1:7]) and self.goal.pose.position.y != float(largest[0][8:14]):
				print('1ST CHOICE', coord.get(largest[0]), largest[0],' d rank ', round(mdict.get(largest[0]),1))
				return largest[0]
			else:
				return 'N'
		else:
			return 'N'
		if self.goal.pose.position.x == float(largest[0][1:7]) and self.goal.pose.position.y == float(largest[0][8:14]):
			print('2ND CHOICE', coord.get(largest[1]), largest[1],' d rank ',round(mdict.get(largest[1]),1))
			return largest[1]
		else:
			print('1ST CHOICE', coord.get(largest[0]), largest[0],' d rank ',round(mdict.get(largest[1]),1))
			return largest[0]
			
	def search_map(self,drone2,drone3):
		global gmap,coord
		#print('WHOOOOOOLE MAP BABY')
		a = int(2*(self.quantify(gmap.info.origin.position.y)+0.5))
		b = int(2*self.quantify(self.quantify(gmap.info.origin.position.y + (gmap.info.resolution*gmap.info.height)-0.5)))
		c = int(2*(self.quantify(gmap.info.origin.position.x)+0.5))
		d = int(2*self.quantify(self.quantify(gmap.info.origin.position.x + (gmap.info.resolution*gmap.info.width)-0.5)))
		for j in range(b,a,-1):
			for i in range(c,d,1):
				string = 'X{:6.1f}Y{:6.1f}'.format(float(i)/2,float(j)/2)
				#print(string)
				#if coord.get(string) >= 0:
				coord[string] = (self.get_distribution(float(i)/2,float(j)/2))
				#print(string,coord.get(string))
		ggg = self.frontier_designator(drone2,drone3)
		return ggg
			

	def calc_dist(self):
		dist = abs(self.pos.pose.pose.position.x - (self.goal.pose.position.x + abs(self.map_data.info.origin.position.x - gmap.info.origin.position.x))) + abs(self.pos.pose.pose.position.y - (self.goal.pose.position.y + abs(self.map_data.info.origin.position.y - gmap.info.origin.position.y)))
		print(dist)
		return dist

	#def check_if_moved(self):
		#global start
		#if (time.time()-start) > 4:
		#	start = time.time()
		#	a = abs(self.pos.pose.pose.position.x - self.lastx)
		#	b = abs(self.pos.pose.pose.position.y - self.lasty)
		#	c = abs(self.yaw - self.lastyaw)
		#	self.lastx = self.pos.pose.pose.position.x
		#	self.lasty = self.pos.pose.pose.position.y
		#	self.lastyaw = self.yaw
		#	if a < 0.8 or b < 0.8 or c < 0.5:
		#		return 1
		#	else:
		#		return 0
		#else:
		#	return 0

	def update_pos(self, msg):
		self.pos = msg
		orientation_list = [self.pos.pose.pose.orientation.x, self.pos.pose.pose.orientation.y, self.pos.pose.pose.orientation.z, self.pos.pose.pose.orientation.w]
		(roll, pitch, self.yaw) = euler_from_quaternion (orientation_list)
		

	def update_map_data(self, msg):
		self.map_data = msg
		
	def update_map(self,msg):
		global gmap
		gmap = msg
	
	def get_status(self, msg):
		self.status = msg.status_list[0].status if len(msg.status_list) else 0

def front_explorer():
	rospy.init_node('front_explorer')
	#self.start = time.time()
	drone1 = object(-5,-22,'X  -5.0Y -22.0')
	drone2 = object(-6,-23,'X  -6.0Y -23.0')
	drone3 = object(-4,-23,'X  -4.0Y -23.0')
	rospy.Subscriber('quadrotor1/ground_truth/state', Odometry, drone1.update_pos)
	rospy.Subscriber('quadrotor1/map', OccupancyGrid, drone1.update_map_data)
	rospy.Subscriber('quadrotor1/move_base/status', GoalStatusArray, drone1.get_status)
	pub1 = rospy.Publisher('quadrotor1/move_base_simple/goal', PoseStamped, queue_size = 50)
	rospy.Subscriber('quadrotor2/ground_truth/state', Odometry, drone2.update_pos)
	rospy.Subscriber('quadrotor2/map', OccupancyGrid, drone2.update_map_data)
	rospy.Subscriber('quadrotor2/move_base/status', GoalStatusArray, drone2.get_status)
	pub2 = rospy.Publisher('quadrotor2/move_base_simple/goal', PoseStamped, queue_size = 50)
	rospy.Subscriber('quadrotor3/ground_truth/state', Odometry, drone3.update_pos)
	rospy.Subscriber('quadrotor3/map', OccupancyGrid, drone3.update_map_data)
	rospy.Subscriber('quadrotor3/move_base/status', GoalStatusArray, drone3.get_status)
	pub3 = rospy.Publisher('quadrotor3/move_base_simple/goal', PoseStamped, queue_size = 50)
	rospy.Subscriber('map', OccupancyGrid, drone1.update_map)
	
	#pub1.publish(drone1.goal)
	#pub2.publish(drone2.goal)
	#pub3.publish(drone3.goal)
	while not gmap.data:
		print('waiting for /map topic')
		time.sleep(1)
	while not drone1.start or not drone2.start or not drone3.start:
		print('waiting for start')
		time.sleep(1)

	drone3.search_map(drone1,drone3)
	#pub1.publish(drone1.goal)
	#pub2.publish(drone2.goal)
	#pub3.publish(drone3.goal)
	#print(drone1.goal.pose.position.x,drone1.goal.pose.position.y,drone2.goal.pose.position.x,drone2.goal.pose.position.y,drone3.goal.pose.position.x,drone3.goal.pose.position.y)

	rate = rospy.Rate(1) # 1hz
	while not rospy.is_shutdown():
		drone1.mark_blocks()
		drone2.mark_blocks()
		drone3.mark_blocks()
		if drone1.counter > 1:
			drone1.reassess_blocks()
		if drone2.counter > 1:
			drone2.reassess_blocks()
		if drone3.counter > 1:
			drone2.reassess_blocks()
		if drone2.counter1 > 10:
			drone2.search_map(drone1,drone3)
			drone2.counter1 = 0
		else:
			drone2.counter1 = drone2.counter1 + 1

		if drone1.counter0 == 3:
			drone1.counter0 = 0
			if drone1.status == 4 or drone1.status == 5 or drone1.status == 9:
				string1 = drone1.namer(drone1.goal.pose.position.x,drone1.goal.pose.position.y)
				print('drone1 ',drone1.status, "it's stuck at ",string1)
				coord[string1] = -1
				string1 = drone1.namer(drone1.pos.pose.pose.position.x,drone1.pos.pose.pose.position.y)
				print(string1)
				drone1.goal.pose.position.x = float(string1[1:7])
				drone1.goal.pose.position.y = float(string1[8:14])
				pub1.publish(drone1.goal)
		else:
			drone1.counter0 = drone1.counter0 + 1
		if drone2.counter0 == 3:
			drone2.counter0 = 0
			if drone2.status == 4 or drone2.status == 5 or drone2.status == 9:
				string2 = drone2.namer(drone2.goal.pose.position.x,drone2.goal.pose.position.y)
				print('drone2 ',drone2.status, "it's stuck at ",string2)
				coord[string2] = -1
				string2 = drone2.namer(drone2.pos.pose.pose.position.x,drone2.pos.pose.pose.position.y)
				print(string2)
				drone2.goal.pose.position.x = float(string2[1:7])
				drone2.goal.pose.position.y = float(string2[8:14])
				pub2.publish(drone2.goal)
		else:
			drone2.counter0 = drone2.counter0 + 1
		if drone3.counter0 == 3:
			drone3.counter0 = 0
			if drone3.status == 4 or drone3.status == 5 or drone3.status == 9:
				string3 = drone3.namer(drone3.goal.pose.position.x,drone3.goal.pose.position.y)
				print('drone3 ',drone3.status, "it's stuck at ",string3)
				coord[string3] = -1
				string3 = drone3.namer(drone3.pos.pose.pose.position.x,drone3.pos.pose.pose.position.y)
				print(string3)
				drone3.goal.pose.position.x = float(string3[1:7])
				drone3.goal.pose.position.y = float(string3[8:14])
				pub3.publish(drone3.goal)
		else:
			drone3.counter0 = drone3.counter0 + 1
		#print(drone1.status,drone3.status,drone3.status)
		#print(round(gmap.info.origin.position.x,1),round(gmap.info.origin.position.y,1),round(drone1.map_data.info.origin.position.x,1),round(drone1.map_data.info.origin.position.y,1))

		if drone1.calc_dist() < 2.0 or (time.time()-drone1.start) > 70:
			drone1.start = time.time()
			string1 = drone1.frontier_designator(drone2,drone3)
			if string1 != 'N':
				drone1.goal.pose.position.x = float(string1[1:7]) + abs(drone1.map_data.info.origin.position.x - gmap.info.origin.position.x)
				drone1.goal.pose.position.y = float(string1[8:14]) + abs(drone1.map_data.info.origin.position.y - gmap.info.origin.position.y)
				pub1.publish(drone1.goal)
				drone1.goal.pose.position.x = float(string1[1:7])
				drone1.goal.pose.position.y = float(string1[8:14])
				print('DRONE 1',drone1.get_distribution(drone1.goal.pose.position.x,drone1.goal.pose.position.y))
			else:
				string1 = drone1.search_map(drone2,drone3)
				if string1 != 'N':
					drone1.goal.pose.position.x = float(string1[1:7]) + abs(drone1.map_data.info.origin.position.x - gmap.info.origin.position.x)
					drone1.goal.pose.position.y = float(string1[8:14]) + abs(drone1.map_data.info.origin.position.y - gmap.info.origin.position.y)
					print('AFTER ALL: ',string1)
					pub1.publish(drone1.goal)
				else:
					print('THE MAPPING IS DONE')

		if drone2.calc_dist() < 2.0 or (time.time()-drone2.start) > 70:
			drone2.start = time.time()
			string2 = drone2.frontier_designator(drone1,drone3)
			if string2 != 'N':
				drone2.goal.pose.position.x = float(string2[1:7]) + abs(drone2.map_data.info.origin.position.x - gmap.info.origin.position.x)
				drone2.goal.pose.position.y = float(string2[8:14]) + abs(drone2.map_data.info.origin.position.y - gmap.info.origin.position.y)
				pub2.publish(drone2.goal)
				drone2.goal.pose.position.x = float(string1[1:7])
				drone2.goal.pose.position.y = float(string1[8:14])
				print('DRONE 2',drone2.get_distribution(drone2.goal.pose.position.x,drone2.goal.pose.position.y))
			else:
				string2 = drone2.search_map(drone1,drone3)
				if string2 != 'N':
					drone2.goal.pose.position.x = float(string2[1:7]) + abs(drone2.map_data.info.origin.position.x - gmap.info.origin.position.x)
					drone2.goal.pose.position.y = float(string2[8:14]) + abs(drone2.map_data.info.origin.position.y - gmap.info.origin.position.y)
					print('AFTER ALL: ',string2)
					pub1.publish(drone2.goal)
				else:
					print('THE MAPPING IS DONE')

		if drone3.calc_dist() < 2.0 or (time.time()-drone3.start) > 70:
			drone3.start = time.time()
			string3 = drone3.frontier_designator(drone1,drone2)
			if string3 != 'N':
				drone3.goal.pose.position.x = float(string3[1:7]) + abs(drone3.map_data.info.origin.position.x - gmap.info.origin.position.x)
				drone3.goal.pose.position.y = float(string3[8:14]) + abs(drone3.map_data.info.origin.position.y - gmap.info.origin.position.y)
				pub3.publish(drone3.goal)
				drone3.goal.pose.position.x = float(string1[1:7])
				drone3.goal.pose.position.y = float(string1[8:14])
				print('DRONE 3',drone3.get_distribution(drone3.goal.pose.position.x,drone3.goal.pose.position.y))
			else:
				string3 = drone3.search_map(drone1,drone2)
				if string3 != 'N':
					drone3.goal.pose.position.x = float(string3[1:7]) + abs(drone3.map_data.info.origin.position.x - gmap.info.origin.position.x)
					drone3.goal.pose.position.y = float(string3[8:14]) + abs(drone3.map_data.info.origin.position.y - gmap.info.origin.position.y)
					print('AFTER ALL: ',string3)
					pub1.publish(drone3.goal)
				else:
					print('THE MAPPING IS DONE')
		rate.sleep()

if __name__ == '__main__':
    try:
        front_explorer()
    except rospy.ROSInterruptException:
        pass
