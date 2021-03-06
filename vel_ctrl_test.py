#!/usr/bin/env python
#*************************
# Dan Cohen
# 2/25/2021
# SES 598
# 120481845
# ************************
# This script controls a cart pole system with a PID position controller and force PID control on the cart
# this does way-point navigation from point to point in task space


from __future__ import print_function


# import needed libs
import math
import random
import time
import numpy as np
import matplotlib as plt

# import rospy and the msgs
import rospy
from std_msgs.msg import (UInt16, Float64)
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from gazebo_msgs.msg import LinkState
from geometry_msgs.msg import Point

PI = 3.14159
pole_x_loc = []
pole_y_loc = []

mean_loc_x = 0.0
mean_loc_y = 0.0
x_list = [0.0, 0.0, 0.0, 0.0]#, 1.0, 1.5, -1.0, -1.0]  # creating the list of x,y points you want to use for testing
y_list = [0.0, 0.0, 0.0, 0.0]#, -0.5, 0.0, -0.2, 0.2]

# our class is called test bed
class Testbed(object):
	""" Testbed, for the pupose of testing cart-pole system """

	def __init__(self):
		self._sub_invpend_states = rospy.Subscriber('/invpend/joint_states', JointState, self.jsCB)
		self._pub_vel_cmd = rospy.Publisher('/invpend/joint1_velocity_controller/command', Float64, queue_size=1)
		self._pub_pos_cmd_joint_2 = rospy.Publisher('/invpend/joint2_position_controller/command', Float64,
													queue_size=1)

		# self._joint_2_state = rospy.Subscriber('/invpend/joint2_positon_controler/state',Joint2State,
		self._pub_set_pole = rospy.Publisher('/gazebo/set_link_state', LinkState)  # publish to link states
		self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
		self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
		self.reset_sim = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
		# where the the robot starts
		self.pos_cart = 0.001
		self.vel_cart = 0.001
		self.pos_pole = 0.0
		self.vel_pole = 0
		self.PoleState = LinkState()
		self.PoleState.link_name = 'pole'
		self.PoleState.pose.position = Point(0.0, -0.25, 2.0)
		self.PoleState.reference_frame = 'world'

	def jsCB(self, data):
		# rospy.loginfo("\n~~~Getting Inverted pendulum joint states~~~\n")
		self.pos_cart = data.position[1] # getting the cart pos and other critical robot info
		self.vel_cart = data.velocity[1]
		self.pos_pole = data.position[0]
		self.vel_pole = data.velocity[0]
		# print("cart_position: {0:.5f}, cart_velocity: {1:.5f}, pole_angle: {2:.5f}, pole_angular_velocity: {3:.5f}".format(self.pos_cart, self.vel_cart, self.pos_pole, self.vel_pole))

		# print("cart_position: {0:.5f}, pole_angle: {2:.5f}".format(self.pos_cart, self.pos_pole))

		if math.fabs(self.pos_cart) >= 2.5:
			# print("=== reset invpend pos ===\n")
			for _ in range(50):
				self._pub_vel_cmd.publish(0)
				self._pub_set_pole.publish(self.PoleState)
				rospy.sleep(1. / 50)
				# self.reset_sim()
	# find the std onf the way points

	def std_maker(self, n):

		mean = sum(n) / len(n)
		SUM = 0
		for i in n:
			SUM += (i - mean) ** 2

		stdeV = math.sqrt(SUM / (len(n) - 1))
		return stdeV
	# find the min value
	def min_finder(self, target, list_enter):

		min_number_ = 1000.0
		for i in range(len(list_enter)):
			min_number = abs(target - list_enter[i])
			if (min_number < min_number_):
				min_number_ = min_number
		return min_number_
	# find the max number
	def max_finder(self, target, list_enter):

		max_number_ = 0.0
		for i in range(len(list_enter)):
			max_number = abs(target - list_enter[i])
			if (max_number > max_number_):
				max_number_ = max_number
		return max_number_
	# main code for sending robot to the right loc
	def wobble(self, loc_x, theta):
		'''
        Cart performs the desird moation .
        '''
		rate = rospy.Rate(50)
		start = rospy.Time.now()
		I = 0
		error_old = 0
		old_pos = self.pos_cart

		# def make_cmd(elapsed):
		# period_factor = .5
		# amplitude_factor = 25
		# w = period_factor * elapsed.to_sec()
		# return amplitude_factor * math.cos(w*2*math.pi)
		# the pole should get with in 0.3 of the goal y and 0.001 of the x this is how we are defining our error
		# this means the cart will come back ot the x spot until it meet our 0.0001 req, we can lower this and it would
		# let the cart roll passed a way point, I looked at like we want to look at something at that point and hover
		# I could also define this as sqrt(x^2+y^2) for the location we want to send it but the y was well bounded that
		# using the abs(x) was that I used
		while not rospy.is_shutdown() and math.fabs(loc_x - self.pos_cart) > 0.001:  # and math.fabs(theta-self.pos_pole)>0.001:
			# print(math.fabs(loc_x-self.pos_cart)>0.01)
			x = self.pos_cart + 0.5 * math.sin(theta)
			y = 0.5 * math.cos(self.pos_pole)
			# print('x',x,'y',y)
			# start of the control loops
			if math.fabs(self.pos_cart) <= 2.4: # as long as the robot has not hit the end
				# elapsed = rospy.Time.now() - start
				dvdt = (-1.0 * error_old - (-loc_x + self.pos_cart)) / (1.0 / 50.0)  # find the rate of change
				# print("dvdt = ",dvdt)
				I = 0.001*I + (1.0 * (loc_x - self.pos_cart) * (1.0 / 50.0)) # not able to get I working yet
				cmd_vel = 1.5 * (loc_x - self.pos_cart) + 5.0 * dvdt  +I #random.uniform(-50, 50) # make_cmd(elapsed)
				pole_x_loc.append(self.pos_cart + 0.5 * math.sin(self.pos_pole)) # store pol loc x
				pole_y_loc.append(0.5 * math.cos(self.pos_pole)) # store pol loc y
				print(self.pos_cart + 0.5 * math.sin(self.pos_pole), ',', 0.5 * math.cos(self.pos_pole)) #  print pole xy
			# print('pole_y',0.5*math.cos(self.pos_pole))

			else:  # otherwise not send 0 vel/force
				cmd_vel = 0

			error_old = loc_x - self.pos_cart  # find the old error
			old_pos = self.pos_cart  # save old pose
			# print("cmd_vel = ", cmd_vel)
			# publish to the the robot
			self._pub_pos_cmd_joint_2.publish(theta)
			self._pub_vel_cmd.publish(cmd_vel)
			# rate.sleep()
			rospy.sleep(1. / 50)
		# publish to robot
		cmd_vel = 0
		self._pub_pos_cmd_joint_2.publish(theta)
		self._pub_vel_cmd.publish(cmd_vel)

	def home(self, loc_x, theta): #when we want to send thr robot home
		'''
        Cart performs the wobbling.
        '''
		rate = rospy.Rate(50)
		start = rospy.Time.now()
		I = 0
		error_old = 0
		old_pos = self.pos_cart
		# check theta

		# def make_cmd(elapsed):
		# period_factor = .5
		# amplitude_factor = 25
		# w = period_factor * elapsed.to_sec()
		# return amplitude_factor * math.cos(w*2*math.pi)

		while not rospy.is_shutdown() and math.fabs(loc_x - self.pos_cart) > 0.01:
			# print(math.fabs(loc_x - self.pos_cart) > 0.01)
			x = self.pos_cart + 0.5 * math.sin(theta)
			y = 0.5 * math.cos(self.pos_pole)
			#print('x', x, 'y', y)

			if math.fabs(self.pos_cart) <= 2.4:
				elapsed = rospy.Time.now() - start
				dvdt = (-1.0 * error_old - (-loc_x + self.pos_cart)) / (1.0 / 50.0)
				# print("dvdt = ",dvdt)
				I = I + (1.0 * (loc_x - self.pos_cart) * (1.0 / 50.0))
				cmd_vel = 1.5 * (loc_x - self.pos_cart) + 5.0 * dvdt  # +I #random.uniform(-50, 50) # make_cmd(elapsed)


			else:
				cmd_vel = 0  # otherwise send a cmd vel of zero

			error_old = loc_x - self.pos_cart
			old_pos = self.pos_cart
			# print("cmd_vel = ", cmd_vel)
			self._pub_pos_cmd_joint_2.publish(theta)
			self._pub_vel_cmd.publish(cmd_vel)
			# rate.sleep()
			rospy.sleep(1. / 50)

		cmd_vel = 0
		self._pub_pos_cmd_joint_2.publish(theta)
		self._pub_vel_cmd.publish(cmd_vel)

	def clean_shutdown(self):
		# print("Shuting dwon...")
		self._pub_vel_cmd.publish(0)
		return True

	def _reset(self):
		rospy.wait_for_service("/gazebo/reset_simulation")
		print("reset simulation===\n")
		self.reset_sim()
		# rospy.wait_for_service("/gazebo/unpause_physics")
		# self.unpause
		# rospy.wait_for_service("/gazebo/pause_physics")
		# self.pause


def main():
	""" Perform testing actions provided by Testbed class
    """
	# print("Initializing node... ")
	rospy.init_node('cart_wobble')
	pole_hight = 0.5
	cart = Testbed()
	rospy.on_shutdown(cart.clean_shutdown)
	#x_list = [0.0, -1.0, 0.0, 0.0, 1.0, 1.5, -1.0, -1.0]
	#y_list = [0.0, 0.2, 0.5, -0.2, -0.5, 0.0, -0.2, 0.2]
	x_list = [0.0, -1.0, 0.0, 0.0]  # , 1.0, 1.5, -1.0, -1.0] #list of way points
	y_list = [0.0, 0.2, 0.5, -0.2]  # , -0.5, 0.0, -0.2, 0.2]
	for index in range(len(x_list)):
		# print(y_list[index] / pole_hight)
		if y_list[index] < 0.0: # is the value less then 0

			theta = -1.0 * math.acos(1.0 * y_list[index] / pole_hight) # find theta
		else:

			theta = 1.0*math.acos(1.0*y_list[index] / pole_hight) # find theta

		offset = math.sin(theta) * pole_hight # find the offset
		# print("theta", theta)
		# print("offset", offset)
		if offset < PI / 2.0 and offset > 3.0 * PI / 2:  # find where the robot offset is
			loc_x_offset = x_list[index] + offset

		else:
			loc_x_offset = x_list[index] - offset
		# theta = -3.14
		cart.wobble(loc_x_offset, theta)
		# print("done", index)
		# print('length of pole_x_loc', len(pole_x_loc))
		# print('length of pole_y_loc', len(pole_y_loc))

		if (len(pole_x_loc) > 0):
			mean_loc_x = sum(pole_x_loc) / len(pole_x_loc)
			mean_loc_y = sum(pole_y_loc) / len(pole_y_loc)

			std_Dev_x = cart.std_maker(pole_x_loc)
			std_Dev_y = cart.std_maker(pole_y_loc)
			# print('x goal location', x_list[index])
			# print('y goal location', y_list[index])
			# print('mean of loc_x in m', mean_loc_x)
			# print('mean of loc_y in m', mean_loc_y)
			# print('var of x m', std_Dev_x)
			# print('var rad', std_Dev_y)
			# print('loc_x min m', cart.min_finder(x_list[index], pole_x_loc))
			# print('loc_x max m', cart.max_finder(x_list[index], pole_x_loc))
			del pole_y_loc[:]
			del pole_x_loc[:]
		del pole_y_loc[:]
		del pole_x_loc[:]

	loc_x = 0.0   # send to home
	theta = 0.0   # send to home
	cart.home(loc_x, theta)
	# print('***********************************************')


# mean_loc_x = sum(pole_x_loc)/len(pole_x_loc)
# mean_loc_y = sum(pole_y_loc)/len(pole_y_loc)

# var_loc_x = statistics.pstdev(pole_x_loc)
# var_loc_y = statistics.pstdev(pole_y_loc)
# std_Dev_x = cart.std_maker(pole_x_loc)
# std_Dev_y = cart.std_maker(pole_y_loc)
# print('x goal location',x_list[0])
# print('y goal location',y_list[0])
# print('mean of loc_x in m',mean_loc_x)
# print('mean of loc_y in m',mean_loc_y)
# print('var of x m',std_Dev_x)
# print('var rad',std_Dev_y)
# print('loc_x min m',cart.min_finder(x_list[0],pole_x_loc))
# print('loc_x max m',cart.max_finder(x_list[0],pole_x_loc))
# del pole_y_loc[:]
# del pole_x_loc[:]
# print('loc_x max m',max(x_list[0]-pole_x_loc))
# print('loc_y min m',min(y_list[0]-pole_y_loc))
# print('loc_y max m',max(y_list[0]-pole_y_loc))

# plt.plot(pole_x_loc)
# plt.show(block=True)


if __name__ == '__main__':
	main()

	# print('done done done ')
