#!/usr/bin/env python

# Dan Cohen
# 2/25/2021
# 

# This script tests if the self-created inverted pendulum can be controled by joint velocity controller


from __future__ import print_function

import math
import random
import time

import rospy
from std_msgs.msg import (UInt16, Float64)
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from gazebo_msgs.msg import LinkState
from geometry_msgs.msg import Point

PI = 3.14159
    
class Testbed(object):
    """ Testbed, for the pupose of testing cart-pole system """
    def __init__(self):
        self._sub_invpend_states = rospy.Subscriber('/invpend/joint_states', JointState, self.jsCB)
        self._pub_vel_cmd = rospy.Publisher('/invpend/joint1_velocity_controller/command', Float64, queue_size=1)
	self._pub_pos_cmd_joint_2 = rospy.Publisher('/invpend/joint2_position_controller/command', Float64, queue_size=1)
        self._pub_set_pole = rospy.Publisher('/gazebo/set_link_state', LinkState)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_sim = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

        self.pos_cart = 0.001
        self.vel_cart = 0.001
        self.pos_pole = 0.0
        self.vel_pole = 0
        self.PoleState = LinkState()
        self.PoleState.link_name = 'pole'
        self.PoleState.pose.position = Point(0.0, -0.25, 2.0)
        self.PoleState.reference_frame = 'world'
	
                
    def jsCB(self, data):
    	rospy.loginfo("\n~~~Getting Inverted pendulum joint states~~~\n")
    	self.pos_cart = data.position[1]
    	self.vel_cart = data.velocity[1]
    	self.pos_pole = data.position[0]
    	self.vel_pole = data.velocity[0]
        print("cart_position: {0:.5f}, cart_velocity: {1:.5f}, pole_angle: {2:.5f}, pole_angular_velocity: {3:.5f}".format(self.pos_cart, self.vel_cart, self.pos_pole, self.vel_pole))
	
        if math.fabs(self.pos_cart) >= 2.4:
            print("=== reset invpend pos ===\n")
            for _ in range(50):
                self._pub_vel_cmd.publish(0)
                self._pub_set_pole.publish(self.PoleState)
                rospy.sleep(1./50)
                # self.reset_sim()

        
    def wobble(self,loc_x,theta):
        '''
        Cart performs the wobbling.
        '''
        rate = rospy.Rate(50)
        start = rospy.Time.now()
	I = 0
	error_old = 0
	old_pos = self.pos_cart

        #def make_cmd(elapsed):
            #period_factor = .5
            #amplitude_factor = 25
            #w = period_factor * elapsed.to_sec()
            #return amplitude_factor * math.cos(w*2*math.pi)
	
        while not rospy.is_shutdown() and math.fabs(loc_x-self.pos_cart)>0.0001:
	    print(math.fabs(loc_x-self.pos_cart)>0.01)

            if math.fabs(self.pos_cart) <= 2.4:
                elapsed = rospy.Time.now() - start
		dvdt = (-1.0*error_old-(-loc_x+self.pos_cart))/(1.0/50.0)
		print("dvdt = ",dvdt)
		I = I + (1.0*(loc_x-self.pos_cart)*(1.0/50.0))
                cmd_vel = 1.5*(loc_x-self.pos_cart)+ 5.0*dvdt#+I #random.uniform(-50, 50) # make_cmd(elapsed)
		
            else:
                cmd_vel = 0

	    error_old = loc_x-self.pos_cart
	    old_pos = self.pos_cart
	    print("cmd_vel = ", cmd_vel)
	    self._pub_pos_cmd_joint_2.publish(theta)
            self._pub_vel_cmd.publish(cmd_vel)
            #rate.sleep()
            rospy.sleep(1./50)

	cmd_vel = 0
	self._pub_pos_cmd_joint_2.publish(theta)
        self._pub_vel_cmd.publish(cmd_vel)
	    
	    
    def home(self,loc_x,theta):
        '''
        Cart performs the wobbling.
        '''
        rate = rospy.Rate(50)
        start = rospy.Time.now()
	I = 0
	error_old = 0
	old_pos = self.pos_cart

        #def make_cmd(elapsed):
            #period_factor = .5
            #amplitude_factor = 25
            #w = period_factor * elapsed.to_sec()
            #return amplitude_factor * math.cos(w*2*math.pi)
	
        while not rospy.is_shutdown():
	    print(math.fabs(loc_x-self.pos_cart)>0.01)

            if math.fabs(self.pos_cart) <= 2.4:
                elapsed = rospy.Time.now() - start
		dvdt = (-1.0*error_old-(-loc_x+self.pos_cart))/(1.0/50.0)
		print("dvdt = ",dvdt)
		I = I + (1.0*(loc_x-self.pos_cart)*(1.0/50.0))
                cmd_vel = 1.5*(loc_x-self.pos_cart)+ 5.0*dvdt#+I #random.uniform(-50, 50) # make_cmd(elapsed)
		
            else:
                cmd_vel = 0

	    error_old = loc_x-self.pos_cart
	    old_pos = self.pos_cart
	    print("cmd_vel = ", cmd_vel)
	    self._pub_pos_cmd_joint_2.publish(theta)
            self._pub_vel_cmd.publish(cmd_vel)
            #rate.sleep()
            rospy.sleep(1./50)

	cmd_vel = 0
	self._pub_pos_cmd_joint_2.publish(theta)
        self._pub_vel_cmd.publish(cmd_vel)    
        
    def clean_shutdown(self):
        print("Shuting dwon...")
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
    print("Initializing node... ")
    rospy.init_node('cart_wobble')
    pole_hight = 0.5
    cart = Testbed()
    rospy.on_shutdown(cart.clean_shutdown)
    x_list = [0.0,0.0,0.0,0.0,1.0,1.5,-1.0,-1.0]
    y_list = [-0.2,0.2,0.5,-0.5,-0.5,0.0,-0.2,0.2]
    for index in range(len(x_list)):
	print(y_list[index]/pole_hight)
	if y_list[index] > 0.5 or  y_list[index] < 0.0:
	
		theta = -1.0*math.acos(-1.0*y_list[index]/pole_hight)
	else:

		theta = math.acos(y_list[index]/pole_hight)

	offset = math.sin(theta)*pole_hight
	print("theta",theta)
	print("offset",offset)
	if offset<PI/2.0 and offset>3.0*PI/2:
		loc_x_offset = x_list[index]+offset
	
	else:
		loc_x_offset = x_list[index]-offset
	#theta = -3.14
    	cart.wobble(loc_x_offset,theta)
   	print("done")


    loc_x = 0.0
    theta = 0.0
    cart.home(loc_x,theta)
    rospy.spin()

if __name__ == '__main__':
    main()
