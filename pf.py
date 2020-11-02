#!/usr/bin/python
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from util import rotateQuaternion, getHeading, multiply_quaternions
from random import random, gauss
from time import time

ROOT_2_PI = math.sqrt(2*math.pi)
MAX_SAMPLE = 500

def find_weighted_random(sd): #use approximate normal to generate random number
	#takes standard deviation as input, returns a value from between -3 standard deviations
	#and 3 standard deviations chosen at random with proabability weighted using
	#expression for normal distribution
	values = np.arange(-3*sd,3*sd,sd/10.0)
	weights = [0]*len(values)
	sum_weights = 0
	for t in range(0,len(values)):
		weights[t] = (1/(ROOT_2_PI))*math.exp(-0.5*(values[t]/sd)**2)
		sum_weights = sum_weights + weights[t]
	z = random()*sum_weights
	for t in range(0,len(values)):
		if z < weights[t]:
			return values[t]
		z = z - weights[t]
	return 0 #base case to avoid infinite loop; should never run


class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # ----- Set motion model parameters
	self.ODOM_ROTATION_NOISE = 0.05 # Odometry model rotation noise 0.5
	self.ODOM_TRANSLATION_NOISE = 0.2 # Odometry model x axis (forward) noise 0.2
	self.ODOM_DRIFT_NOISE = 0.2 # Odometry model y axis (side-to-side) noise 0.2

        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict
        self.DEBUG = True
        self.SortedX = [float]*MAX_SAMPLE
        self.SortedY = [float]*MAX_SAMPLE
        self.Sorted_Orientation = [float]*MAX_SAMPLE
        self.noise = 2.5
       
    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise
        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.
        
        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """
	particle_array = PoseArray()

	particle_array.poses = [Odometry().pose.pose]*MAX_SAMPLE #100 is number of data into array [Size of the Array]

	for i in range(0,len(particle_array.poses)):
		xnoise = find_weighted_random(self.ODOM_TRANSLATION_NOISE)
		ynoise = find_weighted_random(self.ODOM_DRIFT_NOISE)
		znoise = find_weighted_random(self.ODOM_ROTATION_NOISE)

		new_particle = Odometry().pose.pose
		new_particle.position.x = initialpose.pose.pose.position.x + xnoise
		new_particle.position.y = initialpose.pose.pose.position.y + ynoise
		new_particle.orientation = rotateQuaternion(initialpose.pose.pose.orientation, znoise)
		particle_array.poses[i] = new_particle
                
                if self.DEBUG:
                       print 'X Noise = ' + str(xnoise) + '\tX = ' + str(new_particle.position.x) 
                       print 'Y Noise = ' + str(xnoise) + '\tX = ' + str(new_particle.position.y)  
                       print 'orientation = ' + str(new_particle.orientation)
        
        print("--------------------------------------------")
 
        listX = [float]*MAX_SAMPLE
        listY = [float]*MAX_SAMPLE
        #List_Orien = [float]*MAX_SAMPLE

        if self.DEBUG:
           for i in range(0,len(particle_array.poses)):
                 print(particle_array.poses[i].position.x)
                 listX[i] = particle_array.poses[i].position.x
                
                 print(particle_array.poses[i].position.y)
                 listY[i] = particle_array.poses[i].position.y
     
                 print(particle_array.poses[i].orientation.z)
                 #list_Orien[i] = particle_array.poses[i].orientation

           self.SortedX = sorted(listX, reverse=False)
           self.SortedY = sorted(listY, reverse=False)
           #self.Sorted_Orientation = sorted(particle_array.poses[i].orientation, reverse=False)
           #xxx = sorted(particle_array.poses[i].orientation, reverse=False)
           print("x=", self.SortedX)
           print("y=", self.SortedY)
           #print("Orientation=", xxx)

	return particle_array

 
    
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update
         """

	probs = []
	
	for i in range(0,len(self.particlecloud.poses)):
		probs.append(self.sensor_model.get_weight(scan,self.particlecloud.poses[i])**2)

	if sum(probs)/len(probs) < 28 and max(probs) < 50:

		for i in range(0,len(self.particlecloud.poses)):
			if probs[i] < 2.8:
				self.particlecloud.poses[i].position.x = gauss(self.estimatedpose.pose.pose.position.x,self.noise)
				self.particlecloud.poses[i].position.y = gauss(self.estimatedpose.pose.pose.position.y,self.noise)
				self.particlecloud.poses[i].orientation.z = 0.0
				self.particlecloud.poses[i].orientation.w = 1.0
				self.particlecloud.poses[i].orientation = rotateQuaternion(self.particlecloud.poses[i].orientation, math.pi*(random()*2 - 1))
		self.noise = self.noise + 1
	else:
		self.noise = 2.5
		new_particlecloud = PoseArray()
		for i in range(0,len(self.particlecloud.poses)):
			benchmark = random()*sum(probs)
			j = 0
			k = 0
			while j < benchmark:
				j = j + probs[k]
				k = k + 1
			chosenpose = self.particlecloud.poses[k-1]
			new_particlecloud.poses.append(Odometry().pose.pose)
			new_particlecloud.poses[i].position.x = gauss(chosenpose.position.x, 0.04)
			new_particlecloud.poses[i].position.y = gauss(chosenpose.position.y, 0.04) 
			new_particlecloud.poses[i].orientation = rotateQuaternion(chosenpose.orientation, gauss(0, 0.03))
		self.particlecloud = new_particlecloud



    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).
        
        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.
        
        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after 
        throwing away any which are outliers
        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """

        avgRangeX = 0.0
        avgRangeY = 0.0

        for i in range((MAX_SAMPLE/4),(MAX_SAMPLE/4) * 3):
             avgRangeX = avgRangeX + self.SortedX[i]
             avgRangeY = avgRangeY + self.SortedY[i]

        avgRangeX = avgRangeX / (MAX_SAMPLE/2)
        avgRangeY = avgRangeY / (MAX_SAMPLE/2)

        avgx = 0.0
        avgy = 0.0

	sinsum = 0.0
	cossum = 0.0

        count = 1
        for i in self.particlecloud.poses:
             if ((i.position.x <= avgRangeX or i.position.x <= avgRangeX) and (i.position.y <= avgRangeY or i.position.y <= avgRangeY)):
                  avgx = avgx + i.position.x
                  avgy = avgy + i.position.y
		  sinsum = sinsum + 2*i.orientation.z*i.orientation.w
		  cossum = cossum + i.orientation.w**2 - i.orientation.z**2
                  count = count + 1 
    
	avgx = avgx / count
        avgy = avgy / count

	estPose = Odometry().pose.pose
	estPose.position.x = avgx
	estPose.position.y = avgy



	estPose.orientation.w = 1.0
	estPose.orientation = rotateQuaternion(estPose.orientation,math.atan2(sinsum,cossum))

        return estPose
