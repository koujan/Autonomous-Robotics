#!/usr/bin/env python

# ROS general imports
import roslib
roslib.load_manifest('ar_proj')
import rospy

# Other imports
import math #math library
import numpy as np #numpy library
from probabilistic_lib.functions import angle_wrap #Normalize angles between -pi and pi
from RRT_pyth import RRT
from RRT_smooth import smooth
import tf

#ROS messages
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

class driver(object):
    
    def __init__(self):
        '''
        constructor
        '''
        #Initialize ros node
        rospy.init_node('RRT_driver')        
        
        #Initialize goals
        self.x = np.array([])
        self.y = np.array([])
        self.theta = np.array([])
        #Threshold for distance to goal
        self.goal_th_xy = rospy.get_param('goal_thershold_xy',0.1) #Position threshold
        self.goal_th_ang = rospy.get_param('goal_threshold_ang',0.01) #Orientation threshold
        #self.RRT_goals=RRT()
        self.active_goal = 0
        
        #define subscriber
        
        self.sub =rospy.Subscriber('/odom', Odometry, self.callback)

        #define publisher        
       
	self.pub=rospy.Publisher('/cmd_vel',Twist,queue_size=15)
        
	#define the velocity message
           
        self.vmsg=Twist()   

	#Has the goal been loaded
        self.params_loaded = False            

    def print_goals(self):
        '''
        print_goals prints the list of goals
        '''
        rospy.loginfo("List of goals:")
        rospy.loginfo("X:\t " + str(self.x))
        rospy.loginfo("Y:\t " + str(self.y))
        rospy.loginfo("Theta:\t " + str(self.theta))
    
    def print_goal(self):
        '''
        print_goal prints the next goal
        '''
        rospy.loginfo( "Goal: (" + str(self.x[self.active_goal]) + " , " + str(self.y[self.active_goal]) + " , " + str(self.theta[self.active_goal]) + ")")
        
    def print_pose(self):
        '''
        print_pose pints the robot's actuall position
        '''
        rospy.loginfo( "Pose: (" + str(self.position_x) + " , " + str(self.position_y) + " , " + str(self.position_theta) + " )")
        
    def print_goal_and_pose(self):
        '''
        pritn_goal_and_pose prints both goal and pose
        '''
        rospy.loginfo("\tPose\t\tGoal")
        rospy.loginfo("X:\t%f\t%f",self.position_x,self.x[self.active_goal])
        rospy.loginfo("Y:\t%f\t%f",self.position_y,self.y[self.active_goal])
        rospy.loginfo("A:\t%f\t%f",self.position_theta,self.theta[self.active_goal])
        
    def dist_to_goal_xy(self):
        '''
        dist_to_goal_xy computes the distance in x and y direction to the 
        active goal
        '''
        return math.sqrt(pow(self.position_x-self.x[self.active_goal],2)+pow(self.position_y-self.y[self.active_goal],2))
    
    def dist_to_goal_ang(self):
        '''
        dist_to_goal_ang computes the orientation distance to the active
        goal
        '''
        return np.abs(angle_wrap(self.theta[self.active_goal]-self.position_theta))
        
    def has_arrived_xy(self):
        '''
        has_arrived_xy returns true if the xy distance to the ative goal is
        smaller than the position threshold
        '''
        return self.dist_to_goal_xy()<self.goal_th_xy
        
    def has_arrived_ang(self):
        '''
        has_arrived_ang returns true if the orientation distance to the 
        ative goal is smaller than the orientation threshold
        '''
        return self.dist_to_goal_ang()<self.goal_th_ang
        
    def has_arrived(self):
        '''
        has_arrived return true if the robot is closer than the apropiate 
        threshold to the goal in position and orientation
        '''
        return (self.has_arrived_xy() and self.has_arrived_ang())   
    
    def check_goal(self):
        '''
        check_goal checks if the robot has arrived to the active goal, 
        '''
        if self.has_arrived():
            self.next_goal()
        
    def publish(self):
        '''
        publish: publishes the velocity message in vmsg
        '''
        self.pub.publish(self.vmsg)
        
    def callback(self,msg):
        '''
        callback reads the actuall position of the robot, computes the 
        appropiate velocity, publishes it and check if the goal is reached
        '''
        self.read_position(msg)
        self.compute_velocity()
        self.publish()
        self.check_goal()
        
    def drive(self):
        '''
        drive is a needed function for the ros to run untill somebody stops
        the driver
        '''
        #self.print_goal()
        while not rospy.is_shutdown():
            rospy.sleep(0.03)
        
    def load_goals(self):
        '''
        load_goals loads the goal (or goal list for the optional part) into
        the x y theta variables.
        
        '''
	goal_x = rospy.get_param('x',200)+425
	goal_y = -rospy.get_param('y',200)+375
	RRT_goals=RRT(np.array([goal_x,goal_y]))
	#print RRT_goals[:,:]*np.array([1,-1])+np.array([-425,375])
	RRT_goals=smooth(RRT_goals,1)	
	self.x = np.append( self.x,RRT_goals[1:,0]-425)
	self.y = np.append( self.y,-RRT_goals[1:,1]+375)
	print 'self.x'
	print self.x
	print 'self.y'
	print self.y
	self.theta = np.zeros((len(self.x),1))
	self.params_loaded = True
	#self.print_goals()
        
    def next_goal(self):
        '''
        next_goal increments the index of the goal in 1 and checks whether
        or not the robot has reached the final goal
      
        '''
	self.active_goal=self.active_goal+1
	if(self.active_goal>len(self.x)-1):
		rospy.signal_shutdown('Final goal reached!')
        
    def read_position(self,msg):
        '''
        read_position copy the position received in the message (msg) to the
        internal class variables self.position_x, self.position_y and 
        self.position_theta
     
        '''
	(b,a,self.position_theta) =tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.position_x=msg.pose.pose.position.x
	self.position_y=msg.pose.pose.position.y

    def compute_velocity(self):
        '''
        compute_velocity computes the velocity which will be published.

        '''    	
	self.vmsg.angular.z=math.atan2(self.y[self.active_goal]-self.position_y,self.x[self.active_goal]-self.position_x)-self.position_theta
	self.vmsg.angular.z=angle_wrap(self.vmsg.angular.z)
	if(abs(self.vmsg.angular.z)>math.pi/12): # in order not to enter an infinte loop of rotations
		 self.vmsg.linear.x=0.3
	else:
		self.vmsg.linear.x=0.5*math.sqrt(pow(self.position_x-self.x[self.active_goal],2)+pow(self.position_y-self.y[self.active_goal],2))
	if(self.has_arrived_xy()):
		self.vmsg.linear.x=0
		self.vmsg.angular.z=angle_wrap(self.theta[self.active_goal]-self.position_theta)












