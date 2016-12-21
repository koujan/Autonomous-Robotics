#! /usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import roslib
import actionlib

import tbug.msg

robotpos = Odometry();

def robotposcheck(updatedState):
	global robotpos
	robotpos = updatedState



class GoalServer(object):

	global robotpos


	_feedback = tbug.msg.goalStatusFeedback()
	_result =tbug.msg.goalStatusResult()

	def __init__(self):
		self._as = actionlib.SimpleActionServer('check_goals',tbug.msg.goalStatusAction, execute_cb=self.execute_cb, auto_start=False)
		self._as.start()

	def execute_cb(self, goal):
		
		robot1goalReached = 0

		r = rospy.Rate(1)

		

		if (math.sqrt((goal.x-robotpos.pose.pose.position.x)**2 + (goal.y-robotpos.pose.pose.position.y)**2)<0.5):
			robot0goalReached = 1

		

		self._feedback.x=robotpos.pose.pose.position.x
		self._feedback.y=robotpos.pose.pose.position.y

		self._as.publish_feedback(self._feedback)

		r.sleep()

		if (robot1goalReached):
			self._result.robot1_thereOrNot=1
		else:
			self._result.robot1_thereOrNot=0

		
		self._as.set_succeeded(self._result)

if __name__ == '__main__':
	rospy.init_node('goal1_server')

	rospy.Subscriber("robot_1/base_pose_ground_truth",Odometry,robotposcheck)

	GoalServer()
	rospy.spin()
