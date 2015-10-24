#!/usr/bin/env python
import roslib; roslib.load_manifest('rfh_follow_me')
import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from p2os_msgs.msg import MotorState, SonarArray

class sonar_verification:
	def __init__(self):
		self.sonar_sub = rospy.Subscriber("/sonar", SonarArray, self.callback)
		self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

	def callback(self, data):
		sonar_array = list(data.ranges)
		vel = Twist()

		if max(sonar_array[8:16]) > 0.4:
			vel.linear.x = 0.0
			vel.angular.z = 0.0
		else:
			if max(sonar_array[0:3]) > 0.4:
				vel.linear.x = 0.0
				vel.angular.z = 0.1
			elif max(sonar_array[3:5]) > 0.4:
				vel.linear.x = 0.0
				vel.angular.z = 0.0
			elif max(sonar_array[5:8]) > 0.4:
				vel.linear.x = 0.0
				vel.angular.z = -0.1
			else:
				pass

		self.vel_pub.publish(vel)



def main(args):
	sv = sonar_verification()
	rospy.init_node('sonar_verification', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"
	
if __name__ == '__main__':
	main(sys.argv)