#!/usr/bin/env python
import roslib; roslib.load_manifest('rfh_follow_me')
import rospy
from std_msgs.msg import String	
import tf
import math
from rfh_follow_me.msg import Distance
import numpy as np

global ctr

def receive(info):
	global ctr
	ctr = str(info)[6:]
	print ctr

if __name__ == '__main__':
	global ctr
	
	cn = ctr

	rospy.init_node('rfh_person_tracker')
	rospy.Subscriber('followme', String, receive)

	listener = tf.TransformListener()

	rate = rospy.Rate(10.0)

	rospy.spin()
	
	if cn == '1':
		while not rospy.is_shutdown():
			dist = Distance()
			try:
				# dist.is_calibrated = listener.canTransform('/openni', '/torso_1', rospy.Time())
				# if dist.is_calibrated:
				# 	(trans, rot) = listener.lookupTransform('/openni', '/torso_1', rospy.Time())
				# 	dist.x = trans[0]
				# 	dist.y = trans[1]
				# 	dist.z = trans[2]
				# 	dist.distance = np.linalg.norm(trans)
				listener.waitForTransform("/openni", "/torso_1", rospy.Time(0), rospy.Duration(4.0))
				(trans, rot) = listener.lookupTransform('/openni', '/torso_1', rospy.Time(0))
				dist.x = trans[0]
				dist.y = trans[1]
				dist.z = trans[2]
				dist.distance = np.linalg.norm(trans)
				
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
				print e
				continue

			dist_pub = rospy.Publisher('/distance_person', Distance, queue_size=10)
			dist.header.stamp = rospy.get_rostime()
			dist_pub.publish(dist)
		
			rate.sleep()