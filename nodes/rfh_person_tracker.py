#!/usr/bin/env python
import roslib; roslib.load_manifest('rfh_follow_me')
import rospy
import tf
from rfh_follow_me.msg import Distance
import numpy as np

if __name__ == '__main__':
	rospy.init_node('rfh_person_tracker')

	listener = tf.TransformListener()

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		dist = Distance()
		try:
			if listener.canTransform('/openni', '/torso_1', rospy.Time(0)):
				(trans, rot) = listener.lookupTransform('/openni', '/torso_1', rospy.Time(0))
				dist.distance = np.linalg.norm(trans)

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
			print e
			continue

		dist_pub = rospy.Publisher('/distance_person', Distance, queue_size=10)
		dist.header.stamp = rospy.get_rostime()
		dist_pub.publish(dist)
		
		rate.sleep()