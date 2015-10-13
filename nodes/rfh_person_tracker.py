#!/usr/bin/env python
import roslib; roslib.load_manifest('rfh_follow_me')
import rospy
import tf
import geometry_msgs.msg
from p2os_msgs.msg import MotorState
import numpy as np

if __name__ == '__main__':
	rospy.init_node('rfh_person_tracker')

	listener = tf.TransformListener()

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			(trans, rot) = listener.lookupTransform('/openni', '/torso_1', rospy.Time(0))

			dist = np.linalg.norm(trans)

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
			print e
			continue

		cmd_vel = rospy.Publisher("/cmd_vel", geometry_msgs.msg.Twist, queue_size=10)
		motor_state = rospy.Publisher("/cmd_motor_state", MotorState, queue_size=10)
		vel = geometry_msgs.msg.Twist()
		state = MotorState()
		state.state = 4

		if dist > 0.7 and dist < 2:
			vel.linear.x = 0.1
			vel.angular.z = 0.0
			motor_state.publish(state)
			cmd_vel.publish(vel)

		rate.sleep()