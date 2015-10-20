#!/usr/bin/env python
import roslib; roslib.load_manifest('rfh_follow_me')
import sys
import rospy
import cv
import cv2
import cv2.cv
import message_filters
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from p2os_msgs.msg import MotorState
from rfh_follow_me.msg import Distance
from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime

class follow_person:
	def __init__(self):
		self.bridge = CvBridge()
		
		image_sub = message_filters.Subscriber("camera/rgb/image_color", Image)
		dist_sub = message_filters.Subscriber("distance_person", Distance)
		
		self.ts = message_filters.ApproximateTimeSynchronizer([image_sub, dist_sub], 10, 0.5)
		self.ts.registerCallback(self.callback)
		
		self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
		self.motor_state_pub = rospy.Publisher("/cmd_motor_state", MotorState, queue_size=10)
		
		
	def callback(self, rgb_data, dist_data):
		try:
			image = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
		except CvBridgeError, e:
			print e

		(rows, cols, channels) = image.shape
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		px = cols/2
		py = rows/2

		h_high = 130
		h_low = 75
		s_high = 255
		s_low = 130
		v_high = 255
		v_low = 0

		lower = np.array([h_low,s_low,v_low])
		upper = np.array([h_high,s_high,v_high])
		
		# Threshold the HSV image to get only orange colors
		mask = cv2.inRange(hsv, lower, upper)
		
		#erosion
		kernel = np.ones((5,5),np.uint8)
		erosion = cv2.erode(mask,kernel,iterations=3)
		
		#dilation
		dilation = cv2.dilate(erosion,kernel,iterations=3)

		img = cv2.medianBlur(dilation,21)
		moment = cv2.moments(img)
		try:
			px = int(moment['m10']/moment['m00'])
			py = int(moment['m01']/moment['m00'])
			area = moment['m00']
		except ZeroDivisionError:
			pass

		cv2.circle(img, (px, py), 3, (171,110,0), 2)
		cv2.circle(img, (px, py), 3, (171,110,0), 2)

		cv2.imshow('Color Segmentation', img)

		vel = Twist()
		state = MotorState()
		state.state = 4

		if dist_data.distance > 0.7 and dist_data.distance < 2: 
			self.motor_state_pub.publish(state)
			#rospy.loginfo('lim-left: {0} < px: {1} < lim-right: {2}'.format((cols/3), px, (cols - (cols/3))))
			if ((cols/3) > px and px > 0) or (dist_data.y > 0.2):
				#Goes to the left
				rospy.loginfo('left <<<<<<<<')
				vel.linear.x = 0.3
				vel.angular.z = 0.2
			elif ((cols - (cols/3)) < px and px < cols) or (dist_data.y < -0.2):
				#Goes to the right
				rospy.loginfo('right >>>>>>>>')
				vel.linear.x = 0.3
				vel.angular.z = -0.2
			else:
				rospy.loginfo('Distancia: %.4f' % dist_data.distance)
				#Foward
				vel.linear.x = 0.3
				vel.angular.z = 0.0
			
			self.cmd_vel_pub.publish(vel)
		else:
			vel.linear.x = 0.0
			vel.angular.z = 0.0
			self.cmd_vel_pub.publish(vel)

		cv2.waitKey(3)

def main(args):
	fp = follow_person()
	rospy.init_node('follow_person', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)