#!/usr/bin/env python
import roslib; roslib.load_manifest('rfh_follow_me')
import sys
import rospy
import cv2
import cv2.cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime

class face_recognition:

	def __init__(self):
		cv2.namedWindow("Image Window", 1)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("camera/rgb/image_color", Image, self.callback)
		self.face_cascade = cv2.CascadeClassifier('/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_default.xml')

	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError, e:
			print e

		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		roi_gray = None

		faces = self.face_cascade.detectMultiScale(
			gray,
			scaleFactor=1.3,
			minNeighbors=5,
			minSize=(30,30),
			flags=cv2.cv.CV_HAAR_SCALE_IMAGE
		)

		for (x, y, w, h) in faces:
			x1 = x  + int(w*.1)
			x2 = x1 + int(w*.8)

			y1 = y  + int(h*.2)
			y2 = y1 + int(h*.8)

			roi_gray = gray[y1:y2, x1:x2]
			cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0,255,0), 2)
			cv2.circle(cv_image, (x+(w/2), y+(h/2)), 3, (171,110,0), 2)

		cv2.imshow('Video', cv_image)
		cv2.imwrite('faces/crop_face' + str(datetime.now().time()) +'.png', roi_gray)
		cv2.waitKey(3)

def main(args):
	fr = face_recognition()
	rospy.init_node('face_recognition', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
