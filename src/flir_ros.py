#!/usr/bin/python3

import numpy as np
import cv2
import rospy 
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time

bridge = CvBridge()

cap = cv2.VideoCapture(0)
rospy.init_node('lepton_driver', anonymous=True)

image_pub = rospy.Publisher("/flir_lepton/image", Image, queue_size=1)

while(True and not rospy.is_shutdown()):
    time.sleep(0.1)
    # Capture frame-by-frame
    ret, frame = cap.read()

    frame = frame[:, :, 0]
    frame.astype('int')

    gray = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    print(gray.shape)

    image_message = bridge.cv2_to_imgmsg(frame, encoding="passthrough")

    image_pub.publish(image_message)

    # Our operations on the frame come here

    # Display the resulting frame
    # cv2.imshow('frame',frame)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
