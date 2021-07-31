import numpy as np

import  rospy
import cv2
from sensor_msgs.msg import CompressedImage

class Cyka:
    def __init__(self,name):
        rospy.init_node(name,anonymous=False)
        self.cmpimg_sub = rospy.Subscriber("/d400/color/image_raw/compressed",CompressedImage, self.img_cb, queue_size=1)

    def img_cb(self,msg):
        np_arr = np.fromstring(msg.data,np.uint8)
        img_np = cv2.imdecode(np_arr,cv2.IMREAD_COLOR)

        cv2.imshow('cv_img', img_np)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    cyka = Cyka("cyka_node")
    cyka.run()