import cv_bridge
import cv2 as cv
import rospy
from sensor_msgs.msg import Image,CompressedImage
from pyzbar import pyzbar

class T:
    cnt_ = 0
    def __init__(self):
        rospy.init_node("node_x", anonymous=False)
        cnt_ = 0

    def go(self):
        # rospy.Subscriber("/usb_cam/image_raw",Image,image_cv)
        rospy.Subscriber("/d400/color/image_raw", Image, self.image_cv)
        rospy.spin()

    def image_cv(self,msg):
        # print "Cyka"
        self.cnt_ = self.cnt_ +1
        print self.cnt_
        cvb = cv_bridge.CvBridge()
        img = cvb.imgmsg_to_cv2(msg)
        res = pyzbar.decode(img)
        print res
        img = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
        # cv.blur(img_,img_,cv.Size(3,3))
        # cv.equalizeHist(img_,img_)
        # ret, thresh1 = cv.threshold(img, 127, 255, cv.THRESH_BINARY)
        cv.imshow("SP",img)
        if self.cnt_ % 100 == 0:
            cv.imwrite("RES/img_save"+str(self.cnt_ // 100)+".png",img)
        # qrc = cv.QRCodeDetector()
        # a, b, c = qrc.detectAndDecode(img)
        cv.waitKey(1)
        #
        # print a
        # print b
        # print c

if __name__ == '__main__':
    t = T()
    t.go()

