#!/usr/bin/env python
# coding: UTF-8
import rospy
import numpy as np
from geometry_msgs.msg import Quaternion,PointStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import PositionTarget
from simple_pid import PID

use_qrcode = True
states = {"TRACKING": "t", "HOVERING": "h", "IDLE": "i", "CONVERTING": "c"}
camera_pitch = np.pi * 33 / 180

# TODO：何时进行current值的初始化以避免循环错误？
class Ctrl_(object):
    hover_altitude = None
    current_odom = None # Odometry
    current_target = None
    current_target_position = None
    current_ctrl_msg = None
    current_state = ''

    pid_feedback_x = PID(1., 0., 0.)
    pid_feedback_y = PID(0., 0., 0.)
    pid_feedback_z = None
    pid_feedback_yaw = PID(1., 0., 0.)

    def __init__(self,use_qrcode):
        rospy.init_node('track_ctrl_node', anonymous=False)
        if use_qrcode:
            rospy.Subscriber('<target_topic>', Quaternion, self.qrcode_cb)
        else:
            rospy.Subscriber('<target_topic>', Quaternion, self.target_cb)


        self.ctrl_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=0)
        pass

    def go(self):
        rospy.spin()
        pass

    def target_cb(self, msg):
        """
        由检测跟踪算法发布，频率不定，可能掉帧
        """
        self.current_target = [msg.x, msg.y, msg.z, msg.w]
        if not self.current_target_position:
            self.current_target_position = PointStamped()

        self.current_target_position.header.stamp = rospy.Time.now()
        # TODO: x,y,w,h求解距离及偏差
        self.current_target_position.point.x =
        self.current_target_position.point.y =
        pass

    def qrcode_cb(self,msg):
        """
        目标来源为二维码识别
        """

    def odom_cb(self, msg):
        """
        随着T265(200hz)或GPS更新，频率稳定
        """
        self.current_odom = msg
        if self.current_state == states["IDLE"]: # IDLE 模式下高度浮动，切入OFFBOARD模式后不再更新
            self.hover_altitude = msg.pose.pose.position.z

    def ctrl_step(self, timeEvent):
        """
        以Timer形式定频发布，每一帧都必须发布，否则掉帧超过一定时长fcu自动退出offboard模式
        - IDLE模式：vx,vy = 0, z = hover_altitude 此时fcu无控制响应，但如果不发则fcu无法切入OFFBOARD模式；
        为了确保切入之后飞机不发生突发运动，速度期望应当为0，悬停高度为当前高度
        -------------
        - TRACKING模式：vx,vy = pid(dx,dy) z = hover_altitude
        - HOVERING模式：vx,vy = 0，z = hover_altitude
        - 过渡模式？
        """
        ctrl_msg = PositionTarget()
        ctrl_msg.header.stamp = rospy.Time.now()
        ctrl_msg.coordinate_frame = PositionTarget.FRAME_BODY_NED()
        ctrl_msg.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_VZ | \
                             PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | \
                             PositionTarget.IGNORE_YAW_RATE

        if self.current_state == states["IDLE"]:
            self.hover_altitude = self.current_odom.pose.pose.position.z
            ctrl_msg.position.z = self.hover_altitude
            ctrl_msg.velocity.x = 0
            ctrl_msg.velocity.y = 0

        if self.current_state == states["HOVERING"]:
            ctrl_msg.position.z = self.hover_altitude
            ctrl_msg.velocity.x = 0
            ctrl_msg.velocity.y = 0
            pass

        if self.current_state == states["TRACKING"]:
            """求解过程在target_cb环中完成"""
            vx,vy,vyaw = self.feed_back(dx = self.current_target_position.point.x,
                                        dy = self.current_target_position.point.y,
                                        dyaw = )
            pass


        self.ctrl_pub.publish(ctrl_msg)

    def feed_back(self, dx=None, dy=None, dz=None, dyaw=None):
        return [y(x) for x, y
                in zip([dx, dy, dz, dyaw],
                       [self.pid_feedback_x, self.pid_feedback_y, self.pid_feedback_z, self.pid_feedback_yaw])
                if x is not None]


if __name__ == '__main__':
    pass
