#!/usr/bin/env python
import cv2
import rospy
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import numpy as np
import math

class chase_ball():
    def __init__(self):
        #variable
        self.max_linear = 1
        self.max_angular = 1
        self.x = 0
        self.y = 0
        self.area = 400
        self._hz = rospy.get_param('~hz',50)
        self._running = True
        self._receive = False
        #subscribe
        rospy.Subscriber('/vrep/image',Image,self.callback,queue_size=1)
        #publish
        self._pub = rospy.Publisher('/vrep/cmd_vel',Twist,queue_size=1)
        #bridge
        self.bridge = CvBridge()
    def callback(self,data):
        self._receive = True
        img = self.bridge.imgmsg_to_cv2(data,"bgr8")
        hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        yellow_min = np.array([26,43,46])
        yellow_max = np.array([34,255,255])
        masked_img = cv2.inRange(hsv_img,yellow_min,yellow_max)
        blurry_img = cv2.GaussianBlur(masked_img,(11,11),0)
        binary_img = cv2.Canny(blurry_img,20,160)
        _, cnts, _ = cv2.findContours(binary_img.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        if cnts != []:
            max_area = 0
            max_cnt = cnts[0]
            for cnt in cnts:
                _,_,w,h = cv2.boundingRect(cnt)
                if w*h > max_area:
                    max_cnt = cnt
                    max_area = w*h
            x, y, w, h = cv2.boundingRect(max_cnt)
            print("max_cnt")
            print(x+w/2,y,w,h)
            self.x = x+w/2
            self.y = y
            self.area = w*h
        else:
            self.area = 0
    def get_ball_pos(self):
        return self.x,self.y,self.area
    def run(self):
        rate = rospy.Rate(self._hz)
        while self._running:
            if self.area > 399:
                if self._receive == True:
                    self._receive = False
                    self.pid(250)
            else:
                self._running = False
                print("chaser lost ball")
                twist = Twist()
                twist.linear.x = 0
                twist.angular.z = 0
                self._pub.publish(twist)
            rate.sleep()
    def pid(self,r):
        area = r*r
        err_x = 256.0 - float(self.x)
        err_a = float(area) - float(self.area)
        print("error")
        print(err_x,err_a)
        ex = 0
        if err_x >= 0:
            ex = math.sqrt(err_x)
        else:
            ex = -math.sqrt(-err_x)
        ea = 0
        if err_a >= 0:
            ea = math.sqrt(err_a)
        else:
            ea = -math.sqrt(-err_a)
        linear = ea * 0.005
        if linear > self.max_linear:
            linear = self.max_linear
        if linear < -self.max_linear:
            linear = -self.max_linear
        angular = ex * 0.04
        if angular > self.max_angular:
            angular = self.max_angular
        if angular < -self.max_angular:
            angular = -self.max_angular
        print("chasing at")
        print(linear,angular)
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self._pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('chase_ball')
    app = chase_ball()
    app.run()
