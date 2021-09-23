#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import cv2
import os
import sys
import glob
import numpy as np
import math
import time
import socket
import json
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from std_msgs.msg import Int32

ExpectedSpeed = 0
ExpectedSteer = 0
CurrentSteer = 0
cmdSteer = 0
DirectionError = 0 
last_DirectionError = 0
DirectionError_Sum = 0
DirectionError_Diff = 0
R = 1
flag_lane = 0  # 0二道直线  1 一道右线  2一道左线  3 虚线
last_flag_lane = 0
flag_remain = 0
last_Steer = 0
t = 0

flag_scene = 0
last_midlane_RT = 0
last_R = 0

def scanInfo_callback(msg):
    global flag_scene
    flag_scene = msg.data

def constrain(min, max, input):
    if input <= min:
        output = min
    elif input >= max:
        output = max
    else:
        output = input
    return output

def remap(min1, max1, min2, max2, value):
    value = min1 + (value - min2) * (max1 - min1)/(max2 - min2)
    return value

def Steer_PID_update():
    global ExpectedSteer, CurrentSteer, cmdSteer
    global DirectionError, last_DirectionError, DirectionError_Sum, DirectionError_Diff
    global R
    global last_lane
    global lane_now
    global flag_remain
    global last_Steer
    flag_remain = 0
    #if R == 10000: # 弯道
    #    k_p = 0.035 # 0.04
    #    k_i = 0.001 #0
    #    k_d = 0.1 # 0.3
    #elif R == 0:
    #    k_p = 0.035 # 直道 0.018
    #    k_i = 0.001 #0
    #    k_d = 0.1  # 0.05
    #else:
    #print('no RRRRRRR')
    k_p = 0.04
    k_i = 0.001
    k_d = 0.1
    #k_p = 0.04 # 0.075
    #k_i = 0
    #k_d = 0.1 # 0.3
    DirectionError_Sum = DirectionError_Sum * 0.1 + DirectionError
    DirectionError_Diff = DirectionError - last_DirectionError
    last_DirectionError = DirectionError

    cmdSteer = k_p * DirectionError + k_i * DirectionError_Sum + k_d * DirectionError_Diff
    # last_Steer = cmdSteer

    cmdSteer = constrain(-20, 20, cmdSteer)
    # cmdSteer = remap(-20.0, 20.0, -300.0, 300.0, float(cmdSteer))

# 相机参数
mtx = np.array([[1.15777930e+03, 0, 6.67111054e+02], [0, 1.15282291e+03, 3.86128937e+02], [0, 0, 1]])
dist = np.array([[-0.24688775, -0.02373133, -0.00109842, 0.00035108, -0.00258571]])
map1 = 0
map2 = 0
def labBSelect(img, thresh=(175, 255)):
    # 1) Convert to LAB color space
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2Lab)
    lab_b = lab[:, :, 2]
    # don't normalize if there are no yellows in the image
    if np.max(lab_b) > 100:
        lab_b = lab_b * (255 / np.max(lab_b))
    # 2) Apply a threshold to the L channel
    binary_output = np.zeros_like(lab_b)
    binary_output[((lab_b > thresh[0]) & (lab_b <= thresh[1]))] = 1
    # 3) Return a binary image of threshold result
    return binary_output

#from numba import jit
#@jit(nopython=True)
def for_loop(fitx, fity, warp_zero):
    for i in range(len(fitx)):
        x = fitx[i]
        y = fity[i]
        warp_zero[y][x][0] = 255

def drawing(undist, fitx, fity, Minv):
    warp_zero = np.zeros_like(undist).astype(np.uint8)
    for_loop(fitx.astype(np.int), fity.astype(np.int), warp_zero)
    newwarp = cv2.warpPerspective(warp_zero, Minv, (undist.shape[1], undist.shape[0]))

    # Combine the result with the original image
    # result = cv2.addWeighted(undist, 1, newwarp, 0.3, 0)
    result = newwarp
    return result

def init_undistort():
    frame_size=(1280,720)
    map1, map2=cv2.initUndistortRectifyMap(mtx, dist, None, mtx, frame_size, cv2.CV_32FC1)
    return map1, map2
map1, map2 = init_undistort()


def undistortImage_fast(orig_img, map1, map2):
    dst = cv2.remap(orig_img, map1, map2, cv2.INTER_LINEAR, cv2.BORDER_REFLECT_101)
    return dst

class camera:
    def __init__(self, frame):
        self.frame = frame
        # 发布ros话题
        # self.imagePub = rospy.Publisher('images', Image, queue_size=1)
        self.cmdPub = rospy.Publisher('/lane_vel', Twist, queue_size=1)
        self.cam_cmd = Twist()
        self.cvb = CvBridge()

        self.aP = [0.0, 0.0]
        self.lastP = [0.0, 0.0]
        self.Timer = 0
        self.angularScale = 6
        self.abc = 0
        self.camMat = np.array([[1.15777930e+03, 0, 6.67111054e+02], [0, 1.15282291e+03, 3.86128937e+02], [0, 0, 1]])
        self.camDistortion = np.array([[-0.24688775, -0.02373133, -0.00109842, 0.00035108, -0.00258571]])
        self.queue = []  # 自己控制长度，取10个
    def __del__(self):
        return

    def spin(self):
        global ExpectedSteer, CurrentSteer, cmdSteer
        global DirectionError, last_DirectionError, DirectionError_Sum, DirectionError_Diff
        global R
        global last_lane
        global lane_now
        global flag_lane
        global last_flag_lane
        global flag_scene
        global t
        global last_R
        t += 1
        #print('t:', t)
        global ExpectedSpeed

        ret = True
        img = self.frame
        # 改尺寸的resize预处理
        img = cv2.resize(img, (1280, 720), interpolation=cv2.INTER_AREA)
        if ret == True:
            t0 = time.time()
            # undstrt = cv2.undistort(img, self.camMat, self.camDistortion, None, self.camMat)
            undstrt = undistortImage_fast(img, map1, map2)
            # print('undstrt_time:', time.time()-t0)
            # 适用于RK3399的标定参数
            # src_points = np.array([[430, 430], [780, 430], [1280, 620], [0, 550]], dtype="float32")
            # # 左上，右上，右下，左下
            # dst_points = np.array([[300, 0], [950, 0], [950, 720], [300, 720]], dtype="float32")
            # 适用于Hilens的标定参数
            src_points = np.array([[436, 591], [800, 590], [1049, 718], [225, 718]], dtype="float32")
            # 左上，右上，右下，左下
            dst_points = np.array([[300, 0], [950, 0], [950, 720], [300, 720]], dtype="float32")
            M = cv2.getPerspectiveTransform(src_points, dst_points)
            img_warped = cv2.warpPerspective(undstrt, M, (1280, 720), cv2.INTER_LINEAR)
            # cv2.imshow('img_warped', img_warped)
            binary_warped = labBSelect(img_warped)
            # cv2.imshow('binary_warped', binary_warped*255)
            # cv2.waitKey(0)

            base_detect_graph = binary_warped[-50:, :]  # 先行后列，取出后50行
            #             print(base_detect_graph.shape)
            new_hist_x = np.sum(base_detect_graph[:, :], axis=0)  # axis指定往哪里压...axis=0是往下压
            #             print(np.sum(new_hist_x))
            #             print(np.min(new_hist_x))   # argmax是返回下标
            list_ = []
            lane_base_ = []
            last_unit = 0
            lane_base_LT = 0
            lane_base_RT = 0
            flag_start = 0

            # t11 = time.time()
            for i in range(len(new_hist_x)):
                if new_hist_x[i] > 35:
                    if last_unit == 0:
                        list_.append(i)
                        flag_start = 1
                    else:
                        list_.append(i)
                    last_unit = 1
                else:
                    last_unit = 0
                    if len(list_) > 15 and flag_start == 1:
                        lane_base_.append(int(np.average(list_)))
                        list_ = []
                    flag_start = 0
            t22 = time.time()
            # print("tiiiime:", t22-t11)
            #           print(lane_base_)
            if len(lane_base_) == 1:  # 如果只有一条线，先默认这条线为右线，后续继续判断
                lane_base_LT = 0
                lane_base_RT = lane_base_[0]
            elif len(lane_base_) == 2:
                lane_base_LT = lane_base_[0]
                lane_base_RT = lane_base_[1]
            else:
                pass

            lane_num = len(lane_base_)
            #print('lane_base_',lane_base_)
            # print('lane_num:',lane_num)
###################################################################
            if flag_scene == 0 or flag_scene == 2:  # flag_scene == 2时只需要降低速度
                apx = 700
                apy = 450
                midlane_LT = 0
                midlane_RT = 0
                #print ('lane_num:', lane_num)
                if lane_num == 2:
                    flag_lane = 0
                    # print("baseLT,baseRT:", lane_base_LT, lane_base_RT)
                    if lane_base_LT > 600 and lane_base_RT > 700:  # 人行道干扰保护
                        DirectionError = last_DirectionError
                        print('protection:', DirectionError)
                    else:
                        midlane_LT = lane_base_LT
                        midlane_RT = lane_base_RT
                        DirectionError = int((midlane_LT + midlane_RT) / 2) - apx
                        print('2 lines Error:', DirectionError)
                    #print("lane_NUM2")
                elif lane_num == 1:
                    # lane_base_RT 表征视野中的一根线
                    if lane_base_RT < 500:  # 理想的一道左线
                        if last_flag_lane == 1:  # 如果上一帧是右线，则认为此时仍然是右线
                            flag_lane = 1
                            midlane_LT = 0
                            midlane_RT = lane_base_RT
                            DirectionError = int((midlane_LT + midlane_RT) / 2) - apx
                            print('1 RIGHT protected Error:', DirectionError)
                        else:  # 一根左线
                            midlane_LT = lane_base_RT
                            midlane_RT = 1280
                            DirectionError = int((midlane_LT+midlane_RT) / 2) - apx  # 直道，左侧，一根线,1100
                            print('1 LEFT Error:', DirectionError)
                            flag_lane = 2
                    else:  # 一根右线
                        if last_flag_lane == 2:  # 左线
                            flag_lane = 2
                            midlane_LT = lane_base_RT
                            midlane_RT = 1280
                            DirectionError = int((midlane_LT+midlane_RT) / 2) - apx
                            print('1 LEFT protected Error:', DirectionError)
                        else:
                            midlane_LT = 0
                            midlane_RT = lane_base_RT
                            DirectionError = int((midlane_LT + midlane_RT) / 2) - apx  # 右侧一根线， midlane_LT = 0其实
                            print('1 RIGHT, Error:', DirectionError)
                            flag_lane = 1
                else:
                    print("no lane_num")
                    midlane_LT = 0
                    midlane_RT = 1280

              #  print('midlane_LT, midlane_RT',midlane_LT,midlane_RT)
                last_flag_lane = flag_lane
              #  print('flag_lane:', flag_lane)
    ################################  R #################################
                apy_r1 = 450
                apy_r2 = 550
                apy_r3 = 680
                lane_r1 = 0
                lane_r2 = 0
                lane_r3 = 0
                if lane_num == 2:  # 瞄右线
                   # print("xxxxxxxxxxxxxxxxxx")
                    lane_r1 = binary_warped[apy_r1][550:]
                    lane_r2 = binary_warped[apy_r2][550:]
                    lane_r3 = binary_warped[apy_r3][550:]
                elif lane_num == 1:
                 #   print("yyyyyyyyyyyyyyyyyy")
                    lane_r1 = binary_warped[apy_r1][:]
                    lane_r2 = binary_warped[apy_r2][:]
                    lane_r3 = binary_warped[apy_r3][:]

                apx_r1 = np.mean(np.where(lane_r1))
                apx_r2 = np.mean(np.where(lane_r2))
                apx_r3 = np.mean(np.where(lane_r3))
                #print("x1,x2,x3:",apx_r1, apx_r2, apx_r3)
                k1 = float((apx_r1 - apx_r2)/(apy_r1 - apy_r2))
                k2 = float((apx_r1 - apx_r3)/(apy_r1 - apy_r3))
                k3 = float((apx_r3 - apx_r2)/(apy_r3 - apy_r2))
                RR = (k1 - k2)**2 + (k1 - k3)**2 + (k2 - k3)**2

                if RR < 0.0001:
                    RR = 0
                last_R = RR
                #print('RR is :',RR)
    ######################################################################
                #print('AIMLANE:',midlane_LT,midlane_RT)
                #print('lane_num:',lane_num)
                # flag_scene = 0时的ExpectedSpeed
                if RR < 0.05 and last_R < 0.05:
                    R = 0
                    ExpectedSpeed = 30
                else:
                    R = 10000   # 弯道
                    ExpectedSpeed = 25

                #print("R is:",R)

            # 此时的lane_num会受虚线影响
            elif flag_scene == 1:    # 只循左线，减速.
                R = 10000  # 弯道
                apx = 700
                apy = 450
                right_lane_ref = 700
                midlane_LT = lane_base_LT
                DirectionError = int((midlane_LT + right_lane_ref) / 2) - apx  # error要调
                ExpectedSpeed = 20
                print('1 LEFTTTTT Error:', DirectionError)
                flag_lane = 3
                print('flag_lane:', flag_lane)

            else:
                ExpectedSpeed = 10

            last_DirectionError = DirectionError
            # flag_scene == 3 时由激光雷达计算并发布twist，优先级更高，这里就不做动作了。
            Steer_PID_update()

            self.cam_cmd.angular.z = - cmdSteer  # k * steerAngle  #- cmdSteer
            print('cmdSteer:', cmdSteer)
            self.cam_cmd.linear.x = ExpectedSpeed
            self.cmdPub.publish(self.cam_cmd)
            # print('flag_scene:',flag_scene)
            # print('R :', R)
            print("  ")
            print("  ")
def talker():
    host='192.168.2.111'
    port=7777
    rospy.init_node('lane_vel', anonymous=True)
    pub_traffic = rospy.Publisher('/traffic', Int32, queue_size=10)
    pub_conf = rospy.Publisher('/conf', Int32, queue_size=10)
    rospy.Subscriber("/scanInfo", Int32, scanInfo_callback)
    rate = rospy.Rate(10)
    while True:
        try:
            t1 = time.time()
            conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            conn.connect((host, port))
            json_dic = conn.recv(1024).decode('utf-8','ignore')
            # print('json_dic is:', json_dic)
            dic = json.loads(json_dic)

            # 发布ros标志信息话题
            pub_traffic.publish(int(dic['labelName']))
            pub_conf.publish(int(dic['conf']))

            file_path = os.path.join(os.getcwd(), dic['file_name'])
            with open(file_path, 'wb') as f:
                while dic['file_size'] > 0:
                    file_conet = conn.recv(1024)
                    dic['file_size'] -= len(file_conet)
                    f.write(file_conet)
            t2 = time.time()
            # print('req_time is:', t2 - t1)
            frame = cv2.imread('./frame.jpg', cv2.IMREAD_COLOR)
            cam = camera(frame)
            cam.spin()
            t3 = time.time()
            #print('total_time is:/n', t3 - t1)
            conn.close()
        except (KeyboardInterrupt, ValueError) as error:
            # print(type(error), error)
            if error == KeyboardInterrupt:
                break
            else:
                continue
        rate.sleep()

if __name__ == '__main__':
    last_lane = 0
    lane_now = 0
    talker()
