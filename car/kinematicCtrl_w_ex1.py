#!/usr/bin/env python
#coding=utf-8
import rospy
#倒入自定义的数据类型
import time
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np
import threading

# GLOBAL VARIABLES
# Hilens摄像头识别参数
lane_vel = Twist()
angularScale = 6       # 180/30
servodata=0
ExpectedSpeed = 0

# 标志物识别参数
traffic_data=0
conf = 0

# 陀螺仪参数
z_theta = 0

# 雷达距离参数
dist_data = 10
dist_data_last = 10

# 激光雷达扫描参数
lane_vel_scan = Twist()  # 这个变量是通过激光雷达后计算出的小车需要的速度
servodata_scan = 0
ExpectedSpeed_scan = 0
flag_scan = 0

def thread_job():

    rospy.spin()

def lanecallback(msg):
    global lane_vel
    lane_vel = msg
    _servoCmdMsg = msg.angular.z * angularScale + 90 
    global servodata
    global ExpectedSpeed
    servodata = min(max(0, _servoCmdMsg), 180)
    servodata=100-servodata*100/180
    ExpectedSpeed = msg.linear.x
    #rospy.loginfo('lane_vel.angular.z = %f',lane_vel.angular.z)

def scanVelcallback(msg):
    global flag_scan_first
    # 命名重复
    # global lane_vel
    # 缺少ExpectedSpeed_scan
    global ExpectedSpeed_scan
    global lane_vel_scan
    global servodata_scan

    lane_vel_scan = msg
    flag_scan_first = 1
    _servoCmdMsg = msg.angular.z * angularScale + 90
    servodata_scan = min(max(0, _servoCmdMsg), 180)
    servodata_scan = 100 - servodata_scan * 100 / 180
    ExpectedSpeed_scan = msg.linear.x

def scanInfocallback(msg):
    global flag_scan
    flag_scan = msg.data


def trafficcallback(data):  # 标志物检测的callback
    global traffic_data
    traffic_data = data.data
    labelList = ["green_go", "pedestrian_crossing", "red_stop", "speed_limited_max", "speed_limited_min",
                 "speed_unlimited", "yellow_back"]
    if traffic_data != 7:
        print(traffic_data)
        rospy.loginfo(rospy.get_caller_id() + "traffic_data is %s", labelList[traffic_data])

def confcallback(data):  # 标志物置信度的callback
    global conf
    conf = data.data


def distcallback(data):  # 雷达距离的callback
    global dist_data
    global dist_data_last
    dist_data_last = dist_data
    dist_data = data.data
    # dist_data = min(dist_data, 2000)
    # rospy.loginfo(rospy.get_caller_id() + "dist_data is", dist_data)


def imuzcallback(data):  # 陀螺仪z轴角度callback
    global z_theta
    z_theta = data.data
    # rospy.loginfo(rospy.get_caller_id() + "z_theta is", z_theta)

def myhook():
    direction = 50
    speed = 0
    pub22 = rospy.Publisher('/auto_driver/send/direction', Int32 , queue_size=10)
    pub33 = rospy.Publisher('/auto_driver/send/speed', Int32 , queue_size=10)
    pub22.publish(direction)
    pub33.publish(speed)
    print("5s done, the node kinematic is shutdown!")
rospy.on_shutdown(myhook)

def kineticCtrl():
    labelList = ["green_go", "pedestrian_crossing", "red_stop", "speed_limited_max", "speed_limited_min",
                 "speed_unlimited", "yellow_back"]
    #Publisher 函数第一个参数是话题名称，第二个参数 数据类型，现在就是我们定义的msg 最后一个是缓冲区的大小
    #queue_size: None（不建议）  #这将设置为阻塞式同步收发模式！
    #queue_size: 0（不建议）#这将设置为无限缓冲区模式，很危险！
    #queue_size: 10 or more  #一般情况下，设为10 。queue_size太大了会导致数据延迟不同步。
    stages = ['light', 'speed_limited_max', 'pedestrian_crossing', 'speed_limited_min', 'speed_unlimited']
    stage_idx = -1
    pub1 = rospy.Publisher('/bluetooth/received/manul', Int32 , queue_size=10)
    pub2 = rospy.Publisher('/auto_driver/send/direction', Int32 , queue_size=10)
    pub3 = rospy.Publisher('/auto_driver/send/speed', Int32 , queue_size=10)
    pub4 = rospy.Publisher('/auto_driver/send/gear', Int32 , queue_size=10)
    
    manul=0       # 0 - Automatic
    speed=20      # SPEED
    direction=50  # 0-LEFT-50-RIGHT-100
    gear=1        # 1 - Drive, 2 - Stop 

    cmd_vel = Twist()
    flag=0
    p_flag=1
    servo = 50
    servodata_list=[]
    n_loop=1

    rospy.init_node('kineticCtrl', anonymous=True)

    add_thread = threading.Thread(target = thread_job)

    add_thread.start()

    rate = rospy.Rate(8) # 10hz
    rospy.Subscriber("/lane_vel", Twist, lanecallback)
    rospy.Subscriber("/traffic", Int32, trafficcallback)
    rospy.Subscriber("/conf", Int32, confcallback)
    # rospy.Subscriber("/vcu/SupersonicDistance", Int32, distcallback)
    rospy.Subscriber("/vcu/thetaZ", Int32, imuzcallback)
    rospy.Subscriber("/back_distance", Int32, distcallback)
    rospy.Subscriber("/scan_vel", Twist, scanVelcallback)
    rospy.Subscriber("/scanInfo", Int32, scanInfocallback)
    # rospy.Subscriber("/vcu/ActualVehicleDirection", Int32, Directioncallback)
    #更新频率是1hz
    rospy.loginfo(rospy.is_shutdown())
    n=1
    servodata_list = n * [servodata]
    while not rospy.is_shutdown():
        # KINETIC CONTROL CODE HERE
        #sona_data=rospy.wait_for_message("/vcu/SupersonicDistance", Int32, timeout=0.2)
        #traffic_light=rospy.wait_for_message("/traffic_light", Int32,timeout=None)
        #traffic_signs=rospy.wait_for_message("/traffic_signs", Int32,timeout=None)
   
        # if State is GO AROUND
        #if(lane_vel.linear.x == 0): # STOP SIGNAL FROM HILENS
        if flag_scan_first != 0:  # 激光雷达优先级更高
            servo = servodata_scan
            speed = ExpectedSpeed_scan
        else:
            servo = servodata
            speed = ExpectedSpeed
 
        servodata_list[0:n-1] = servodata_list[1:n]
        servodata_list[n-1] = servo

        sumii = 0
        for i in servodata_list:
            sumii += i
            #print("servodata %f",i)
        servodata_mean = sumii/n
        direction = servodata_mean

        # 道路标志物不影响方向，所以提前pub
        pub2.publish(direction)
        #print("servodata %f %f",servodata_mean,servodata)

        # WRITE YOUR CONDITION STATEMENT HERE
        # TO CHANGE: GEAR, DIRECTION(IF DRIVE, USE servodata_mean)

        #  针对不同的道路标志做出反应
        if stage_idx == -1 and traffic_data == 7:  # 初始状态
            gear = 1
            pub1.publish(manul)
            # pub2.publish(direction)
            pub3.publish(speed)
            pub4.publish(gear)
            continue

        if (traffic_data == 0 or traffic_data == 2 or traffic_data == 6) and conf > 30:  # 看到交通灯
            if stage_idx == -1 or stage_idx == 4:
                stage_idx = stage_idx + 1
                if traffic_data == 0:
                    gear = 1
                if traffic_data == 2:
                    gear = 3  # 驻车
                    stage_idx = stage_idx - 1
                if traffic_data == 6:
                    while True:
                        # print('dist_data is:', dist_data)
                        gear = 4
                        # 以下进行泊车操作
                        speed = 5
                        direction = 50

                        pub2.publish(direction)
                        pub3.publish(speed)
                        pub4.publish(gear)
                        # print('publish done!')
                        if dist_data == 0:
                            continue
                        if dist_data <= 10 or float(abs(dist_data - dist_data_last)) / float(dist_data) > 0.6:
                            continue
                        elif dist_data < 500:
                            break
                    # print(dist_data)
                    gear = 3
                    speed = 0
                    # print('finished!')
                    pub4.publish(gear)
                    pub3.publish(speed)
                    break

        elif traffic_data == 3 and conf > 92:  # 看到最大限速标志
            if stage_idx == 0:
                stage_idx = stage_idx + 1
                gear = 1
                speed = min(speed, 50)
        elif traffic_data == 1 and conf > 30:  # 看到人行横道
            if stage_idx == 1:
                stage_idx = stage_idx + 1
                gear = 3
                # 进行等待操作
                # 示例
        elif traffic_data == 4 and conf > 92:  # 看到最低限速标志
            if stage_idx == 2:
                stage_idx = stage_idx + 1
                gear = 1
                speed = max(50, speed)
        elif traffic_data == 5 and conf > 80:  # 看到解限速标志
            if stage_idx == 3:
                stage_idx = stage_idx + 1
                gear = 1
             

        # 直接控制底盘的指令
        pub1.publish(manul)
        # pub2.publish(direction)
        pub3.publish(speed)
        pub4.publish(gear)
        rate.sleep()

if __name__ == '__main__':
    kineticCtrl()
 
