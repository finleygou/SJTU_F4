#!/usr/bin/env python
# use encoding: utf-8
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

ExpectedSteer = 0
CurrentSteer = 0
cmdSteer = 0
DirectionError = 0
last_DirectionError = 0
DirectionError_Sum = 0
DirectionError_Diff = 0
ExpectedSpeed_scan = 0

Round = 0  # 记录圈数
left_forward_PN = 0
right_forward_PN = 0
right_PN = 0
left_PN = 0
LF_threshold = 200
RF_threshold = 200
L_threshold = 400
R_threshold = 400

#  avoid noise
counter1 = 0
counter2 = 0
counter3 = 0
counter4 = 0

flag_scene = 0  # 0 正常行驶  1 进入虚线弯道  2 进入桥洞前  3 上坡前中  4 停车
is_board = 0
backDist = 0
flag_lidar_assist = 0

scan_vel = Twist()

flag_obstacle = 0

def constrain(min, max, input):
    if input <= min:
        output = min
    elif input >= max:
        output = max
    else:
        output = input
    return output


def Steer_PID_update():
    global ExpectedSteer, CurrentSteer, cmdSteer
    global DirectionError, last_DirectionError, DirectionError_Sum, DirectionError_Diff

    k_p = 20  # Error的数量级在0.1以下
    k_i = 0
    k_d = 5  # 0.3

    DirectionError_Sum += DirectionError
    DirectionError_Diff = DirectionError - last_DirectionError
    last_DirectionError = DirectionError

    cmdSteer = k_p * DirectionError + k_i * DirectionError_Sum + k_d * DirectionError_Diff
    cmdSteer = constrain(-20, 20, cmdSteer)


def laserCallback(scan):
    global left_PN, right_PN, right_forward_PN, left_forward_PN
    global LF_threshold, RF_threshold, L_threshold, R_threshold
    global flag_scene
    global counter1, counter2, counter3, counter4
    global DirectionError
    global Round
    global backDist
    global flag_lidar_assist
    global ExpectedSpeed_scan
    global flag_obstacle
    ################ lidar data into XY points ###############
    # Store maxAngle of lidar
    maxAngle = scan.angle_max
    # Store minAngle of lidar
    minAngle = scan.angle_min
    # Store angleInc of lidar
    angleInc = scan.angle_increment
    # Store maxLength in lidar distances
    maxLength = scan.range_max
    # Store array of ranges
    ranges = scan.ranges
    # Calculate the number of points in array of ranges
    num_pts = len(ranges)
    xy_scan = np.zeros((num_pts, 2))

    left_PN = 0
    right_PN = 0
    right_forward_PN = 0
    left_forward_PN = 0
    left_back_PN = 0
    right_back_PN = 0
    for i in range(num_pts):  # can be optimated by np
        # Check that distance is not longer than it should be
        if math.isnan(ranges[i]):
            continue
        else:
            if i <= int(num_pts / 4):
                if ranges[i] < 1.2:
                    right_PN += 1
            elif int(num_pts / 4) < i < int(num_pts / 2):
                if ranges[i] < 1.2:
                    right_PN += 1
                    right_forward_PN += 1
            elif int(num_pts / 2) <= i < int(num_pts * 3 / 4):
                if ranges[i] < 1.5:
                    left_PN += 1
                    left_forward_PN += 1
            elif i >= int(num_pts * 3 / 4):
                if ranges[i] < 1.5:
                    left_PN += 1
    left_back_PN = left_PN - left_forward_PN
    right_back_PN = right_PN - right_forward_PN
    # print('LFPN', left_forward_PN)
    # print('RFPN', right_forward_PN)
    # print('RBPN',right_back_PN)
    # print('LBPN',left_back_PN)
    # print('LPN', left_PN)
    # print('RPN', right_PN)

    forward_i = int(15 * num_pts / 360)
    board_counter = 0
    for i in range(forward_i):
        if ranges[int(num_pts / 2) - i] < 3:
            board_counter += 1
        if ranges[int(num_pts / 2) + i] < 3:
            board_counter += 1
    # print('board_counter:', board_counter)

    if board_counter > 50:
        is_board = 1
    else:
        is_board = 0

    if (right_forward_PN > 60 or right_PN > 60) and left_forward_PN < 20 and left_PN < 20 and is_board == 1:  # scene 1
        counter1 += 1
        if counter1 > 2:
            flag_scene = 1
            flag_lidar_assist = 1

    elif right_forward_PN > 60 and left_forward_PN > 60:
        counter2 += 1
        if counter2 > 2:
            flag_scene = 2
            if left_back_PN > 70 or right_back_PN > 70:    # 60，60
                flag_scene = 3

                right_lane = ranges[240:480]
                right_lane = list(right_lane)  # 雷达收到的是tuple
                theta = [-math.sin(minAngle + i * angleInc) for i in range(240, 480)]
                inf = float("inf")
                for i in range(len(right_lane) - 1, -1, -1):
                    if right_lane[i] == inf:  # 删除inf的点
                        del right_lane[i]
                        del theta[i]
                # print(right_lane)
                avg_dist_rt = np.mean(np.array(right_lane) * np.array(theta))
                # lf_range = np.mean(left_lane)
                DirectionError = avg_dist_rt - 0.5
                # print('avg_dist_rt:', avg_dist_rt)
                # print('DirectionError:', DirectionError)
                # print('cmdSteer:', cmdSteer)
                ExpectedSpeed_scan = 20
                Steer_PID_update()

                scan_vel.angular.z = - cmdSteer
                scan_vel.linear.x = ExpectedSpeed_scan  # scan expectedspeed
                pub2.publish(scan_vel)
    else:
        if counter1 > 5:
            counter1 = 0
        counter2 = 0
        counter3 = 0
        flag_scene = 0
        # print("xxxxxxxxxxxxxxxxxxx")

    if flag_lidar_assist == 1 and board_counter > 40:
        flag_scene = 1

    if flag_scene == 0:  # 保证停车那段路一直是flag_scene = 1
        flag_lidar_assist = 0

    print('flag_scene:', flag_scene)
    pub1.publish(flag_scene)

    ###################### get back distance ##################
    rb_range = ranges[:40]  # 0-39
    rb_range = list(rb_range)
    lb_range = ranges[-40:]  # 40 nums
    lb_range = list(lb_range)
    back_range = rb_range + lb_range
    # back_range = list(back_range)
    theta1 = [-math.cos(minAngle + i * angleInc) for i in range(0, 40)]  # 0-39
    theta2 = [-math.cos(minAngle + i * angleInc) for i in range(num_pts - 40, num_pts)]  # 后40个数
    theta = theta1 + theta2
    inf = float("inf")
    for i in range(len(back_range) - 1, -1, -1):
        if back_range[i] == inf:  # 删除inf的点
            del back_range[i]
            del theta[i]

    # print(back_range)
    # print(theta)
    backDist = np.mean(np.array(back_range) * np.array(theta))
    backDist = int(backDist * 100)  # cm
    pub3.publish(backDist)
    print('back_distance :', backDist)

############################## obstacle detection ##############################
    # 根据规则，只在flag_scene = 0时检测是否有机器人
    if flag_scene == 0:
        obs_range = ranges[650:790]   # some degree，点的数目限制确保不会与flag_scene = 2冲突（70+70）
        dist_min = np.min(obs_range)
        if dist_min < 0.8:  # 最近的点
            flag_obstacle = 1
        else:
            flag_obstacle = 0
            pub4.publish(flag_obstacle)


    print('   ')
    print('   ')


if __name__ == "__main__":
    try:
        rospy.init_node('board_detection', anonymous=True)
        rospy.Subscriber('/scan', LaserScan, laserCallback)
        pub1 = rospy.Publisher('/scanInfo', Int32, queue_size=10)  # publish vel&steer
        pub2 = rospy.Publisher('/scan_vel', Twist, queue_size=10)
        pub3 = rospy.Publisher('/back_distance', Int32, queue_size=10)
        pub4 = rospy.Publisher('/obstacle_detection', Int32, queue_size=10)
        rate = rospy.Rate(10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
