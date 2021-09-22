# 交大F4

## lidar_test.py

- #### 功能


1. 场景判断
2. 充当超声波传感器



- #### 场景判断

  使用变量flag_scene表示：

  flag_scene = 0：正常巡线；

  flag_scene = 1：停车时过虚线那个弯的阶段；

  flag_scene = 2:   进桥洞或者上桥前的阶段（注意是前），有一个减速动作；

  flag_scene = 3:   桥洞中以及桥面上的阶段，使用雷达测距的pid控制。这个状态下将发布一个Twist控制指令，由kinematics.py订阅，话题名为 /scan_vel. kinematics中根据优先级选择使用/scan_vel或者/lane_vel控制小车；

  

​		将flag_scene发布，由lane_detection_SK_Demo订阅，话题名为 /scanInfo，int32



- #### 超声传感器

  以话题名 /back_distance 发布。int32类型。停车使用，由kinematics.py订阅



## lane_detection_SK_Demo.py

- #### 功能


1. 发布交通标志和置信度，由kinematics.py订阅
2. 处理图像，得到俯视黑白图binary_warped
3. 取后50行判断lane_num车道线根数并计算lane_base_RT/LT
4. 分段计算DirectionError
5. pid update
6. 发布/lane_vel



- #### binary_warped

  720*1280

  binary_warped[ y ] [ x ],  y轴向下x轴向右



- #### lane_base计算

  若有两条线，即lane_num=2, 认为第一根为lane_base_LT, 第二根为lane_base_RT

  如果只有一条线，先默认这条线为右线lane_base_RT，后续继续判断

```python
# flag_lane = 0: 正常2线
# flag_lane = 1: 一根右线
# flag_lane = 2: 一根左线
# flag_lane = 3: 虚线处的一根左线

apx = 700
apy = 450  # 以前取的450，现在重新以lane_base作为预瞄点，apy可以忽略
midlane_LT = 0
midlane_RT = 0 # 这两个点是刻画预瞄点的横坐标。现在与lane_base相同

if flag_scene == 0 or flag_scene == 2:  # flag_scene == 2时只需要降低速度
    if lane_num == 2:
        if 人行道处需要丢线保护:
            DirectionError = lastDirectionError
        else: # 正常的两根线
        	DirectionError = int((midlane_LT + midlane_RT) / 2) - apx
    elif lane_num == 1:
        if lane_base_RT < 500:  # 视野中的单线理想的一道左线
        	if last_flag_lane == 1:  # 如果上一帧是右线，则认为此时仍然是右线。保护，不让他从一根左线跳到一根右线
            	flag_lane = 1
            	midlane_LT = 0
                midlane_RT = lane_base_RT
                DirectionError = int((midlane_LT + midlane_RT) / 2) - apx
            else:  # 一根左线
                midlane_LT = lane_base_RT
                midlane_RT = 1280
                DirectionError = int((midlane_LT+midlane_RT) / 2) - apx  # 直道，左侧，一根线,1100
				flag_lane = 2
		else:  # 一根右线
             if last_flag_lane == 2:  # 左线
             	flag_lane = 2
                midlane_LT = lane_base_RT
                midlane_RT = 1280
        	 else:
				midlane_LT = 0
				midlane_RT = lane_base_RT
               	DirectionError = int((midlane_LT + midlane_RT) / 2) - apx  # 右侧一根线， midlane_LT = 0其实
                flag_lane = 1 
    else:
		print("no lane_num")
        midlane_LT = 0
        midlane_RT = 1280
   
	计算R，判断直道弯道
    
elif flag_scene == 1:
	right_lane_ref = 700
    midlane_LT = lane_base_LT
    DirectionError = int((midlane_LT + right_lane_ref) / 2) - apx  # error要调
    ExpectedSpeed = 20
    
else:
    pass  # flag_scene = 3, 激光雷达控制


updatePID()

发布消息
```

