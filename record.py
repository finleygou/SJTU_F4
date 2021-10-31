import socket
# import time
import json
import os
import time
import cv2
def talker():
    host='192.168.2.111'
    port=7777
    # conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # conn.connect((host, port))
    labels_list =  ["green_go", "pedestrian_crossing", "red_stop", "speed_limited_max", "speed_limited_min",
                     "speed_unlimited", "yellow_back"]

    # 图片保存定义
    count = 0
    while True:
        try:
            # t1 = time.time()
            conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            conn.connect((host, port))
            json_dic = conn.recv(1024).decode('utf-8')
            # print("recv_done", json_dic)
            dic = json.loads(json_dic)
            label = dic['labelName']
            # print('label is:', label)
            conf = dic['conf']
            if label == 7:
                labelName = "None"
            else:
                labelName = labels_list[label]
            print('labelName is:', labelName, '\t conf is:', conf)
            file_path = os.path.join(os.getcwd(), dic['file_name'])
            # print(file_path)
            with open(file_path, 'wb') as f:
                while dic['file_size'] > 0:
                    file_conet = conn.recv(1024)
                    dic['file_size'] -= len(file_conet)
                    f.write(file_conet)
                # print('size is:', os.path.getsize(file_path))
            # print('file_path is:', file_path)
            frame = cv2.imread('frame.jpg')
            # print(frame)
            # cv2.imwrite('E:\\华为无人车挑战杯\\决赛\\SJTU_F4\\img\\img.jpg', frame)
            if count % 5 == 0:  # 每隔20帧保存图片
                img_path = './imgs/img_' + str(int(count / 20)) + '.jpg'
                cv2.imwrite(img_path, frame)
            count += 1
            # t2 = time.time()
            # print(t2 - t1)

            conn.close()
        except:
            # print("error")
            continue
    # conn.close()

if __name__ == '__main__':
    talker()
    # print("done")
