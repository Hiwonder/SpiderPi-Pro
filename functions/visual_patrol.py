#!/usr/bin/python3
# coding=utf8
#3.AI视觉学习课程/第2课 智能巡线(5.AI Vision Basic Lesson\Lesson 4 Line Following)
import sys
import cv2
import time
import math
import threading
import numpy as np
from common import yaml_handle
from calibration.camera import Camera 
from calibration.CalibrationConfig import *
from common import kinematics
from sensor.ultrasonic_sensor import Ultrasonic
import arm_ik.arm_move_ik as AMK

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

lab_data = None
servo_data = None
def load_config():
    global lab_data, servo_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

load_config()

__target_color = ('red',)
# 设置检测颜色(set target color)
def setLineTargetColor(target_color):
    global __target_color

    __target_color = target_color
    return (True, ())

# 初始位置(initial position)
def init_move():
    ultrasonic.setRGBMode(0)
    ultrasonic.setRGB(0, (0, 0, 0))
    ultrasonic.setRGB(1, (0, 0, 0)) 
    ak.setPitchRangeMoving((0, 8, 15), -60, -90, 100, 2)


line_center = -2
last_line_center = 0
# 变量重置(reset variables)
def reset():
    global last_line_center
    global line_center
    global __target_color

    last_line_center = 0
    line_center = -2
    __target_color = ()
    
# app初始化调用(call app initialization)
def init():
    print("VisualPatrol Init")
    load_config()
    init_move()

__isRunning = False
# app开始玩法调用(call app start game)
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("VisualPatrol Start")

# app停止玩法调用(call app stop game)
def stop():
    global __isRunning
    __isRunning = False
    print("VisualPatrol Stop")

# app退出玩法调用(call app exit game)
def exit():
    global __isRunning
    __isRunning = False
    ik.stand(ik.initial_pos)
    print("VisualPatrol Exit")

# 找出面积最大的轮廓(find the contour with the maximum area)
# 参数为要比较的轮廓的列表(parameter is the list of contour to be compared)
def get_area_maxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0

    area_max_contour = None
    max_area = 0

    for c in contours:  # 历遍所有轮廓(traverse through all contours)
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积(calculate contour area)
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp >= 10:  # 只有在面积大于设定时，最大面积的轮廓才是有效的，以过滤干扰(Only when the area is greater than the set value, the contour with the maximum area is considered valid to filter out interference)
                area_max_contour = c
                max_area = contour_area_temp

    return area_max_contour, max_area  # 返回最大的轮廓(return the largest contour)

img_center_x = 320
def move():
    global last_line_center
    global line_center

    while True:
        if __isRunning:
            if line_center >= 0:              
                if abs(line_center -img_center_x) < 60:
                    ik.go_forward(ik.initial_pos, 2, 60, 50, 1)
                elif line_center -img_center_x >= 60:
                    ik.turn_right(ik.initial_pos, 2, 30, 50, 1)
                else:
                    ik.turn_left(ik.initial_pos, 2, 30, 50, 1)
                last_line_center = line_center

            elif line_center == -1:
                if last_line_center >= img_center_x:
                    ik.turn_left(ik.initial_pos, 2, 30, 50, 1)
                else:
                    ik.turn_right(ik.initial_pos, 2, 30, 50, 1)
        else:
            time.sleep(0.01)



# 运行子线程(run sub-thread)
threading.Thread(target=move, args=(), daemon=True).start()

roi = [(240, 280,  0, 640)]
def run(img):
    global line_center
    global __target_color

    if not __isRunning or __target_color == ():
        return img

    frame_gb = cv2.GaussianBlur(img, (3, 3), 3)
    
    for r in roi:
        blobs = frame_gb[r[0]:r[1], r[2]:r[3]]
        frame_lab = cv2.cvtColor(blobs, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(convert the image to LAB space)

        for i in lab_data:
            if i in __target_color:
                detect_color = i
                frame_mask = cv2.inRange(frame_lab,
                                         (lab_data[i]['min'][0],
                                          lab_data[i]['min'][1],
                                          lab_data[i]['min'][2]),
                                         (lab_data[i]['max'][0],
                                          lab_data[i]['max'][1],
                                          lab_data[i]['max'][2]))  #对原图像和掩模进行位运算(perform bitwise operation to original image and mask)
                eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  #腐蚀(erode)
                dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  #膨胀(dilate)
                cnts = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  #找出所有轮廓(find all contours)
                cnt_large, area = get_area_maxContour(cnts)  #找到最大面积的轮廓(find the largest contour)
                if area > 10:
                    rect = cv2.minAreaRect(cnt_large)  #最小外接矩形(the minimum bounding rectangle)
                    
                    box = np.intp(cv2.boxPoints(rect))  #最小外接矩形的四个顶点(the four corner points of the minimum bounding rectangle)
                    for j in range(4):
                        box[j, 1] = box[j, 1] + r[0]

                    cv2.drawContours(img, [box], -1, (0, 255, 255), 2)  #画出四个点组成的矩形(draw the rectangle composed of the four points)

                    #获取矩形的对角点(obtain the diagonal points of the rectangle)
                    pt1_x, pt1_y = box[0, 0], box[0, 1]
                    pt3_x, pt3_y = box[2, 0], box[2, 1]
                    line_center_x, line_center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2  #中心点(center point)
                    cv2.circle(img, (int(line_center_x), int(line_center_y)), 5, (0, 0, 255), -1)  #画出中心点(draw the center point)
                    line_center = line_center_x
                else:
                    if line_center != -2:
                        line_center = -1
        
    return img

if __name__ == '__main__':
    from common.ros_robot_controller_sdk import Board
    from sensor.ultrasonic_sensor import Ultrasonic
    
    board = Board()
    ik = kinematics.IK(board)  # 实例化逆运动学库(instantiate inverse kinematics library)
    ultrasonic = Ultrasonic()
    ak = AMK.ArmIK()

    #加载参数(load parameter)
    param_data = np.load(calibration_param_path + '.npz')

    #获取参数(obtain parameter)
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)

    init()
    start()
    camera = Camera()
    camera.camera_open()

    __target_color = ('red',)
  
    while True:
        img = camera.frame
        if img is not None:
            frame = img.copy()
            frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)  # 畸变矫正(distortion correction)
            Frame = run(frame)           
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    camera.camera_close()
    cv2.destroyAllWindows()
