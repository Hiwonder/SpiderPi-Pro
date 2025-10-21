#!/usr/bin/python3
# coding=utf8
#第9章 机械臂正逆运动学课程\2.机械臂正逆运动学实战应用\第1课 机械臂色块追踪(9.Forward & Inverse Kinematic/2.Forward & Inverse Kinematics Practical Application\Lesson 1 Color Tracking)
import sys
import cv2
import math
import time
import threading
import numpy as np
from common import misc
from common.pid import PID
from common import yaml_handle
from common import kinematics
from common.ros_robot_controller_sdk import Board
from calibration.camera import Camera
import arm_ik.arm_move_ik as AMK
from sensor.ultrasonic_sensor import Ultrasonic


board = Board()
ik = kinematics.IK(board)
ultrasonic = Ultrasonic()
ak = AMK.ArmIK()

# 机械臂色块追踪(robotic arm tracks the block)

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
    


range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255)}

# 变量定义(define variables)
size = (640, 480)
x,y,z = (0, 15, 5)
world_x, world_y = -1, -1
lab_data = None
K,R,T = None,None,None

# 读取颜色阈值及坐标变换参数(read color threshold and parameters of coordinate transformation)
def load_config():
    global lab_data,K,R,T
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
    camera_cal = yaml_handle.get_yaml_data(yaml_handle.camera_file_path)['block_params']
    K = np.array(camera_cal['K'], dtype=np.float64).reshape(3, 3)
    R = np.array(camera_cal['R'], dtype=np.float64).reshape(3, 1)
    T = np.array(camera_cal['T'], dtype=np.float64).reshape(3, 1)
        

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
            if contour_area_temp >= 100:  # 只有在面积大于设定时，最大面积的轮廓才是有效的，以过滤干扰(Only when the area is greater than the set value, the contour with the maximum area is considered valid to filter out interference)
                area_max_contour = c
                max_area = contour_area_temp

    return area_max_contour, max_area  # 返回最大的轮廓, 面积(Return the largest contour and area)

# 初始位置(initial position)
def init_move():
    ultrasonic.setRGBMode(0)
    ultrasonic.setRGB(0, (0, 0, 0))
    ultrasonic.setRGB(1, (0, 0, 0))
    board.bus_servo_set_position(1, [[25, 240]])
    ik.stand(ik.initial_pos)
    ak.setPitchRangeMoving((x,y,z), -90, -90, 100, 2)

centre_x = 320
centre_y = 240
x_pid = PID(P=0.08, I=0.005, D=0.008)#pid初始化(PID initialization)
y_pid = PID(P=0.08, I=0.005, D=0.008)
# 机器人移动函数(robot movement function)
def move():
    global x,y,z
    global world_x, world_y
    
    while True:
        if world_x > 0 or world_y > 0:
            # X轴PID处理(X-axis PID processing)
            if abs(world_x-centre_x) >= 10:
                x_pid.SetPoint = centre_x  #设定(set)
            else:
                x_pid.SetPoint = world_x
            x_pid.update(world_x)  #当前(current)
            dx = int(x_pid.output)/10 # 输出(output)
            x -= dx
            
            # Y轴PID处理(Y-axis PID processing)
            if abs(world_y-centre_y) >= 10:
                y_pid.SetPoint = centre_y  #设定(set)
            else:
                y_pid.SetPoint = world_y
            y_pid.update(world_y)  #当前(current)
            dy = int(y_pid.output)/10 # 输出(output)
            y += dy
            if x > 10 : x = 10
            if x < -10 : x = -10
            if y > 24 : y = 24
            if y < 6 : y = 6
            ak.setPitchRangeMoving((x, y, 5), -90, -90, 100, 0.08) # 移动到目标位置(move to the target position)
            time.sleep(0.05)
            world_x, world_y = -1, -1
            
        else:
            time.sleep(0.01)
        

# 运行子线程(run sub-thread)
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()


# 颜色识别函数(color recognition function)
def color_detect(img, color='red'): 
    global world_x, world_y
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)      
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(convert the image to LAB space)
    frame_mask = cv2.inRange(frame_lab,
                             (lab_data[color]['min'][0],
                              lab_data[color]['min'][1],
                              lab_data[color]['min'][2]),
                             (lab_data[color]['max'][0],
                              lab_data[color]['max'][1],
                              lab_data[color]['max'][2]))  #对原图像和掩模进行位运算(perform bitwise operation to original image and mask)
    eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  #腐蚀(erode)
    dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))) #膨胀(dilate)
    contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  #找出轮廓(find contours)
    areaMaxContour, area_max = get_area_maxContour(contours)  #找出最大轮廓(find the largest contour)
    
    if area_max > 500:  # 有找到最大面积(the maximum area has been found)
        ((centerX, centerY), radius) = cv2.minEnclosingCircle(areaMaxContour)  # 获取最小外接圆(obtain the minimum circumscribed circle)
        centerX = int(misc.map(centerX, 0, size[0], 0, img_w))
        centerY = int(misc.map(centerY, 0, size[1], 0, img_h))
        radius = int(misc.map(radius, 0, size[0], 0, img_w))
        
        cv2.circle(img, (centerX, centerY), radius, range_rgb[color], 2)#画圆(draw the circle)
        cv2.putText(img, "Color: " + color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, range_rgb[color], 2)
        
        world_x, world_y = centerX, centerY
    else:
        world_x, world_y = -1, -1
            
    return img

# 主要图像处理函数(main image processing function)
def run(img):
    img = color_detect(img)
        
    return img

if __name__ == '__main__':

    init_move()
    load_config()
    camera = Camera()
    camera.camera_open()
    while True:
        img = camera.frame
        if img is not None:
            frame = img.copy()
            Frame = run(frame)
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    camera.camera_close()
    cv2.destroyAllWindows()
    
    
