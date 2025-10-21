#!/usr/bin/python3
# coding=utf8
#第5章 AI视觉基础课程/第5课 人脸识别(5.AI Vision Basic Lesson\Lesson 5 Face Detection)
import sys
import cv2
import time
import sys
import numpy as np
import threading
import mediapipe as mp
from common import yaml_handle
from calibration.camera import Camera 
from calibration.CalibrationConfig import *
from common.action_group_controller import ActionGroupController
from common.ros_robot_controller_sdk import Board
from calibration.camera import Camera 
from common import kinematics
import arm_ik.arm_move_ik as AMK

debug = False

iHWSONAR = None
board = None
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
 
# 导入人脸识别模块(import facial recognition module)
Face = mp.solutions.face_detection
# 自定义人脸识别方法，最小的人脸检测置信度0.5(Customize face recognition method, and the minimum face detection confidence is 0.5)
faceDetection = Face.FaceDetection(min_detection_confidence=0.8)

lab_data = None
servo_data = None
def load_config():
    global lab_data, servo_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

load_config()

x_pulse = 500
# 初始位置(initial position)
def init_move():
    ultrasonic.setRGBMode(0)
    ultrasonic.setRGB(0, (0, 0, 0))
    ultrasonic.setRGB(1, (0, 0, 0)) 

    ak.setPitchRangeMoving((0, 12, 35), 30, -90, 100, 1)
    board.bus_servo_set_position(1, [[21, x_pulse]])    

    

d_pulse = 5
start_greet = False
action_finish = True
# 变量重置(reset variables)
def reset():
    global d_pulse
    global start_greet
    global x_pulse    
    global action_finish

 
    start_greet = False
    action_finish = True
    x_pulse = 500 
    init_move()  
    
# app初始化调用(call app initialization)
def init():
    print("FaceDetect Init")
    reset()

__isRunning = False
# app开始玩法调用(call app start game)
def start():
    global __isRunning
    __isRunning = True
    print("FaceDetect Start")

# app停止玩法调用(call app stop game)
def stop():
    global __isRunning
    __isRunning = False
    reset()
    print("FaceDetect Stop")

# app退出玩法调用(call app exit game)
def exit():
    global __isRunning
    __isRunning = False
    ik.stand(ik.initial_pos)
    print("FaceDetect Exit")

def move():
    global start_greet
    global action_finish
    global d_pulse, x_pulse    

    LOCK_SERVOS={'21':500, '22':750, '23':33, '24':466}
    
    while True:
        if __isRunning:
            if start_greet:
                start_greet = False
                action_finish = False
                agc.run_action('wave',lock_servos=LOCK_SERVOS) # 识别到人脸时执行的动作(If the face is detected, execute the action)
                ak.setPitchRangeMoving((0, 12, 35), 30, -90, 100, 1)   
                action_finish = True
                time.sleep(0.5)
            else:
                if x_pulse >= 700 or x_pulse <= 300:
                    d_pulse = -d_pulse
            
                x_pulse += d_pulse

                board.bus_servo_set_position(0.05, [[21,x_pulse]])    
                time.sleep(0.05)
        else:
            time.sleep(0.01)
            
# 运行子线程(run sub-thread)
threading.Thread(target=move, args=(), daemon=True).start()


size = (320, 240)
def run(img):
    global __isRunning, area
    global center_x, center_y
    global center_x, center_y, area
    global start_greet
    global action_finish
    if not __isRunning:   # 检测是否开启玩法，没有开启则返回原图像(Detect if the game is started, if not, return the original image)
        return img
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
     
    imgRGB = cv2.cvtColor(img_copy, cv2.COLOR_BGR2RGB) # 将BGR图像转为RGB图像(convert BGR image to RGB image)
    results = faceDetection.process(imgRGB) # 将每一帧图像传给人脸识别模块(transmit the image of each frame to facial recognition module)

    if results.detections:  # 如果检测不到人脸那就返回None(If no face is detected, return None)

        for index, detection in enumerate(results.detections):  # 返回人脸索引index(第几张脸)，和关键点的坐标信息(Return the face index (which face) and the coordinate information of the keypoints)
            scores = list(detection.score)
            if scores and scores[0] > 0.7:

                bboxC = detection.location_data.relative_bounding_box  # 设置一个边界框，接收所有的框的xywh及关键点信息(Set a bounding box to receive xywh and keypoint information for all received boxes)
                
                # 将边界框的坐标点,宽,高从比例坐标转换成像素坐标(Convert the coordinates' width and height of the bounding box from proportional coordinates to pixel coordinates)
                bbox = (
                    int(bboxC.xmin * img_w),
                    int(bboxC.ymin * img_h),
                    int(bboxC.width * img_w),
                    int(bboxC.height * img_h)
                )
                cv2.rectangle(img, bbox, (0, 255, 0), 2)  # 在每一帧图像上绘制矩形框(draw a rectangle on each frame of the image)
                
                # 获取识别框的信息, xy为左上角坐标点(Get information about the recognition box, where xy is the coordinates of the upper left corner)
                x, y, w, h = bbox
                center_x = int(x + (w / 2))
                center_y = int(y + (h / 2))
                area = int(w * h)
                if action_finish:
                    start_greet = True


    else:
        center_x, center_y, area = -1, -1, 0
            
    return img

if __name__ == '__main__':
    from common.ros_robot_controller_sdk import Board
    from sensor.ultrasonic_sensor import Ultrasonic
    from common.action_group_controller import ActionGroupController

    board = Board()
    ik = kinematics.IK(board)  # 实例化逆运动学库(instantiate inverse kinematics library)
    ultrasonic = Ultrasonic()
    agc = ActionGroupController(board)
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
