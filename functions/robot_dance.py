#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import math
import threading
import numpy as np
from common import kinematics # 运动学库已加密，只供调用
from common.ros_robot_controller_sdk import Board
import arm_ik.arm_move_ik as AMK

board = Board()
ik = kinematics.IK(board)  # 实例化逆运动学库(instantiate inverse kinematics library)
ak = AMK.ArmIK() 

ik.stand(ik.initial_pos, t=500)  # 立正，姿态为ik.initial_pos，时间500ms(Stand at attention with the posture of 'ik.initial_pos' for 500ms)
ak.setPitchRangeMoving((0, 10, 30), 0, -90, 100, 1)
time.sleep(1)

'''
moveToPosition(leg, leg_p, p, o):
控制机体的中心运动(control the movement of robot center)
:param leg: 1~6, 表示6条腿，1表示头部朝前时左上角所在腿, 逆时针表示1-6(1 to 6 represent the 6 legs of the robot. 1 represents the leg located at the upper left corner when the robot's head is facing forward, and the legs are numbered in a counterclockwise direction from 1 to 6)
:param leg_p: 足末端点相对中心的坐标,单位mm(The coordinate of the foot end point relative to the center, in millimeters)
:param p: 方向角(direction angle)
:param o: 旋转角(rotation angle)
:param speed: 运行该动作的速度,单位ms(the running speed of the action, in the unit of ms)
:return:

setStepMode(pos, mode, step_velocity, step_amplitude, step_height, movement_direction, rotation_angle, rotation, p, o, speed, times):
:param pos: 初始位置/mm(initial position/mm)
:param mode: 步态模式(gait mode)， 1:Ripple Gait，2:Tripod Gait, 3:四足(quadruped)，4：四足(quadruped)
:param step_velocity: 速度(speed)
:param step_amplitude: 幅度/mm(amplitude/mm)
:param step_height: 高度/mm(height/mm)
:param movement_direction: 方向/0-360(direction/0-360)
:param rotation: 旋转角/-1~1(rotation angle/-1~1)
:param speed: 舵机速度/ms, 一个周期10个动作，前进幅度为step_amplitude，所以移动速度为step_amplitude/(speed*10)(servo speed/ms, with 10 actions per cycle. The forward movement amplitude is 'step_amplitude', therefore, the moving speed is 'step_amplitude/(speed*10)')
:param times: 执行次数(execution number of times)
:return:

ik.circle(distance, times):
   
ik.circle_roll(distance, angle, times):
'''

'''#####################################################################################'''

# 原地踏步(stepping)
def mark_time(times=1, speed=50, o=[0,0,0]):
# times:运行次数(times: running number of times)
# speed:运行时间/ms(speed:runtime/ms)
# o:[滚转角,俯仰角,偏航角](o:roll angle, pitch angle, yaw angle)

    ik.setStepMode(ik.initial_pos, 2, 2, 0, 30, 0, 0, 0, [0, 0, 0], o, speed, times)


def drumbeat(times=1, speed=500):
    ik.moveBody(ik.initial_pos, [0,50,0], [0,0,0], 1)
    ak.setPitchRangeMoving((-5, 10, 30), 0, -90, 100, 1)
    time.sleep(1)
    for i in range(times):
        ak.setPitchRangeMoving((-5, 10+5, 30), 0, -90, 100, speed/1000)
        ik.moveBody(ik.initial_pos, [30,50,0], [0,0,0], speed)
        time.sleep(speed/1000)
        ak.setPitchRangeMoving((-5, 10, 30), 0, -90, 100, speed/1000)
        ik.moveBody(ik.initial_pos, [0,50,0], [0,0,0], speed)
        time.sleep(speed/1000)
        
    ik.moveBody(ik.initial_pos, [0,-50,0], [0,0,0], 2000)
    ak.setPitchRangeMoving((5, 10, 30), 0, -90, 100, 2)
    time.sleep(2) 
    for i in range(times):
        ak.setPitchRangeMoving((5, 10+5, 30), 0, -90, 100, speed/1000)
        ik.moveBody(ik.initial_pos, [30,-50,0], [0,0,0], speed)
        time.sleep(speed/1000)
        ak.setPitchRangeMoving((5, 10, 30), 0, -90, 100, speed/1000)
        ik.moveBody(ik.initial_pos, [0,-50,0], [0,0,0], speed)
        time.sleep(speed/1000)

# 原地扭动1(twist in place 1)
def twist(times=1, speed=100):
# times:运行次数(times: running number of times)
# speed:运行时间/ms(speed:runtime/ms)

    for i in range(times):
        for i in range(0, -21, -5):
            ak.setPitchRangeMoving((0, 10-abs(i/10*2), 30+abs(i/10*2)), 0, -90, 100, speed/1000)
            ik.moveBody(ik.initial_pos, [0,0,abs(i*2)], [0,0, math.radians(i)], speed)
            time.sleep(speed/1000.0)
            
        for i in range(-20, 21, 5):
            ak.setPitchRangeMoving((0, 10-abs(i/10*2), 30+abs(i/10*2)), 0, -90, 100, speed/1000)
            ik.moveBody(ik.initial_pos, [0,0,abs(i*2)], [0,0, math.radians(i)], speed)
            time.sleep(speed/1000.0)
            
        for i in range(20, -1, -5):
            ak.setPitchRangeMoving((0, 10-abs(i/10*2), 30+abs(i/10*2)), 0, -90, 100, speed/1000)
            ik.moveBody(ik.initial_pos, [0,0,abs(i*2)], [0,0, math.radians(i)], speed)
            time.sleep(speed/1000.0)
            
# 原地扭动2(twist in place 2)
def circle(distance=30,times=1):
# distance:扭动幅度/mm(distance: twist amplitude/mm)
# times:运行次数(times: running number of times)
   
    p = [0, 0, 0]
    o = [0, 0, 0]
    p[1] = p[1] + distance * math.cos(math.radians(0))
    p[0] = p[0] + distance * math.sin(math.radians(0))
    x = round(p[0]/10, 1)
    y = round(p[1]/10, 1)
    ak.setPitchRangeMoving((0+x, 10+y, 30), 0, -90, 100, 0.4)
    ik.moveBody(ik.initial_pos, p, o, 500)
    time.sleep(0.5)
    
    for j in range(times):
        for i in range(10, 370, 10):
            p = [0, 0, 0]
            o = [0, 0, 0]
            p[1] = p[1] + distance*math.cos(math.radians(i))
            p[0] = p[0] + distance*math.sin(math.radians(i))
            x = round(p[0]/10, 1)
            y = round(p[1]/10, 1)
            ak.setPitchRangeMoving((0+x, 10+y, 30), 0, -90, 100, 0.05)
            if ik.moveBody(ik.initial_pos, p, o, 50):
                time.sleep(0.05)
              
    p = [0, 0, 0]
    ak.setPitchRangeMoving((0, 10, 30), 0, -90, 100, 0.50)
    ik.moveBody(ik.initial_pos, p, o, 500)
    time.sleep(0.5)

# 原地扭动3(twist in place 3)
def circle_roll( distance=5, angle=5, times=1):
# distance:扭动幅度/mm(distance: twist amplitude/mm)
# angle:扭动角度(angle:twist angle)
# times:运行次数(times: running number of times)

    p = [0, 0, 0]
    o = [0, 0, 0]
    p[1] = p[1] + distance * math.cos(math.radians(0))
    p[0] = p[0] + distance * math.sin(math.radians(0))
    o[0] = o[0] + math.radians(angle) * math.cos(math.radians(0))
    o[1] = o[1] - math.radians(angle) * math.sin(math.radians(0))
    x = round(p[0], 1)
    z = round(p[1]/2, 1)
    ak.setPitchRangeMoving((0+x, 10, 30+z), 0, -90, 100, 0.5)
    ik.moveBody(ik.initial_pos, p, o, 500)
    time.sleep(0.5)
    for j in range(times):
        for i in range(0, 370, 10):
            p = [0, 0, 0]
            o = [0, 0, 0]
            p[1] = p[1] + distance*math.cos(math.radians(i))
            p[0] = p[0] + distance*math.sin(math.radians(i))
            o[0] = o[0] + math.radians(angle)*math.cos(math.radians(i))
            o[1] = o[1] - math.radians(angle)*math.sin(math.radians(i))
            x = round(p[0], 1)
            z = round(p[1]/2, 1)
            ak.setPitchRangeMoving((0+x, 10, 30+z), 0, -90, 100, 0.050)
            if ik.moveBody(ik.initial_pos, p, o, 50):
                time.sleep(0.05)
        for i in range(360, -10, -10):
            p = [0, 0, 0]
            o = [0, 0, 0]
            p[1] = p[1] + distance*math.cos(math.radians(i))
            p[0] = p[0] + distance*math.sin(math.radians(i))
            o[0] = o[0] + math.radians(angle)*math.cos(math.radians(i))
            o[1] = o[1] - math.radians(angle)*math.sin(math.radians(i))
            x = round(p[0], 1)
            z = round(p[1]/2, 1)
            ak.setPitchRangeMoving((0+x, 10, 30+z), 0, -90, 100, 0.050)
            if ik.moveBody(ik.initial_pos, p, o, 50):
                time.sleep(0.05)
                
    p = [0, 0, 0]
    o = [0, 0, 0]
    ak.setPitchRangeMoving((0, 10, 30), 0, -90, 100, 0.50)
    ik.moveBody(ik.initial_pos, p, o, 500)
    time.sleep(0.5)

# 机体左移(the robot moves to the left)
def left_move():
    ak.setPitchRangeMoving((15, 10, 30), 0, -90, 100, 1.2)
    ik.left_move(ik.initial_pos, 2, 50, 30, 3)
    ik.stand(ik.initial_pos, t=200)  # 立正(stand at attention)
    time.sleep(0.2)
    ak.setPitchRangeMoving((0, 10, 30), 0, -90, 100, 1.2)
    ik.right_move(ik.initial_pos, 2, 50, 30, 3)
    ik.stand(ik.initial_pos, t=200)  # 立正(stand at attention)
    time.sleep(0.2)

# 机体右移(the robot moves to the right)
def right_move():
    ak.setPitchRangeMoving((-15, 10, 30), 0, -90, 100, 1.2)
    ik.right_move(ik.initial_pos, 2, 50, 30, 3)
    ik.stand(ik.initial_pos, t=200)  # 立正(stand at attention)
    time.sleep(0.2)
    ak.setPitchRangeMoving((0, 10, 30), 0, -90, 100, 1.2)
    ik.left_move(ik.initial_pos, 2, 50, 30, 3)
    ik.stand(ik.initial_pos, t=200)  # 立正(stand at attention)
    time.sleep(0.2)

# 机体前后移动(the robot moves forward and backward)
def forward_and_back():
    for i in range(4):
        ak.setPitchRangeMoving((0, 10 + 5*(i + 1), 30), 0, -90, 100, 0.5)
        ik.setStepMode_whitout_delay(ik.initial_pos, 2, 2, -50, 20, 0, 0, 0, [0, 0, 0], [0, 0, 0], 50, 1)
    ik.stand(ik.initial_pos, t=300)

    for i in range(4):
        ak.setPitchRangeMoving((0, 30.1 - 5*(i + 1), 30), 0, -90, 100, 0.5)
        ik.setStepMode_whitout_delay(ik.initial_pos, 2, 2, 50, 20, 0, 0, 0, [0, 0, 0], [0, 0, 0], 50, 1)
    ik.stand(ik.initial_pos, t=300)
    
    for i in range(2):
        ak.setPitchRangeMoving((0, 10.1 - 5*(i + 1), 30), 0, -90, 100, 0.5)
        ik.setStepMode_whitout_delay(ik.initial_pos, 2, 2, 50, 20, 0, 0, 0, [0, 0, 0], [0, 0, 0], 50, 1)
    ik.stand(ik.initial_pos, t=300)
    
    for i in range(2):
        ak.setPitchRangeMoving((0, 0.1 + 5*(i + 1), 30), 0, -90, 100, 0.5)
        ik.setStepMode_whitout_delay(ik.initial_pos, 2, 2, -50, 20, 0, 0, 0, [0, 0, 0], [0, 0, 0], 50, 1)
    ik.stand(ik.initial_pos, t=300)
    
# 机体中心原地前后移动(the robot body moves forward and backward in place)
def dolly_moves(distance=30,times=1, speed=100):
# distance:扭动幅度/mm(distance:twist amplitude/mm)
# times:运行次数(times:running number of times)
# speed:运行时间/ms(speed:runtime/ms)
     
        for i in range(0, -(distance+1), -5):
            ak.setPitchRangeMoving((0, 10+(i/10), 30), 0, -90, 100, speed/1000)
            ik.moveBody(ik.initial_pos, [i,0,0], [0,0,0], speed)
            time.sleep(speed/1000.0)
            
        for i in range(times):
            for i in range(-distance, distance+1, 5):
                ak.setPitchRangeMoving((0, 10+(i/10), 30), 0, -90, 100, speed/1000)
                ik.moveBody(ik.initial_pos, [i,0,0], [0,0,0], speed)
                time.sleep(speed/1000.0)
            for i in range(distance, -(distance+1), -5):
                ak.setPitchRangeMoving((0, 10+(i/10), 30), 0, -90, 100, speed/1000)
                ik.moveBody(ik.initial_pos, [i,0,0], [0,0,0], speed)
                time.sleep(speed/1000.0)

        for i in range(-distance, 1, 5):
            ak.setPitchRangeMoving((0, 10+(i/10), 30), 0, -90, 100, speed/1000)
            ik.moveBody(ik.initial_pos, [i,0,0], [0,0,0], speed)
            time.sleep(speed/1000.0)
            
# 原地左右旋转(rotate left and right in place)
def rotate(times=1, speed=300):
# times:运行次数(times:running number of times)
# speed:运行时间/ms(speed:runtime/ms)

    for j in range(times):
        ik.moveBody(ik.initial_pos, [0,0,0], [0,0, math.radians(20)], 500)
        ak.setPitchRangeMoving((5, 5, 30), 0, -90, 100, 0.50)
        time.sleep(0.5)
        o = [0,0, math.radians(20)]
        th = threading.Thread(target=mark_time, args=(3,speed/6,o), daemon=True)
        th.start()
        for i in range(3):
            board.bus_servo_set_position(speed/1000, [[25, 1000]])
            ak.setPitchRangeMoving((5, 5, 26), 0, -90, 100, speed/1000)
            time.sleep(speed/1000)
            ak.setPitchRangeMoving((5, 5, 30), 0, -90, 100, speed/1000)
            board.bus_servo_set_position(speed/1000, [[25, 500]])
            time.sleep(speed/1000)

        ik.moveBody(ik.initial_pos, [0,0,0], [0,0, math.radians(0)], 800)
        ak.setPitchRangeMoving((0, 10, 30), 0, -90, 100, 0.50)
        time.sleep(1)

        ik.moveBody(ik.initial_pos, [0,0,0], [0,0, math.radians(-20)], 500)
        ak.setPitchRangeMoving((-5, 5, 30), 0, -90, 100, 0.50)
        time.sleep(0.5)
        o = [0,0, math.radians(-20)]
        th = threading.Thread(target=mark_time, args=(3,speed/6,o), daemon=True)
        th.start()
        for i in range(3):
            board.bus_servo_set_position(speed/1000, [[25, 1000]])
            ak.setPitchRangeMoving((-5, 5, 26), 0, -90, 100, speed/1000)
            time.sleep(speed/1000)
            ak.setPitchRangeMoving((-5, 5, 30), 0, -90, 100, speed/1000)
            board.bus_servo_set_position(speed/1000, [[25, 500]])
            time.sleep(speed/1000)

        ik.moveBody(ik.initial_pos, [0,0,0], [0,0, math.radians(0)], 800)
        ak.setPitchRangeMoving((0, 10, 30), 0, -90, 100, 0.50)
        time.sleep(1)
        
############################################################################################

def dance():
    mark_time(2,45)
    rotate(1,260)
    left_move()
    right_move()
    circle(30,2)
    forward_and_back()
    dolly_moves(80,2,10)
    twist(2, 100)
    circle_roll(10,10,1)
    mark_time(2)
    ik.stand(ik.initial_pos, t=500)  # 立正(stand at attention)



if __name__ == '__main__':

    mark_time(2,45)
    rotate(1,260)
    left_move()
    right_move()
    circle(30,2)
    forward_and_back()
    dolly_moves(80,2,10)
    twist(2, 100)
    circle_roll(10,10,1)
    mark_time(2)
    ik.stand(ik.initial_pos, t=500)  # 立正(stand at attention)
