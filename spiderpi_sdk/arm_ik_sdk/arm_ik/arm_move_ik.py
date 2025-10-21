#!/usr/bin/env python3
# encoding:utf-8
import sys
sys.path.append('/home/pi/SpiderPi/')
import time
import numpy as np
from math import sqrt
import matplotlib.pyplot as plt
from arm_ik.inverse_kinematics import *
#from HiwonderSDK.Board import setBusServoPulse, getBusServoPulse
from common.ros_robot_controller_sdk import Board

board = Board()
#机械臂根据逆运动学算出的角度进行移动(robotic arm moves based on the angle calculated by inverse kinematics)
ik = IK()

class ArmIK:
    servo24Range = (0, 1000.0, 0, 240.0) #脉宽， 角度(pulse width, angle)
    servo23Range = (0, 1000.0, 240.0, 0)
    servo22Range = (0, 1000.0, 0, 240.0)
    servo21Range = (0, 1000.0, 0, 240.0)

    def __init__(self):
        self.setServoRange()

    def setServoRange(self, servo24_Range=servo24Range, servo23_Range=servo23Range, servo22_Range=servo22Range, servo21_Range=servo21Range):
        # 适配不同的舵机(adapt to different servos)
        self.servo24Range = servo24_Range
        self.servo23Range = servo23_Range
        self.servo22Range = servo22_Range
        self.servo21Range = servo21_Range
        self.servo24Param = (self.servo24Range[1] - self.servo24Range[0]) / (self.servo24Range[3] - self.servo24Range[2])
        self.servo23Param = (self.servo23Range[1] - self.servo23Range[0]) / (self.servo23Range[3] - self.servo23Range[2])
        self.servo22Param = (self.servo22Range[1] - self.servo22Range[0]) / (self.servo22Range[3] - self.servo22Range[2])
        self.servo21Param = (self.servo21Range[1] - self.servo21Range[0]) / (self.servo21Range[3] - self.servo21Range[2])

    def transformAngelAdaptArm(self, theta24, theta23, theta22, theta21):
        #将逆运动学算出的角度转换为舵机对应的脉宽值(convert the angle calculated by inverse kinematics to the corresponding pulse width value for the servo)
        servo24 = int(round(theta24 * self.servo24Param + (self.servo24Range[1] + self.servo24Range[0])/2))
        if servo24 > self.servo24Range[1] or servo24 < self.servo24Range[0] + 60:
            logger.info('servo24(%s)超出范围(%s, %s)', servo24, self.servo24Range[0] + 60, self.servo24Range[1])
            return False

        servo23 = int(round(theta23 * self.servo23Param + (self.servo23Range[1] + self.servo23Range[0])/2))
        if servo23 > self.servo23Range[1] or servo23 < self.servo23Range[0]:
            logger.info('servo23(%s)超出范围(%s, %s)', servo23, self.servo23Range[0], self.servo23Range[1])
            return False

        servo22 = int(round((self.servo22Range[1] + self.servo22Range[0])/2 - (90.0 - theta22) * self.servo22Param))
        if servo22 > ((self.servo22Range[1] + self.servo22Range[0])/2 + 90*self.servo22Param) or servo22 < ((self.servo22Range[1] + self.servo22Range[0])/2 - 90*self.servo22Param):
            logger.info('servo22(%s)超出范围(%s, %s)', servo22, self.servo22Range[0], self.servo22Range[1])
            return False
        
        if theta21 < -(self.servo21Range[3] - self.servo21Range[2])/2:
            servo21 = int(round(((self.servo21Range[3] - self.servo21Range[2])/2 + (90 + (180 + theta21))) * self.servo21Param))
        else:
            servo21 = int(round(((self.servo21Range[3] - self.servo21Range[2])/2 - (90 - theta21)) * self.servo21Param))
        if servo21 > self.servo21Range[1] or servo21 < self.servo21Range[0]:
            logger.info('servo21(%s)超出范围(%s, %s)', servo21, self.servo21Range[0], self.servo21Range[1])
            return False

        return {"servo24": servo24, "servo23": servo23, "servo22": servo22, "servo21": servo21}

    def servosMove(self, servos, movetime=None):
        #驱动24,23,22,21号舵机转动(drive servos No.21 to No.24 to rotate)
        time.sleep(0.02)
        if movetime is None:
            max_d = 0
            for i in  range(0, 4):
                d = abs(board.bus_servo_read_position(i + 3)[0] - servos[i])
                if d > max_d:
                    max_d = d
            movetime = int(max_d*4)
        board.bus_servo_set_position(movetime, [[24, servos[0]], [23, servos[1]], [22, servos[2]], [21, servos[3]]])
        #setBusServoPulse(24, servos[0], movetime)
        #setBusServoPulse(23, servos[1], movetime)
        #setBusServoPulse(22, servos[2], movetime)
        #setBusServoPulse(21, servos[3], movetime)

        return movetime

    def setPitchRange(self, coordinate_data, alpha1, alpha2, da = 1):
        #给定坐标coordinate_data和俯仰角的范围alpha1，alpha2, 自动在范围内寻找到的合适的解(Given the coordinate 'coordinate_data' and the range of pitch angles 'alpha1' and 'alpha2', automatically search for a suitable solution within the range)
        #如果无解返回False,否则返回对应舵机角度,俯仰角(If there is no solution, return False. Otherwise, return the corresponding servo angle and pitch angle)
        #坐标单位cm， 以元组形式传入，例如(0, 5, 10)(The coordinate unit is cm and is transmitted in the form of tuple, for example, (0, 5, 10))
        #da为俯仰角遍历时每次增加的角度('da' represents the angle increment when iterating through the pitch angle)
        x, y, z = coordinate_data
        if alpha1 >= alpha2:
            da = -da
        for alpha in np.arange(alpha1, alpha2, da):#遍历求解(iterat to calculate)
            result = ik.getRotationAngle((x, y, z), alpha)
            if result:
                theta24, theta23, theta22, theta21 = result['theta24'], result['theta23'], result['theta22'], result['theta21']
                servos = self.transformAngelAdaptArm(theta24, theta23, theta22, theta21)
                if servos != False:
                    return servos, alpha

        return False

    def setPitchRangeMoving(self, coordinate_data, alpha, alpha1, alpha2, movetime=None):
        #给定坐标coordinate_data和俯仰角alpha,以及俯仰角范围的范围alpha1, alpha2，自动寻找最接近给定俯仰角的解，并转到目标位置(Given the coordinate 'coordinate_data' and the pitch angle 'alpha', as well as the range of pitch angle 'alpha1' and 'alpha2', automatically search for the solution closest to the given pitch angle and move to the target position)
        #如果无解返回False,否则返回舵机角度、俯仰角、运行时间(If there is no solution, return False. Otherwise, return the servo angle, pitch angle, and running time)
        #坐标单位cm， 以元组形式传入，例如(0, 5, 10)(The coordinate unit is cm and is transmitted in the form of tuple, for example, (0, 5, 10))
        #alpha为给定俯仰角('alpha' represents the given pitch angle)
        #alpha1和alpha2为俯仰角的取值范围('alpha1' and 'alpha2' represent the range of pitch angle values)
        #movetime为舵机转动时间，单位ms, 如果不给出时间，则自动计算('movetime' represents the rotation time of the servo in the unit of ms. If the time is not given, it will be automatically calculated)
        x, y, z = coordinate_data
        result1 = self.setPitchRange((x, y, z), alpha, alpha1)
        result2 = self.setPitchRange((x, y, z), alpha, alpha2)
        if result1 != False:
            data = result1
            if result2 != False:
                if abs(result2[1] - alpha) < abs(result1[1] - alpha):
                    data = result2
        else:
            if result2 != False:
                data = result2
            else:
                return False
        servos, alpha = data[0], data[1]

        movetime = self.servosMove((servos["servo24"], servos["servo23"], servos["servo22"], servos["servo21"]), movetime)

        return servos, alpha, movetime
    
    
if __name__ == "__main__":
    AK = ArmIK()
    
    print(AK.setPitchRangeMoving((0, 10, 25), 0, -90, 100, 1000))
   
    
