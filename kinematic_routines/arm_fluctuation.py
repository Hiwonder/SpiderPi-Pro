#!/usr/bin/python3
# coding=utf8
#第9章 机械臂正逆运动学课程\2.机械臂正逆运动学实战应用\第2课 机械臂高度调节(9.Forward & Inverse Kinematic/2.Forward & Inverse Kinematics Practical Application\Lesson 2 Robotic Arm Height Adjustment)
import time
from common import kinematics
from common.ros_robot_controller_sdk import Board
import arm_ik.arm_move_ik as AMK


board = Board()
ik = kinematics.IK(board)
ak = AMK.ArmIK()


# 机械臂上下移动(robotic arm moves up and down)
     

if __name__ == '__main__':
    ak.setPitchRangeMoving((0, 15, 30), 0, -90, 100, 2)
    ik.stand(ik.initial_pos)
    time.sleep(2)
    
    ak.setPitchRangeMoving((0, 15, 35), 0, -90, 100, 1)
    time.sleep(1)
    for i in range(3):
       ak.setPitchRangeMoving((0, 15, 25), 0, -90, 100, 2)
       time.sleep(2)
       ak.setPitchRangeMoving((0, 15, 35), 0, -90, 100, 2)
       time.sleep(2)
        
    ak.setPitchRangeMoving((0, 15, 30), 0, -90, 100, 1)
    time.sleep(1)