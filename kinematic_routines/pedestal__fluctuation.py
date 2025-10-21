#!/usr/bin/python3
# coding=utf8
#第9章 机械臂正逆运动学课程\2.机械臂正逆运动学实战应用\第3课 底盘高度调节(9.Forward & Inverse Kinematic/2.Forward & Inverse Kinematics Practical Application\Lesson 3 Chassis Height Adjustment)
import time
import copy
from common import kinematics
from common.ros_robot_controller_sdk import Board
import arm_ik.arm_move_ik as AMK

# 底盘上下移动(chassis up and down movement)

board = Board()
ik = kinematics.IK(board)
ak = AMK.ArmIK()


def Stand(height, mode, t):
    if height > 160:
        ik.current_pos = copy.deepcopy(ik.initial_pos_high)
    else:
        ik.current_pos = copy.deepcopy(ik.initial_pos)
        
    pos = ik.current_pos
    for i in range(6):
        pos[i][2] = -float(height)
        
    ik.stand(pos, t)
        
      

if __name__ == '__main__':
    ak.setPitchRangeMoving((0, 15, 30), 0, -90, 100, 2)
    ik.stand(ik.initial_pos)
    time.sleep(2)

    Stand(50,2,1000)
    for i in range(3):
        Stand(150,2,2000)
        
        Stand(50,2,2000)
    
    Stand(100,2,1000)