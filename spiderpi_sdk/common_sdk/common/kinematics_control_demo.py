#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import numpy as np
import kinematics  # 运动学库已加密，只供调用
from common.ros_robot_controller_sdk import Board

board =Board()
ik = kinematics.IK(board)  # 实例化逆运动学库(instantiate the inverse kinematics library)
ik.stand(ik.initial_pos, t=500)  # 立正，姿态为ik.initial_pos，时间500ms(Stand up at attention, with the posture of ik.initial_pos, for 500ms)
print('姿态:\n', np.array(ik.initial_pos))  # 打印查看姿态(print and check the posture)
# 姿态数据为3x6的数组，表示6条腿的的末端的x，y，z坐标，单位mm(The posture data is a 3x6 array, representing the x, y, and z coordinates of the end of 6 legs, in units of mm)
# 头部朝前的方向为x轴， 头部朝前位置为负方向，右边为y轴正， 竖直朝上为z轴正， 从中间两条腿的连线的中点做垂线与上下板相交，取连线中心为零点(The direction of the head forward is the x-axis, with the position of that as the negative direction, the right side is the positive y-axis, and the vertical upward is the positive z-axis. A perpendicular line is drawn from the midpoint of the line connecting the two middle legs to intersect with the upper and lower boards, and the center of the line is taken as the zero point)
# 第一条腿表示头部朝前时左上角所在腿, 逆时针表示1-6(The first leg represents the leg located in the upper left corner when the head is facing forward, and the legs are numbered counterclockwise from 1 to 6)
# [[-199.53, -177.73, -100.0],
#  [0.0,     -211.27, -100.0],
#  [199.53,  -177.73, -100.0],
#  [199.53,  177.73,  -100.0],
#  [0.0,     211.27,  -100.0],
#  [-199.53, 177.73,  -100.0]]

# 参数1：姿态；参数2：模式，2为六足模式，4为四足模式，当为4足模式时相应的姿态需要为initial_pos_quadruped(Parameter 1: posture; Parameter 2: mode, 2 is for hexapod mode and 4 is for quadruped mode. When in quadruped mode, the corresponding posture should be initial_pos_quadruped)
# 参数3：步幅，单位mm （转弯时为角度，单位度）；参数4：速度，单位mm/s；参数5：执行次数，单位0时表示无限循环(Parameter 3: stride in the unit of mm (with unit degree for turning); Parameter 4: speed in the unit of mm/s; Parameter 5: number of executions, with 0 indicating infinite looping)

ik.go_forward(ik.initial_pos, 2, 50, 80, 1)  # 朝前直走50mm(go straight ahead for 50mm)

ik.back(ik.initial_pos, 2, 100, 80, 1)  # 朝后直走100mm(go straight back for 50mm)

ik.turn_left(ik.initial_pos, 2, 30, 100, 1)  # 原地左转30度(turn left for 30 degrees in place)

ik.turn_right(ik.initial_pos, 2, 30, 100, 1)  # 原地右转30度(turn right for 30 degrees in place)

ik.left_move(ik.initial_pos, 2, 100, 100, 1)  # 左移100mm(move left for 100mm)

ik.right_move(ik.initial_pos, 2, 100, 100, 1)  # 右移100mm(move right for 100mm)

ik.stand(ik.initial_pos, t=500)
