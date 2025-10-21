import os, sys
dir_path = '/home/pi/spiderpi/spiderpi_sdk/camera_calibration_sdk/calibration'

#相邻两个角点间的实际距离，单位cm(The actual distance between adjacent corner points is in the unit of cm)
corners_length = 1.4

#木块边长3cm(the side length of the block is 3cm)
square_length = 3

#标定棋盘大小, 列， 行, 指内角点个数，非棋盘格(The calibration chessboard size is determined by the number of internal corners, excluding the non-chessboard corners, in both the row and column directions)
calibration_size = (8, 6)

#采集标定图像存储路径(storage path for calibration image acquisition)
save_path = dir_path + '/calibration_images/'

#标定参数存储路径(storage path for the calibration parameters)
calibration_param_path = dir_path + '/calibration_param'

#映射参数存储路径(storage path for mapping parameters)
map_param_path = dir_path + '/map_param'
