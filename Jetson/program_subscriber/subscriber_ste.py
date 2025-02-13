#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from livox_ros_driver.msg import CustomMsg
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import datetime
import time
import os
from multiprocessing import Process, Value, Array


MSG_TYPE = 1 # [0]=PointCloud2, [1]=CustomMsg
GPS_FLAG = False
GPS_LOG_DIRS = "/workspace/bind_data"


def get_device_time(cmd):
    tmp_time = datetime.datetime.now()

    # datetime.datetime
    date_time = tmp_time + datetime.timedelta(hours=9)
    # float
    unix_time = date_time.timestamp()
    # convert(no-milliseconds)
    conv_str = date_time.strftime("%Y-%m-%d_%H-%M-%S")
    # convert(milliseconds)
    conv_str_milli = date_time.strftime("%Y-%m-%d_%H-%M-%S_%f")

    if cmd == "date_time":
        return date_time
    elif cmd == "unix_time":
        return unix_time
    elif cmd == "conv_str":
        return conv_str
    elif cmd == "conv_str_milli":
        return conv_str_milli


def callback_lidar(point_cloud):
    
    ros_time_raw = point_cloud.header.stamp # ROS時間
    jetson_time_unix = time.time() # Jetson時間
    
    print("\n========================================")
    ros_time_unix = float(str(ros_time_raw.secs) + "." + str(ros_time_raw.nsecs).zfill(9)[:3])
    ros_time_dt = datetime.datetime.utcfromtimestamp(ros_time_unix)
    ros_time_dt_jst = ros_time_dt + datetime.timedelta(hours=9)
    print("ROS_Time: " + ros_time_dt_jst.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3])
    
    jetson_time_dt = datetime.datetime.utcfromtimestamp(jetson_time_unix)
    jetson_time_dt_jst = jetson_time_dt + datetime.timedelta(hours=9)
    print("Jetson_Time: " + jetson_time_dt_jst.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3])

    diff_time = jetson_time_unix - ros_time_unix
    print("[Time] Delay: " + str(diff_time))

    if MSG_TYPE == 0:
        points = pc2.read_points(point_cloud, field_names=("x", "y", "z", "intensity"), skip_nans=True)
        num_points = sum(1 for _ in points)
        print("[Type] PointCloud2")
        print("[Message] Point_Num: " + str(num_points))
    elif MSG_TYPE == 1:
        num_points = point_cloud.point_num
        print("[Type] CustomMsg")
        print("[Message] Point_Num: " + str(num_points))


def process_ros():
    rospy.init_node("research_subscriber")

    if MSG_TYPE == 0:
        rospy.Subscriber("/livox/lidar", PointCloud2, callback_lidar)
    elif MSG_TYPE == 1:
        rospy.Subscriber("/livox/lidar", CustomMsg, callback_lidar)

    rospy.spin()


def process_gps(log_filepass):
    print("")


def main():
    if GPS_FLAG == True:
        # Logファイルパスの設定
        log_filepass = GPS_LOG_DIRS + "/" + "log_" + str(get_device_time("conv_str"))
        os.makedirs(log_filepass)
        print("[Debug] Log file save to: {}".format(log_filepass))
        
        # GPSプロセスの設定
        p_gps = Process(target=process_gps, args=(log_filepass,))
        p_gps.start()

    # ROSプロセスの設定
    p_ros = Process(target=process_ros)
    p_ros.start()


if __name__ == "__main__":
    main()