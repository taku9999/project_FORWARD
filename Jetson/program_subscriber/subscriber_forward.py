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
from pymavlink import mavutil


# ---------- 点群関係の設定 ----------
MSG_TYPE = 1 # [0]=PointCloud2, [1]=CustomMsg

# ---------- GPS関係の設定 ----------
GPS_FLAG = False
LOG_DIRS = "/workspace/bind_data"
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200



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


def callback_lidar(point_cloud, args):
    lidar_time = args[0]
    jetson_time = args[1]
    gps_lat = args[2]
    gps_lng = args[3]
    gps_alt = args[4]
    
    lidar_time_raw = point_cloud.header.stamp # LiDAR時間
    jetson_time_unix = time.time() # Jetson時間
    
    print("\n========================================")
    lidar_time_unix = float(str(lidar_time_raw.secs) + "." + str(lidar_time_raw.nsecs).zfill(9)[:3])
    lidar_time_dt = datetime.datetime.utcfromtimestamp(lidar_time_unix)
    lidar_time_dt_jst = lidar_time_dt + datetime.timedelta(hours=9)
    print("LiDAR_Time: " + lidar_time_dt_jst.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3])
    
    jetson_time_dt = datetime.datetime.utcfromtimestamp(jetson_time_unix)
    jetson_time_dt_jst = jetson_time_dt + datetime.timedelta(hours=9)
    print("Jetson_Time: " + jetson_time_dt_jst.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3])

    if MSG_TYPE == 0:
        points = pc2.read_points(point_cloud, field_names=("x", "y", "z", "intensity"), skip_nans=True)
        num_points = sum(1 for _ in points)
        print("[Type] PointCloud2")
        print("[Message] Point_Num: " + str(num_points))
    elif MSG_TYPE == 1:
        num_points = point_cloud.point_num
        print("[Type] CustomMsg")
        print("[Message] Point_Num: " + str(num_points))
        
    print("[GPS] lat={}, lng={}, alt={}".format(gps_lat.value, gps_lng.value, gps_alt.value))
    
    # 共有メモリの更新
    lidar_time.value = lidar_time_unix
    jetson_time.value = jetson_time_unix


def process_ros(g_lidar_time, g_jetson_time, g_gps_lat, g_gps_lng, g_gps_alt):
    rospy.init_node("research_subscriber")

    if MSG_TYPE == 0:
        rospy.Subscriber("/livox/lidar", PointCloud2, callback_lidar, (g_lidar_time, g_jetson_time, g_gps_lat, g_gps_lng, g_gps_alt))
    elif MSG_TYPE == 1:
        rospy.Subscriber("/livox/lidar", CustomMsg, callback_lidar, (g_lidar_time, g_jetson_time, g_gps_lat, g_gps_lng, g_gps_alt))

    rospy.spin()


def process_gps(g_lidar_time, g_jetson_time, g_gps_lat, g_gps_lng, g_gps_alt, g_gps_time):
    # ログファイルの設定
    log_file_path = LOG_DIRS + "/log_" + str(get_device_time("conv_str")) + ".csv"
    print("[Debug] Log file save to: {}".format(log_file_path))
    csv_header = "jetson_time,lidar_time,gps_lat,gps_lng,gps_alt,gps_time"
    with open(log_file_path, mode="w") as f:
        f.write(csv_header + "\n")
    
    # シリアルポート経由でMAVLink接続を確立
    the_connection = mavutil.mavlink_connection(SERIAL_PORT, BAUD_RATE)
    print("[Debug] MAVLink connected! Waiting data...")
    the_connection.wait_heartbeat()
    print("[Debug] MAVLink heartbeat receive!")
    
    # GPS情報の取得とログ出力
    while True:
        location = the_connection.location()
        g_gps_lat.value = location.lat
        g_gps_lng.value = location.lng
        g_gps_alt.value = location.alt
        g_gps_time.value = the_connection.timestamp
        
        with open(log_file_path, mode="a") as f:
            f.write(str(g_jetson_time.value))
            f.write("," + str(g_lidar_time.value))
            f.write("," + str(location.lat))
            f.write("," + str(location.lng))
            f.write("," + str(g_gps_alt.value))
            f.write("," + str(g_gps_time.value) + "\n")


def main():
    # 共有メモリの設定
    m_lidar_time = Value("d", 0.0)
    m_jetson_time = Value("d", 0.0)
    m_gps_lat = Value("d", 0.0)
    m_gps_lng = Value("d", 0.0)
    m_gps_alt = Value("d", 0.0)
    m_gps_time = Value("d", 0.0)
    
    # GPSプロセスの設定
    if GPS_FLAG == True:
        p_gps = Process(target=process_gps, args=(m_lidar_time, m_jetson_time, m_gps_lat, m_gps_lng, m_gps_alt, m_gps_time))
        p_gps.start()

    # ROSプロセスの設定
    p_ros = Process(target=process_ros, args=(m_lidar_time, m_jetson_time, m_gps_lat, m_gps_lng, m_gps_alt))
    p_ros.start()


if __name__ == "__main__":
    main()
