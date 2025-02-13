import subprocess
import datetime

# 現在の日時を取得してフォーマット
timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

# ROS Launchコマンドを構築
launch_command = [
    "screen",
    "roslaunch",
    "livox_ros_driver",
    "livox_lidar_msg.launch",
    f"rosbag_file_name:={timestamp}"
]

# コマンドを実行して出力をリアルタイムで表示
try:
    subprocess.run(launch_command, check=True)
except subprocess.CalledProcessError as e:
    print(f"Launch failed with error: {e}")

