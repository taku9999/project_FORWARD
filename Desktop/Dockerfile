FROM nvidia/cuda:11.7.1-devel-ubuntu20.04
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN ln -sf /usr/share/zoneinfo/Asia/Tokyo /etc/localtime

# Install basic package
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
      iproute2 iputils-ping net-tools traceroute \
      curl make nano wget

# Install Git
RUN apt-get install -y software-properties-common && \
    add-apt-repository ppa:git-core/ppa && \
    apt-get update && apt-get upgrade -y && \
    apt-get install -y git

# Install Cmake (for Ubuntu 20.04 focal)
WORKDIR /
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
      software-properties-common ca-certificates gpg wget
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null && \
    echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main' | tee /etc/apt/sources.list.d/kitware.list >/dev/null && \
    apt-get update && apt-get install -y cmake


# ======= [ Python ] ================================================
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
      python3 python3-pip && \
    pip3 install --upgrade pip


# ======= [ PyTorch ] ================================================
# Install PyTorch v2.0.1 (for cuda 11.7)
RUN pip3 install torch==2.0.1 torchvision==0.15.2 torchaudio==2.0.2


# ======= [ Open3D ] ================================================
# Install Open3D v0.18.0 (for cuda 11.7 & pytorch 2.0.x support)
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
      libgl1-mesa-glx && \
    pip3 install tensorboard
RUN pip3 install open3d==0.18.0


# ======= [ Livox-ros-driver ] ================================================
# Install Cmake (for Ubuntu 20.04 focal)
WORKDIR /
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
      software-properties-common ca-certificates gpg wget
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null && \
    echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main' | tee /etc/apt/sources.list.d/kitware.list >/dev/null && \
    apt-get update && apt-get install -y cmake

# Install ROS-Noetic
WORKDIR /
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
      locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
      software-properties-common curl && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg  && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros.list > /dev/null && \
    apt-get update && apt-get install -y ros-noetic-desktop-full && \
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    source ~/.bashrc && \
    apt-get install -y \
      python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool python3-catkin-tools build-essential && \
    rosdep init && \
    rosdep update

# Install Livox-SDK
WORKDIR /
RUN mkdir workspace
WORKDIR /workspace
RUN git clone https://github.com/Livox-SDK/Livox-SDK.git
WORKDIR /workspace/Livox-SDK/build
RUN cmake .. && \
    make && \
    make install

# Install Livox-ros-driver
WORKDIR /workspace
RUN git clone https://github.com/Livox-SDK/livox_ros_driver.git ws_livox/src
WORKDIR /workspace/ws_livox
RUN apt-get update && apt-get install -y python3-empy
RUN source /opt/ros/noetic/setup.sh; catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 && \
    echo "source /workspace/ws_livox/devel/setup.sh" >> ~/.bashrc && \
    source ~/.bashrc


# ======= [ Fast-LIO ] ================================================
# Require for Fast-LIO
RUN pip3 install pcl rospkg

# Install Fast-LIO
WORKDIR /workspace
RUN mkdir -p ws_fast-lio/src && \
    cp -r /workspace/ws_livox/src/livox_ros_driver/ /workspace/ws_fast-lio/src/
WORKDIR /workspace/ws_fast-lio/src
RUN git clone https://github.com/hku-mars/FAST_LIO.git
WORKDIR /workspace/ws_fast-lio/src/FAST_LIO
RUN git submodule update --init
WORKDIR /workspace/ws_fast-lio/
RUN source /opt/ros/noetic/setup.sh; catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 && \
    echo "source /workspace/ws_fast-lio/devel/setup.bash" >> ~/.bashrc && \
    source ~/.bashrc


# ======= [ Git config ] ================================================
WORKDIR /workspace
RUN rm -rf /workspace/ws_livox/src/.git && \
    rm -rf /workspace/ws_livox/src/.gitignore && \
    rm -rf /workspace/Livox-SDK/.git && \
    rm -rf /workspace/Livox-SDK/.gitignore && \
    rm -rf /workspace/ws_fast-lio/src/FAST_LIO/.git && \
    rm -rf /workspace/ws_fast-lio/src/FAST_LIO/.gitignore && \
    rm -rf /workspace/ws_fast-lio/src/FAST_LIO/.gitmodules && \
    rm -rf /workspace/ws_fast-lio/src/FAST_LIO/.github && \
    rm -rf /workspace/ws_fast-lio/src/FAST_LIO/rviz_cfg/.gitignore && \
    rm -rf /workspace/ws_fast-lio/src/FAST_LIO/include/ikd-Tree/.git


# ===== [ ws_fast-lio ] ================================================
# LiDAR config（livox_lidar_config.json）
WORKDIR /workspace/ws_fast-lio/src/livox_ros_driver/config
RUN sed -i '3,11d' livox_lidar_config.json && \
    sed -i '4s/"broadcast_code": "0TFDG3U99101431"/"broadcast_code": "3JEDJCP00136071"/' livox_lidar_config.json && \
    sed -i '5s/"enable_connect": false/"enable_connect": true/' livox_lidar_config.json && \
    sed -i '8s/"imu_rate": 0/"imu_rate": 1/' livox_lidar_config.json

# PCD_mapping config (laserMapping.cpp)
WORKDIR /workspace/ws_fast-lio/src/FAST_LIO/src
RUN sed -i '1019s/file_name = string("scans.pcd")/file_name = string("scans_map.pcd")/' laserMapping.cpp && \
    sed -i '1020s/all_points_dir(string(string(ROOT_DIR) + "PCD\/")/all_points_dir(string("\/workspace\/bind_data\/")/' laserMapping.cpp && \
    sed -i '1022s/"current scan saved to \/PCD\/"/"current scan saved to \/workspace\/bind_data\/"/' laserMapping.cpp
WORKDIR /workspace/ws_fast-lio/
RUN source /opt/ros/noetic/setup.sh; catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3

# PCD_save mapping config (avia.yaml)
WORKDIR /workspace/ws_fast-lio/src/FAST_LIO/config
RUN sed -i '33s/pcd_save_en: true/pcd_save_en: true/' avia.yaml && \
    sed -i '34s/interval: -1/interval: -1/' avia.yaml

# Publish config (livox_lidar_msg.launch)
WORKDIR /workspace/ws_fast-lio/src/livox_ros_driver/launch
RUN sed -i '8s/"publish_freq" default="10.0"/"publish_freq" default="10.0"/' livox_lidar_msg.launch && \
    sed -i '11s/"rosbag_enable" default="false"/"rosbag_enable" default="true"/' livox_lidar_msg.launch && \
    sed -i '40s/args="-a"/args="-a -O \/workspace\/bind_data\/rosbag_202X-XX-XX_XX-XX-XX.bag"/' livox_lidar_msg.launch

# RVIZ config (mapping_avia.launch, loam_livox.rviz)
WORKDIR /workspace/ws_fast-lio/src/FAST_LIO/launch
RUN sed -i '4s/arg name="rviz" default="true"/arg name="rviz" default="false"/' mapping_avia.launch
WORKDIR /workspace/ws_fast-lio/src/FAST_LIO/rviz_cfg
RUN sed -i '298s/Frame Rate: 10/Frame Rate: 60/' loam_livox.rviz && \
    sed -i '119s/Queue Size: 100000/Queue Size: 1000000000/' loam_livox.rviz && \
    sed -i '101s/Alpha: 0.10000000149011612/Alpha: 0.5/' loam_livox.rviz && \
    sed -i '112s/Decay Time: 1000/Decay Time: 3000/' loam_livox.rviz && \
    sed -i '337s/Target Frame: global/Target Frame: body/' loam_livox.rviz