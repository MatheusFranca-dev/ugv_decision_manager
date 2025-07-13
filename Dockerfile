FROM nvidia/cudagl:11.1.1-base-ubuntu20.04
 
# Minimal setup
RUN apt-get update \
 && apt-get install -y locales lsb-release curl
ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-reconfigure locales
ARG ROS_DISTRO=noetic

# Install ROS ROS_DISTRO
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update \
 && apt-get install -y ros-${ROS_DISTRO}-desktop-full
RUN apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
RUN rosdep init \
 && rosdep fix-permissions \
 && rosdep update

ARG PROJECT=decision_manager_ws
ARG WS_PATH=/home/${PROJECT}

WORKDIR ${WS_PATH}/src
RUN git clone https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2.git && \
    git clone https://github.com/BehaviorTree/Groot.git && \
    git clone https://github.com/MatheusFranca-dev/ugv_decision_manager.git

# Install POLARIS repo
RUN apt install -y ros-${ROS_DISTRO}-ackermann-msgs ros-${ROS_DISTRO}-geometry2 \
    ros-${ROS_DISTRO}-hector-gazebo ros-${ROS_DISTRO}-hector-models ros-${ROS_DISTRO}-jsk-rviz-plugins \
    ros-${ROS_DISTRO}-ros-control ros-${ROS_DISTRO}-ros-controllers ros-${ROS_DISTRO}-velodyne-simulator

# Install and Build Groot
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-behaviortree-cpp-v3 ros-${ROS_DISTRO}-mbf-msgs \
    qt5-default ros-${ROS_DISTRO}-moveit-ros ros-${ROS_DISTRO}-moveit-visual-tools \
    qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev
WORKDIR ${WS_PATH}/src/Groot
RUN git submodule update --init --recursive && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install

RUN apt install -y python3-pip
RUN pip3 install numpy
RUN pip3 install matplotlib
RUN apt install -y ros-${ROS_DISTRO}-robot-localization

WORKDIR ${WS_PATH}
RUN apt install -y ros-${ROS_DISTRO}-catkin
RUN /bin/bash -c '. /opt/ros/${ROS_DISTRO}/setup.sh \
        && catkin_make'
