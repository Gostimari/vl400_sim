FROM ros:jazzy-ros-base

LABEL maintainer="Duarte Cruz <duarte.cruz@adai.pt>"

SHELL ["/bin/bash","-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Install packages
RUN apt update \
    && apt install -y \
    build-essential \
    apt-utils \
    curl \
    git \
    git-lfs \
    wget \
    vim \
    nano \
    software-properties-common \
    cmake \
    lsb-release

# Install some python packages
RUN apt -y install \
    python3 \
    python3-scipy


RUN apt -y install \
    ros-$ROS_DISTRO-ament-cmake \
    ros-$ROS_DISTRO-rosidl-default-generators \
    ros-$ROS_DISTRO-rosidl-default-runtime \
    ros-$ROS_DISTRO-std-msgs \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-nav-msgs \
    ros-$ROS_DISTRO-rosgraph-msgs \
    ros-$ROS_DISTRO-rclcpp \
    ros-$ROS_DISTRO-rclpy \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-ament-index-cpp \
    ros-$ROS_DISTRO-gz-sim-vendor \
    ros-$ROS_DISTRO-gz-math-vendor \
    ros-$ROS_DISTRO-gz-msgs-vendor \
    ros-$ROS_DISTRO-gz-transport-vendor \
    ros-$ROS_DISTRO-gz-plugin-vendor \
    ros-$ROS_DISTRO-ros-gz-sim \
    ros-$ROS_DISTRO-ros-gz-bridge \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-cyclonedds

#Configure catkin workspace
ENV ROS2_WS=/root/ros2_ws
RUN mkdir -p $ROS2_WS/src

WORKDIR $ROS2_WS/src
RUN git lfs install
RUN git clone https://github.com/Gostimari/vl400_sim.git

#Compile workspace
WORKDIR $ROS2_WS
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc
RUN echo "cd ros2_ws/" >> ~/.bashrc
RUN echo "export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia" >> ~/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

# Clean-up
WORKDIR /root
RUN apt upgrade -y

CMD ["bash"]
