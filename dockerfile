FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV COLCON_WS=/root/ros2_ws
WORKDIR ${COLCON_WS}

RUN apt-get update && apt-get install -y \
    apt-get update && apt-get install -y \
    git \
    python3-opencv python3-numpy \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-vision-opencv \
    ros-${ROS_DISTRO}-ros-base \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    udev \
    vim nano \
 && rm -rf /var/lib/apt/lists/*


RUN rosdep init || true && rosdep update


RUN mkdir -p ${COLCON_WS}/src

# Host의 blue_segmentation 패키지를 컨테이너로 복사
# (빌드 컨텍스트에서 src/blue_segmentation 이 존재해야 함)
COPY src/blue_segmentation ${COLCON_WS}/src/blue_segmentation


# 의존성 설치 (librealsense2는 호스트/다른 컨테이너에서 제공 시 skip)
RUN apt-get update && \
    rosdep update && \
    rm -rf /var/lib/apt/lists/*

# 빌드
RUN bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${COLCON_WS} && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# 편의: 셸 진입 시 ROS/WS 자동 소스
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source ${COLCON_WS}/install/setup.bash" >> /root/.bashrc && \
    echo "alias cb='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=30" >> /root/.bashrc


CMD ["bash"]
