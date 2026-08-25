ARG  BASE_IMAGE=buaadocker.xuhao1.me/ros_base_noetic:arm64
FROM $BASE_IMAGE

ARG ROS_VERSION=noetic
ARG LIO_WS=/root/lio_ws

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y git libgoogle-glog-dev  libsuitesparse-dev libatlas-base-dev htop net-tools libeigen3-dev


RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    mkdir -p Livox-SDK2/build && \
    cd Livox-SDK2/build  && \
    cmake ..  && \
    make -j8  && \
    make install && \
    rm -rf /Livox-SDK2

# Ceres solver for calibration
RUN git clone https://github.com/HKUST-Swarm/ceres-solver -b D2SLAM && \
      cd ceres-solver && \
      mkdir build && cd build && \
      cmake  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF -DCUDA=OFF .. && \
      make -j8 install && \
      rm -rf ../../ceres-solver && \
      apt-get clean all

RUN   mkdir -p ${LIO_WS}/src/ && \
      cd ${LIO_WS}/src/ && \
      git clone https://github.com/Livox-SDK/livox_ros_driver2.git && \
      git clone https://github.com/xuhao1/LiDAR_IMU_Init.git

COPY ./ ${LIO_WS}/src/FAST_LIO/

WORKDIR $LIO_WS
SHELL ["/bin/bash", "-c"]
RUN   . "/opt/ros/${ROS_VERSION}/setup.bash" && \
      cd $LIO_WS/src/livox_ros_driver2 && \
      ./build.sh ROS1 && \
      cd $LIO_WS/ && \
      catkin_make -DCMAKE_BUILD_TYPE=Release
COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
