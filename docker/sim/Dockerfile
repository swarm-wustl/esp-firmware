FROM ros:humble-ros-core

SHELL ["/bin/bash", "-c"]

# humble to match the agent and the micro-ROS component, but the firmware
# headers the sim compiles need g++ 13 (std::unreachable, ranges::fold_left)
# and jammy ships 11
RUN apt-get update \
 && apt-get install -y --no-install-recommends software-properties-common \
 && add-apt-repository -y ppa:ubuntu-toolchain-r/test \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
      cmake \
      g++-13 \
      make \
      ros-humble-ament-cmake \
      ros-humble-foxglove-bridge \
      ros-humble-robot-localization \
      ros-humble-teleop-twist-keyboard \
      ros-humble-tf2-msgs \
      ros-humble-tf2-ros \
 && rm -rf /var/lib/apt/lists/*

ENV CXX=g++-13

RUN echo '. /opt/ros/humble/setup.bash' >> /root/.bashrc

CMD ["bash"]
