FROM ros:humble-ros-core

SHELL ["/bin/bash", "-c"]

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      ros-humble-foxglove-bridge \
      ros-humble-robot-localization \
      ros-humble-teleop-twist-keyboard \
 && rm -rf /var/lib/apt/lists/*

RUN echo '. /opt/ros/humble/setup.bash' >> /root/.bashrc

CMD ["bash"]
