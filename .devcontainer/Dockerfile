# Custom build environment for the swarm project.
#
# The official microros/esp-idf-microros image is stale (ESP-IDF v5.0, 2023),
# which is missing APIs the code uses (e.g. ESP_INTR_CPU_AFFINITY_AUTO, added in
# IDF v5.1). We instead base off Espressif's official IDF image and add the
# micro-ROS Python build dependencies. The micro_ros_espidf_component itself
# compiles from source out of components/, so nothing else is needed.
#
# Bump the IDF version by changing the tag below (release-v5.2 ... release-v5.5).
ARG IDF_VERSION=release-v5.5
FROM espressif/idf:${IDF_VERSION}

# micro-ROS build dependencies, installed into the IDF Python virtualenv.
RUN . $IDF_PATH/export.sh && \
    pip3 install --no-cache-dir catkin_pkg colcon-common-extensions lark

ENV LC_ALL=C.UTF-8
ENV LANG=C.UTF-8
