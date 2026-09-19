# Written with Claude

# pinned by digest: espressif/idf:release-v5.5 is a mutable tag and drifted out
# from under this repo once already (empy 3 -> 4 broke micro-ROS codegen)
ARG IDF_IMAGE=espressif/idf@sha256:5976da28fd46671f66d8f96908e166f99bc34d1fc24f516ead9a8fbef9bdc109
FROM ${IDF_IMAGE}

# micro-ROS build dependencies
# empy must stay on 3.x: rosidl's generators use em.BUFFERED_OPT, dropped in empy 4
RUN IDF_PATH_FORCE=1 . $IDF_PATH/export.sh && \
    pip3 install --no-cache-dir catkin_pkg colcon-common-extensions lark "empy==3.3.4"

# test runner: pytest-embedded drives the unity menu (host + device); see test/run.py
# pytest-subtests reports each unity case individually
RUN IDF_PATH_FORCE=1 . $IDF_PATH/export.sh && \
    pip3 install --no-cache-dir pytest-embedded pytest-embedded-idf pytest-embedded-serial-esp pytest-subtests

# clangd for in-container LSP (see scripts/clangd.sh, .nvim.lua)
# See scripts/clangd.sh
RUN IDF_PATH_FORCE=1 . $IDF_PATH/export.sh && \
    python "$IDF_PATH/tools/idf_tools.py" install esp-clang && \
    ln -sf "$(ls /opt/esp/tools/esp-clang/*/esp-clang/bin/clangd | head -1)" \
        /usr/local/bin/clangd

ENV LC_ALL=C.UTF-8
ENV LANG=C.UTF-8
