# Written with Claude

ARG IDF_VERSION=release-v5.5
FROM espressif/idf:${IDF_VERSION}

# micro-ROS build dependencies
RUN IDF_PATH_FORCE=1 . $IDF_PATH/export.sh && \
    pip3 install --no-cache-dir catkin_pkg colcon-common-extensions lark

# clangd for in-container LSP (see scripts/clangd.sh, .nvim.lua)
# See scripts/clangd.sh
RUN IDF_PATH_FORCE=1 . $IDF_PATH/export.sh && \
    python "$IDF_PATH/tools/idf_tools.py" install esp-clang && \
    ln -sf "$(ls /opt/esp/tools/esp-clang/*/esp-clang/bin/clangd | head -1)" \
        /usr/local/bin/clangd

ENV LC_ALL=C.UTF-8
ENV LANG=C.UTF-8
