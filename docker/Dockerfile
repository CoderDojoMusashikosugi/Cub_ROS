ARG BASE_IMAGE_NAME
FROM ${BASE_IMAGE_NAME}

LABEL org.opencontainers.image.source=https://github.com/CoderDojoMusashikosugi/Cub_ROS

COPY scripts/libuvc_installation.sh  /libuvc_installation.sh
RUN mkdir /build_temp && cd /build_temp && mv /libuvc_installation.sh ./ && \
    sudo ./libuvc_installation.sh 

# Install additional packages specified in configuration
ARG ADDITIONAL_PKGS=""
RUN --mount=type=bind,source=ros/,dst=/ros \
    apt update && \
    apt -y install $ADDITIONAL_PKGS && \
    . /opt/ros/humble/setup.sh && \
    rosdep install -i -y --from-paths /ros

# Install editting tool of Behavior Tree
RUN cd /prebuilt_ws && apt -y install qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev && \
    git clone --recurse-submodules https://github.com/BehaviorTree/Groot.git && \
    cd Groot && \
    cmake -S . -B build && cmake --build build -j$(($(nproc)-1)) && cmake --install build && rm -r build

COPY docker/internal/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
