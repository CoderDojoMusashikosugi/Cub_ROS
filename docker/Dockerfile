ARG BASE_IMAGE_NAME
# FROM ubuntu:22.04
FROM ${BASE_IMAGE_NAME}

LABEL org.opencontainers.image.source=https://github.com/CoderDojoMusashikosugi/Cub_ROS

COPY scripts/libuvc_installation.sh  /libuvc_installation.sh
RUN mkdir /build_temp && cd /build_temp && mv /libuvc_installation.sh ./ && \
    sudo ./libuvc_installation.sh 

COPY docker/additional_pkgs.bash /additional_pkgs.bash
RUN --mount=type=bind,source=ros/,dst=/ros \
    /additional_pkgs.bash && \
    . /opt/ros/humble/setup.sh && \
    rosdep install -i -y --from-paths /ros

# Install editting tool of Behavior Tree
RUN cd /prebuilt_ws && git clone --recurse-submodules https://github.com/BehaviorTree/Groot.git && \
    cd Groot && \
    cmake -S . -B build && cmake --build build -j$(($(nproc)-1)) && cmake --install build && rm -r build

COPY docker/internal/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
