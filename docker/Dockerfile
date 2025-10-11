ARG BASE_IMAGE_NAME
FROM ${BASE_IMAGE_NAME}

LABEL org.opencontainers.image.source=https://github.com/CoderDojoMusashikosugi/Cub_ROS
SHELL ["/bin/bash", "-c"]

ENV ROS_DISTRO=jazzy
# Install additional packages specified in ./docker/envirnment/*.conf
ARG ADDITIONAL_PKGS=""
RUN --mount=type=bind,source=ros/,dst=/ros \
    DEBIAN_FRONTEND=noninteractive apt update && \
    DEBIAN_FRONTEND=noninteractive apt -y install python3-rosdep $ADDITIONAL_PKGS
       # ensure ROS environment is available for rosdep
RUN --mount=type=bind,source=ros/,dst=/ros \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep init || true && \
    rosdep update || true && \
    rosdep install -i --skip-keys micro_ros_agent -y --from-paths ./ros

COPY docker/internal/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
