ARG BASE_IMAGE_NAME
# FROM ubuntu:22.04
FROM ${BASE_IMAGE_NAME}

LABEL org.opencontainers.image.source=https://github.com/CoderDojoMusashikosugi/Cub_ROS

COPY docker/additional_pkgs.bash /additional_pkgs.bash
RUN --mount=type=bind,source=ros/,dst=/ros \
    /additional_pkgs.bash && \
    . /opt/ros/humble/setup.sh && \
    rosdep init && \
    rosdep update --rosdistro=humble && \
    rosdep install -i -y --from-paths /ros

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
