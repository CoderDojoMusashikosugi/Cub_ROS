ARG BASE_IMAGE_NAME
ARG ROS_DISTRO=humble
FROM ${BASE_IMAGE_NAME}

LABEL org.opencontainers.image.source=https://github.com/CoderDojoMusashikosugi/Cub_ROS
SHELL ["/bin/bash", "-c"]

COPY docker/rosdep.yaml /etc/ros/rosdep/cub.yaml
RUN echo "yaml file:///etc/ros/rosdep/cub.yaml" > /etc/ros/rosdep/sources.list.d/10-cub.list

# Install additional packages specified in ./docker/envirnment/*.conf
ARG ADDITIONAL_PKGS=""
RUN --mount=type=bind,source=ros/,dst=/ros \
    apt update && \
    apt -y install $ADDITIONAL_PKGS && \
    source /etc/user.bashrc && \
    rosdep update --rosdistro="${ROS_DISTRO}" && \
    rosdep install -i -y --from-paths /ros

COPY docker/internal/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
