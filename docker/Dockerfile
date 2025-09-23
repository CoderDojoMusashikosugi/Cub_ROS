ARG BASE_IMAGE_NAME
FROM ${BASE_IMAGE_NAME}

LABEL org.opencontainers.image.source=https://github.com/CoderDojoMusashikosugi/Cub_ROS
SHELL ["/bin/bash", "-c"]

# Install additional packages specified in ./docker/envirnment/*.conf
ARG ADDITIONAL_PKGS=""
RUN --mount=type=bind,source=ros/,dst=/ros \
    apt update && \
    apt -y install $ADDITIONAL_PKGS && \
    source /etc/user.bashrc && \
    rosdep install -i -y --from-paths /ros

COPY docker/internal/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
