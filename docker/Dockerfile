ARG BASE_IMAGE_NAME
# FROM ubuntu:22.04
FROM ${BASE_IMAGE_NAME}

LABEL org.opencontainers.image.source=https://github.com/CoderDojoMusashikosugi/Cub_ROS

COPY docker/additional_pkgs.bash /additional_pkgs.bash
RUN /additional_pkgs.bash

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
