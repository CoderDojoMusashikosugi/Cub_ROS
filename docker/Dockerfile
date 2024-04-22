FROM ubuntu:22.04

COPY docker/install_ros.bash /install_ros.bash

RUN sed -i.bak -e "s/http:\/\/ports\.ubuntu\.com/http:\/\/jp\.ports\.ubuntu\.com/g" /etc/apt/sources.list && \
    apt update && \
    apt -y install sudo x11-apps net-tools terminator && \
    /install_ros.bash && \
    rm -rf /var/lib/apt/lists/

COPY docker/additional_pkgs.bash /additional_pkgs.bash
RUN /additional_pkgs.bash

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
