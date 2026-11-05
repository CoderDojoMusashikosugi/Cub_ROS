FROM ros:noetic-ros-base-focal
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential ros-noetic-pcl-ros ros-noetic-grid-map python3-yaml \
    && rm -rf /var/lib/apt/lists/*
COPY source /ws/src/map_cleaner
COPY patch_batch.py /tmp/patch_batch.py
RUN python3 /tmp/patch_batch.py /ws/src/map_cleaner
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash && cd /ws && catkin_make -j2 -DCMAKE_BUILD_TYPE=Release'
COPY batch.launch /ws/src/map_cleaner/launch/cub_batch.launch
COPY entrypoint.sh /cub_entrypoint.sh
ENTRYPOINT ["/bin/bash", "/cub_entrypoint.sh"]
