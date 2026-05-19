apt -y install git python3-ply meson python3-jinja2 python3-yaml
git clone https://github.com/raspberrypi/libcamera.git
cd libcamera
meson setup build --buildtype=release -Dgstreamer=disabled -Dpycamera=disabled
ninja -C build install -j 2
