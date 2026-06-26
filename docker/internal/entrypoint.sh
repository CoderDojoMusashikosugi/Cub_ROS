#!/bin/sh

# username: cub
# password: same as username   (openssl passwd $USER_NAME)
if ! id -u $USER_NAME > /dev/null 2>&1; then
    groupadd -g $HOST_GID $USER_NAME
    useradd -u $HOST_UID -g $HOST_GID -m $USER_NAME --password '$1$1MParYB8$R8c8vAbst5CsASYDC8Bmw1' -s /bin/bash
    echo "$USER_NAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
    gpasswd -a $USER_NAME sudo
fi

# Configure chrony for GNSS Time Sync
CHRONY_CONF="/etc/chrony/chrony.conf"
if [ ! -f "$CHRONY_CONF.bak" ]; then
    cp "$CHRONY_CONF" "$CHRONY_CONF.bak"
    cat <<EOF > "$CHRONY_CONF"
# GNSS NMEA data via SHM 0 (provided by gnss_time_node)
refclock SHM 0 poll 0 delay 0.1 refid NMEA precision 1e-1 offset 0.0
# PPS signal from /dev/pps0
refclock PPS /dev/pps0 poll 0 lock NMEA refid PPS
# Allow stepping the clock if it's far off
makestep 1 -1
EOF
fi

# Pre-create SHM segment with 0666 permissions so non-root ROS node can write to it
# 0x4e545030 is NTP0, 96 is the size, 0o1000 is IPC_CREAT, 0o666 is the permission
python3 -c "import ctypes; libc = ctypes.CDLL('libc.so.6'); libc.shmget(0x4e545030, 96, 0o1000 | 0o666)"

# Start chrony
# Remove stale pid file if exists (e.g. after host reboot)
rm -f /var/run/chrony/chronyd.pid
service chrony start

su - $USER_NAME -c "/bin/bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash &&
	    source ~/.user_config.bash &&
	        ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765'"
/bin/bash
