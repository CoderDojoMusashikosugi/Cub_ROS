apt update && apt -y install wget
wget -P /etc/apt/keyrings/ http://archive.raspberrypi.com/debian/raspberrypi.gpg.key
echo "deb [signed-by=/etc/apt/keyrings/raspberrypi.gpg.key] http://archive.raspberrypi.com/debian/ bookworm main" > /etc/apt/sources.list.d/raspi.list
apt update && apt install -y libpigpio-dev
