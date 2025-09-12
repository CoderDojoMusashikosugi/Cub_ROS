apt update && apt -y install wget
wget -P /etc/apt/keyrings/ http://archive.raspberrypi.com/debian/raspberrypi.gpg.key
echo "deb [signed-by=/etc/apt/keyrings/raspberrypi.gpg.key] http://archive.raspberrypi.com/debian/ bookworm main" > /etc/apt/sources.list.d/raspi.list
sudo tee /etc/apt/preferences.d/90-raspberrypi-pin > /dev/null <<'EOF'
Package: *
Pin: origin "archive.raspberrypi.com"
Pin-Priority: 100
EOF
apt update && apt install -y libpigpio-dev
