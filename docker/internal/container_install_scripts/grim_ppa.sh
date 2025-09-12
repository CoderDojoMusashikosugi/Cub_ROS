export DEBIAN_FRONTEND=noninteractive
apt-get install -y  curl
 
# Automatically setup PPA via online script
curl -s https://koide3.github.io/ppa/setup_ppa.sh | bash
