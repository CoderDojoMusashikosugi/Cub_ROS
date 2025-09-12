#!/bin/bash
apt-get update

for script in "$@"; do
  echo "running: $script"
  ls "/container_install_scripts/${script}"
  "/container_install_scripts/${script}"
done
