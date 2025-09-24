#!/bin/bash

export DEBIAN_FRONTEND=noninteractive
apt update && apt -y install ros-humble-foxglove-bridge
