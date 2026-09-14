#!/bin/bash

echo "This script setups host autostart settings."
if [ $(whoami) != "root" ]; then
    echo "[error] run as sudo"
    echo "for example: sudo ./mcub_autostart_settings.sh"
    exit
fi

# Configure Swap (Expand to 16GB if current swap is ~2GB)
SWAP_FILE="/swapfile"
if [ -f "$SWAP_FILE" ]; then
    SWAP_BYTES=$(stat -c %s "$SWAP_FILE" 2>/dev/null || echo 0)
    # Check if swap is ~2GB (between ~1.9GB and ~2.1GB, 2GiB = 2147483648 bytes)
    if [ "$SWAP_BYTES" -ge 2040109465 ] && [ "$SWAP_BYTES" -le 2254857830 ]; then
        echo "[INFO] Current swap is ~2GB. Replacing with 16GB swap..."
        swapoff "$SWAP_FILE" 2>/dev/null || true
        rm -f "$SWAP_FILE"
        fallocate -l 16G "$SWAP_FILE"
        chmod 600 "$SWAP_FILE"
        mkswap "$SWAP_FILE"
        swapon "$SWAP_FILE"
        echo "[INFO] 16GB swap configured successfully."
    elif [ "$SWAP_BYTES" -eq 17179869184 ]; then
        echo "[INFO] Swap is already 16GB. Skipping swap creation."
    else
        echo "[WARN] Swap file exists with size ${SWAP_BYTES} bytes (not 2GB). Skipping swap replacement."
    fi
else
    echo "[INFO] No swapfile found at ${SWAP_FILE}. Creating 16GB swap..."
    fallocate -l 16G "$SWAP_FILE"
    chmod 600 "$SWAP_FILE"
    mkswap "$SWAP_FILE"
    swapon "$SWAP_FILE"
    echo "[INFO] 16GB swap configured successfully."
fi

# Ensure swap persistence in /etc/fstab
if ! grep -q "^[[:space:]]*${SWAP_FILE}" /etc/fstab; then
    echo "${SWAP_FILE} none swap sw 0 0" >> /etc/fstab
    echo "[INFO] Added ${SWAP_FILE} to /etc/fstab."
fi

echo 'KERNEL=="ttyUSB*",  ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="ttyATOM"' > /etc/udev/rules.d/99-atom.rules
# echo 'KERNEL=="ttyACM*",  ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="ttyGPS"' > /etc/udev/rules.d/99-gps.rules
echo 'KERNEL=="ttyUSB*", ENV{ID_SERIAL_SHORT}=="b69c7db1d29de8118347301338b01545", SYMLINK+="ttyMULIMU"' > /etc/udev/rules.d/99-multiIMU.rules
echo "reboot to apply"
