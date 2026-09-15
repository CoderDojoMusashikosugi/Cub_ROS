#!/bin/bash

echo "This script setups host autostart settings."
if [ $(whoami) != "root" ]; then
    echo "[error] run as sudo"
    echo "for example: sudo ./install_host_settings.sh"
    exit
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

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

# setup pps input
DTS_FILE="${SCRIPT_DIR}/../support_tools/jetson_pps/jetson-pps-gpio07.dts"
DTBO_FILE="/boot/jetson-pps-gpio07.dtbo"
EXTLINUX_CONF="/boot/extlinux/extlinux.conf"
FDT_FILE="/boot/dtb/kernel_tegra234-p3768-0000+p3767-0005-nv-super.dtb"
if [ ! -f "$FDT_FILE" ]; then
    FALLBACK_FDT=$(ls /boot/dtb/kernel_tegra234*.dtb 2>/dev/null | head -n 1)
    if [ -n "$FALLBACK_FDT" ]; then
        FDT_FILE="$FALLBACK_FDT"
    fi
fi

if [ -f "$DTS_FILE" ]; then
    echo "[INFO] Compiling PPS device tree overlay ($DTS_FILE -> $DTBO_FILE)..."
    dtc -@ -I dts -O dtb -o "$DTBO_FILE" "$DTS_FILE"
else
    echo "[ERROR] DTS file not found: $DTS_FILE"
fi

# Configure extlinux.conf (FDT and OVERLAYS for PPS)
if [ -f "$EXTLINUX_CONF" ]; then
    HAS_PPS_OVERLAY=false
    if grep -q "jetson-pps-gpio07\.dtbo" "$EXTLINUX_CONF"; then
        HAS_PPS_OVERLAY=true
    fi

    HAS_FDT=false
    if grep -q "^[[:space:]]*FDT[[:space:]]" "$EXTLINUX_CONF"; then
        HAS_FDT=true
    fi

    if [ "$HAS_PPS_OVERLAY" = true ] && [ "$HAS_FDT" = true ]; then
        echo "[INFO] extlinux.conf is already configured with FDT and PPS OVERLAYS. Skipping."
    else
        echo "[INFO] Updating $EXTLINUX_CONF for PPS..."
        if [ ! -f "${EXTLINUX_CONF}.bak" ]; then
            cp "$EXTLINUX_CONF" "${EXTLINUX_CONF}.bak"
            echo "[INFO] Created backup: ${EXTLINUX_CONF}.bak"
        fi

        if [ "$HAS_FDT" = false ] && [ "$HAS_PPS_OVERLAY" = false ]; then
            if grep -q "^[[:space:]]*INITRD[[:space:]]" "$EXTLINUX_CONF"; then
                sed -i '/^[[:space:]]*INITRD[[:space:]]/a \      FDT '"$FDT_FILE"'\n      OVERLAYS '"$DTBO_FILE" "$EXTLINUX_CONF"
            elif grep -q "^[[:space:]]*APPEND[[:space:]]" "$EXTLINUX_CONF"; then
                sed -i '/^[[:space:]]*APPEND[[:space:]]/i \      FDT '"$FDT_FILE"'\n      OVERLAYS '"$DTBO_FILE" "$EXTLINUX_CONF"
            fi
            echo "[INFO] Added FDT and OVERLAYS to $EXTLINUX_CONF."
        elif [ "$HAS_FDT" = true ] && [ "$HAS_PPS_OVERLAY" = false ]; then
            sed -i '/^[[:space:]]*FDT[[:space:]]/a \      OVERLAYS '"$DTBO_FILE" "$EXTLINUX_CONF"
            echo "[INFO] Added OVERLAYS to $EXTLINUX_CONF."
        elif [ "$HAS_FDT" = false ] && [ "$HAS_PPS_OVERLAY" = true ]; then
            sed -i '/^[[:space:]]*OVERLAYS[[:space:]]/i \      FDT '"$FDT_FILE" "$EXTLINUX_CONF"
            echo "[INFO] Added FDT to $EXTLINUX_CONF."
        fi
    fi
else
    echo "[WARN] $EXTLINUX_CONF not found. Skipping extlinux configuration."
fi


echo 'KERNEL=="ttyUSB*",  ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="ttyATOM"' > /etc/udev/rules.d/99-atom.rules
# echo 'KERNEL=="ttyACM*",  ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="ttyGPS"' > /etc/udev/rules.d/99-gps.rules
echo 'KERNEL=="ttyUSB*", ENV{ID_SERIAL_SHORT}=="b69c7db1d29de8118347301338b01545", SYMLINK+="ttyMULIMU"' > /etc/udev/rules.d/99-multiIMU.rules
echo "reboot to apply"
