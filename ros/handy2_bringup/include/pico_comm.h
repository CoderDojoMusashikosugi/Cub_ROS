#pragma once
#include <string>
#include <cstdint>

namespace picoComm {
    // Initialize the 1-Wire connection by finding the DS2431 device in sysfs
    // Automatically scans /sys/bus/w1/devices/ for a 2D-xxxx device.
    bool init();
    
    // Send the enable shutter keep-alive signal (writes 1 to offset 16)
    bool send_shutter_keep_alive();

    // Send the RTC time sync command (writes 7 bytes to offset 0)
    // Year should be 0-99 (e.g. 26 for 2026)
    bool sync_time(int year, int month, int day, int hour, int min, int sec);

    // Set the shutter parameters (writes 8 bytes to offset 8)
    bool set_shutter_params(uint32_t on_time_us, uint32_t offset_us);
}
