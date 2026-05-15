#include "pico_comm.h"
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <string.h>

namespace picoComm {
    static std::string w1_device_path = "";

    bool init() {
        DIR *dir = opendir("/sys/bus/w1/devices/");
        if (!dir) {
            std::cerr << "Error: /sys/bus/w1/devices/ not found. Is w1-gpio enabled?" << std::endl;
            return false;
        }

        struct dirent *entry;
        while ((entry = readdir(dir)) != NULL) {
            if (strncmp(entry->d_name, "2d-", 3) == 0 || strncmp(entry->d_name, "2D-", 3) == 0) {
                w1_device_path = std::string("/sys/bus/w1/devices/") + entry->d_name + "/eeprom";
                break;
            }
        }
        closedir(dir);

        if (w1_device_path.empty()) {
            std::cerr << "Error: No DS2431 (Family 2D) device found on 1-Wire bus." << std::endl;
            return false;
        }

        std::cout << "Found 1-Wire EEPROM: " << w1_device_path << std::endl;
        return true;
    }

    static bool write_eeprom(int offset, const uint8_t* data, size_t len) {
        if (w1_device_path.empty()) return false;
        
        std::fstream fs(w1_device_path, std::ios::in | std::ios::out | std::ios::binary);
        if (!fs.is_open()) {
            std::cerr << "Failed to open EEPROM for writing." << std::endl;
            return false;
        }
        
        fs.seekp(offset);
        fs.write((const char*)data, len);
        bool success = fs.good();
        fs.close();
        return success;
    }

    bool send_shutter_keep_alive() {
        uint8_t val = 1;
        return write_eeprom(16, &val, 1);
    }

    bool sync_time(int year, int month, int day, int hour, int min, int sec) {
        uint8_t buf[8] = {0};
        buf[0] = (uint8_t)year;
        buf[1] = (uint8_t)month;
        buf[2] = (uint8_t)day;
        buf[3] = (uint8_t)hour;
        buf[4] = (uint8_t)min;
        buf[5] = (uint8_t)sec;
        buf[6] = 1; // Update flag
        
        return write_eeprom(0, buf, 7);
    }

    bool set_shutter_params(uint32_t on_time_us, uint32_t offset_us) {
        uint8_t buf[8];
        memcpy(&buf[0], &on_time_us, 4);
        memcpy(&buf[4], &offset_us, 4);
        return write_eeprom(8, buf, 8);
    }
}
