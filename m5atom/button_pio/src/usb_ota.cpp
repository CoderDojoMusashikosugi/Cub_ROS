#include "usb_ota.h"
#include "usb_ncm.h"
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <atomic>

namespace {
std::atomic<bool> updating{false};

void serve_ota(void *) {
    ArduinoOTA.setHostname("m5dial");
    ArduinoOTA.setMdnsEnabled(false); // Connect directly to the USB IP.
    ArduinoOTA.setPort(3232);
    ArduinoOTA.setTimeout(10000);
    ArduinoOTA.onStart([] { updating.store(true); });
    ArduinoOTA.onError([](ota_error_t) { updating.store(false); });
    bool started = false;
    for (;;) {
        if (usb_ncm_connected()) {
            if (!started) {
                ArduinoOTA.begin();
                started = true;
            }
            ArduinoOTA.handle();
        } else if (started) {
            ArduinoOTA.end();
            updating.store(false);
            started = false;
        }
        delay(5);
    }
}
} // namespace

bool usb_ota_begin() {
    // Agent discovery and OTA transfers run independently.
    return xTaskCreate(serve_ota, "usb_ota", 8192, nullptr, 1, nullptr) == pdPASS;
}

bool usb_ota_updating() { return updating.load(); }
