#include "M5Dial.h"
#include "USB.h"
#include "usb_ncm.h"
#include "usb_ota.h"
#include "build_timestamp.h"
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/bool.h>

extern "C" {
#include "lwip/sockets.h"
}

#ifndef DIAL_ROS_DOMAIN_ID
#define DIAL_ROS_DOMAIN_ID 0
#endif
#ifndef DIAL_AGENT_PORT
#define DIAL_AGENT_PORT 8888
#endif

namespace {
constexpr int BEEP_FREQ = 4000;
constexpr int BEEP_DURATION_MS = 50;
constexpr int CONNECT_BEEP_DURATION_MS = 100;
constexpr int DISCONNECT_BEEP_DURATION_MS = 200;

enum class DialStatus { Updating, Error, Pressed, Ready };

int udp_socket = -1;
rclc_support_t support{};
rcl_node_t node = rcl_get_zero_initialized_node();
rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
bool support_ready = false;
bool node_ready = false;
bool publisher_ready = false;
bool pressed = false;
bool last_published = false;
bool connected = false;
bool updating = false;
bool usb_connected = false;
uint32_t connected_host = 0;
unsigned long last_publish = 0;
unsigned long last_agent_check = 0;

bool udp_open(uxrCustomTransport *) {
    if (udp_socket >= 0) close(udp_socket);
    udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_socket < 0) return false;
    sockaddr_in peer{};
    peer.sin_family = AF_INET;
    peer.sin_port = htons(DIAL_AGENT_PORT);
    peer.sin_addr.s_addr = usb_ncm_host_address();
    if (connect(udp_socket, reinterpret_cast<sockaddr *>(&peer), sizeof(peer)) != 0) {
        close(udp_socket);
        udp_socket = -1;
        return false;
    }
    return true;
}

bool udp_close(uxrCustomTransport *) {
    if (udp_socket >= 0) close(udp_socket);
    udp_socket = -1;
    return true;
}

size_t udp_write(uxrCustomTransport *, const uint8_t *buffer, size_t size, uint8_t *error) {
    if (udp_socket < 0) { *error = 1; return 0; }
    const int result = send(udp_socket, buffer, size, 0);
    *error = result < 0 ? 1 : 0;
    return result < 0 ? 0 : result;
}

size_t udp_read(uxrCustomTransport *, uint8_t *buffer, size_t size, int timeout, uint8_t *error) {
    if (udp_socket < 0) { *error = 1; return 0; }
    if (timeout < 0) timeout = 0;
    fd_set sockets;
    FD_ZERO(&sockets);
    FD_SET(udp_socket, &sockets);
    timeval wait{timeout / 1000, (timeout % 1000) * 1000};
    const int ready = select(udp_socket + 1, &sockets, nullptr, nullptr, &wait);
    if (ready <= 0) {
        *error = ready < 0 ? 1 : 0;
        return 0;
    }
    const int result = recv(udp_socket, buffer, size, 0);
    *error = result < 0 ? 1 : 0;
    return result < 0 ? 0 : result;
}

void destroy_ros() {
    if (support_ready)
        rmw_uros_set_context_entity_destroy_session_timeout(
            rcl_context_get_rmw_context(&support.context), 0);
    if (publisher_ready) (void)rcl_publisher_fini(&publisher, &node);
    if (node_ready) (void)rcl_node_fini(&node);
    if (support_ready) (void)rclc_support_fini(&support);
    publisher = rcl_get_zero_initialized_publisher();
    node = rcl_get_zero_initialized_node();
    support = {};
    publisher_ready = node_ready = support_ready = connected = false;
    rcl_reset_error();
}

bool create_ros() {
    auto allocator = rcl_get_default_allocator();
    auto options = rcl_get_zero_initialized_init_options();
    if (rcl_init_options_init(&options, allocator) != RCL_RET_OK) return false;
    rcl_ret_t result = rcl_init_options_set_domain_id(&options, DIAL_ROS_DOMAIN_ID);
    if (result == RCL_RET_OK)
        result = rclc_support_init_with_options(&support, 0, nullptr, &options, &allocator);
    (void)rcl_init_options_fini(&options);
    if (result != RCL_RET_OK) return false;
    support_ready = true;
    if (rclc_node_init_default(&node, "m5dial_button", "", &support) != RCL_RET_OK) return false;
    node_ready = true;
    if (rclc_publisher_init_default(&publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/dial/button") != RCL_RET_OK)
        return false;
    publisher_ready = true;
    rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&publisher), 100);
    connected_host = usb_ncm_host_address();
    return true;
}

DialStatus current_status() {
    if (updating) return DialStatus::Updating;
    if (!connected) return DialStatus::Error;
    return pressed ? DialStatus::Pressed : DialStatus::Ready;
}

void display_state() {
    uint16_t background = TFT_BLACK;
    uint16_t foreground = TFT_GREEN;
    const char *title = "READY";
    const char *detail = "ROS CONNECTED";
    switch (current_status()) {
    case DialStatus::Updating:
        foreground = TFT_YELLOW;
        title = "UPDATING";
        detail = "OTA";
        break;
    case DialStatus::Error:
        background = pressed ? TFT_RED : TFT_BLACK;
        foreground = pressed ? TFT_WHITE : TFT_RED;
        title = "ERROR";
        detail = usb_connected ? "ROS WAITING" : "USB WAITING";
        break;
    case DialStatus::Pressed:
        background = TFT_BLUE;
        foreground = TFT_WHITE;
        title = "PRESSED";
        break;
    case DialStatus::Ready:
        break;
    }
    M5Dial.Display.fillScreen(background);
    M5Dial.Display.setTextDatum(middle_center);
    M5Dial.Display.setTextColor(foreground);
    M5Dial.Display.setTextSize(2);
    M5Dial.Display.drawString(title, 120, 65);
    M5Dial.Display.drawString(detail, 120, 105);
    M5Dial.Display.setTextSize(1);
    M5Dial.Display.drawString("USB: 192.168.77.1", 120, 140);
    M5Dial.Display.drawString("/dial/button", 120, 160);
    M5Dial.Display.setTextColor(TFT_LIGHTGREY);
    M5Dial.Display.drawString("Build " DIAL_BUILD_DATE, 120, 210);
    M5Dial.Display.drawString(DIAL_BUILD_TIME, 120, 222);
}
} // namespace

void setup() {
    auto config = M5.config();
    config.serial_baudrate = 0;
    M5Dial.begin(config, false, false);
    rmw_uros_set_custom_transport(false, nullptr, udp_open, udp_close, udp_write, udp_read);
    if (!usb_ncm_begin() || !usb_ota_begin()) {
        M5Dial.Display.fillScreen(TFT_RED);
        M5Dial.Display.drawString("USB INIT FAILED", 10, 100);
        while (true) delay(1000);
    }
    display_state();
}

void loop() {
    usb_ncm_poll();
    M5Dial.update();
    const bool was_pressed = pressed;
    pressed = M5Dial.BtnA.isPressed();
    if (pressed && !was_pressed && connected)
        M5Dial.Speaker.tone(BEEP_FREQ, BEEP_DURATION_MS);
    const bool was_usb_connected = usb_connected;
    usb_connected = usb_ncm_connected();
    const bool was_updating = updating;
    updating = usb_ota_updating();
    const bool was_connected = connected;
    const unsigned long now = millis();
    if (connected && (!usb_connected || connected_host != usb_ncm_host_address()))
        destroy_ros();
    if (!updating && usb_connected && now - last_agent_check >= 1000) {
        last_agent_check = now;
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
            if (!connected) {
                connected = create_ros();
                if (!connected) destroy_ros();
            }
        } else if (connected) {
            destroy_ros();
        }
    }
    if (!updating && connected && (was_connected != connected || pressed != last_published ||
                      now - last_publish >= 100)) {
        std_msgs__msg__Bool message{};
        message.data = pressed;
        if (rcl_publish(&publisher, &message, nullptr) != RCL_RET_OK) {
            destroy_ros();
        } else {
            last_published = pressed;
            last_publish = now;
        }
    }
    if (!was_connected && connected)
        M5Dial.Speaker.tone(BEEP_FREQ, CONNECT_BEEP_DURATION_MS);
    else if (was_connected && !connected)
        M5Dial.Speaker.tone(BEEP_FREQ, DISCONNECT_BEEP_DURATION_MS);
    if (was_pressed != pressed || was_connected != connected || was_updating != updating ||
        was_usb_connected != usb_connected)
        display_state();
    delay(5);
}
