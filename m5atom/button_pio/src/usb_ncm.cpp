#include "usb_ncm.h"
#include <Arduino.h>
#include <Network.h>
#include <USB.h>
#include <atomic>
#include <cstring>

extern "C" {
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_netif_defaults.h"
#include "esp32-hal-tinyusb.h"
#include "device/usbd_pvt.h"
#include "lwip/ip4_addr.h"
#include "dhcpserver/dhcpserver.h"
}

namespace {
constexpr size_t MTU = 1514;
esp_netif_t *usb_interface = nullptr;
std::atomic<uint32_t> host_address{0};
SemaphoreHandle_t tx_slots = nullptr;
std::atomic<bool> link_started{false};
char host_mac_string[13];

struct TxPacket {
    uint16_t length;
    uint8_t data[MTU];
};

void free_packet(TxPacket *packet) {
    free(packet);
    xSemaphoreGive(tx_slots);
}

void send_in_usb_task(void *argument) {
    auto *packet = static_cast<TxPacket *>(argument);
    if (tud_mounted() && tud_network_can_xmit(packet->length)) {
        tud_network_xmit(packet, packet->length);
    } else {
        free_packet(packet);
    }
}

esp_err_t transmit(void *, void *buffer, size_t length) {
    if (length > MTU || !tud_mounted()) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(tx_slots, 0) != pdTRUE) return ESP_ERR_NO_MEM;
    auto *packet = static_cast<TxPacket *>(malloc(sizeof(TxPacket)));
    if (!packet) {
        xSemaphoreGive(tx_slots);
        return ESP_ERR_NO_MEM;
    }
    packet->length = length;
    memcpy(packet->data, buffer, length);
    // USB class APIs execute in the TinyUSB task, not the lwIP task.
    usbd_defer_func(send_in_usb_task, packet, false);
    return ESP_OK;
}

void free_receive(void *, void *buffer) { free(buffer); }

void assigned_ip(void *, esp_event_base_t, int32_t, void *data) {
    const auto *event = static_cast<ip_event_ap_staipassigned_t *>(data);
    if (event->esp_netif == usb_interface) host_address.store(event->ip.addr);
}

uint16_t descriptor(uint8_t *destination, uint8_t *interface_number) {
    const uint8_t mac_index = tinyusb_add_string_descriptor(host_mac_string);
    const uint8_t name_index = tinyusb_add_string_descriptor("M5Dial USB Network");
    const uint8_t notify = tinyusb_get_free_in_endpoint();
    const uint8_t data = tinyusb_get_free_duplex_endpoint();
    if (!notify || !data) return 0;
    const uint8_t bytes[] = {
        TUD_CDC_NCM_DESCRIPTOR(*interface_number, name_index, mac_index,
                              static_cast<uint8_t>(0x80 | notify), 64, data,
                              static_cast<uint8_t>(0x80 | data), 64,
                              MTU, 16, 0)
    };
    *interface_number += 2;
    memcpy(destination, bytes, sizeof(bytes));
    return sizeof(bytes);
}
} // namespace

extern "C" {
uint8_t tud_network_mac_address[6];
bool tud_network_default_link_state_cb(void) { return true; }
void tud_network_init_cb(void) {}

bool tud_network_recv_cb(const uint8_t *source, uint16_t length) {
    if (link_started.load() && length <= MTU) {
        void *copy = malloc(length);
        if (copy) {
            memcpy(copy, source, length);
            // esp-netif owns copy and releases it through free_receive.
            esp_netif_receive(usb_interface, copy, length, nullptr);
        }
    }
    tud_network_recv_renew();
    return true;
}

uint16_t tud_network_xmit_cb(uint8_t *destination, void *reference, uint16_t) {
    auto *packet = static_cast<TxPacket *>(reference);
    const uint16_t length = packet->length;
    memcpy(destination, packet->data, length);
    free_packet(packet);
    return length;
}
}

bool usb_ncm_begin() {
    if (!Network.begin()) return false;
    tx_slots = xSemaphoreCreateCounting(8, 8);
    if (!tx_slots) return false;
    uint8_t device_mac[6];
    ESP_ERROR_CHECK(esp_efuse_mac_get_default(device_mac));
    device_mac[0] = (device_mac[0] | 2) & 0xfe;
    memcpy(tud_network_mac_address, device_mac, 6);
    tud_network_mac_address[5] ^= 1; // Distinct MAC for the Linux end.
    snprintf(host_mac_string, sizeof(host_mac_string), "%02X%02X%02X%02X%02X%02X",
             tud_network_mac_address[0], tud_network_mac_address[1],
             tud_network_mac_address[2], tud_network_mac_address[3],
             tud_network_mac_address[4], tud_network_mac_address[5]);

    esp_netif_ip_info_t ip{};
    ip.ip.addr = ipaddr_addr(DIAL_USB_ADDRESS);
    ip.netmask.addr = ipaddr_addr("255.255.255.0");
    ip.gw.addr = 0;
    esp_netif_inherent_config_t base = ESP_NETIF_INHERENT_DEFAULT_ETH();
    base.flags = static_cast<esp_netif_flags_t>(ESP_NETIF_DHCP_SERVER | ESP_NETIF_FLAG_AUTOUP);
    base.if_key = "DIAL_USB";
    base.if_desc = "M5Dial USB NCM";
    base.ip_info = &ip;
    base.get_ip_event = 0;
    base.lost_ip_event = 0;
    const esp_netif_driver_ifconfig_t driver = {
        .handle = &usb_interface, // esp-netif requires a non-null driver handle.
        .transmit = transmit,
        .transmit_wrap = nullptr,
        .driver_free_rx_buffer = free_receive,
    };
    const esp_netif_config_t config = {
        .base = &base, .driver = &driver, .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH
    };
    usb_interface = esp_netif_new(&config);
    if (!usb_interface) return false;
    ESP_ERROR_CHECK(esp_netif_set_mac(usb_interface, device_mac));

    dhcps_lease_t lease{};
    lease.enable = true;
    lease.start_ip.addr = ipaddr_addr("192.168.77.2");
    // ESP-IDF 5.5 requires start_ip < end_ip, even for a single USB peer.
    lease.end_ip.addr = ipaddr_addr("192.168.77.3");
    ESP_ERROR_CHECK(esp_netif_dhcps_option(usb_interface, ESP_NETIF_OP_SET,
        ESP_NETIF_REQUESTED_IP_ADDRESS, &lease, sizeof(lease)));
    dhcps_offer_t no_offer = 0;
    ESP_ERROR_CHECK(esp_netif_dhcps_option(usb_interface, ESP_NETIF_OP_SET,
        ESP_NETIF_ROUTER_SOLICITATION_ADDRESS, &no_offer, sizeof(no_offer)));
    ESP_ERROR_CHECK(esp_netif_dhcps_option(usb_interface, ESP_NETIF_OP_SET,
        ESP_NETIF_DOMAIN_NAME_SERVER, &no_offer, sizeof(no_offer)));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED,
                                             assigned_ip, nullptr));

    if (tinyusb_enable_interface(USB_INTERFACE_CUSTOM, TUD_CDC_NCM_DESC_LEN, descriptor) != ESP_OK)
        return false;
    USB.productName("M5Dial micro-ROS NCM");
    USB.PID(0x1001);
    return USB.begin();
}

void usb_ncm_poll() {
    const bool mounted = tud_mounted();
    if (mounted && !link_started) {
        esp_netif_action_start(usb_interface, nullptr, 0, nullptr);
        link_started = true;
    } else if (!mounted && link_started) {
        host_address.store(0);
        esp_netif_action_stop(usb_interface, nullptr, 0, nullptr);
        link_started = false;
    }
}

bool usb_ncm_connected() { return link_started && host_address.load() != 0; }
uint32_t usb_ncm_host_address() { return host_address.load(); }
