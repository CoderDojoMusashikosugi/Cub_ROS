#pragma once
#include <stdint.h>

constexpr char DIAL_USB_ADDRESS[] = "192.168.77.1";

bool usb_ncm_begin();
void usb_ncm_poll();
bool usb_ncm_connected();
uint32_t usb_ncm_host_address(); // Network byte order; zero until DHCP ACK.
