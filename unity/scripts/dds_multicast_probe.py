#!/usr/bin/env python3

import argparse
import socket
import struct
import time


LOCATOR_PARAMETERS = {
    0x002F: "unicast",
    0x0030: "multicast",
    0x0031: "default_unicast",
    0x0032: "metatraffic_unicast",
    0x0033: "metatraffic_multicast",
}


def rtps_locators(payload):
    result = []
    for parameter_id, label in LOCATOR_PARAMETERS.items():
        marker = struct.pack("<HH", parameter_id, 24)
        offset = 0
        while True:
            offset = payload.find(marker, offset)
            if offset < 0:
                break
            data = offset + 4
            kind, port = struct.unpack_from("<iI", payload, data)
            address = socket.inet_ntoa(payload[data + 20 : data + 24])
            result.append(f"{label}={address}:{port}/kind{kind}")
            offset = data + 24
    return result


def main():
    parser = argparse.ArgumentParser(description="Listen for Fast DDS discovery multicast packets.")
    parser.add_argument("--interface", required=True, help="Local IPv4 interface address")
    parser.add_argument("--group", default="239.255.0.1")
    parser.add_argument("--port", type=int, default=7400)
    parser.add_argument("--timeout", type=float, default=8.0)
    args = parser.parse_args()

    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    receiver.bind(("", args.port))
    membership = socket.inet_aton(args.group) + socket.inet_aton(args.interface)
    receiver.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)
    receiver.settimeout(0.5)

    deadline = time.monotonic() + args.timeout
    count = 0
    sources = set()
    while time.monotonic() < deadline:
        try:
            payload, source = receiver.recvfrom(65535)
        except socket.timeout:
            continue
        count += 1
        sources.add(source[0])
        is_rtps = payload.startswith(b"RTPS") and len(payload) >= 20
        protocol = "RTPS" if is_rtps else payload[:4].hex()
        guid_prefix = payload[8:20].hex(":") if is_rtps else "-"
        host_id = payload[8:12].hex(":") if is_rtps else "-"
        print(
            f"{source[0]}:{source[1]} bytes={len(payload)} protocol={protocol} "
            f"host_id={host_id} guid_prefix={guid_prefix} "
            f"locators={','.join(rtps_locators(payload)) or '-'}"
        )

    print(f"packets={count} sources={','.join(sorted(sources)) or '-'}")
    return 0 if count else 1


if __name__ == "__main__":
    raise SystemExit(main())
