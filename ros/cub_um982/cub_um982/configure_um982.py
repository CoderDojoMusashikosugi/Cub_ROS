#!/usr/bin/env python3
"""
configure_um982.py

Utility script to configure Unicore UM982 receiver via serial port.
Supports piping commands via stdin, loading from a file, or applying
the standard Cub default configuration preset.
"""

import argparse
import sys
import time
import serial

DEFAULT_COMMANDS = [
    "UNLOG COM1",
    "UNLOG COM2",
    "UNLOG COM3",
    "CONFIG COM1 115200 8 n 1",
    "CONFIG COM3 9600 8 n 1",
    "MODE ROVER",
    "CONFIG HEADING FIXLENGTH",
    "CONFIG HEADING LENGTH 20 2",
    "GPGGA COM1 0.1",
    "GPGGAH COM1 0.1",
    "HEADINGA COM1 0.1",
    "GPSUTCA COM1 ONCHANGED",
    "RECTIMEA COM1 60",
    "GPRMC COM3 1.0",
    "CONFIG PPS ENABLE GPS POSITIVE 100000 1000 0 0",
    "SAVECONFIG",
]


def load_commands(file_path: str = None) -> list[str]:
    """Load commands from stdin (if piped), from a file, or default list."""
    commands = []

    if file_path:
        with open(file_path, "r", encoding="utf-8") as f:
            lines = f.readlines()
    elif not sys.stdin.isatty():
        # Input piped via stdin
        lines = sys.stdin.readlines()
        # docker exec without a TTY also supplies an empty stdin.
        if not lines:
            return DEFAULT_COMMANDS.copy()
    else:
        # Use default preset
        return DEFAULT_COMMANDS.copy()

    for line in lines:
        cleaned = line.strip()
        # Skip empty lines and comment lines
        if cleaned and not cleaned.startswith("#") and not cleaned.startswith("//"):
            commands.append(cleaned)

    return commands


def send_commands(
    port: str,
    baud: int,
    commands: list[str],
    line_delay: float = 0.1,
    dry_run: bool = False,
):
    print("=" * 60)
    print(f" UM982 Configuration Tool")
    print(f" Target Port: {port} @ {baud} bps")
    print(f" Commands count: {len(commands)}")
    if dry_run:
        print(" [DRY RUN MODE - No commands will be sent]")
    print("=" * 60)

    if dry_run:
        for idx, cmd in enumerate(commands, 1):
            print(f"[{idx:02d}/{len(commands):02d}] (dry-run) >>> {cmd}")
        print("\nDry run completed.")
        return

    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=0.5,
            write_timeout=1.0,
        )
    except serial.SerialException as e:
        print(f"\n[ERROR] Failed to open serial port {port}: {e}", file=sys.stderr)
        sys.exit(1)

    try:
        # Flush existing input buffer
        time.sleep(0.2)
        if ser.in_waiting:
            ser.read(ser.in_waiting)

        for idx, cmd in enumerate(commands, 1):
            print(f"[{idx:02d}/{len(commands):02d}] >>> {cmd}")
            ser.write(f"{cmd}\r\n".encode("ascii"))
            ser.flush()

            # SAVECONFIG needs slightly longer to write to NVRAM / Flash
            wait_time = 1.5 if "SAVECONFIG" in cmd.upper() else line_delay
            time.sleep(wait_time)

            # Read any responses
            response = ""
            while ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting).decode("ascii", errors="replace")
                response += chunk
                time.sleep(0.05)

            if response:
                for resp_line in response.strip().splitlines():
                    # Indent responses
                    print(f"       <<< {resp_line}")

        print("=" * 60)
        print(" Configuration successfully completed!")
        print("=" * 60)

    except KeyboardInterrupt:
        print("\n[INFO] Configuration interrupted by user.")
    except Exception as e:
        print(f"\n[ERROR] Communication error: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        ser.close()


def main():
    parser = argparse.ArgumentParser(
        description="Send configuration commands to Unicore UM982 GNSS receiver."
    )
    parser.add_argument(
        "--port",
        "-p",
        default="/dev/ttyUSB0",
        help="Serial port path (default: /dev/ttyUSB0)",
    )
    parser.add_argument(
        "--baud",
        "-b",
        type=int,
        default=115200,
        help="Serial baud rate (default: 115200)",
    )
    parser.add_argument(
        "--file",
        "-f",
        default=None,
        help="Path to command file (optional. If omitted and no stdin piped, default preset is used)",
    )
    parser.add_argument(
        "--delay",
        "-d",
        type=float,
        default=0.1,
        help="Delay between commands in seconds (default: 0.1s)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Display commands without sending them to the receiver",
    )

    args = parser.parse_args()

    commands = load_commands(args.file)
    if not commands:
        print("[ERROR] No commands to send.", file=sys.stderr)
        sys.exit(1)

    send_commands(
        port=args.port,
        baud=args.baud,
        commands=commands,
        line_delay=args.delay,
        dry_run=args.dry_run,
    )


if __name__ == "__main__":
    main()
