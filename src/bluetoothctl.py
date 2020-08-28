# ReachView code is placed under the GPL license.
# Written by Egor Fedorov (egor.fedorov@emlid.com)
# Copyright (c) 2015, Emlid Limited
# All rights reserved.
# Source: https://gist.github.com/egorf/66d88056a9d703928f93

import os
import time
import pexpect
import subprocess
import sys

class BluetoothctlError(Exception):
    """This exception is raised, when bluetoothctl fails to start."""
    pass


class Bluetoothctl:
    """A wrapper for bluetoothctl utility."""

    def __init__(self, logger = None):
        try:
            out = subprocess.check_output("rfkill unblock bluetooth", shell = True)
        except Exception as e:
            print("rfkill error: %s"%e)
        self.child = pexpect.spawn("bluetoothctl", echo = False)
        self.logger = logger
        if self.logger is None:
            def default_logger(m):
                print("[bluetoothctl] {}".format(m))
            self.logger = default_logger

    def get_output(self, command, pause = 0):
        """Run a command in bluetoothctl prompt, return output as a list of lines."""
        self.logger("Send command: {}".format(command))

        self.child.send(command + "\n")
        time.sleep(pause)
        start_failed = self.child.expect(["bluetooth", "Wireless Controller", pexpect.EOF])

        if start_failed > 1:
            raise BluetoothctlError("Bluetoothctl failed after running " + command)

        out = self.child.before.split(b"\r\n")

        for line in out:
            self.logger("[stdout] {}".format(line))

        return out

    def start_scan(self):
        """Start bluetooth scanning process."""
        try:
            out = self.get_output("scan on")
        except BluetoothctlError as e:
            self.logger(e)
            return None

    def make_discoverable(self):
        """Make device discoverable."""
        try:
            out = self.get_output("discoverable on")
        except BluetoothctlError as e:
            print(e)
            return None

    def parse_device_info(self, info_string):
        """Parse a string corresponding to a device."""
        device = {}
        block_list = ["[\x1b[0;", "removed"]
        string_valid = not any(keyword in info_string for keyword in block_list)

        if string_valid:
            try:
                device_position = info_string.index("Device")
            except ValueError:
                pass
            else:
                if device_position > -1:
                    attribute_list = info_string[device_position:].split(" ", 2)
                    device = {
                        "mac_address": attribute_list[1],
                        "name": attribute_list[2]
                    }

        return device

    def get_available_devices(self):
        """Return a list of tuples of paired and discoverable devices."""
        try:
            out = self.get_output("devices")
        except BluetoothctlError as e:
            print(e)
            return None
        else:
            available_devices = []
            for line in out:
                device = self.parse_device_info(line)
                if device:
                    available_devices.append(device)

            return available_devices

    def get_paired_devices(self):
        """Return a list of tuples of paired devices."""
        try:
            out = self.get_output("paired-devices")
        except BluetoothctlError as e:
            print(e)
            return None
        else:
            paired_devices = []
            for line in out:
                device = self.parse_device_info(line)
                if device:
                    paired_devices.append(device)

            return paired_devices

    def get_discoverable_devices(self):
        """Filter paired devices out of available."""
        available = self.get_available_devices()
        paired = self.get_paired_devices()

        return [d for d in available if d not in paired]

    def get_device_info(self, mac_address):
        """Get device info by mac address."""
        try:
            out = self.get_output("info " + mac_address)
        except BluetoothctlError as e:
            print(e)
            return None
        else:
            return out

    def pair(self, mac_address):
        """Try to pair with a device by mac address."""
        try:
            out = self.get_output("pair " + mac_address, 4)
        except BluetoothctlError as e:
            print(e)
            return None
        else:
            res = self.child.expect(["Failed to pair", "Pairing successful", pexpect.EOF], timeout=1)
            success = True if res == 1 else False
            return success

    def remove(self, mac_address):
        """Remove paired device by mac address, return success of the operation."""
        try:
            out = self.get_output("remove {}".format(mac_address), 3)
        except BluetoothctlError as e:
            print(e)
            return None
        else:
            res = self.child.expect(["not available", "Device has been removed", pexpect.EOF])
            success = res == 1
            return success

    def connect(self, mac_address):
        """Try to connect to a device by mac address."""
        try:
            out = self.get_output("connect " + mac_address, 2)
        except BluetoothctlError as e:
            print(e)
            return None
        else:
            res = self.child.expect(["Failed to connect", "Connection successful", pexpect.EOF], timeout=1)
            success = True if res == 1 else False
            return success

    def disconnect(self, mac_address):
        """Try to disconnect to a device by mac address."""
        try:
            out = self.get_output("disconnect " + mac_address, 2)
        except BluetoothctlError as e:
            print(e)
            return None
        else:
            res = self.child.expect(["Failed to disconnect", "Successful disconnected", pexpect.EOF])
            success = True if res == 1 else False
            return success


if __name__ == "__main__":
    mac_addr = sys.argv[1] if len(sys.argv) > 1 else None
    if mac_addr is None:
        mac_addr = os.environ.get("BLUE_MAC", None)
    if mac_addr is None:
        mac_addr = "8C:41:F2:E1:76:BE" # TODO remove
    mac_addr = mac_addr.upper()
    print("Target device mac_addr: {}".format(mac_addr))

    print("Init bluetooth. . .")
    bl = Bluetoothctl()

    time.sleep(1)

    print("Ready! Removing the target and reconnect. . .")
    try:
        bl.remove(mac_addr)
    except BluetoothctlError as e:
        print(e)
    except pexpect.exceptions.TIMEOUT:
        pass

    print("Start scanning and wait for device. . ."),
    bl.start_scan()
    for i in range(0, 2):
        print("."),
        sys.stdout.flush()
        time.sleep(1)
    print("")

    print("Pairing, DS4 in pairing mode (LED blink fast!). ."),
    while True:
        try:
            print("."),
            sys.stdout.flush()
            resp = bl.pair(mac_addr)
            if resp == True:
                break
        except BluetoothctlError as e:
            print(e)
        except pexpect.exceptions.TIMEOUT:
            pass
    print("")

    print("Connecting. . ."),
    while True:
        try:
            print("."),
            sys.stdout.flush()
            resp = bl.connect(mac_addr)
            if resp == True:
                break
        except BluetoothctlError as e:
            print(e)
        except pexpect.exceptions.TIMEOUT:
            pass
    print("\a")

    print("Done.")
