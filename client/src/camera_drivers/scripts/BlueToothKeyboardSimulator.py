#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sun Sep 14 10:45:40 2014

@author: mathieu
"""


import bluetooth as bt
import os
import dbus
import sys
import time
import thread
import rospy

keytable = {
    "KEY_RESERVED": 0,
    "KEY_ESC": 41,
    "KEY_1": 30,
    "KEY_2": 31,
    "KEY_3": 32,
    "KEY_4": 33,
    "KEY_5": 34,
    "KEY_6": 35,
    "KEY_7": 36,
    "KEY_8": 37,
    "KEY_9": 38,
    "KEY_0": 39,
    "KEY_MINUS": 45,
    "KEY_EQUAL": 46,
    "KEY_BACKSPACE": 42,
    "KEY_TAB": 43,
    "KEY_Q": 20,
    "KEY_W": 26,
    "KEY_E": 8,
    "KEY_R": 21,
    "KEY_T": 23,
    "KEY_Y": 28,
    "KEY_U": 24,
    "KEY_I": 12,
    "KEY_O": 18,
    "KEY_P": 19,
    "KEY_LEFTBRACE": 47,
    "KEY_RIGHTBRACE": 48,
    "KEY_ENTER": 40,
    "KEY_LEFTCTRL": 224,
    "KEY_A": 4,
    "KEY_S": 22,
    "KEY_D": 7,
    "KEY_F": 9,
    "KEY_G": 10,
    "KEY_H": 11,
    "KEY_J": 13,
    "KEY_K": 14,
    "KEY_L": 15,
    "KEY_SEMICOLON": 51,
    "KEY_APOSTROPHE": 52,
    "KEY_GRAVE": 53,
    "KEY_LEFTSHIFT": 225,
    "KEY_BACKSLASH": 50,
    "KEY_Z": 29,
    "KEY_X": 27,
    "KEY_C": 6,
    "KEY_V": 25,
    "KEY_B": 5,
    "KEY_N": 17,
    "KEY_M": 16,
    "KEY_COMMA": 54,
    "KEY_DOT": 55,
    "KEY_SLASH": 56,
    "KEY_RIGHTSHIFT": 229,
    "KEY_KPASTERISK": 85,
    "KEY_LEFTALT": 226,
    "KEY_SPACE": 44,
    "KEY_CAPSLOCK": 57,
    "KEY_F1": 58,
    "KEY_F2": 59,
    "KEY_F3": 60,
    "KEY_F4": 61,
    "KEY_F5": 62,
    "KEY_F6": 63,
    "KEY_F7": 64,
    "KEY_F8": 65,
    "KEY_F9": 66,
    "KEY_F10": 67,
    "KEY_NUMLOCK": 83,
    "KEY_SCROLLLOCK": 71,
    "KEY_KP7": 95,
    "KEY_KP8": 96,
    "KEY_KP9": 97,
    "KEY_KPMINUS": 86,
    "KEY_KP4": 92,
    "KEY_KP5": 93,
    "KEY_KP6": 94,
    "KEY_KPPLUS": 87,
    "KEY_KP1": 89,
    "KEY_KP2": 90,
    "KEY_KP3": 91,
    "KEY_KP0": 98,
    "KEY_KPDOT": 99,
    "KEY_ZENKAKUHANKAKU": 148,
    "KEY_102ND": 100,
    "KEY_F11": 68,
    "KEY_F12": 69,
    "KEY_RO": 135,
    "KEY_KATAKANA": 146,
    "KEY_HIRAGANA": 147,
    "KEY_HENKAN": 138,
    "KEY_KATAKANAHIRAGANA": 136,
    "KEY_MUHENKAN": 139,
    "KEY_KPJPCOMMA": 140,
    "KEY_KPENTER": 88,
    "KEY_RIGHTCTRL": 228,
    "KEY_KPSLASH": 84,
    "KEY_SYSRQ": 70,
    "KEY_RIGHTALT": 230,
    "KEY_HOME": 74,
    "KEY_UP": 82,
    "KEY_PAGEUP": 75,
    "KEY_LEFT": 80,
    "KEY_RIGHT": 79,
    "KEY_END": 77,
    "KEY_DOWN": 81,
    "KEY_PAGEDOWN": 78,
    "KEY_INSERT": 73,
    "KEY_DELETE": 76,
    "KEY_MUTE": 239,
    "KEY_VOLUMEDOWN": 238,
    "KEY_VOLUMEUP": 237,
    "KEY_POWER": 102,
    "KEY_KPEQUAL": 103,
    "KEY_PAUSE": 72,
    "KEY_KPCOMMA": 133,
    "KEY_HANGEUL": 144,
    "KEY_HANJA": 145,
    "KEY_YEN": 137,
    "KEY_LEFTMETA": 227,
    "KEY_RIGHTMETA": 231,
    "KEY_COMPOSE": 101,
    "KEY_STOP": 243,
    "KEY_AGAIN": 121,
    "KEY_PROPS": 118,
    "KEY_UNDO": 122,
    "KEY_FRONT": 119,
    "KEY_COPY": 124,
    "KEY_OPEN": 116,
    "KEY_PASTE": 125,
    "KEY_FIND": 244,
    "KEY_CUT": 123,
    "KEY_HELP": 117,
    "KEY_CALC": 251,
    "KEY_SLEEP": 248,
    "KEY_WWW": 240,
    "KEY_COFFEE": 249,
    "KEY_BACK": 241,
    "KEY_FORWARD": 242,
    "KEY_EJECTCD": 236,
    "KEY_NEXTSONG": 235,
    "KEY_PLAYPAUSE": 232,
    "KEY_PREVIOUSSONG": 234,
    "KEY_STOPCD": 233,
    "KEY_REFRESH": 250,
    "KEY_EDIT": 247,
    "KEY_SCROLLUP": 245,
    "KEY_SCROLLDOWN": 246,
    "KEY_F13": 104,
    "KEY_F14": 105,
    "KEY_F15": 106,
    "KEY_F16": 107,
    "KEY_F17": 108,
    "KEY_F18": 109,
    "KEY_F19": 110,
    "KEY_F20": 111,
    "KEY_F21": 112,
    "KEY_F22": 113,
    "KEY_F23": 114,
    "KEY_F24": 115
}


class BluetoothKeyBoard():
    CTRL_PORT = 17
    INTR_PORT = 19
    HOST = 0  # BT Mac address
    PORT = 1  # Bluetooth Port Number
    LOCK = thread.allocate_lock()
    CONTROL_POOL = []
    INTERUPT_POOL = []
    TERMINATED = False

    def __init__(self, maxConnection=1, sdpRecordFile="sdp_record.xml"):
        self.maxConnection = maxConnection
        os.system("hciconfig hci0 class 0x002540")  # Keyboard class
        #os.system("hciconfig hci0 name CameraNetwork")
        os.system("hciconfig hci0 piscan")  # discovery mode
        #os.system("hciconfig hci0 leadv")
        self.socket_ctrl = bt.BluetoothSocket(bt.L2CAP)
        self.socket_intr = bt.BluetoothSocket(bt.L2CAP)

        self.socket_ctrl.bind(("", BluetoothKeyBoard.CTRL_PORT))
        self.socket_intr.bind(("", BluetoothKeyBoard.INTR_PORT))

        self._init_dbus()
        self._load_sdp_record(sdpRecordFile)

    def __del__(self):
        BluetoothKeyBoard.TERMINATED = True
        self.socket_ctrl.close()
        self.socket_intr.close()

    def _init_dbus(self):
        self.bus = dbus.SystemBus()
        try:
            self.manager = dbus.Interface(
                self.bus.get_object(
                    "org.bluez",
                    "/"),
                "org.bluez.Manager")
            adapter_path = self.manager.DefaultAdapter()
            self.service = dbus.Interface(
                self.bus.get_object(
                    "org.bluez",
                    adapter_path),
                "org.bluez.Service")
        except:
            rospy.logfatal(
                "Could not configure bluetooth. Is bluetoothd started?")

    def _load_sdp_record(self, filename):
        try:
            fh = open(filename, "r")
        except:
            rospy.logfatal("Could not open the sdp record: " + filename)
        self.service_record = fh.read()
        fh.close()

    def listen(self):
        self.service_handle = self.service.AddRecord(self.service_record)
        self.socket_ctrl.listen(self.maxConnection)
        self.socket_intr.listen(self.maxConnection)
        #bt.advertise_service(self.socket_ctrl, "test ctrl", 
        #                     service_classes = [bt.HID_CLASS],
        #                     profiles = [bt.HID_PROFILE])

        #bt.advertise_service(self.socket_intr, "test intr", 
        #                     service_classes = [bt.HID_CLASS],
        #                     profiles = [bt.HID_PROFILE])
        thread.start_new_thread(self._listen_thread, ())

    def _listen_thread(self):
        while(not BluetoothKeyBoard.TERMINATED):
            if len(BluetoothKeyBoard.INTERUPT_POOL) < self.maxConnection:
                _ccontrol, self.cinfo = self.socket_ctrl.accept()
                _cinterrupt, self.cinfo = self.socket_intr.accept()
                rospy.loginfo(
                    "Got a connection from " +
                    self.cinfo[
                        BluetoothKeyBoard.HOST])
                with BluetoothKeyBoard.LOCK:
                    BluetoothKeyBoard.CONTROL_POOL.append(_ccontrol)
                    BluetoothKeyBoard.INTERUPT_POOL.append(_cinterrupt)

    def raise_volume(self):
        self.send_input(keytable['KEY_VOLUMEUP'])
        self.send_input(0)

    def send_input(self, value):
        '''
        Take a specific value into an hex array and convert it to a string
        '''
        state = [
            0xA1,  # This is an input report
            0x01,  # Usage report = Keyboard
            # Bit array for Modifier keys
            [0,   # Right GUI - (usually the Windows key)
                0,   # Right ALT
                0,   # Right Shift
                0,   # Right Control
                0,   # Left GUI - (again, usually the Windows key)
                0,   # Left ALT
                0,   # Left Shift
                0],   # Left Control
            0x00,  # Vendor reserved
            value,  # Rest is space for 6 keys
            0x00,
            0x00,
            0x00,
            0x00,
            0x00]
        hex_str = ""
        for element in state:
            if isinstance(element, list):
                # This is our bit array - convert it to a single byte represented
                # as a char
                bin_str = ""
                for bit in element:
                    bin_str += str(bit)
                hex_str += chr(int(bin_str, 2))
            else:
                # This is a hex value - we can convert it straight to a char
                hex_str += chr(element)
        # Send an input report
        count = 0
        with BluetoothKeyBoard.LOCK:
            for c in BluetoothKeyBoard.INTERUPT_POOL:
                try:
                    c.send(hex_str)
                except:
                    BluetoothKeyBoard.INTERUPT_POOL.remove(c)
                count += 1


if __name__ == "__main__":
    # We can only run as root
    rospy.init_node('test')
    #if not os.geteuid() == 0:
    #    sys.exit("Only root can run this script")
    dentbleu = BluetoothKeyBoard()
    dentbleu.listen()
    while(1):
        time.sleep(4)
        dentbleu.raise_volume()
