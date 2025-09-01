#!/usr/bin/env python3
# MIT Non-Commercial License
# Copyright (c) 2025 Rick Lan
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, for non-commercial purposes only, subject to the following conditions:
#
# - The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
# - Commercial use (e.g., use in a product, service, or activity intended to generate revenue) is prohibited without explicit written permission from Rick Lan. Contact ricklan@gmail.com for inquiries.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

from cereal import messaging
from opendbc.can.parser import CANParser
from panda import Panda
import hashlib
import json
import sys
import time

BUS = 1 # CAN1
SET_CONFIG_CAN_ID = 0x200 # 512
GET_CONFIG_CAN_ID = 0x201 # 513

SET_CONFIG_BITMASK = b'\xFF\xFF\xC0\x00\xFF\xFC\xFF\xFF'
GET_CONFIG_BITMASK = b'\x40\xFF\xC0\x03\xF7\xFE\xE0\xFF'

SET_CONFIG_CMD  = b'\xFF\x14\x00\x00\x08\x9C\x03\x10'
EXPECTED_CONFIG = b'\x40\x14\x00\x00\x10\xF4\x00\x24'

def bitmasked_bytestr(value, bitmask):
    value = int.from_bytes(value)
    bitmask = int.from_bytes(bitmask)
    return (value & bitmask).to_bytes(8)

def get_raw_radar_config(panda):
  sys.stdout.write("Waiting for radar message")
  while True:
    sys.stdout.write(".")
    sys.stdout.flush()
    msgs = panda.can_recv()
    for msg in msgs:
      addr, zero, data, bus = msg[0], msg[1], msg[2], msg[3]
      if addr == GET_CONFIG_CAN_ID and bus == BUS:
        print("\nRECV", msg)
        data = bitmasked_bytestr(data, GET_CONFIG_BITMASK)
        print("MASKED", data)
        for i, b in enumerate(data):
            print(f"{i}: {b:08b}")
        return data

  sys.stdout.write("\n")
  sys.stdout.flush()
  return None

def get_parsed_radar_config(can_parser, can_sock):
  while True:
    can_strings = messaging.drain_sock_raw(can_sock, wait_for_one=True)
    can_parser.update_strings(can_strings)
    msg = can_parser.vl["RadarState"]
    if msg["NVMReadStatus"] == 1:
      del msg["NVMWriteStatus"]
      return msg
  return None

def set_radar_config(panda, cfg):
  panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
  print("SEND", (SET_CONFIG_CAN_ID, cfg, BUS))
  panda.can_send(SET_CONFIG_CAN_ID, cfg, BUS, timeout=0)
  panda.set_safety_mode(panda.SAFETY_TOYOTA)
  time.sleep(3)

if __name__ == "__main__":
  can_parser = CANParser("mr76_radar", [("RadarState", 0)], BUS)
  can_sock = messaging.sub_sock("can", timeout=100)
  with Panda() as panda:
    config_msg = get_raw_radar_config(panda)
    parsed_msg = get_parsed_radar_config(can_parser, can_sock)
    print(json.dumps(parsed_msg, indent=2))
    if not config_msg:
      print("Radar Config Message not found, maybe on a different bus?")
      sys.exit(1)
    elif config_msg == EXPECTED_CONFIG:
      print("Radar Configuration is correct.")
      sys.exit(0)
    else:
      print("Invalid Radar Configuration, Setting now...")
      set_radar_config(panda, SET_CONFIG_CMD)
      config_msg = get_raw_radar_config(panda)
      parsed_msg = get_parsed_radar_config(can_parser, can_sock)
      print(json.dumps(parsed_msg, indent=2))
      if config_msg == EXPECTED_CONFIG:
        print("Radar Configuration update successful.")
        sys.exit(0)
      else:
        print("Radar Configuration update failed, please try again.")
        sys.exit(2)
