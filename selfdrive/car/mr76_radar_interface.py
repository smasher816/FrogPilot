'''
MIT Non-Commercial License
Copyright (c) 2025 Rick Lan

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, for non-commercial purposes only, subject to the following conditions:

- The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
- Commercial use (e.g., use in a product, service, or activity intended to generate revenue) is prohibited without explicit written permission from Rick Lan. Contact ricklan@gmail.com for inquiries.
- Any project that uses the Software must visibly mention the following acknowledgment: "This project uses software from Rick Lan and is licensed under a custom license requiring permission for use."

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
'''

from cereal import car
from collections import deque, defaultdict
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.interfaces import RadarInterfaceBase
import math

# DOCS - https://github.com/user-attachments/files/22085632/MR76.V1.2_20200307.1.pdf

MIN_SAMPLES = 5

# Connect to CAN1L/H in camera harness
CANBUS = 1

# Correct radar position
RADAR_DREL_OFFSET = 0. # long distance from center of car hood to radar (m)
RADAR_YREL_OFFSET = 0. # lat distance from center of car hood to radar (m)

# Remove old points if they haven't been sampled recently
IDLE_CYCLES = 2 # ~60ms retention
IDLE_FRAMES = 60 # ~1s

class DynProp:
  MOVING = 0
  STATIONARY = 1
  ONCOMING = 2
  CROSSING_LEFT = 3
  CROSSING_RIGHT = 4
  UNKNOWN = 5
  STOPPED = 6

class ObjClass:
  POINT = 0    # Pedestrians, trees, and fences
  VEHICLE = 1  # Trucks and cars
  RESERVED = 2 # Unused

class MR76RadarPoint:
  def __init__(self):
    self.rp = car.RadarData.RadarPoint()
    self.rp.samples = 0
    self.cycle = None

  def update(self, cycle, trackId, dRel, yRel, vRel, yvRel, aRel, rcs, dynProp, objClass):
    self.cycle = cycle
    self.dynProp = dynProp
    self.objClass = objClass
    self.rp.trackId = trackId
    self.rp.dRel = dRel
    self.rp.yRel = yRel
    self.rp.vRel = vRel
    self.rp.yvRel = yvRel
    self.rp.aRel = aRel
    self.rp.rcs = rcs
    self.rp.samples += 1
    # Only consider a point measured if it has sufficient samples
    self.rp.measured = self.rp.samples > MIN_SAMPLES and abs(vRel) > 0.01

  @property
  def angle(self):
    return math.atan2(self.rp.yRel, self.rp.dRel)

  def __str__(self):
    return f"MR76RadarPoint(trackId={self.rp.trackId}, cycle={self.cycle}, samples={self.rp.samples}, dynProp={self.dynProp}, objClass={self.objClass}, rcs={self.rp.rcs}, dRel={self.rp.dRel}, yRel={self.rp.yRel}, vRel={self.rp.vRel}, yvRel={self.rp.yvRel}, angle={self.angle})"


class MR76RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.rcp = CANParser('mr76_radar', [("Status", 0), ("ObjectData", 0)], CANBUS)
    self.radar_points = defaultdict(lambda: MR76RadarPoint())
    self.cycle = 0 # Current radar measurement cycle (60hz)
    self.num_objects = 0 # Number of currently tracked objects
    self.frame = 0 # Current frame
    self.cycle_start = 0 # Frame current cycle started at

  # Called by card.py at 100hz (every 10ms)
  def update(self, can_strings):
    self.frame += 1
    if self.rcp is None:
      return super().update(None)

    updated_addrs = self.rcp.update_strings(can_strings)

    # Start of new radar cycle
    if 0x60A in updated_addrs:
      # Print all objects from current cycle
      # if self.radar_points:
      #   print(f"MR76: {len(self.radar_points)}/{self.num_objects} objects\n -", '\n - '.join([str(rp) for rp in self.radar_points.values()]))
      # else:
      #   print("MR76: Idle")

      # Start new cycle
      self.cycle = int(self.rcp.vl['Status']['MeasCount'])
      self.num_objects = int(self.rcp.vl['Status']['NoOfObjects'])
      self.cycle_start = self.frame

      # Mark points from previous cycles as unmeasured. Remove if too old.
      to_remove = []
      for track_id, radar_point in self.radar_points.items():
        radar_point.measured = False
        if self.cycle - radar_point.cycle > IDLE_CYCLES:
          to_remove.append(track_id)
      for track_id in to_remove:
        del self.radar_points[track_id]

    # We expect to see a cycle every 1.6 frames. Mark data as stale if it's been 2x longer than that.
    if self.frame - self.cycle_start >= 3:
      for radar_point in self.radar_points.values():
        radar_point.measured = False

    # Clear screen if nothing has been published in a while
    if self.frame - self.cycle_start > IDLE_FRAMES:
      self.radar_points.clear()

    # Update current cycle
    if 0x60B in updated_addrs:
      objects = zip(
        self.rcp.vl_all['ObjectData']['ID'],       # trackId
        self.rcp.vl_all['ObjectData']['DistLong'], # dRed
        self.rcp.vl_all['ObjectData']['DistLat'],  # yRel
        self.rcp.vl_all['ObjectData']['VRelLong'], # vRel
        self.rcp.vl_all['ObjectData']['VRelLat'],  # yvRel
        self.rcp.vl_all['ObjectData']['DynProp'],
        self.rcp.vl_all['ObjectData']['Class'],
        self.rcp.vl_all['ObjectData']['RCS'],
      )

      # Update radar_points array with newest valid data available (skip over invalid updates)
      for track_id, dist_long, dist_lat, vrel_long, vrel_lat, dyn_prop, obj_class, rcs in objects:
        track_id = int(track_id)
        dist_long += RADAR_DREL_OFFSET
        dist_lat += RADAR_YREL_OFFSET
        if dist_long < 0:
          dist_long = 0

        # wrong object type (not car)
        if obj_class in [ObjClass.RESERVED]:
          continue
        # wrong movement type
        if dyn_prop in [DynProp.CROSSING_LEFT, DynProp.CROSSING_RIGHT]:
          continue

        # Update radarPoint
        self.radar_points[track_id].update(
          cycle=self.cycle,
          trackId=track_id,
          dynProp=int(dyn_prop),
          objClass=int(obj_class),
          dRel=dist_long,
          yRel=dist_lat,
          vRel=vrel_long,
          aRel=float('nan'),
          yvRel=vrel_lat,
          rcs=rcs,
        )


    # publish to liveTracks at 33hz (every ~30ms)
    # liveTracks updates listeners at 20hz (every 50ms)
    if self.frame % 3 == 0:
      ret = car.RadarData()
      if not self.rcp.can_valid:
        ret.errors.canError = True
      ret.points = [radar_point.rp for radar_point in self.radar_points.values()]
      return ret

    # return None to card.py so it will not try to publish to liveTracks
    return None
