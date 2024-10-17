import struct

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.common.numpy_fast import clip
from opendbc.car.interfaces import V_CRUISE_MAX
from opendbc.car.tesla.values import CANBUS, CarControllerParams

def can_cksum(mm):
  s = 0
  for c in mm:
    s += (c>>4)
    s += c & 0xF
  s = 8-s
  s %= 0x10
  return s

def fix(msg, addr):
  msg2 = msg[0:-1] + bytes([msg[-1] | can_cksum(struct.pack("I", addr)+msg)])
  return msg2

def make_can_msg(addr, dat, idx, alt):
  if idx is not None:
    dat += bytes([idx << 4])
    dat = fix(dat, addr)
  return [addr, dat, alt]

class TeslaCAN:
  def __init__(self, packer):
    self.packer = packer

  @staticmethod
  def checksum(msg_id, dat):
    ret = (msg_id & 0xFF) + ((msg_id >> 8) & 0xFF)
    ret += sum(dat)
    return ret & 0xFF

  def create_steering_control(self, angle, enabled, counter):
    values = {
      "DAS_steeringAngleRequest": -angle,
      "DAS_steeringHapticRequest": 0,
      "DAS_steeringControlType": 1 if enabled else 0,
      "DAS_steeringControlCounter": counter,
    }

    data = self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)[1]
    values["DAS_steeringControlChecksum"] = self.checksum(0x488, data[:3])
    return self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_longitudinal_command(self, acc_state, accel, cntr, active):
    values = {
      "DAS_setSpeed": 0 if (accel < 0 or not active) else V_CRUISE_MAX,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": CarControllerParams.JERK_LIMIT_MIN,
      "DAS_jerkMax": CarControllerParams.JERK_LIMIT_MAX,
      "DAS_accelMin": accel,
      "DAS_accelMax": max(accel, 0),
      "DAS_controlCounter": cntr,
      "DAS_controlChecksum": 0,
    }
    data = self.packer.make_can_msg("DAS_control", CANBUS.party, values)[1]
    values["DAS_controlChecksum"] = self.checksum(0x2b9, data[:7])
    return self.packer.make_can_msg("DAS_control", CANBUS.party, values)

  def create_steering_allowed(self, counter):
    values = {
      "APS_eacAllow": 1,
      "APS_eacMonitorCounter": counter,
    }

    data = self.packer.make_can_msg("APS_eacMonitor", CANBUS.party, values)[1]
    values["APS_eacMonitorChecksum"] = self.checksum(0x27d, data[:2])
    return self.packer.make_can_msg("APS_eacMonitor", CANBUS.party, values)

  def create_radar_commands(self, v_ego, idx):
    commands = []

    v_ego_kph = clip(int(round(v_ego * CV.MS_TO_KPH)), 0, 255)
    msg_0x300 = bytes([0xf9, v_ego_kph, 0x8a, 0xd0, 0x20 if idx == 0 or idx == 3 else 0x00, 0x00, 0x00])

    commands.append(make_can_msg(0x300, msg_0x300, idx, 1))
    commands.append(make_can_msg(0x301, b"\x00\x00\x56\x02\x58\x00\x00", idx, 1))
    return commands
