from opendbc.car.interfaces import V_CRUISE_MAX
from opendbc.car.tesla.values import CANBUS, CarControllerParams


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
      "DAS_steeringControlType": 1 if enabled else 0, # 1=ANGLE_CONTROL
      "DAS_steeringControlCounter": counter,
    }

    data = self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)[1]
    values["DAS_steeringControlChecksum"] = self.checksum(0x488, data[:3])
    return self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_steering_allowed(self, counter):
    values = {
      "APS_eacAllow": 1,
      "APS_eacMonitorCounter": counter,
    }

    data = self.packer.make_can_msg("APS_eacMonitor", CANBUS.party, values)[1]
    values["APS_eacMonitorChecksum"] = self.checksum(0x27d, data[:2])
    return self.packer.make_can_msg("APS_eacMonitor", CANBUS.party, values)

  def create_longitudinal_command(self, acc_state, accel, cntr, active):
    values = {
      "DAS_setSpeed": 0 if (accel < 0 or not active) else V_CRUISE_MAX,
      "DAS_accState": acc_state, # 4=ACC_ON 13=ACC_CANCEL_GENERIC_SILENT
      "DAS_aebEvent": 0, # 0=NOT_ACTIVE
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


  def create_status(self, counter, enabled):
    msgs = []

    values = {
      "DAS_autopilotState": 3 if enabled else 2, # 2=AVAILABLE 3=ACTIVE_NOMINAL 6=FSD
      "DAS_suppressSpeedWarning": 1, # 1=Suppress_Speed_Warning
      "DAS_forwardCollisionWarning": 3, # 3=SNA
      "DAS_csaState": 2, # 2=ENABLE
      "DAS_laneDepartureWarning": 5, # 5=SNA
      "DAS_autopilotHandsOnState": 1, # 0=NOT_REQD 1=REQD_DETECTED 2=REQD_NOT_DETECTED
      "DAS_autoLaneChangeState": 8, # 1=UNAVAILABLE_NO_LANES 8=AVAILABLE_BOTH
      "DAS_visionOnlySpeedLimit": 155, # 155=NONE
      "DAS_fusedSpeedLimit": 90,
      "DAS_statusCounter": counter,
    }
    data = self.packer.make_can_msg("DAS_status", CANBUS.party, values)[1]
    values["DAS_statusChecksum"] = self.checksum(0x39b, data[:7])
    msgs.append(self.packer.make_can_msg("DAS_status", CANBUS.party, values))

    values = {
      "DAS_accSpeedLimit": 204.4, # normally speed limit except 204.4 when on highway
      "DAS_ACC_report": 24, # ACC_REPORT_BEHAVIOR_REPORT
      "DAS_lssState": 3 if enabled else 1, # 1=LDW 2=LKA 3=ELK
      "DAS_longCollisionWarning": 15, # 15=SNA
      "DAS_status2Counter": counter,
    }
    data = self.packer.make_can_msg("DAS_status2", CANBUS.party, values)[1]
    values["DAS_status2Checksum"] = self.checksum(0x389, data[:7])
    msgs.append(self.packer.make_can_msg("DAS_status2", CANBUS.party, values))

    return msgs
