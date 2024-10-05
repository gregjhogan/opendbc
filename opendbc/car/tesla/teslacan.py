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
      "DAS_steeringControlType": 1 if enabled else 0,
      "DAS_steeringControlCounter": counter,
    }

    data = self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)[1]
    values["DAS_steeringControlChecksum"] = self.checksum(0x488, data[:3])
    return self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_longitudinal_command(self, acc_state, accel, cntr):
    values = {
      "DAS_setSpeed": 0 if accel < 0 else V_CRUISE_MAX,
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


  def create_das_status2(self, das_status2, counter, enabled):
    values = {s: das_status2[s] for s in [
      "DAS_activationRequest",
      "DAS_pmmObstacleSeverity",
      "DAS_pmmLoggingRequest",
      "DAS_activationFailureStatus",
      "DAS_pmmUltrasonicsFaultReason",
      "DAS_pmmRadarFaultReason",
      "DAS_pmmSysFaultReason",
      "DAS_pmmCameraFaultReason",
      "DAS_ACC_report",
      "DAS_lssState",
      "DAS_radarTelemetry",
      "DAS_robState",
      "DAS_driverInteractionLevel",
      "DAS_ppOffsetDesiredRamp",
      "DAS_longCollisionWarning",
    ]}

    values["DAS_activationRequest"] = 1 if enabled else 0
    values["DAS_status2Counter"] = counter
    data = self.packer.make_can_msg("DAS_status2", CANBUS.party, values)[1]
    values["DAS_status2Checksum"] = self.checksum(0x389, data[:7])
    return self.packer.make_can_msg("DAS_status2", CANBUS.party, values)