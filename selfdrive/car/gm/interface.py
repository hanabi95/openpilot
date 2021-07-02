#!/usr/bin/env python3
from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.car.gm.values import CAR, CruiseButtons, AccState
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase
from common.params import Params

ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName

class CarInterface(CarInterfaceBase):

  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 4.0

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), has_relay=False, car_fw=None):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint, has_relay)
    ret.carName = "gm"
    ret.safetyModel = car.CarParams.SafetyModel.gm
    ret.enableCruise = False  # stock cruise control is kept off

    # GM port is a community feature
    # TODO: make a port that uses a car harness and it only intercepts the camera
    ret.communityFeature = True

    # Presence of a camera on the object bus is ok.
    # Have to go to read_only if ASCM is online (ACC-enabled cars),
    # or camera is on powertrain bus (LKA cars without ACC).
    ret.enableCamera = True
    ret.enableGasInterceptor = 0x201 in fingerprint[0]
    ret.openpilotLongitudinalControl = ret.enableCamera and ret.enableGasInterceptor

    params = Params()
    LQR_enabled = params.get_bool("LQR_Selected")
    INDI_enabled = params.get_bool("INDI_Selected")

    if LQR_enabled:
      ret.lateralTuning.init('lqr')
      ret.lateralTuning.lqr.scale = 1950.0
      ret.lateralTuning.lqr.ki = 0.055
      ret.lateralTuning.lqr.a = [0., 1., -0.22619643, 1.21822268]
      ret.lateralTuning.lqr.b = [-1.92006585e-04, 3.95603032e-05]
      ret.lateralTuning.lqr.c = [1., 0.]
      ret.lateralTuning.lqr.k = [-110.73572306, 451.22718255]
      ret.lateralTuning.lqr.l = [0.3233671, 0.3185757]
      ret.lateralTuning.lqr.dcGain = 0.002237852961363602
    elif not LQR_enabled and INDI_enabled:
      ret.lateralTuning.init('indi')
      ret.lateralTuning.indi.innerLoopGainBP = [10., 30.]
      ret.lateralTuning.indi.innerLoopGainV = [5.5, 8.0]
      ret.lateralTuning.indi.outerLoopGainBP = [10., 30.]
      ret.lateralTuning.indi.outerLoopGainV = [4.5, 7.0]
      ret.lateralTuning.indi.timeConstantBP = [10., 30.]
      ret.lateralTuning.indi.timeConstantV = [1.8, 3.5]
      ret.lateralTuning.indi.actuatorEffectivenessBP = [0.]
      ret.lateralTuning.indi.actuatorEffectivenessV = [2.0]
    elif not LQR_enabled and not INDI_enabled:
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[10., 30.0], [10., 30.0]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.12, 0.18], [0.02, 0.03]]
      ret.lateralTuning.pid.kdBP = [0.]
      ret.lateralTuning.pid.kdV = [0.]  #corolla from shane fork : 0.725
      ret.lateralTuning.pid.kf = 0.000045

    ret.steerRateCost = 1.0
    ret.steerActuatorDelay = 0.1  # Default delay, not measured yet

    if candidate == CAR.VOLT:
      # supports stop and go, but initial engage must be above 18mph (which include conservatism)
      ret.minEnableSpeed = 18 * CV.MPH_TO_MS
      ret.minSteerSpeed = 7 * CV.MPH_TO_MS
      ret.mass = 1607. + STD_CARGO_KG
      ret.wheelbase = 2.69
      ret.steerRatio = 15.7
      ret.steerRatioRear = 0.
      ret.centerToFront = ret.wheelbase * 0.4  # wild guess
      tire_stiffness_factor = 0.444  # not optimized yet

    elif candidate == CAR.BOLT:
      # initial engage unkown - copied from Volt. Stop and go unknown.
      ret.minEnableSpeed = -1
      ret.minSteerSpeed = 5
      ret.mass = 1625. + STD_CARGO_KG
      ret.safetyModel = car.CarParams.SafetyModel.gm
      ret.wheelbase = 2.60096
      ret.steerRatio = 15.2
      ret.steerRatioRear = 0.
      ret.centerToFront = ret.wheelbase * 0.49
      tire_stiffness_factor = 1.0

    elif candidate == CAR.MALIBU:
      # supports stop and go, but initial engage must be above 18mph (which include conservatism)
      ret.minEnableSpeed = 18 * CV.MPH_TO_MS
      ret.minSteerSpeed = 7 * CV.MPH_TO_MS
      ret.mass = 1496. + STD_CARGO_KG
      ret.wheelbase = 2.83
      ret.steerRatio = 15.8
      ret.steerRatioRear = 0.
      ret.centerToFront = ret.wheelbase * 0.4  # wild guess
      tire_stiffness_factor = 0.444  # not optimized yet

    elif candidate == CAR.HOLDEN_ASTRA:
      ret.mass = 1363. + STD_CARGO_KG
      ret.wheelbase = 2.662
      # Remaining parameters copied from Volt for now
      ret.centerToFront = ret.wheelbase * 0.4
      ret.minEnableSpeed = 18 * CV.MPH_TO_MS
      ret.minSteerSpeed = 7 * CV.MPH_TO_MS
      ret.steerRatio = 15.7
      ret.steerRatioRear = 0.
      tire_stiffness_factor = 0.444  # not optimized yet

    elif candidate == CAR.ACADIA:
      ret.minEnableSpeed = -1.  # engage speed is decided by pcm
      ret.minSteerSpeed = 7 * CV.MPH_TO_MS
      ret.mass = 4353. * CV.LB_TO_KG + STD_CARGO_KG
      ret.wheelbase = 2.86
      ret.steerRatio = 14.4  # end to end is 13.46
      ret.steerRatioRear = 0.
      ret.centerToFront = ret.wheelbase * 0.4
      tire_stiffness_factor = 0.444  # not optimized yet

    elif candidate == CAR.BUICK_REGAL:
      ret.minEnableSpeed = 18 * CV.MPH_TO_MS
      ret.minSteerSpeed = 7 * CV.MPH_TO_MS
      ret.mass = 3779. * CV.LB_TO_KG + STD_CARGO_KG  # (3849+3708)/2
      ret.wheelbase = 2.83  # 111.4 inches in meters
      ret.steerRatio = 14.4  # guess for tourx
      ret.steerRatioRear = 0.
      ret.centerToFront = ret.wheelbase * 0.4  # guess for tourx
      tire_stiffness_factor = 0.444  # not optimized yet

    elif candidate == CAR.CADILLAC_ATS:
      ret.minEnableSpeed = 18 * CV.MPH_TO_MS
      ret.minSteerSpeed = 7 * CV.MPH_TO_MS
      ret.mass = 1601. + STD_CARGO_KG
      ret.wheelbase = 2.78
      ret.steerRatio = 15.3
      ret.steerRatioRear = 0.
      ret.centerToFront = ret.wheelbase * 0.49
      tire_stiffness_factor = 0.444  # not optimized yet

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    ret.longitudinalTuning.kpBP = [0.0, 5.0, 35.0] #CRV_EU
    ret.longitudinalTuning.kpV = [1.2, 0.8, 0.5] #CRV_EU
    ret.longitudinalTuning.kiBP = [0., 35.] #CRV_EU
    ret.longitudinalTuning.kiV = [0.18, 0.12] #CRV_EU
    ret.longitudinalTuning.kfBP = [0.]
    ret.longitudinalTuning.kfV = [1.]

    if ret.enableGasInterceptor:
      ret.gasMaxBP = [0.]
      ret.gasMaxV =  [0.6]  #value from honda

    ret.stoppingControl = True
    ret.startAccel = 1.0

    ret.steerLimitTimer = 0.4
    ret.radarTimeStep = 0.0667  # GM radar runs at 15Hz instead of standard 20Hz

    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    self.cp.update_strings(can_strings)

    ret = self.CS.update(self.cp)

    ret.cruiseState.enabled = self.CS.main_on or self.CS.adaptive_Cruise
    ret.canValid = self.cp.can_valid
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    buttonEvents = []

    if self.CS.cruise_buttons != self.CS.prev_cruise_buttons and self.CS.prev_cruise_buttons != CruiseButtons.INIT:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.unknown
      if self.CS.cruise_buttons != CruiseButtons.UNPRESS:
        be.pressed = True
        but = self.CS.cruise_buttons
      else:
        be.pressed = False
        but = self.CS.prev_cruise_buttons
      if but == CruiseButtons.RES_ACCEL:
        be.type = ButtonType.accelCruise
      elif but == CruiseButtons.DECEL_SET:
        be.type = ButtonType.decelCruise
      elif but == CruiseButtons.CANCEL:
        be.type = ButtonType.cancel
      elif but == CruiseButtons.MAIN:
        be.type = ButtonType.altButton3
      buttonEvents.append(be)

    ret.buttonEvents = buttonEvents

    events = self.create_common_events(ret)

    if ret.vEgo < self.CP.minEnableSpeed:
      events.add(EventName.belowEngageSpeed)
    if self.CS.park_brake:
      events.add(EventName.parkBrake)
    if ret.vEgo < self.CP.minSteerSpeed:
      events.add(car.CarEvent.EventName.belowSteerSpeed)
    #if self.CP.enableGasInterceptor:
    #  if self.CS.adaptive_Cruise and ret.brakePressed:
    #    events.add(EventName.pedalPressed)
    #    self.CS.adaptive_Cruise = False
    #    self.CS.enable_lkas = True

    # handle button presses
    if not self.CS.main_on and self.CP.enableGasInterceptor:
      for b in ret.buttonEvents:
        if (b.type == ButtonType.decelCruise and not b.pressed) and not self.CS.adaptive_Cruise:
          self.CS.adaptive_Cruise = True
          self.CS.enable_lkas = True
          events.add(EventName.buttonEnable)
        if (b.type == ButtonType.accelCruise and not b.pressed) and not self.CS.adaptive_Cruise:
          self.CS.adaptive_Cruise = True
          self.CS.enable_lkas = False
          events.add(EventName.buttonEnable)
        if (b.type == ButtonType.cancel and b.pressed) and self.CS.adaptive_Cruise:
          self.CS.adaptive_Cruise = False
          self.CS.enable_lkas = True
          events.add(EventName.buttonCancel)
    elif self.CS.main_on:
      self.CS.adaptive_Cruise = False
      self.CS.enable_lkas = True

    ret.events = events.to_msg()

    # copy back carState packet to CS
    self.CS.out = ret.as_reader()

    return self.CS.out

  def apply(self, c):
    hud_v_cruise = c.hudControl.setSpeed
    if hud_v_cruise > 70:
      hud_v_cruise = 0

    # For Openpilot, "enabled" includes pre-enable.
    can_sends = self.CC.update(c.enabled, self.CS, self.frame,
                               c.actuators,
                               hud_v_cruise, c.hudControl.lanesVisible,
                               c.hudControl.leadVisible, c.hudControl.visualAlert)

    self.frame += 1
    return can_sends
