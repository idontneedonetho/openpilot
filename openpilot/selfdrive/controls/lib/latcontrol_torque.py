import math
import numpy as np

from openpilot.cereal import log
from opendbc.car.lateral import FRICTION_THRESHOLD, get_friction
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.pid import PIDController
from openpilot.selfdrive.controls.lib.latcontrol import LatControl

# Lateral torque controller with NNFF-Lite feedforward, ported from FrogPilot
# (FrogAi/FrogPilot, branch FrogPilot-Staging):
#   frogpilot/selfdrive/controls/lib/neural_network_feedforward.py
# Lite path only — no: the neural-net model machinery (FluxModel, model JSON, full
# NNFF branch) is intentionally omitted. No toggle — this is the standard
# lateral torque controller for this Prius-only fork.
#
# Deltas vs the stock lat-accel-space controller:
#   1) PID error computed in TORQUE space (handles the non-linear rack).
#   2) Low-speed curvature blend into the setpoint/measurement.
#   3) IMU yaw-rate curvature blend below ~5 m/s (self.yaw_rate, set by
#      controlsd from the calibrated device pose).
#   4) Friction feedforward with the FrogPilot line-342 fix.
#
# kp/ki are the historical NNFF constants (StarPilot/FrogPilot hardcode
# PIDController(1.0, 0.3, ...)). This fork keeps kp/ki in the unset CarParams
# deprecated group, so they cannot be read at runtime.

KP = 1.0
KI = 0.3

# Low-speed blend: weight curvature (m^-1) against lateral accel (m/s^2) in the
# PID error so the controller has authority at parking speeds where the v^2
# term vanishes. Squared to sharpen the falloff (12 -> 0 over 0..30 m/s).
LOW_SPEED_X = [0, 10, 20, 30]
LOW_SPEED_Y = [12, 3, 1, 0]

# IMU yaw-rate curvature is trusted only below this speed (m/s).
LLK_BLEND_SPEEDS = [2.0, 5.0]

VERSION = 1


class LatControlTorque(LatControl):
  def __init__(self, CP, CI, dt):
    super().__init__(CP, CI, dt)
    # Build a private builder so per-frame update_torque_parameters can mutate
    # the live-tuned params without touching the (read-only) CarParams.
    self.torque_params = CP.lateralTuning.torque.as_builder()
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.pid = PIDController(KP, KI, rate=1 / self.dt,
                             pos_limit=self.steer_max, neg_limit=-self.steer_max)
    self.use_steering_angle = bool(self.torque_params.deprecated.useSteeringAngle)
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    # Set by controlsd only from a valid calibrated device pose
    # (calibrated_pose.angular_velocity.yaw); None until the first valid frame
    # so the IMU blend stays off (pure vehicle-model curvature) before that.
    self.yaw_rate = None

  def update_torque_parameters(self, latAccelFactor, latAccelOffset, friction):
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, lat_delay):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = VERSION
    if not active:
      output_torque = 0.0
      pid_log.active = False
    else:
      roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY

      actual_curvature_vm = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
      if self.use_steering_angle:
        actual_curvature = actual_curvature_vm
        curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
      else:
        # Torque-controlled rack: blend in IMU yaw rate below ~5 m/s where the
        # angle sensor is noisy. yaw_rate (rad/s) / vEgo (m/s) = curvature (m^-1),
        # left-positive, same sign as actual_curvature_vm / desired_curvature.
        if self.yaw_rate is not None and CS.vEgo > 0.1:
          actual_curvature_llk = self.yaw_rate / CS.vEgo
          actual_curvature = float(np.interp(CS.vEgo, LLK_BLEND_SPEEDS, [actual_curvature_vm, actual_curvature_llk]))
        else:
          actual_curvature = actual_curvature_vm
        curvature_deadzone = 0.0

      desired_lateral_accel = desired_curvature * CS.vEgo ** 2
      actual_lateral_accel = actual_curvature * CS.vEgo ** 2
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

      # Low-speed curvature blend (FrogPilot Lite).
      low_speed_factor = float(np.interp(CS.vEgo, LOW_SPEED_X, LOW_SPEED_Y)) ** 2
      setpoint = desired_lateral_accel + low_speed_factor * desired_curvature
      measurement = actual_lateral_accel + low_speed_factor * actual_curvature

      # PID error in torque space (handles the non-linear rack).
      torque_from_setpoint = self.torque_from_lateral_accel(setpoint, self.torque_params)
      torque_from_measurement = self.torque_from_lateral_accel(measurement, self.torque_params)
      pid_log.error = float(torque_from_setpoint - torque_from_measurement)

      # Friction feedforward (FrogPilot line-342 fix): the unblended lateral
      # accel error feeds the friction response. For a torque-controlled car the
      # lookahead jerk term is deliberately zero (FrogPilot lines 275-278).
      friction_input = desired_lateral_accel - actual_lateral_accel
      friction_compensation = get_friction(friction_input, lateral_accel_deadzone, FRICTION_THRESHOLD, self.torque_params)

      # Feedforward = torque to hold the gravity-adjusted desired accel, plus
      # friction. (FrogPilot Lite does not apply latAccelOffset here — the
      # offset is only stored on torque_params, never read back in this path.)
      gravity_adjusted_lateral_accel = desired_lateral_accel - roll_compensation
      ff = self.torque_from_lateral_accel(
        gravity_adjusted_lateral_accel + friction_compensation,
        self.torque_params)

      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5
      output_torque = self.pid.update(pid_log.error, speed=CS.vEgo, feedforward=ff, freeze_integrator=freeze_integrator)

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.d = float(self.pid.d)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(-output_torque)
      pid_log.actualLateralAccel = float(actual_lateral_accel)
      pid_log.desiredLateralAccel = float(desired_lateral_accel)
      pid_log.desiredLateralJerk = 0.0
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
