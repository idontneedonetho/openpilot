from openpilot.common.test import OpenpilotTestCase
from openpilot.common.parameterized import parameterized

from openpilot.cereal import log
from opendbc.car.structs import car
from opendbc.car.car_helpers import interfaces
from opendbc.car.toyota.values import CAR as TOYOTA
from opendbc.car.vehicle_model import VehicleModel
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque


def get_controller(car_name):
  CarInterface = interfaces[car_name]
  CP = CarInterface.get_non_essential_params(car_name)
  CI = CarInterface(CP)
  VM = VehicleModel(CP)
  controller = LatControlTorque(CP.as_reader(), CI, DT_CTRL)
  return controller, VM


class TestLatControlTorqueNNFFLite(OpenpilotTestCase):

  @parameterized.expand([(TOYOTA.TOYOTA_PRIUS,)])
  def test_friction_feedforward_engages(self, car_name):
    # NNFF-Lite adds a friction feedforward term (get_friction on the lateral
    # accel error). With a non-zero curvature error the feedforward (lac_log.f)
    # must be non-zero; with zero error it must be zero.
    controller, VM = get_controller(car_name)
    controller.update_torque_parameters(1.6, 0.0, 0.15)
    CS = car.CarState.new_message()
    CS.vEgo = 15
    CS.steeringAngleDeg = 0.0
    CS.steeringPressed = False
    params = log.VehicleParameters.new_message()

    _, _, lac = controller.update(True, CS, VM, params, False, 0.01, False, 0.2)
    self.assertGreater(abs(lac.f), 0.0, "friction feedforward should engage with a curvature error")

    controller.reset()
    _, _, lac0 = controller.update(True, CS, VM, params, False, 0.0, False, 0.2)
    self.assertEqual(lac0.f, 0.0, "friction feedforward should be zero with zero error")

  @parameterized.expand([(TOYOTA.TOYOTA_PRIUS,)])
  def test_torque_space_error_direction(self, car_name):
    # The PID error is computed in torque space. A left vs a right turn must
    # request opposite-sign corrective torque (before the final sign flip).
    controller, VM = get_controller(car_name)
    controller.update_torque_parameters(1.6, 0.0, 0.15)
    controller.yaw_rate = None
    CS = car.CarState.new_message()
    CS.vEgo = 15
    CS.steeringAngleDeg = 0.0
    CS.steeringPressed = False
    params = log.VehicleParameters.new_message()

    pos, _, _ = controller.update(True, CS, VM, params, False, 0.01, False, 0.2)
    controller.reset()
    neg, _, _ = controller.update(True, CS, VM, params, False, -0.01, False, 0.2)
    self.assertNotEqual(pos, 0.0)
    self.assertNotEqual(neg, 0.0)
    self.assertLess(pos * neg, 0.0, "left and right turns must request opposite-sign torque")
