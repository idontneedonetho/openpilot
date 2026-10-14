import pytest

from openpilot.selfdrive.ui.mici.onroad.arrow_view import FULL_ACCEL, PULSE_OFF_ACCEL, PULSE_ON_ACCEL, PULSE_SPEEDS, pulse_on, pulse_speed


@pytest.mark.parametrize("pulsing, accel, on", [
  (False, 0.0, False),
  (False, 0.2, True),     # slight accel already pulses
  (False, -0.2, True),    # and slight braking
  (True, 0.12, True),     # between the thresholds it keeps what it was, no flicker
  (False, 0.12, False),
  (True, 0.05, False),
])
def test_pulse_on(pulsing, accel, on):
  assert pulse_on(pulsing, accel) == on


def test_pulse_speed_grows_with_accel():
  assert pulse_speed(PULSE_ON_ACCEL) == pytest.approx(PULSE_SPEEDS[0])
  assert pulse_speed(-FULL_ACCEL) == pytest.approx(PULSE_SPEEDS[1])
  assert pulse_speed(0.5) < pulse_speed(1.0)
  assert PULSE_OFF_ACCEL < PULSE_ON_ACCEL
