from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState, long_control_state_trans


class TestLongControlStateTransition(OpenpilotTestCase):
  def test_stay_stopped(self):
    active = True
    current_state = LongCtrlState.stopping
    next_state = long_control_state_trans(active, current_state, v_ego=0.1,
                                should_stop=True, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(active, current_state, v_ego=0.1,
                                should_stop=False, brake_pressed=True, cruise_standstill=False)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(active, current_state, v_ego=0.1,
                                should_stop=False, brake_pressed=False, cruise_standstill=True)
    assert next_state == LongCtrlState.stopping
    # starting condition met: enter starting, then hand off with a zero planner target
    next_state = long_control_state_trans(active, current_state, v_ego=1.0,
                                should_stop=False, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.starting
    next_state = long_control_state_trans(active, next_state, v_ego=1.1,
                                should_stop=False, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.pid
    active = False
    next_state = long_control_state_trans(active, current_state, v_ego=1.0,
                                should_stop=False, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.off

  def test_engage(self):
    active = True
    current_state = LongCtrlState.off
    next_state = long_control_state_trans(active, current_state, v_ego=0.1,
                               should_stop=True, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(active, current_state, v_ego=0.1,
                               should_stop=False, brake_pressed=True, cruise_standstill=False)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(active, current_state, v_ego=0.1,
                               should_stop=False, brake_pressed=False, cruise_standstill=True)
    assert next_state == LongCtrlState.stopping
    # starting condition met: enter starting (strong start is always on),
    # then hand off with a zero planner target
    next_state = long_control_state_trans(active, current_state, v_ego=0.1,
                               should_stop=False, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.starting
    next_state = long_control_state_trans(active, next_state, v_ego=0.2,
                               should_stop=False, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.pid

  def test_starting(self):
    active = True
    current_state = LongCtrlState.starting
    next_state = long_control_state_trans(active, current_state, v_ego=0.1,
                               should_stop=False, brake_pressed=False, cruise_standstill=False,
                               a_target=0.5)
    assert next_state == LongCtrlState.starting
    next_state = long_control_state_trans(active, current_state, v_ego=1.0,
                               should_stop=False, brake_pressed=False, cruise_standstill=False,
                               a_target=1.2)
    assert next_state == LongCtrlState.pid
