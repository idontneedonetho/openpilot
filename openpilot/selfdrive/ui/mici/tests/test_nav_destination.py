import pytest

from openpilot.common.params import Params
from openpilot.selfdrive.ui.mici.layouts.nav_destination import picker_visible


@pytest.mark.parametrize("started, navigation_enabled, destination_set, visible", [
  (False, True, False, True),   # offroad, swiping right shows the picker
  (False, True, True, True),    # offroad it stays, to change the destination
  (True, True, False, True),    # onroad without a destination, the picker first
  (True, True, True, False),    # onroad with one, the arrow and map
  (False, False, False, False),  # navigation off
  (True, False, False, False),
])
def test_picker_visible(started, navigation_enabled, destination_set, visible):
  assert picker_visible(started, navigation_enabled, destination_set) == visible


def test_no_default_destination():
  # a default address hid the picker, and came back after every arrival
  assert Params().get_default_value("NavDestination") is None
