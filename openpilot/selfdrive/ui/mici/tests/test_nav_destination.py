from unittest.mock import call, patch

import pytest

import openpilot.cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.selfdrive.navigation.geocode import Place
from openpilot.selfdrive.ui.mici.layouts.nav_destination import active_card, card_text, pick_place, picker_visible, saved_position
from openpilot.selfdrive.ui.ui_state import ui_state


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


HOME = {"latitude": 32.75, "longitude": -117.19, "place_name": "3911 Harney Street", "place_details": "Old Town, San Diego"}
PARK = {"latitude": 32.73, "longitude": -117.14, "place_name": "Balboa Park", "place_details": "San Diego"}


@pytest.mark.parametrize("destination, home, work, active", [
  (None, HOME, None, None),
  (HOME, HOME, PARK, "home"),
  (PARK, HOME, PARK, "work"),
  (PARK, HOME, None, "search"),  # a searched destination lights the pin
])
def test_active_card(destination, home, work, active):
  assert active_card(destination, home, work) == active


def test_pick_written_immediately():
  # params writes are async by default, the periodic reload right after a pick read no destination and showed the picker again
  place = Place(name=PARK["place_name"], details=PARK["place_details"], latitude=PARK["latitude"], longitude=PARK["longitude"])
  with patch.object(ui_state, "params") as params:
    pick_place(place, save_as="NavWork")
  assert params.put.call_args_list == [call("NavWork", place.to_param(), block=True), call("NavDestination", place.to_param(), block=True)]


def test_saved_position_from_ubloxd():
  # ubloxd saves its last fix as a capnp event, offroad search biases results around it
  msg = messaging.new_message('gpsLocationExternal')
  msg.gpsLocationExternal.hasFix = True
  msg.gpsLocationExternal.latitude, msg.gpsLocationExternal.longitude = 32.75, -117.19
  assert saved_position(msg.to_bytes()) == pytest.approx((32.75, -117.19))
  assert saved_position(None) is None
  assert saved_position(b'{"latitude": 32.75}') is None  # the old json format


def test_card_text():
  # holding to pick home or work again wasn't discoverable, the card says it
  assert card_text("home", None) == ("home", "tap to set")
  assert card_text("home", HOME) == ("3911 Harney Street", "hold to change")
  assert card_text("work", {"latitude": 1, "longitude": 2}) == ("work", "hold to change")
