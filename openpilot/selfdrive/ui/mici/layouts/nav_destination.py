import threading

import pyray as rl
import requests

from openpilot.cereal import log
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.navigation.geocode import Place, add_route_summary, search
from openpilot.selfdrive.navigation.helpers import latest_gps_source
from openpilot.selfdrive.ui.mici.widgets.dialog import BigInputDialog, PADDING
from openpilot.selfdrive.ui.ui_state import device, ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight, MousePos
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.scroller import NavScroller

RESULTS_TIMEOUT = 30  # seconds
SAVED_PLACES = {"NavHome": "nav_home", "NavWork": "nav_work"}  # param, ui_state attribute
ICONS = "icons_mici/navigation/"
CARD_BORDER = 48  # px of the card texture's corners and glow kept unscaled
TEXT_SIZE = 35
GREY = rl.Color(255, 255, 255, int(255 * 0.55))
GREEN = rl.Color(70, 230, 90, 255)
KM_PER_MILE = 1.609344


def picker_visible(started: bool, navigation_enabled: bool, destination_set: bool) -> bool:
  # offroad it's always there to pick or change the destination, onroad until one is set
  return navigation_enabled and (not started or not destination_set)


def active_card(destination: dict | None, home: dict | None, work: dict | None) -> str | None:
  # which picker card stands for the current destination, the search pin for anything not saved
  if destination is None:
    return None
  return "home" if destination == home else "work" if destination == work else "search"


def card_text(name: str, saved: dict | None) -> tuple[str, str]:
  # a saved place shows its name and that holding picks another, an unset one that a tap sets it
  if saved is None:
    return name, "tap to set"
  return saved.get("place_name") or name, "hold to change"


def set_destination(destination: dict):
  ui_state.params.put("NavDestination", destination, block=True)
  ui_state.nav_destination = destination


def pick_place(place: Place, save_as: str | None = None):
  if save_as is not None:
    ui_state.params.put(save_as, place.to_param(), block=True)
    setattr(ui_state, SAVED_PLACES[save_as], place.to_param())
  set_destination(place.to_param())


def open_destination_search(hint: str = "enter address...", save_as: str | None = None, text: str = ""):
  gui_app.push_widget(SearchDialog(hint, text, lambda query: gui_app.push_widget(DestinationResults(query, hint, save_as))))


def saved_position(data: bytes | None) -> tuple[float, float] | None:
  # ubloxd saves its last fix as a gpsLocationExternal event
  if not data:
    return None
  try:
    with log.Event.from_bytes(data) as evt:
      gps = evt.gpsLocationExternal
      return (gps.latitude, gps.longitude) if gps.hasFix else None
  except Exception:
    return None


def near() -> tuple[float, float] | None:
  # gps only runs onroad, offroad the last saved fix is the best guess
  source = latest_gps_source(ui_state.sm)
  if source is not None:
    return ui_state.sm[source].latitude, ui_state.sm[source].longitude
  return saved_position(ui_state.params.get("LastGPSPosition"))


def format_distance(meters: float) -> str:
  if ui_state.is_metric:
    return f"{meters / 1000:.1f} km"
  return f"{meters / 1000 / KM_PER_MILE:.1f} mi"


def format_duration(seconds: float) -> str:
  minutes = max(1, round(seconds / 60))
  return f"{minutes} min" if minutes < 60 else f"{minutes // 60} h {minutes % 60} min"


def fit(font: rl.Font, text: str, size: int, width: float) -> str:
  if measure_text_cached(font, text, size).x <= width:
    return text
  while text and measure_text_cached(font, text + "...", size).x > width:
    text = text[:-1]
  return text.rstrip() + "..."


class NavCard(Widget):
  """The settings cards' look (glow, inner shadow) at any size."""

  def __init__(self, width: float, height: float):
    super().__init__()
    self.set_rect(rl.Rectangle(0, 0, width, height))
    self._bg = gui_app.texture("icons_mici/buttons/button_rectangle.png", 402, 180)
    self._bg_pressed = gui_app.texture("icons_mici/buttons/button_rectangle_pressed.png", 402, 180)

  def _render(self, rect: rl.Rectangle):
    rl.draw_rectangle_rounded(rect, 0.3, 8, rl.Color(0, 0, 0, int(255 * 0.5)))
    texture = self._bg_pressed if self.is_pressed else self._bg
    patch = rl.NPatchInfo(rl.Rectangle(0, 0, texture.width, texture.height), CARD_BORDER, CARD_BORDER, CARD_BORDER, CARD_BORDER,
                          rl.NPatchLayout.NPATCH_NINE_PATCH)
    rl.draw_texture_n_patch(texture, patch, rect, rl.Vector2(0, 0), 0, rl.WHITE)
    self._draw_content(rect)

  def _draw_content(self, rect: rl.Rectangle):
    pass


class IconCard(NavCard):
  TITLE_SIZE, HINT_SIZE = 28, 22
  PAD = 14

  def __init__(self, icon: str, icon_width: int, icon_height: int):
    super().__init__(0, 0)
    self._icon = gui_app.texture(ICONS + icon, icon_width, icon_height)
    self._bold, self._roman = gui_app.font(FontWeight.BOLD), gui_app.font(FontWeight.ROMAN)
    self.active = False
    self.title = ""
    self.hint = ""

  def _draw_content(self, rect: rl.Rectangle):
    # the icon, and under it what the card is and what a tap or hold does
    text_h = self.TITLE_SIZE + (self.HINT_SIZE + 4 if self.hint else 0) + 10
    top = rect.y + (rect.height - self._icon.height - text_h) / 2
    rl.draw_texture_ex(self._icon, rl.Vector2(rect.x + (rect.width - self._icon.width) / 2, top), 0, 1, GREEN if self.active else rl.WHITE)
    y = top + self._icon.height + 10
    for text, font, size, color in ((self.title, self._bold, self.TITLE_SIZE, rl.WHITE), (self.hint, self._roman, self.HINT_SIZE, GREY)):
      if not text:
        continue
      text = fit(font, text, size, rect.width - 2 * self.PAD)
      width = measure_text_cached(font, text, size).x
      rl.draw_text_ex(font, text, rl.Vector2(rect.x + (rect.width - width) / 2, y), size, 0, color)
      y += size + 4


class DestinationPicker(Widget):
  """Home, work and search, shown onroad in place of the arrow and map when there's no destination.
  Home and work go straight there once set, a long press picks them again."""
  GAP = 12
  MARGIN = 8

  def __init__(self):
    super().__init__()
    self._home, self._work, self._search = IconCard("home.png", 75, 65), IconCard("work.png", 77, 65), IconCard("pin.png", 52, 65)
    for card, param, label in ((self._home, "NavHome", "home"), (self._work, "NavWork", "work")):
      card.set_click_callback(lambda p=param, l=label: self._go(p, l))
      card.set_long_press_callback(lambda p=param, l=label: open_destination_search(f"{l} address...", save_as=p))
    self._search.set_click_callback(open_destination_search)
    for card in (self._home, self._work, self._search):
      card.set_touch_valid_callback(self._touch_valid)

  @staticmethod
  def _go(param: str, label: str):
    saved = getattr(ui_state, SAVED_PLACES[param])
    if saved is None:
      open_destination_search(f"{label} address...", save_as=param)
    else:
      set_destination(saved)

  def _render(self, rect: rl.Rectangle):
    # the card of the current destination is green, the pin for a searched one
    active = active_card(ui_state.nav_destination, ui_state.nav_home, ui_state.nav_work)
    for name, card in (("home", self._home), ("work", self._work), ("search", self._search)):
      card.active = name == active
    for card, name, saved in ((self._home, "home", ui_state.nav_home), (self._work, "work", ui_state.nav_work)):
      card.title, card.hint = card_text(name, saved)
    self._search.title = "search"

    # two wide cards and a narrow one, like the settings scroller's first cards
    unit = (rect.width - 2 * self.MARGIN - 2 * self.GAP) / 4
    x, y, h = rect.x + self.MARGIN, rect.y + self.MARGIN, rect.height - 2 * self.MARGIN
    for card, width in ((self._home, 1.5 * unit), (self._work, 1.5 * unit), (self._search, unit)):
      card.render(rl.Rectangle(x, y, width, h))
      x += width + self.GAP


class SearchDialog(BigInputDialog):
  """The keyboard with a magnifier in place of the enter arrow."""

  def __init__(self, hint: str, text: str, confirm_callback):
    super().__init__(hint, default_text=text, confirm_callback=confirm_callback)
    self._enter_img = gui_app.texture(ICONS + "search_enter.png", 76, 62)
    self._enter_disabled_img = gui_app.texture(ICONS + "search_enter_disabled.png", 76, 62)


class ResultCard(NavCard):
  WIDTH, HEIGHT = 360, 160
  PAD = 26

  def __init__(self, place: Place):
    super().__init__(self.WIDTH, self.HEIGHT)
    self.place = place
    self._bold, self._roman = gui_app.font(FontWeight.BOLD), gui_app.font(FontWeight.ROMAN)
    self._pin = gui_app.texture(ICONS + "pin.png", 44, 55)

  def _draw_content(self, rect: rl.Rectangle):
    x, width = rect.x + self.PAD, rect.width - 2 * self.PAD - self._pin.width - 12
    rl.draw_text_ex(self._bold, fit(self._bold, self.place.name, 44, width), rl.Vector2(x, rect.y + 18), 44, 0, rl.WHITE)
    rl.draw_text_ex(self._roman, fit(self._roman, self.place.details, 30, width), rl.Vector2(x, rect.y + 66), 30, 0, rl.WHITE)
    if self.place.distance is not None and self.place.duration is not None:
      rl.draw_text_ex(self._roman, format_distance(self.place.distance), rl.Vector2(x, rect.y + 108), 30, 0, GREY)
      rl.draw_text_ex(self._roman, format_duration(self.place.duration), rl.Vector2(x + 150, rect.y + 108), 30, 0, GREY)
    pin = rl.Vector2(rect.x + rect.width - self.PAD - self._pin.width, rect.y + (rect.height - self._pin.height) / 2)
    rl.draw_texture_ex(self._pin, pin, 0, 1, rl.Color(255, 255, 255, int(255 * 0.9)))


class StatusCard(NavCard):
  def __init__(self, text: str, detail: str):
    super().__init__(ResultCard.WIDTH, ResultCard.HEIGHT)
    self.text, self.detail = text, detail
    self._bold, self._roman = gui_app.font(FontWeight.BOLD), gui_app.font(FontWeight.ROMAN)

  def _draw_content(self, rect: rl.Rectangle):
    rl.draw_text_ex(self._bold, self.text, rl.Vector2(rect.x + ResultCard.PAD, rect.y + 34), 44, 0, rl.WHITE)
    rl.draw_text_ex(self._roman, self.detail, rl.Vector2(rect.x + ResultCard.PAD, rect.y + 90), 30, 0, GREY)


class DestinationResults(NavScroller):
  """Search results under the search bar, with route length and driving time. Tapping the bar edits the search."""
  HEADER = 62

  def __init__(self, query: str, hint: str, save_as: str | None = None):
    super().__init__()
    self._query, self._hint, self._save_as = query, hint, save_as
    self._font = gui_app.font(FontWeight.ROMAN)
    self._search_icon = gui_app.texture(ICONS + "search_enter.png", 76, 62)
    self._backspace = gui_app.texture("icons_mici/settings/keyboard/backspace.png", 42, 36)
    self._result: list[Place] | str | None = None  # places, or an error message
    self._status = StatusCard("searching...", query)
    self._scroller.add_widget(self._status)
    threading.Thread(target=self._search, args=(query, near()), daemon=True).start()

  def show_event(self):
    super().show_event()
    # onroad dialogs close after 5 s untouched, the search alone can take that long
    device.set_override_interactive_timeout(RESULTS_TIMEOUT)

  def hide_event(self):
    super().hide_event()
    device.set_override_interactive_timeout(None)

  def _search(self, query: str, start: tuple[float, float] | None):
    try:
      places = search(query, start)
    except requests.exceptions.RequestException:
      cloudlog.exception("nav destination search failed")
      self._result = "check your internet connection"
      return
    self._result = places
    if start is not None:
      for place in places:  # cards show each summary as it arrives
        add_route_summary(place, start)

  def _update_state(self):
    super()._update_state()
    if self._result is None:
      return
    result, self._result = self._result, None
    if isinstance(result, str):
      self._status.text, self._status.detail = "search failed", result
    elif not result:
      self._status.text, self._status.detail = "no matches", "try adding a city"
    else:
      self._status.set_visible(False)
      for place in result:
        card = ResultCard(place)
        card.set_click_callback(lambda p=place: self.dismiss(lambda: pick_place(p, self._save_as)))
        self._scroller.add_widget(card)

  def _handle_mouse_release(self, mouse_pos: MousePos):
    super()._handle_mouse_release(mouse_pos)
    if mouse_pos.y < self._rect.y + self.HEADER and not self.is_dismissing:
      self.dismiss(lambda: open_destination_search(self._hint, self._save_as, self._query))

  def _render(self, rect: rl.Rectangle):
    rl.draw_texture_ex(self._search_icon, rl.Vector2(rect.x + PADDING / 2, rect.y), 0, 1, rl.WHITE)
    text_x = rect.x + PADDING / 2 + self._search_icon.width + PADDING
    backspace_x = rect.x + rect.width - self._backspace.width - 27
    query = fit(self._font, self._query, TEXT_SIZE, backspace_x - text_x - PADDING)
    rl.draw_text_ex(self._font, query, rl.Vector2(text_x, rect.y + PADDING - 5), TEXT_SIZE, 0, rl.WHITE)
    rl.draw_texture_ex(self._backspace, rl.Vector2(backspace_x, rect.y + 14), 0, 1, rl.WHITE)
    self._scroller.render(rl.Rectangle(rect.x, rect.y + self.HEADER, rect.width, rect.height - self.HEADER))
