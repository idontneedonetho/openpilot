import numpy as np
import pyray as rl


from msgq.visionipc import VisionIpcClient
from openpilot.cereal.visionipc import VisionStreamType
from openpilot.selfdrive.navigation.helpers import latest_gps_source
from openpilot.selfdrive.navigation.renderer import EGO, ROUTE_INTENSITY, SIZE
from openpilot.selfdrive.ui.mici.onroad.cameraview import CONNECTION_RETRY_INTERVAL
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import FontWeight, TextAlignmentVertical
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.label import gui_label

STALE_SECONDS = 3
MARKER_MARGIN = 18  # pixels kept below the ego so its marker stays visible

# Recolor the model's grayscale frame for people: dim the white roads and make the grey route pop.
MAP_COLORS = np.repeat(np.round(np.arange(256) * 0.45).astype(np.uint8)[:, None], 3, axis=1)
ROUTE_COLOR = (72, 235, 132)


def colorize(gray: np.ndarray) -> np.ndarray:
  # only 2x2 blocks at the route's level are route, antialiased road edges pass through that grey as 1 pixel lines
  rgb = MAP_COLORS[gray]
  level = np.abs(gray.astype(np.int16) - ROUTE_INTENSITY) <= 1
  block = level[:-1, :-1] & level[1:, :-1] & level[:-1, 1:] & level[1:, 1:]
  route = np.zeros_like(level)
  route[:-1, :-1] |= block
  route[1:, :-1] |= block
  route[:-1, 1:] |= block
  route[1:, 1:] |= block
  rgb[route] = ROUTE_COLOR
  return rgb


class NavigationPanel(Widget):
  """The map frames mapsd publishes for the model, full screen."""

  def __init__(self):
    super().__init__()
    self._client = VisionIpcClient('mapsd', VisionStreamType.VISION_STREAM_MAP, True)
    self._last_connect = 0.0
    self._frame_time = 0.0
    self._texture = None
    self._accent = rl.Color(155, 230, 185, 255)
    self._muted = rl.Color(162, 175, 185, 255)

  def _update_state(self):
    if not self._client.is_connected():
      if rl.get_time() - self._last_connect < CONNECTION_RETRY_INTERVAL:
        return
      self._last_connect = rl.get_time()
      if not self._client.connect(False) or not self._client.num_buffers:
        return
      self._unload_texture()
      self._texture = rl.load_texture_from_image(rl.Image(None, SIZE, SIZE, 1, rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8))
    frame = self._client.recv(timeout_ms=0)
    if frame is not None:
      gray = np.frombuffer(frame.data, dtype=np.uint8, count=SIZE * SIZE).reshape(SIZE, SIZE)
      rl.update_texture(self._texture, rl.ffi.cast("void *", rl.ffi.from_buffer(colorize(gray))))
      # Without a usable map, mapsd publishes black frames with a zero timestamp.
      self._frame_time = rl.get_time() if self._client.timestamp_sof else 0.0

  @property
  def routing(self):
    return ui_state.sm.valid['navInstruction'] and ui_state.sm.alive['navInstruction']

  @property
  def map_ready(self):
    return self._texture is not None and 0 <= rl.get_time() - self._frame_time < STALE_SECONDS

  def _draw_status(self, rect):
    """What the map is waiting on, in place of the map."""
    statuses = (
      ('GPS fix', latest_gps_source(ui_state.sm) is not None),
      ('Route', self.routing),
      ('Map frames', self.map_ready),
    )
    for i, (name, ok) in enumerate(statuses):
      gui_label(rl.Rectangle(rect.x + 24, rect.y + 24 + i * 40, rect.width - 48, 36), f'{name}: {"yes" if ok else "no"}', 32,
                self._accent if ok else self._muted, FontWeight.MEDIUM, alignment_vertical=TextAlignmentVertical.TOP)

  def _render(self, rect):
    rl.draw_rectangle_rec(rect, rl.BLACK)
    if not self.map_ready:
      self._draw_status(rect)
      return
    # The frame scaled to the full width, cropped to what is ahead of the ego with its marker near the bottom.
    scale = rect.width / SIZE
    height = min(SIZE, rect.height / scale)
    source = rl.Rectangle(0, min(SIZE - height, max(0, EGO[1] + MARKER_MARGIN - height)), SIZE, height)
    rl.draw_texture_pro(self._texture, source, rl.Rectangle(rect.x, rect.y, rect.width, height * scale), rl.Vector2(0, 0), 0, rl.WHITE)

  def _unload_texture(self):
    if self._texture:
      rl.unload_texture(self._texture)
      self._texture = None

  def close(self):
    self._unload_texture()
