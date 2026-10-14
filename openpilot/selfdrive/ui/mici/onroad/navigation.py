import time

import numpy as np
import pyray as rl


from msgq.visionipc import VisionIpcClient
from openpilot.cereal.visionipc import VisionStreamType
from openpilot.selfdrive.navigation.helpers import latest_gps_source
from openpilot.selfdrive.navigation.renderer import EGO, ROUTE_INTENSITY, SIZE
from openpilot.selfdrive.navigation.route import RouteTracker
from openpilot.selfdrive.ui.mici.onroad.cameraview import CONNECTION_RETRY_INTERVAL
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import FontWeight, TextAlignmentVertical
from openpilot.system.ui.lib.shader_polygon import draw_polygon
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.label import gui_label

STALE_SECONDS = 3
MARKER_MARGIN = 18  # pixels kept below the ego so its marker stays visible
PLAN_START_M = 12.0  # starts past the ego marker's tip
PLAN_LENGTH_M = 90.0
PLAN_WIDTH = 6  # pixels
PLAN_COLOR = rl.Color(255, 150, 40, 255)
TANGENT_HALF_WINDOW = 5.0  # meters, the route's direction right at the car
COURSE_MIN_SPEED = 4.0  # m/s, below that the gps course is noise
COURSE_MAX_ERROR = 10.0  # degrees of reported bearing accuracy

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


def plan_rotation(chord: float, tangent: float, course: float | None = None) -> float:
  """Degrees clockwise from the map's up to the car's heading. The map is headed along the route's +-50 m chord,
  the car along the route right where it is, or along its gps course when that is trustworthy."""
  heading = tangent if course is None else course
  return (heading - chord + 180) % 360 - 180


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
    self._route: np.ndarray | None = None
    self._tracker: RouteTracker | None = None

  def _update_state(self):
    sm = ui_state.sm
    if sm.updated['navRoute']:
      route = np.array([(c.longitude, c.latitude) for c in sm['navRoute'].coordinates], dtype=np.float64).reshape((-1, 2))
      if self._route is None or not np.array_equal(route, self._route):
        self._route = route
        self._tracker = RouteTracker(route) if len(route) >= 2 else None

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
    self._draw_plan(rl.Vector2(rect.x + EGO[0] * scale, rect.y + (EGO[1] - source.y) * scale), scale)

  def _rotation(self) -> float:
    """The car's heading on the map, found the way mapsd places the map."""
    sm = ui_state.sm
    source = latest_gps_source(sm)
    if self._tracker is None or source is None:
      return 0.0
    gps = sm[source]
    progress = self._tracker.project(gps.longitude, gps.latitude) + max(gps.speed, 0.) * (time.monotonic() - sm.recv_time[source])
    chord, tangent = self._tracker.pose(progress)[2], self._tracker.pose(progress, TANGENT_HALF_WINDOW)[2]
    trusted = gps.speed > COURSE_MIN_SPEED and 0 < gps.bearingAccuracyDeg < COURSE_MAX_ERROR
    return plan_rotation(chord, tangent, gps.bearingDeg if trusted else None)

  def _draw_plan(self, ego: rl.Vector2, scale: float):
    """The model's planned path, turned from the car's frame onto the route-up map at 1 m per pixel."""
    model = ui_state.sm['modelV2']
    if not ui_state.sm.alive['modelV2'] or len(model.position.x) < 2:
      return
    xs, ys = np.array(model.position.x), np.array(model.position.y)
    # resample every meter along the path so the far, sparse plan points don't show as corners
    along = np.r_[0, np.cumsum(np.hypot(np.diff(xs), np.diff(ys)))]
    s = np.arange(PLAN_START_M, min(along[-1], PLAN_LENGTH_M), 1.0)
    if len(s) < 2:
      return
    # plan frame is x forward, y right, the car's forward is turned clockwise from the map's up
    x, y = np.interp(s, along, xs), np.interp(s, along, ys)
    r = np.radians(self._rotation())
    px = ego.x + (x * np.sin(r) + y * np.cos(r)) * scale
    py = ego.y + (-x * np.cos(r) + y * np.sin(r)) * scale
    tx, ty = np.gradient(px), np.gradient(py)
    norm = np.hypot(tx, ty)
    nx, ny = -ty / norm * PLAN_WIDTH / 2, tx / norm * PLAN_WIDTH / 2
    # left edge first, then the right edge back, so the strip faces the camera
    points = np.concatenate([np.stack([px - nx, py - ny], axis=1), np.stack([px + nx, py + ny], axis=1)[::-1]]).astype(np.float32)
    lo, hi = points.min(axis=0), points.max(axis=0)
    draw_polygon(rl.Rectangle(lo[0], lo[1], hi[0] - lo[0], hi[1] - lo[1]), points, PLAN_COLOR)
    for i in (0, -1):  # round caps
      rl.draw_circle_v(rl.Vector2(px[i], py[i]), PLAN_WIDTH / 2, PLAN_COLOR)

  def _unload_texture(self):
    if self._texture:
      rl.unload_texture(self._texture)
      self._texture = None

  def close(self):
    self._unload_texture()
