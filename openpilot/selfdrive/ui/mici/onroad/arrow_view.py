import numpy as np
import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.shader_polygon import draw_polygon
from openpilot.system.ui.widgets import Widget

# a top down view of the model's lane lines and plan, smoothed over time
DISTANCE = 50.0  # meters the arrow spans
LATERAL_SCALE = 14.0  # px per meter on a REF_WIDTH panel, lateral is stretched and distance squished to fit the panel
REF_WIDTH = 179
BOTTOM_MARGIN = 22
TOP_MARGIN = 28
SMOOTH_RC = 0.25  # seconds
MATCH_DISTANCE = 1.5  # meters a lane line can move between frames and still be the same line
OUTER_LINE_PROB = 0.3  # the outer lines only show when the model sees a lane there
LANE_CHANGE_LENGTH = 0.5  # part of the arrow the lane change crosses over in
PULSE_ON_ACCEL = 0.15  # m/s^2, slight accel or braking already pulses
PULSE_OFF_ACCEL = 0.1  # back to solid below this, the gap keeps it from flickering
FULL_ACCEL = 1.5  # m/s^2 where the bands flow fastest
PULSE_PERIOD = 44  # px from one bright band to the next
PULSE_SPEEDS = (20.0, 100.0)  # px/s the bands flow at PULSE_ON_ACCEL and FULL_ACCEL

PATH_WIDTH = 10
LANE_WIDTH = 3
OUTER_LANE_WIDTH = 2
HEAD_LENGTH = 15  # length of each arrowhead arm
LEAD_BAR_WIDTH = 30

WHITE = rl.Color(255, 255, 255, 255)
GREEN = rl.Color(70, 230, 90, 255)
GREY = rl.Color(110, 110, 110, 255)
LIGHT_GREY = rl.Color(200, 200, 200, 255)
DIM = rl.Color(60, 60, 60, 255)
ORANGE = rl.Color(255, 120, 30, 255)
RED = rl.Color(235, 45, 35, 255)
MAX_HEADING = 45.0  # degrees on screen the arrow bends to at most
MAX_LAT_ACCEL = 3.0  # m/s^2, ISO limit openpilot steers to
HIGH_EFFORT = 0.65


def _blend(a: rl.Color, b: rl.Color, t: float) -> rl.Color:
  t = float(np.clip(t, 0, 1))
  return rl.Color(int(a.r + (b.r - a.r) * t), int(a.g + (b.g - a.g) * t), int(a.b + (b.b - a.b) * t), int(a.a + (b.a - a.a) * t))


def _effort_color(base: rl.Color, effort: float) -> rl.Color:
  # the lane line keeps its color until 65% of the steering limit, then orange, red at the limit
  if effort < HIGH_EFFORT:
    return _blend(base, ORANGE, (effort - HIGH_EFFORT + 0.1) / 0.1)
  return _blend(ORANGE, RED, (effort - HIGH_EFFORT) / (1 - HIGH_EFFORT))


def pulse_on(pulsing: bool, accel: float) -> bool:
  return abs(accel) > (PULSE_OFF_ACCEL if pulsing else PULSE_ON_ACCEL)


def pulse_speed(accel: float) -> float:
  return float(np.interp(abs(accel), (PULSE_ON_ACCEL, FULL_ACCEL), PULSE_SPEEDS))


def _strip(px: np.ndarray, py: np.ndarray, width: float, color: rl.Color):
  tx, ty = np.gradient(px), np.gradient(py)
  norm = np.maximum(np.hypot(tx, ty), 1e-6)
  nx, ny = -ty / norm * width / 2, tx / norm * width / 2
  # left edge first, then the right edge back, so the strip faces the camera
  points = np.concatenate([np.stack([px - nx, py - ny], axis=1), np.stack([px + nx, py + ny], axis=1)[::-1]]).astype(np.float32)
  lo, hi = points.min(axis=0), points.max(axis=0)
  draw_polygon(rl.Rectangle(lo[0], lo[1], hi[0] - lo[0] + 1, hi[1] - lo[1] + 1), points, color)


class _Line:
  def __init__(self, y: np.ndarray):
    self.y = y
    self.alpha = 0.4
    self.seen = True


class ArrowView(Widget):
  """The model's lane lines and planned path from above, with the car at the bottom center. Lines are tracked by position,
  so a line the car crosses during a lane change moves under it instead of jumping."""

  def __init__(self):
    super().__init__()
    self._k = (1 / gui_app.target_fps) / (SMOOTH_RC + 1 / gui_app.target_fps)
    self._xs = np.linspace(0.0, DISTANCE, 40)
    self._lines: list[_Line] = []
    self._plan: np.ndarray | None = None
    self._effort = 0.0
    self._curvature = 0.0
    self._lane_change = 0.0
    self._approaching = False  # a lane change's line is the nearest to the car
    self._crossed = False  # and the car got past it
    self._lane_width = 3.5
    self._target = 0.0  # lateral offset of the lane change's target lane center
    self._lead = DISTANCE
    self._lead_alpha = 0.0
    self._accel = 0.0
    self._phase = 0.0
    self._pulsing = False

  def _smooth(self, old, new):
    return old + (new - old) * self._k

  def _update_state(self):
    sm = ui_state.sm
    model = sm['modelV2']
    if len(model.position.x) < 2:
      return

    plan = np.interp(self._xs, model.position.x, model.position.y)
    self._plan = plan if self._plan is None else self._smooth(self._plan, plan)

    # match this frame's lines to the tracked ones by where they are next to the car
    for line in self._lines:
      line.seen = False
    for i, lane_line in enumerate(model.laneLines):
      if len(lane_line.x) < 2:
        continue
      if i in (0, 3) and not self._road_there(model, i):
        continue
      y = np.interp(self._xs, lane_line.x, lane_line.y)
      free = [t for t in self._lines if not t.seen]
      match = min(free, key=lambda t: abs(t.y[0] - y[0]), default=None)
      if match is not None and abs(match.y[0] - y[0]) < MATCH_DISTANCE:
        match.y = self._smooth(match.y, y)
      else:
        match = _Line(y)
        self._lines.append(match)
      match.seen = True
    for line in self._lines:
      # solid while seen, fades in and out as lines appear and drop
      line.alpha = self._smooth(line.alpha, 1.0 if line.seen else 0.0)
    self._lines = [line for line in self._lines if line.seen or line.alpha > 0.02]

    # how much of the steering limit is used, torque output when the car has one
    lateral = sm['controlsState'].lateralControlState
    if lateral.which() == 'torqueState':
      effort = abs(lateral.torqueState.output)
    else:
      effort = abs(model.action.desiredCurvature) * sm['carState'].vEgo ** 2 / MAX_LAT_ACCEL
    self._effort = self._smooth(self._effort, min(effort, 1.0))
    self._curvature = self._smooth(self._curvature, model.action.desiredCurvature)
    self._lane_change = self._smooth(self._lane_change, 0.0 if model.meta.laneChangeState == 'off' else 1.0)
    self._update_lane_change(model.meta)

    # the bands flow toward the car accelerating, away from it braking, faster the harder
    self._accel = self._smooth(self._accel, sm['carState'].aEgo)
    self._pulsing = pulse_on(self._pulsing, self._accel)
    if self._pulsing:
      self._phase += np.sign(self._accel) * pulse_speed(self._accel) / gui_app.target_fps

    lead = None
    if sm.alive['radarState'] and sm['radarState'].leadOne.present:
      lead = sm['radarState'].leadOne.dRel
    elif len(model.leadsV3) and model.leadsV3[0].prob > 0.5:
      lead = model.leadsV3[0].x[0]
    if lead is not None and lead < DISTANCE:
      self._lead = self._smooth(self._lead, lead) if self._lead_alpha > 0.05 else lead
      self._lead_alpha = self._smooth(self._lead_alpha, 1.0)
    else:
      self._lead_alpha = self._smooth(self._lead_alpha, 0.0)

  def _draw_path(self, px: np.ndarray, py: np.ndarray, color: rl.Color):
    # solid coasting, accelerating or braking the path dims and bright bands flow along it
    if not self._pulsing:
      _strip(px, py, PATH_WIDTH, color)
      return
    _strip(px, py, PATH_WIDTH, rl.Color(color.r, color.g, color.b, 40))
    band = color
    s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(px), np.diff(py)))])
    radius = PATH_WIDTH / 2
    for start in np.arange(-(self._phase % PULSE_PERIOD), s[-1], PULSE_PERIOD):
      # round capped bands, the caps count toward their length
      a, b = max(start + radius, 0.0), min(start + PULSE_PERIOD / 2 - radius, s[-1])
      if b - a > 1.0:
        ss = np.linspace(a, b, max(int((b - a) / 4), 2))
        qx, qy = np.interp(ss, s, px), np.interp(ss, s, py)
        _strip(qx, qy, PATH_WIDTH, band)
        rl.draw_circle_v(rl.Vector2(float(qx[0]), float(qy[0])), radius, band)
        rl.draw_circle_v(rl.Vector2(float(qx[-1]), float(qy[-1])), radius, band)

  @staticmethod
  def _road_there(model, i: int) -> bool:
    # an outer line past the road edge or one the model barely sees is no lane
    if len(model.laneLineProbs) > i and model.laneLineProbs[i] < OUTER_LINE_PROB:
      return False
    edge = model.roadEdges[0 if i == 0 else 1] if len(model.roadEdges) == 2 else None
    if edge is None or len(edge.y) == 0:
      return True
    line = model.laneLines[i].y[0]
    return line > edge.y[0] if i == 0 else line < edge.y[0]

  def _update_lane_change(self, meta):
    left = max((line for line in self._lines if line.seen and line.y[0] < 0), key=lambda line: line.y[0], default=None)
    right = min((line for line in self._lines if line.seen and line.y[0] >= 0), key=lambda line: line.y[0], default=None)
    if left is None or right is None:
      return
    changing = meta.laneChangeState in ('laneChangeStarting', 'laneChangeFinishing') and meta.laneChangeDirection != 'none'
    if not changing:
      self._approaching = self._crossed = False
      self._lane_width = float(np.clip(right.y[0] - left.y[0], 2.5, 4.5))
      target = (left.y[0] + right.y[0]) / 2
    else:
      # the model snaps the crossed line from one side of the car to the other, so the car is in the new lane
      # once the nearest line flips away from the turn side, after it got nearest first
      direction = -1 if meta.laneChangeDirection == 'left' else 1
      ahead, behind = (left, right) if direction < 0 else (right, left)
      ahead_nearest = abs(ahead.y[0]) < abs(behind.y[0])
      self._crossed = self._crossed or (self._approaching and not ahead_nearest)
      self._approaching = self._approaching or ahead_nearest
      if self._crossed:
        target = (left.y[0] + right.y[0]) / 2
      else:
        target = ahead.y[0] + direction * self._lane_width / 2
    self._target = self._smooth(self._target, float(target))

  def _render(self, rect: rl.Rectangle):
    rl.draw_rectangle_rec(rect, rl.BLACK)
    if not ui_state.sm.alive['modelV2'] or self._plan is None:
      return

    origin_x, origin_y = rect.x + rect.width / 2, rect.y + rect.height - BOTTOM_MARGIN
    span = rect.height - BOTTOM_MARGIN - TOP_MARGIN
    plan = self._plan
    left = max((line for line in self._lines if line.y[0] < 0), key=lambda line: line.y[0], default=None)
    right = min((line for line in self._lines if line.y[0] >= 0), key=lambda line: line.y[0], default=None)
    dist, long_scale = DISTANCE, span / DISTANCE
    lateral_scale = LATERAL_SCALE * rect.width / REF_WIDTH

    def to_screen(x, y):
      return origin_x + np.asarray(y) * lateral_scale, origin_y - np.asarray(x) * long_scale

    # the heading cap in meters that looks like MAX_HEADING on screen, lateral is stretched
    cap = np.arctan(np.tan(np.radians(MAX_HEADING)) * long_scale / lateral_scale)

    def arc(y0, length):
      # an arc of the steering curvature, straight at the car and bending toward the top, offset sideways by y0
      ss = np.linspace(0.0, length, 40)
      theta = cap * np.tanh(self._curvature * dist / cap) * (ss / dist) ** 2
      ds = np.diff(ss, prepend=0.0)
      ax, ay = np.cumsum(np.cos(theta) * ds), np.cumsum(np.sin(theta) * ds)
      return ax - y0 * np.sin(theta), ay + y0 * np.cos(theta)

    def line_points(y_line):
      return to_screen(*arc(y_line[0], dist))

    def plan_points(end):
      ax, ay = arc(plan[0], end)
      if self._lane_change > 0.01:
        # lane change: a short s into the target lane's center, then straight on in it
        u = np.clip(ax / max(float(ax[-1]) * LANE_CHANGE_LENGTH, 1.0), 0.0, 1.0)
        ay = ay + (self._target - plan[0]) * (3 * u ** 2 - 2 * u ** 3) * self._lane_change
      return to_screen(ax, ay)

    engaged = ui_state.status == UIStatus.ENGAGED
    override = ui_state.status == UIStatus.OVERRIDE
    rl.begin_scissor_mode(int(rect.x), int(rect.y), int(rect.width), int(rect.height))

    # lanes: the two lines around the car green engaged and white on override, the rest grey, all dim when off
    # near the steering limit the outside line of the curve lights up, the car drifts out there on low torque
    # model frame, positive curvature turns right
    outside = left if self._curvature > 0 else right
    for line in self._lines:
      inner = line is left or line is right
      if engaged or override:
        color = (GREEN if engaged else WHITE) if inner else GREY
      else:
        color = DIM
      if line is outside and engaged:
        color = _effort_color(color, self._effort)
      color = rl.Color(color.r, color.g, color.b, int(255 * line.alpha))
      if color.a > 2:
        _strip(*line_points(line.y), LANE_WIDTH if inner else OUTER_LANE_WIDTH, color)

    # the planned path, ending in an arrowhead that flattens into a bar at the lead car, light grey on override, hidden when off
    if engaged or override:
      path_color = WHITE if engaged else LIGHT_GREY
      lead_t = self._lead_alpha
      end = dist + (min(self._lead, dist) - dist) * lead_t
      px, py = plan_points(max(end, 4.0))
      self._draw_path(px, py, path_color)

      tip = rl.Vector2(float(px[-1]), float(py[-1]))
      dx, dy = px[-1] - px[-2], py[-1] - py[-2]
      norm = max(float(np.hypot(dx, dy)), 1e-6)
      dx, dy = dx / norm, dy / norm
      # arms swept back 45 degrees, opening to 90 into the lead bar
      angle = np.radians(45 + 45 * lead_t)
      length = HEAD_LENGTH + (LEAD_BAR_WIDTH / 2 - HEAD_LENGTH) * lead_t
      for side in (-1, 1):
        c, s = np.cos(angle), np.sin(angle) * side
        ax, ay = -(dx * c - dy * s), -(dx * s + dy * c)
        end_point = rl.Vector2(tip.x + ax * length, tip.y + ay * length)
        rl.draw_line_ex(tip, end_point, PATH_WIDTH, path_color)
        rl.draw_circle_v(end_point, PATH_WIDTH / 2, path_color)
      rl.draw_circle_v(tip, PATH_WIDTH / 2, path_color)

    rl.end_scissor_mode()
