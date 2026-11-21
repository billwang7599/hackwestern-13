"""Field geometry shared by the vision side (see goalie/docs/architecture.md).

Origin: centre of the goal mouth on the goal line. Metres.
  x: across the goal, +x = right in the overhead image (goal at the bottom).
  y: out from the goal line towards the shooter; incoming shots have vy < 0.

FIELD_W is not in the contract (only the goal width and field length are);
it is whatever you measure between the side walls, passed to calibrate.py.
"""
from __future__ import annotations

GOAL_HALF_W = 0.11        # 22 cm goal
FIELD_L = 0.60            # goal line -> shooter end
FIELD_W = 0.40            # side wall to side wall (default; calib.json overrides)
BALL_R = 0.02             # ping-pong ball radius
DEFAULT_PORT = 5556
WD_MS = 300               # Pi watchdog: send something at least this often


def in_goal(x: float) -> bool:
    return -GOAL_HALF_W <= x <= GOAL_HALF_W
