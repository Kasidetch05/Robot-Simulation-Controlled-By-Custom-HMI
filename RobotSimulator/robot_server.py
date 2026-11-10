"""Robot packing cell simulator: the physics and the web page.

Your robot_io.py is the connection to the PLC (Modbus TCP on port 5022).
This program moves the robot, the conveyors, the products and the cartons,
and shows the cell in your browser at http://localhost:8091

    Infeed conveyor   products arrive from the left and wait at the pick stop
    Camera            checks the product at the pick stop (dent, no label, wrong colour)
    Robot             SCARA-type arm with a vacuum gripper and its own controller:
                      position numbers and the GO / BUSY / DONE handshake
    Carton conveyor   empty cartons come from the left; the pack-station stop holds one
                      in place; OUTFEED_RUN switching on lets the full one go
    Take-away         after the pack station; always running; CARTON_OUT pulses at the exit
    Reject bin        bad products go here

Random factory events (like real life) come by themselves while the machine runs:
weak vacuum, slow camera, supply gaps, slipping belts, a technician opening the door,
a robot servo overload (press RESET). Choose Off / Realistic / Stress on the web page.

The simulator only acts and measures. Your PLC program decides what happens.

Run:  python robot_server.py          (or double-click "Start Robot.bat")
"""

import argparse
import collections
import json
import math
import os
import random
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import robot_io

HERE = os.path.dirname(os.path.abspath(__file__))
PAGE = os.path.join(HERE, "robot_view.html")
CLASSIC = os.path.join(HERE, "robot_view_classic.html")     # the old flat 2D page

# ---------------------------------------------------------------- robot (1 picture unit = 1 mm)
BASE = (620.0, 340.0)    # robot base
ARM1 = 150.0             # upper arm length
ARM2 = 130.0             # forearm length
LIFT_TIME = 0.25         # s to lift or lower the gripper at 100 % speed
TRAVEL_MIN = 0.3         # s for a tiny move at 100 % speed
TRAVEL_PER_PI = 0.7      # extra s for half a turn of the busiest joint at 100 % speed
VACUUM_TIME = 0.3        # s for the vacuum to build up and hold a product
GO_SETTLE = 0.2          # s after GO rises before the robot reads ROBOT_TARGET (the PLC writes
                         # coils and registers in separate Modbus messages, so give the number time)
MOVING = 99              # ROBOT_POS while the robot is not at a position

HOME = (700.0, 240.0)    # position 0
PICK = (450.0, 200.0)    # position 1, at the infeed pick stop
REJECT = (830.0, 200.0)  # position 20, above the reject bin
BIN_W, BIN_H = 90.0, 80.0
BIN_SIZE = 15

# ---------------------------------------------------------------- products and camera
INFEED_Y = PICK[1]
SPAWN_X = 50.0
PRODUCT_D = 34.0
INFEED_SPEED = 120.0     # units per second
SPAWN_EVERY = 2.5        # s between new products from upstream
SCAN_TIME = 0.4          # s the camera needs for one check
DEFECTS = {1: "dent", 2: "no label", 3: "wrong colour"}
NO_PRODUCT = 9           # CAMERA_CODE when nothing is at the pick stop

# ---------------------------------------------------------------- cartons
CARTON_Y = 500.0
CARTON_W = 140.0         # cartons are square
CARTON_GAP = 20.0
CARTON_ENTRY = 300.0     # new empty cartons appear here
PACK_X = 620.0           # pack station (the stop holds a carton here)
TAKEAWAY_X = 720.0       # past this point the take-away belt carries the carton
CARTON_EXIT = 1010.0     # the carton leaves here and CARTON_OUT pulses
OUTFEED_SPEED = 100.0    # units per second (outfeed and take-away)
SLOT_PITCH = 40.0
GRIDS = {4: (2, 2), 6: (3, 2), 9: (3, 3)}   # PATTERN: (columns, rows)
DEFAULT_PATTERN = 6
PULSE_TIME = 0.5         # s CARTON_OUT stays TRUE

# ---------------------------------------------------------------- random factory events
# mean = average seconds between two events of that kind in "realistic" mode (stress: 3x more often)
EVENT_MODES = {"off": {"defect_rate": 5.0, "scale": None},
               "realistic": {"defect_rate": 10.0, "scale": 1.0},
               "stress": {"defect_rate": 20.0, "scale": 1.0 / 3.0}}
RANDOM_EVENTS = {
    "vacuum_weak": {"mean": 140, "dur": (2.5, 5.0), "needs": "running",
                    "start": "Vacuum pressure low: the gripper struggles to hold",
                    "end": "Vacuum pressure back to normal"},
    "camera_slow": {"mean": 170, "dur": (1.5, 3.5), "needs": "running",
                    "start": "Camera image processing is slow", "end": "Camera back to normal speed"},
    "supply_gap": {"mean": 190, "dur": (8.0, 18.0), "needs": "running",
                   "start": "Upstream supply gap: no products arriving", "end": "Products arriving again"},
    "infeed_jam": {"mean": 230, "dur": (2.5, 5.0), "needs": "infeed",
                   "start": "Infeed belt slipping", "end": "Infeed belt running normally"},
    "outfeed_jam": {"mean": 260, "dur": (2.5, 5.0), "needs": "outfeed",
                    "start": "Carton conveyor stalled", "end": "Carton conveyor moving again"},
    "door_open": {"mean": 360, "dur": (4.0, 8.0), "needs": "running",
                  "start": "A technician opened the guard door", "end": "The technician closed the guard door"},
    "servo_fault": {"mean": 540, "dur": None, "needs": "robot_moving",
                    "start": "Robot servo overload on axis 2: press RESET", "end": None},
    "product_slip": {"mean": 200, "dur": None, "needs": "holding", "stress_only": True,
                     "start": "A product slipped from the gripper", "end": None},
}
BIN_EMPTY_AT = 12        # an operator empties the reject bin when it holds this many
FLOOR_CLEAN_AFTER = 25.0  # s until an operator picks up a dropped product

GEOMETRY = {
    "base": BASE, "arm1": ARM1, "arm2": ARM2, "home": HOME, "pick": PICK, "reject": REJECT,
    "bin": [BIN_W, BIN_H], "bin_size": BIN_SIZE, "product_d": PRODUCT_D,
    "infeed": {"x0": 20, "x1": PICK[0] + PRODUCT_D / 2 + 4, "y": INFEED_Y, "w": 56},
    "carton": {"y": CARTON_Y, "w": CARTON_W, "entry": CARTON_ENTRY, "pack": PACK_X,
               "takeaway": TAKEAWAY_X, "exit": CARTON_EXIT, "x0": 215, "x1": 1088, "belt_w": 180},
    "pitch": SLOT_PITCH, "grids": {str(k): v for k, v in GRIDS.items()},
    "defects": {str(k): v for k, v in DEFECTS.items()}, "moving": MOVING, "no_product": NO_PRODUCT,
}

# Every name the physics uses. They must match the lists in your robot_io.py.
NEEDED = {
    "ROBOT_INPUTS": ["ESTOP_OK", "DOOR_CLOSED", "PB_START", "PB_STOP", "PB_RESET", "SEL_REMOTE",
                     "PRODUCT_AT_PICK", "CAMERA_DONE", "CAMERA_PASS", "ROBOT_READY", "ROBOT_BUSY",
                     "ROBOT_DONE", "ROBOT_AT_HOME", "ROBOT_FAULT", "VACUUM_OK", "CARTON_AT_PACK",
                     "CARTON_OUT"],
    "ROBOT_REGISTERS": ["ROBOT_POS", "ROBOT_ANGLE", "ROBOT_REACH", "CAMERA_CODE"],
    "ROBOT_COILS": ["INFEED_RUN", "OUTFEED_RUN", "CAMERA_TRIGGER", "ROBOT_ENABLE", "ROBOT_GO",
                    "ROBOT_RESET", "VACUUM_ON", "LAMP_GREEN", "LAMP_AMBER", "LAMP_RED", "BUZZER"],
    "ROBOT_HOLDING": ["ROBOT_TARGET", "ROBOT_SPEED", "PATTERN"],
}


def check_names(station):
    """Stop with a clear message if robot_io.py is missing a name the physics needs."""
    have = {
        "ROBOT_INPUTS": station.input_names, "ROBOT_REGISTERS": station.register_names,
        "ROBOT_COILS": station.coil_names, "ROBOT_HOLDING": getattr(station, "holding_names", []),
    }
    problems = []
    for list_name, names in NEEDED.items():
        missing = [n for n in names if n not in have[list_name]]
        if missing:
            problems.append("%s in robot_io.py is missing: %s" % (list_name, ", ".join(missing)))
    if problems:
        print("\n".join(problems))
        print("Fix the spelling in robot_io.py, then start again.")
        sys.exit(1)


# ---------------------------------------------------------------- arm geometry
def inverse(point):
    """Joint angles (shoulder, elbow) in radians that put the gripper on point."""
    x = point[0] - BASE[0]
    y = BASE[1] - point[1]                      # picture y points down, maths y points up
    d = min(ARM1 + ARM2 - 0.5, max(abs(ARM1 - ARM2) + 0.5, math.hypot(x, y)))
    c2 = (d * d - ARM1 * ARM1 - ARM2 * ARM2) / (2 * ARM1 * ARM2)
    t2 = math.acos(max(-1.0, min(1.0, c2)))
    t1 = math.atan2(y, x) - math.atan2(ARM2 * math.sin(t2), ARM1 + ARM2 * math.cos(t2))
    return t1, t2


def forward(t1, t2):
    """Elbow and gripper points for the joint angles."""
    ex = BASE[0] + ARM1 * math.cos(t1)
    ey = BASE[1] - ARM1 * math.sin(t1)
    return (ex, ey), (ex + ARM2 * math.cos(t1 + t2), ey - ARM2 * math.sin(t1 + t2))


def wrap(angle):
    """The same angle, between -pi and +pi (the short way round)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def smooth(s):
    """Speed up and slow down gently (0..1 -> 0..1)."""
    return s * s * (3 - 2 * s)


def slot_point(pattern, slot, carton_x=PACK_X):
    """Centre of slot 1..pattern in a carton (row by row from the top-left)."""
    cols, rows = GRIDS[pattern]
    col = (slot - 1) % cols
    row = (slot - 1) // cols
    return (carton_x + (col - (cols - 1) / 2) * SLOT_PITCH,
            CARTON_Y + (row - (rows - 1) / 2) * SLOT_PITCH)


class Product:
    def __init__(self, number, x, defect):
        self.number = number
        self.x = x
        self.y = INFEED_Y
        self.defect = defect                    # None, or 1 dent, 2 no label, 3 wrong colour
        self.since = 0.0                        # when it fell on the floor

    @property
    def good(self):
        return self.defect is None

    def what(self):
        return "good" if self.good else DEFECTS[self.defect]

    def to_json(self):
        return {"n": self.number, "x": round(self.x, 1), "y": round(self.y, 1), "d": self.defect}


class Carton:
    def __init__(self, number, x, pattern):
        self.number = number
        self.x = x
        self.pattern = pattern
        self.slots = {}                         # slot number -> products in that slot
        self.released = False                   # the pack-station stop let it go

    def count(self):
        return sum(len(v) for v in self.slots.values())

    def to_json(self):
        return {"n": self.number, "x": round(self.x, 1), "pattern": self.pattern,
                "released": self.released, "count": self.count(),
                "slots": [[s, [p.defect for p in ps]] for s, ps in sorted(self.slots.items())]}


class Cell:
    def __init__(self, station, defect_rate, spawn_every, events_mode="realistic"):
        self.station = station
        self.lock = threading.Lock()
        self.t = 0.0

        # What the PLC sent last (filled in every physics step)
        self.coils = {n: False for n in NEEDED["ROBOT_COILS"]}
        self.holding = {n: 0 for n in NEEDED["ROBOT_HOLDING"]}
        self.was_connected = False

        # Faults and tests from the web page
        self.faults = {"vacuum_leak": False, "camera_dead": False, "no_cartons": False, "starve": False,
                       "infeed_jam": False, "outfeed_jam": False}
        self.drop_next = False
        self.next_defect = None
        self.events_mode = events_mode if events_mode in EVENT_MODES else "realistic"
        self.defect_rate = EVENT_MODES[self.events_mode]["defect_rate"] if defect_rate is None else defect_rate
        self.spawn_every = spawn_every
        self.next_event = {}                    # kind -> time the next one is due
        self.active = {}                        # kind -> time it ends (None = until reset)
        self.bin_full_since = None

        # Local operator panel at the cell
        self.buttons = {"PB_START": False, "PB_STOP": False, "PB_RESET": False}
        self.estop_pressed = False
        self.door_closed = True
        self.remote = False

        # Robot controller
        self.j1, self.j2 = inverse(HOME)
        self.z = 0.0                            # 0 = up (travel height), 1 = down
        self.move = None
        self.pos = 0                            # None while between positions
        self.done = False
        self.go_wait = None                     # counting down GO_SETTLE after a GO rising edge
        self.fault = None
        self.prev_go = False
        self.prev_reset = False
        self.held = None
        self.vac_timer = 0.0

        # Products
        self.products = []
        self.next_product = 1
        for x in (330.0, 270.0, 210.0):
            self.add_product(x)
        self.spawn_timer = 0.0
        self.floor = []
        self.bin = []

        # Camera
        self.prev_trigger = False
        self.scan_timer = 0.0
        self.cam_done = False
        self.cam_pass = False
        self.cam_code = 0
        self.last_scan = ""

        # Cartons: one at the pack station, two waiting
        self.cartons = []
        self.next_carton = 1
        for x in (PACK_X, PACK_X - CARTON_W - CARTON_GAP, CARTON_ENTRY):
            self.add_carton(x)
        self.prev_outfeed = False
        self.carton_pulse = 0.0

        # Belt animation
        self.offsets = {"infeed": 0.0, "outfeed": 0.0, "takeaway": 0.0}

        # Score: the simulator knows the truth about every product
        self.score = {"packed_good": 0, "rejected_bad": 0, "packed_bad": 0, "rejected_good": 0,
                      "double": 0, "short": 0, "dropped": 0, "shipped": 0}
        self.packed_times = collections.deque()
        self.events = collections.deque(maxlen=40)
        self.said = {}
        self.notes = []

    # ------------------------------------------------------------ helpers
    def log(self, text, level="info"):
        """level: info, good, warn, fault (machine problem), event (random factory event)."""
        minutes, seconds = divmod(int(self.t), 60)
        self.events.appendleft({"t": "%02d:%02d" % (minutes, seconds), "text": text, "level": level})

    def log_once(self, key, text, every=5.0, level="warn"):
        """Log a message, but not again for the same key within a few seconds."""
        last = self.said.get(key)
        if last is None or self.t - last[0] > every or last[1] != text:
            self.said[key] = (self.t, text)
            self.log(text, level)

    def add_product(self, x):
        defect = None
        if self.next_defect:
            defect, self.next_defect = self.next_defect, None
        elif random.random() * 100 < self.defect_rate:
            defect = random.choice(list(DEFECTS))
        self.products.append(Product(self.next_product, x, defect))
        self.next_product += 1

    def add_carton(self, x):
        self.cartons.append(Carton(self.next_carton, x, self.pattern()))
        self.next_carton += 1

    def pattern(self):
        raw = int(self.holding["PATTERN"])
        return raw if raw in GRIDS else DEFAULT_PATTERN

    def speed(self):
        return min(100, max(10, int(self.holding["ROBOT_SPEED"])))

    def tool(self):
        return forward(self.j1, self.j2)[1]

    def product_at_pick(self):
        for p in self.products:
            if abs(p.x - PICK[0]) < 1.0:
                return p
        return None

    def carton_at_pack(self):
        for c in self.cartons:
            if abs(c.x - PACK_X) < 1.0:
                return c
        return None

    def target_point(self, target):
        """Where a position number is, and whether the gripper goes down there."""
        pattern = self.pattern()
        if target == 0:
            return HOME, False
        if target == 1:
            return PICK, True
        if target == 20:
            return REJECT, False
        if 11 <= target <= 10 + pattern:
            return slot_point(pattern, target - 10), True
        return None, False

    def not_ready(self, coils):
        """Why the robot can't move now (None = it can)."""
        if self.fault:
            return "robot fault (%s), pulse ROBOT_RESET" % self.fault
        if not coils["ROBOT_ENABLE"]:
            return "ROBOT_ENABLE is off"
        if not self.door_closed:
            return "the guard door is open"
        return None

    # ------------------------------------------------------------ physics
    def step(self, dt):
        connected = self.station.connected()
        with self.station.lock:
            coils = dict(self.station.coils)
            holding = dict(self.station.holding)
        if not connected:
            # A real I/O module switches its outputs off when the PLC goes quiet
            coils = {n: False for n in coils}

        with self.lock:
            self.t += dt
            if connected != self.was_connected:
                self.log("PLC connected" if connected else "PLC link lost: every output is OFF",
                         "info" if connected else "warn")
                self.was_connected = connected
            self.coils = coils
            self.holding = holding
            self.step_events(coils)
            self.update_notes()
            self.step_safety()
            self.step_robot(dt, coils)
            self.step_vacuum(dt, coils)
            self.step_infeed(dt, coils)
            self.step_camera(dt, coils)
            self.step_cartons(dt, coils)
            inputs, registers = self.sensors(coils)

        with self.station.lock:
            self.station.inputs.update(inputs)
            self.station.registers.update(registers)

    # ------------------------------------------------------------ random factory events
    def schedule(self, kind):
        scale = EVENT_MODES[self.events_mode]["scale"]
        spec = RANDOM_EVENTS[kind]
        if scale is None or (spec.get("stress_only") and self.events_mode != "stress"):
            self.next_event.pop(kind, None)
            return
        self.next_event[kind] = self.t + random.expovariate(1.0 / (spec["mean"] * scale))

    def event_possible(self, kind, c):
        need = RANDOM_EVENTS[kind]["needs"]
        if need == "infeed":
            return c["INFEED_RUN"]
        if need == "outfeed":
            return c["OUTFEED_RUN"]
        if need == "robot_moving":
            return self.move is not None
        if need == "holding":
            return self.held is not None and self.move is not None
        return True

    def start_event(self, kind):
        spec = RANDOM_EVENTS[kind]
        if kind == "vacuum_weak":
            self.faults["vacuum_leak"] = True
        elif kind == "camera_slow":
            self.faults["camera_dead"] = True
        elif kind == "supply_gap":
            self.faults["starve"] = True
        elif kind in ("infeed_jam", "outfeed_jam"):
            self.faults[kind] = True
        elif kind == "door_open":
            self.door_closed = False
        elif kind == "servo_fault":
            self.fault = "servo overload, axis 2"
            self.abort("servo overload")
        elif kind == "product_slip":
            self.drop_next = True
        self.log(spec["start"], "event")
        if spec["dur"]:
            self.active[kind] = self.t + random.uniform(*spec["dur"])
        else:
            self.schedule(kind)

    def end_event(self, kind):
        self.active.pop(kind, None)
        if kind == "vacuum_weak":
            self.faults["vacuum_leak"] = False
        elif kind == "camera_slow":
            self.faults["camera_dead"] = False
        elif kind == "supply_gap":
            self.faults["starve"] = False
        elif kind in ("infeed_jam", "outfeed_jam"):
            self.faults[kind] = False
        elif kind == "door_open":
            self.door_closed = True
        if RANDOM_EVENTS[kind]["end"]:
            self.log(RANDOM_EVENTS[kind]["end"], "info")
        self.schedule(kind)

    def set_events_mode(self, mode):
        for kind in list(self.active):
            self.end_event(kind)
        self.events_mode = mode
        self.defect_rate = EVENT_MODES[mode]["defect_rate"]
        self.next_event.clear()
        self.log("Random factory events: %s" % mode.upper(), "info")

    def step_events(self, c):
        """Random things that happen in a real factory, plus the operators who tidy up."""
        for kind, end in list(self.active.items()):
            if end is not None and self.t >= end:
                self.end_event(kind)

        # Housekeeping by the operators (in every mode)
        if len(self.bin) >= BIN_EMPTY_AT:
            if self.bin_full_since is None:
                self.bin_full_since = self.t
            elif self.t - self.bin_full_since > 6.0:
                self.bin.clear()
                self.bin_full_since = None
                self.log("An operator emptied the reject bin", "info")
        else:
            self.bin_full_since = None
        old = [p for p in self.floor if self.t - p.since > FLOOR_CLEAN_AFTER]
        if old:
            self.floor = [p for p in self.floor if p not in old]
            self.log("An operator picked up %d product%s from the floor" % (len(old), "" if len(old) == 1 else "s"), "info")

        if EVENT_MODES[self.events_mode]["scale"] is None:
            return
        running = (self.was_connected and c["ROBOT_ENABLE"] and not self.estop_pressed
                   and self.fault is None)
        for kind, spec in RANDOM_EVENTS.items():
            if kind in self.active:
                continue
            if spec.get("stress_only") and self.events_mode != "stress":
                continue
            due = self.next_event.get(kind)
            if due is None:
                self.schedule(kind)
                continue
            if self.t < due:
                continue
            if not running or not self.event_possible(kind, c) or self.active:
                self.next_event[kind] = self.t + 3.0      # try again a little later
                continue
            self.start_event(kind)

    def update_notes(self):
        notes = []
        raw_speed = int(self.holding["ROBOT_SPEED"])
        if raw_speed < 10 or raw_speed > 100:
            notes.append("ROBOT_SPEED = %d is outside 10-100, the robot uses %d %%" % (raw_speed, self.speed()))
        raw_pattern = int(self.holding["PATTERN"])
        if raw_pattern not in GRIDS:
            notes.append("PATTERN = %d is not 4, 6 or 9, the cartons use %d" % (raw_pattern, DEFAULT_PATTERN))
        if self.move and self.not_ready(self.coils):
            notes.append("Move paused: " + self.not_ready(self.coils))
        self.notes = notes

    def step_safety(self):
        if self.estop_pressed and self.fault is None:
            self.fault = "E-stop"
            self.abort("E-stop pressed")

    def abort(self, why):
        if self.move:
            self.log("Robot stopped on the way to %d (%s)" % (self.move["target"], why), "warn")
            self.pos = None
        self.move = None
        self.go_wait = None
        self.done = False

    def step_robot(self, dt, c):
        go = c["ROBOT_GO"]
        reset = c["ROBOT_RESET"]
        target = int(self.holding["ROBOT_TARGET"])

        if reset and not self.prev_reset and self.fault:
            if self.estop_pressed:
                self.log("ROBOT_RESET ignored: release the E-stop first", "warn")
            else:
                self.log("Robot fault reset (%s)" % self.fault, "good")
                self.fault = None
                if go and self.move is None and self.go_wait is None and not self.done:
                    # GO is still on from the interrupted move: carry on with it
                    self.go_wait = GO_SETTLE
                    self.log("ROBOT_GO is still on: the robot continues its move", "info")
        self.prev_reset = reset

        reason = self.not_ready(c)
        if go and not self.prev_go and self.move is None and self.go_wait is None:
            if reason:
                self.log_once("go", "ROBOT_GO ignored: " + reason)
            else:
                self.go_wait = GO_SETTLE              # BUSY at once, read the target in a moment
                self.done = False
        self.prev_go = go

        if self.go_wait is not None:
            if not go:
                self.go_wait = None                   # GO went off before the robot started
            elif reason is None:
                self.go_wait -= dt
                if self.go_wait <= 0:
                    self.go_wait = None
                    self.start_move(target)

        if go and self.move is None and self.go_wait is None and self.done and target != self.pos:
            self.log_once("go_on", "ROBOT_TARGET is %d but ROBOT_GO is still on from the move to %d. "
                          "Turn GO off, then on again" % (target, self.pos), every=10.0)

        if self.move and reason is None:
            self.advance(dt)

        if self.done and not go:
            self.done = False                         # DONE follows GO: off when GO goes off

    def start_move(self, target):
        point, down = self.target_point(target)
        if point is None:
            self.fault = "bad target %d" % target
            self.log("Robot fault: %d is not a position (0, 1, 11-%d or 20)" % (target, 10 + self.pattern()), "fault")
            return
        self.move = {"target": target, "point": point, "down": down, "phase": "lift", "progress": 0.0}
        self.done = False
        self.pos = None
        if self.z <= 0:
            self.begin_travel()

    def begin_travel(self):
        m = self.move
        e1, e2 = inverse(m["point"])
        m.update(phase="travel", progress=0.0, a1=self.j1, a2=self.j2,
                 d1=wrap(e1 - self.j1), d2=e2 - self.j2)
        m["duration"] = TRAVEL_MIN + TRAVEL_PER_PI * max(abs(m["d1"]), abs(m["d2"])) / math.pi

    def advance(self, dt):
        rate = self.speed() / 100.0
        m = self.move
        if m["phase"] == "lift":
            self.z = max(0.0, self.z - dt * rate / LIFT_TIME)
            if self.z <= 0:
                self.begin_travel()
        elif m["phase"] == "travel":
            m["progress"] = min(1.0, m["progress"] + dt * rate / m["duration"])
            s = smooth(m["progress"])
            self.j1 = m["a1"] + m["d1"] * s
            self.j2 = m["a2"] + m["d2"] * s
            if self.held and self.drop_next and m["progress"] >= 0.5:
                self.drop_next = False
                p, self.held = self.held, None
                x, y = self.tool()
                self.to_floor(p, x, y, "fell off the gripper on the way")
            if m["progress"] >= 1.0:
                if m["down"]:
                    m["phase"] = "lower"
                else:
                    self.finish_move()
        elif m["phase"] == "lower":
            self.z = min(1.0, self.z + dt * rate / LIFT_TIME)
            if self.z >= 1.0:
                self.finish_move()

    def finish_move(self):
        self.pos = self.move["target"]
        self.move = None
        self.done = True
        if not self.coils["ROBOT_GO"]:
            self.log_once("go_off", "The move to %d finished, but ROBOT_GO was already off, so ROBOT_DONE "
                          "could not show it. Keep GO on until DONE comes" % self.pos, every=10.0)

    # ------------------------------------------------------------ gripper
    def step_vacuum(self, dt, c):
        x, y = self.tool()
        if c["VACUUM_ON"]:
            if self.held is None:
                found = None
                if self.z >= 0.95:
                    for p in self.products:
                        if math.hypot(p.x - x, p.y - y) < 10:
                            found = p
                if found and self.faults["vacuum_leak"]:
                    self.vac_timer = 0.0
                    self.log_once("leak", "Vacuum leak: the gripper can't hold product %d" % found.number)
                elif found:
                    self.vac_timer += dt
                    if self.vac_timer >= VACUUM_TIME:
                        self.products.remove(found)
                        self.held = found
                        self.vac_timer = 0.0
                else:
                    self.vac_timer = 0.0
        else:
            self.vac_timer = 0.0
            if self.held:
                p, self.held = self.held, None
                self.release(p, x, y)

    def release(self, p, x, y):
        """The vacuum went off: where does the product land?"""
        if abs(x - REJECT[0]) <= BIN_W / 2 and abs(y - REJECT[1]) <= BIN_H / 2:
            if len(self.bin) >= BIN_SIZE:
                self.to_floor(p, x + 30, y + 50, "bounced out of the full reject bin")
                return
            self.bin.append(p)
            if p.good:
                self.score["rejected_good"] += 1
                self.log("Good product %d went into the reject bin" % p.number, "fault")
            else:
                self.score["rejected_bad"] += 1
                self.log("Product %d rejected (%s)" % (p.number, p.what()))
            return
        for carton in self.cartons:
            if abs(x - carton.x) <= CARTON_W / 2 and abs(y - CARTON_Y) <= CARTON_W / 2:
                self.into_carton(p, carton, x, y)
                return
        if math.hypot(x - PICK[0], y - PICK[1]) < 12 and self.product_at_pick() is None:
            p.x, p.y = PICK
            self.products.append(p)
            self.log("Product %d was put back on the infeed" % p.number)
            return
        self.to_floor(p, x, y, "was dropped")

    def into_carton(self, p, carton, x, y):
        cols, rows = GRIDS[carton.pattern]
        col = min(cols - 1, max(0, int(round((x - carton.x) / SLOT_PITCH + (cols - 1) / 2))))
        row = min(rows - 1, max(0, int(round((y - CARTON_Y) / SLOT_PITCH + (rows - 1) / 2))))
        slot = row * cols + col + 1
        here = carton.slots.setdefault(slot, [])
        here.append(p)
        self.packed_times.append(self.t)
        if len(here) > 1:
            self.score["double"] += 1
            self.log("Product %d landed on top of another one (carton %d, slot %d)" % (p.number, carton.number, slot + 10), "fault")
        if p.good:
            self.score["packed_good"] += 1
        else:
            self.score["packed_bad"] += 1
            self.log("Bad product %d (%s) was packed into carton %d" % (p.number, p.what(), carton.number), "fault")

    def to_floor(self, p, x, y, why):
        self.floor.append(Product(p.number, x, p.defect))
        self.floor[-1].y = y
        self.floor[-1].since = self.t
        self.score["dropped"] += 1
        self.log("Product %d %s" % (p.number, why), "fault")

    # ------------------------------------------------------------ infeed and camera
    def step_infeed(self, dt, c):
        if c["INFEED_RUN"] and not self.estop_pressed and not self.faults["infeed_jam"]:
            dx = INFEED_SPEED * dt
            self.offsets["infeed"] = (self.offsets["infeed"] + dx) % 24
            limit = PICK[0]
            for p in sorted(self.products, key=lambda q: -q.x):
                p.x = max(p.x, min(p.x + dx, limit))
                limit = p.x - PRODUCT_D - 2
        self.spawn_timer += dt
        if self.spawn_timer >= self.spawn_every and not self.faults["starve"]:
            if all(p.x > SPAWN_X + PRODUCT_D + 2 for p in self.products):
                self.add_product(SPAWN_X)
                self.spawn_timer = 0.0

    def step_camera(self, dt, c):
        trigger = c["CAMERA_TRIGGER"]
        if trigger and not self.prev_trigger:
            self.scan_timer = SCAN_TIME
            self.cam_done = self.cam_pass = False
            self.cam_code = 0
        if not trigger:
            self.scan_timer = 0.0
            self.cam_done = self.cam_pass = False
            self.cam_code = 0
        elif self.scan_timer > 0:
            if self.faults["camera_dead"]:
                self.log_once("camera", "The camera does not answer")
            else:
                self.scan_timer -= dt
                if self.scan_timer <= 0:
                    self.scan_timer = 0.0
                    self.finish_scan()
        self.prev_trigger = trigger

    def finish_scan(self):
        p = self.product_at_pick()
        self.cam_done = True
        if p is None:
            self.cam_pass = False
            self.cam_code = NO_PRODUCT
            self.last_scan = "No product at the pick stop"
            return
        self.cam_pass = p.good
        self.cam_code = p.defect or 0
        self.last_scan = "Product %d: %s" % (p.number, "PASS" if p.good else "FAIL, " + p.what())

    # ------------------------------------------------------------ cartons
    def step_cartons(self, dt, c):
        run = c["OUTFEED_RUN"]
        if run and not self.prev_outfeed:
            at_pack = self.carton_at_pack()
            if at_pack:
                at_pack.released = True
        self.prev_outfeed = run

        stopped = self.estop_pressed
        dx = OUTFEED_SPEED * dt
        belt = run and not self.faults["outfeed_jam"]
        if belt and not stopped:
            self.offsets["outfeed"] = (self.offsets["outfeed"] + dx) % 24
        if not stopped:
            self.offsets["takeaway"] = (self.offsets["takeaway"] + dx) % 24

        limit = float("inf")
        for carton in sorted(self.cartons, key=lambda k: -k.x):
            if not carton.slots:
                carton.pattern = self.pattern()       # an empty carton follows PATTERN
            if stopped:
                step = 0.0
            elif carton.x >= TAKEAWAY_X:
                step = dx                             # the take-away belt always runs
            elif belt:
                step = dx
            else:
                step = 0.0
            new = carton.x + step
            if not carton.released and carton.x <= PACK_X + 0.001 and new >= PACK_X:
                new = PACK_X                          # caught by the pack-station stop
            new = min(new, limit - CARTON_W - CARTON_GAP)
            carton.x = max(carton.x, new)
            limit = carton.x

        for carton in [k for k in self.cartons if k.x >= CARTON_EXIT]:
            self.cartons.remove(carton)
            self.ship(carton)

        if not self.faults["no_cartons"]:
            if all(k.x >= CARTON_ENTRY + CARTON_W + CARTON_GAP - 0.01 for k in self.cartons):
                self.add_carton(CARTON_ENTRY)

        self.carton_pulse = max(0.0, self.carton_pulse - dt)
        while self.packed_times and self.t - self.packed_times[0] > 60:
            self.packed_times.popleft()

    def ship(self, carton):
        self.carton_pulse = PULSE_TIME
        self.score["shipped"] += 1
        n = carton.count()
        if n < carton.pattern:
            self.score["short"] += 1
            self.log("Carton %d left with only %d of %d products" % (carton.number, n, carton.pattern), "fault")
        else:
            self.log("Carton %d shipped (%d/%d)" % (carton.number, n, carton.pattern), "good")

    # ------------------------------------------------------------ what the PLC reads
    def sensors(self, c):
        x, y = self.tool()
        angle = int(round(math.degrees(math.atan2(BASE[1] - y, x - BASE[0])))) % 360
        reach = int(round(math.hypot(x - BASE[0], y - BASE[1])))
        inputs = {
            "ESTOP_OK": not self.estop_pressed,
            "DOOR_CLOSED": self.door_closed,
            "PB_START": self.buttons["PB_START"],
            "PB_STOP": self.buttons["PB_STOP"],
            "PB_RESET": self.buttons["PB_RESET"],
            "SEL_REMOTE": self.remote,
            "PRODUCT_AT_PICK": self.product_at_pick() is not None,
            "CAMERA_DONE": self.cam_done,
            "CAMERA_PASS": self.cam_pass,
            "ROBOT_READY": self.not_ready(c) is None,
            "ROBOT_BUSY": self.move is not None or self.go_wait is not None,
            "ROBOT_DONE": self.done,
            "ROBOT_AT_HOME": self.move is None and self.pos == 0,
            "ROBOT_FAULT": self.fault is not None,
            "VACUUM_OK": self.held is not None and c["VACUUM_ON"],
            "CARTON_AT_PACK": self.carton_at_pack() is not None,
            "CARTON_OUT": self.carton_pulse > 0,
        }
        registers = {
            "ROBOT_POS": MOVING if self.pos is None else self.pos,
            "ROBOT_ANGLE": angle,
            "ROBOT_REACH": reach,
            "CAMERA_CODE": self.cam_code,
        }
        return inputs, registers

    # ------------------------------------------------------------ web page
    def command(self, data):
        cmd = data.get("cmd")
        value = data.get("value")
        with self.lock:
            if cmd == "button" and data.get("name") in self.buttons:
                self.buttons[data["name"]] = bool(value)
            elif cmd == "estop":
                self.estop_pressed = bool(value)
                self.log("E-stop pressed" if self.estop_pressed else "E-stop released",
                         "fault" if self.estop_pressed else "info")
            elif cmd == "door":
                self.door_closed = bool(value)
                self.log("Guard door closed" if self.door_closed else "Guard door opened",
                         "info" if self.door_closed else "warn")
            elif cmd == "remote":
                self.remote = bool(value)
                self.log("Key switch: %s" % ("REMOTE (SCADA)" if self.remote else "LOCAL (panel)"))
            elif cmd == "defect_rate":
                self.defect_rate = float(min(50, max(0, float(value))))
            elif cmd == "next_defect" and int(value) in DEFECTS:
                self.next_defect = int(value)
                self.log("The next product will arrive with: %s" % DEFECTS[self.next_defect])
            elif cmd == "fault" and data.get("name") in self.faults:
                name = data["name"]
                self.faults[name] = bool(value)
                names = {"vacuum_leak": "Vacuum leak", "camera_dead": "Camera dead",
                         "no_cartons": "Out of cartons", "starve": "No products from upstream",
                         "infeed_jam": "Infeed jam", "outfeed_jam": "Carton conveyor jam"}
                self.log("%s %s" % (names[name], "ON" if value else "cleared"), "event")
            elif cmd == "servo_fault":
                self.fault = "servo fault, axis 2"
                self.abort("servo fault")
                self.log("Robot fault: servo fault on axis 2", "fault")
            elif cmd == "drop_next":
                self.drop_next = True
                self.log("The next product the robot carries will fall off")
            elif cmd == "empty_bin":
                self.bin.clear()
                self.log("Reject bin emptied")
            elif cmd == "clear_floor":
                self.floor.clear()
                self.log("Floor cleaned")
            elif cmd == "take_from_gripper":
                if self.held:
                    self.log("Product %d taken off the gripper by hand" % self.held.number)
                    self.held = None
            elif cmd == "remove_at_pick":
                p = self.product_at_pick()
                if p:
                    self.products.remove(p)
                    self.log("Product %d taken off the infeed by hand" % p.number)
            elif cmd == "events_mode" and value in EVENT_MODES:
                self.set_events_mode(value)
            elif cmd == "reset_score":
                for k in self.score:
                    self.score[k] = 0
                self.packed_times.clear()
            else:
                return False
        return True

    def snapshot(self):
        with self.station.lock:
            inputs = dict(self.station.inputs)
            registers = dict(self.station.registers)
        with self.lock:
            elbow, tool = forward(self.j1, self.j2)
            moving_to = None
            if self.move:
                moving_to = {"target": self.move["target"], "x": round(self.move["point"][0], 1),
                             "y": round(self.move["point"][1], 1)}
            pack = self.carton_at_pack()
            return {
                "t": round(self.t, 2),
                "link": {"up": self.station.connected(), "requests": self.station.requests,
                         "port": self.station.port},
                "inputs": inputs,
                "registers": registers,
                "coils": dict(self.coils),
                "holding": dict(self.holding),
                "robot": {
                    "elbow": [round(elbow[0], 1), round(elbow[1], 1)],
                    "tool": [round(tool[0], 1), round(tool[1], 1)],
                    "z": round(self.z, 3), "moving_to": moving_to,
                    "phase": self.move["phase"] if self.move else None,
                    "pos": self.pos, "done": self.done, "fault": self.fault,
                    "held": self.held.to_json() if self.held else None,
                    "vacuum": round(self.vac_timer / VACUUM_TIME, 2),
                    "speed": self.speed(), "pattern": self.pattern(),
                    "not_ready": self.not_ready(self.coils),
                },
                "products": [p.to_json() for p in self.products],
                "floor": [p.to_json() for p in self.floor],
                "bin": [p.defect for p in self.bin],
                "cartons": [k.to_json() for k in self.cartons],
                "pack": pack.to_json() if pack else None,
                "stop_up": not (pack and pack.released),
                "camera": {"scanning": self.scan_timer > 0 and self.prev_trigger,
                           "last": self.last_scan, "dead": self.faults["camera_dead"]},
                "offsets": {k: round(v, 1) for k, v in self.offsets.items()},
                "panel": {"buttons": dict(self.buttons), "estop": self.estop_pressed,
                          "door_closed": self.door_closed, "remote": self.remote},
                "faults": dict(self.faults),
                "events_mode": self.events_mode,
                "active_events": [{"kind": k, "text": RANDOM_EVENTS[k]["start"],
                                   "left": None if end is None else round(max(0.0, end - self.t), 1)}
                                  for k, end in self.active.items()],
                "drop_next": self.drop_next,
                "next_defect": self.next_defect,
                "defect_rate": self.defect_rate,
                "score": dict(self.score),
                "per_minute": len(self.packed_times),
                "notes": list(self.notes),
                "events": list(self.events),
            }


def run_physics(cell):
    last = time.monotonic()
    while True:
        time.sleep(0.02)
        now = time.monotonic()
        cell.step(min(0.1, now - last))
        last = now


def make_web_handler(cell):
    class WebHandler(BaseHTTPRequestHandler):
        def send_body(self, code, body, content_type):
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                with open(PAGE, "rb") as f:
                    self.send_body(200, f.read(), "text/html; charset=utf-8")
            elif self.path == "/classic" and os.path.exists(CLASSIC):
                with open(CLASSIC, "rb") as f:
                    self.send_body(200, f.read(), "text/html; charset=utf-8")
            elif self.path == "/state":
                self.send_body(200, json.dumps(cell.snapshot()).encode(), "application/json")
            elif self.path == "/geometry":
                self.send_body(200, json.dumps(GEOMETRY).encode(), "application/json")
            else:
                self.send_body(404, b"not found", "text/plain")

        def do_POST(self):
            if self.path != "/cmd":
                self.send_body(404, b"not found", "text/plain")
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                data = json.loads(self.rfile.read(size) or b"{}")
                ok = cell.command(data)
            except (ValueError, TypeError):
                ok = False
            self.send_body(200 if ok else 400, json.dumps({"ok": ok}).encode(), "application/json")

        def log_message(self, *args):
            pass

    return WebHandler


def main():
    parser = argparse.ArgumentParser(description="Robot packing cell simulator")
    parser.add_argument("--web-port", type=int, default=8091, help="web page port (default 8091)")
    parser.add_argument("--port", type=int, help="use another Modbus port (testing)")
    parser.add_argument("--defect-rate", type=float, default=None, help="%% of products with a defect (default: set by --events)")
    parser.add_argument("--events", choices=sorted(EVENT_MODES), default="realistic",
                        help="random factory events: off, realistic (default) or stress")
    parser.add_argument("--spawn", type=float, default=SPAWN_EVERY, help="seconds between new products")
    parser.add_argument("--no-browser", action="store_true", help="don't open the browser")
    args = parser.parse_args()

    station = robot_io.make_station()
    if args.port:
        station.port = args.port
    check_names(station)

    try:
        station.start()
    except OSError:
        print("Port %d is already in use." % station.port)
        print("Close the other program that uses it (robot_io.py, or another simulator window) and try again.")
        return 1

    cell = Cell(station, args.defect_rate, args.spawn, args.events)
    threading.Thread(target=run_physics, args=(cell,), daemon=True).start()

    try:
        web = ThreadingHTTPServer(("127.0.0.1", args.web_port), make_web_handler(cell))
    except OSError:
        print("Web port %d is already in use. Is the simulator already running?" % args.web_port)
        return 1
    web.daemon_threads = True
    url = "http://localhost:%d" % args.web_port
    print("Robot cell picture:", url)
    print("Press Ctrl + C to stop.")
    if not args.no_browser:
        threading.Timer(1.0, webbrowser.open, args=(url,)).start()
    try:
        web.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
