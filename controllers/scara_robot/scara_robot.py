"""Epson SCARA T6-602S. The same program runs on SCARA A and SCARA B.

PLC -> robot (JSON):
  {"cmd": "pick_place", "id": n, "part": DEF, "pick": [x, y, z], "place": [x, y, z]}
  {"cmd": "hold"} / {"cmd": "release_hold"} / {"cmd": "speed", "value": 0..1}
Robot -> PLC:
  {"evt": "ready", "side", "fk_err"}
  {"evt": "picked", "id"}     vacuum on, still in the shared column
  {"evt": "clear", "id"}      TCP is at the via point, on its own side; the column is free
  {"evt": "placed", "id", "part"}
  {"evt": "done", "id"}
  {"evt": "error", "msg"}
The cup is kinematic: with the vacuum on, the part follows the TCP. The controller is a supervisor.
"""
import json
import math
import os
import sys

from controller import Supervisor

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "lib"))
import layout as L  # noqa: E402
from geom import ik_base  # noqa: E402
from motion import Carrier, Executor  # noqa: E402
from scara_kin import fk  # noqa: E402

JOINT_SPEED = 5.0
DOWN_SPEED = 0.45


class Scara(Supervisor):
    def __init__(self):
        super().__init__()
        self.dt = int(self.getBasicTimeStep())
        self.side = "A" if self.getName().strip().endswith("A") else "B"
        names = ["base_arm", "arm", "shaft_linear", "shaft_rotation"]
        self.motors = [self.getDevice(n + "_motor") for n in names]
        self.sensors = [self.getDevice(n + "_position") for n in names]
        for s in self.sensors:
            s.enable(self.dt)
        self.rx = self.getDevice("scara_rx")
        self.rx.enable(self.dt)
        self.tx = self.getDevice("scara_tx")
        self.led = self.getDevice("epson_led")
        self.carrier = Carrier(self, "SCARA_%s_TCP" % self.side)
        self.base = (0.0, 0.0, 0.0, 0.0)
        self.job = None

    def read_base(self):
        bp = self.getSelf().getPosition()
        R = self.getSelf().getOrientation()
        self.base = (bp[0], bp[1], bp[2], math.atan2(R[3], R[0]))

    def send(self, **m):
        m.setdefault("side", self.side)
        self.tx.send(json.dumps(m))

    def to_base(self, p):
        bx, by, bz, yaw = self.base
        c, s = math.cos(-yaw), math.sin(-yaw)
        u, v = p[0] - bx, p[1] - by
        return (c * u - s * v, s * u + c * v, p[2] - bz, 0.0)

    def _ik(self, t, q):
        return ik_base(self.base, self.side, t, q)

    def plan(self, m):
        pick, place = self.to_base(m["pick"]), self.to_base(m["place"])
        via = self.to_base(L.via_world(self.side))
        home = self.to_base(L.home_world(self.side))
        up = lambda p: (p[0], p[1], L.CLEAR_Z - self.base[2], 0.0)
        jid, part = m["id"], m["part"]

        def grip():
            self.carrier.grasp([part])
            self.led.set(1)

        def release():
            def settle(n):
                p = n.getPosition()
                n.getField("translation").setSFVec3f([p[0], p[1], m["place"][2] - L.PART_H / 2])
                n.getField("rotation").setSFRotation([0, 0, 1, 0])
            names = self.carrier.release(settle)
            self.led.set(0)
            self.send(evt="placed", id=jid, part=names[0] if names else part)

        def done():
            self.job = None
            self.send(evt="done", id=jid)

        self.job = jid
        self.ex.load([
            ("j", up(pick)), ("l", pick, DOWN_SPEED), ("a", grip, 0.12),
            ("l", up(pick), DOWN_SPEED), ("a", lambda: self.send(evt="picked", id=jid), 0.0),
            ("j", up(via)), ("a", lambda: self.send(evt="clear", id=jid), 0.0),
            ("j", up(place)), ("l", place, DOWN_SPEED), ("a", release, 0.10),
            ("l", up(place), DOWN_SPEED),
            ("j", up(home)), ("a", done, 0.0),
        ], self.ex.tcp)

    def poll(self):
        while self.rx.getQueueLength() > 0:
            m = json.loads(self.rx.getString())
            self.rx.nextPacket()
            c = m.get("cmd")
            if c == "hold":
                self.ex.hold = True
            elif c == "release_hold":
                self.ex.hold = False
            elif c == "speed":
                self.ex.override = max(0.05, min(1.0, float(m["value"])))
            elif c == "pick_place" and self.job is None and m.get("side", self.side) == self.side:
                self.plan(m)

    def fk_error(self):
        for _ in range(20):
            self.step(self.dt)
        q = [s.getValue() for s in self.sensors]
        x, y, z, _ = fk(q, L.SCARA_TOOL)
        w = self._world(x, y, z)
        return math.dist(w, self.carrier.tcp.getPosition()), q

    def _world(self, x, y, z):
        bx, by, bz, yaw = self.base
        c, s = math.cos(yaw), math.sin(yaw)
        return (bx + c * x - s * y, by + s * x + c * y, bz + z)

    def run(self):
        self.step(self.dt)
        self.read_base()
        q0 = [s.getValue() for s in self.sensors]
        self.ex = Executor(self._ik, q0, JOINT_SPEED)
        home = self.to_base(L.home_world(self.side))
        self.ex.tcp = home
        self.ex.load([("j", home)], home)
        ready = False
        while self.step(self.dt) != -1:
            self.poll()
            q = self.ex.update(self.dt / 1000.0)
            if self.ex.error:
                self.send(evt="error", msg=self.ex.error)
                self.ex.error = None
                self.job = None
            q[2] = min(0.0, q[2])
            for mtr, v in zip(self.motors, q):
                mtr.setPosition(v)
            self.carrier.update()
            if not ready and not self.ex.busy():
                ready = True
                err, _ = self.fk_error()
                self.send(evt="ready", fk_err=err)


Scara().run()
