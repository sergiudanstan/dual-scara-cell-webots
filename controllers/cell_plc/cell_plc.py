"""Cell PLC (supervisor) for two Epson SCARA T6 robots.

CONTROL: orders come from the sensors (B1 part at the stop, B5 accumulation, B2A/B2B tray present),
from the robots' events, and from the area scanner (operator position, same idea as a tag).
PROCESS: the belt, the tray exit, the door and the stack light are moved here. Parts and trays have
no physics. The robots move on their own motors.

Keys, after a click in the 3D view, or letters written into this robot's customData:
  E  emergency stop (press / release)     D  open / close the guard door     R  reset
  P  operator walks through the scanner fields and back
  S  end-of-cycle stop / restart

controllerArgs "--smoke=<seconds>" runs a scripted self-test and quits 0 (pass) or 1 (fail).
"""
import json
import math
import os
import sys
import time

from controller import Keyboard, Supervisor

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "lib"))
import layout as L  # noqa: E402
from dispatch import choose  # noqa: E402

SIDES = ("A", "B")
LEAVE_T, OUT_DWELL, LOAD_T = 2.2, 1.4, 1.0
LAMP = {"RED": "0.9 0.1 0.08", "AMBER": "1 0.6 0.05", "GREEN": "0.1 0.85 0.2"}


def arm_state():
    return {"ready": False, "busy": False, "holding": False, "tray": True,
            "pockets_left": L.PARTS_PER_TRAY, "placed": 0, "id": 0, "part": None, "pocket": 0}


class PLC(Supervisor):
    def __init__(self):
        super().__init__()
        self.dt = int(self.getBasicTimeStep())
        self.sens = {}
        for n in ("B1", "B5", "B2A", "B2B"):
            d = self.getDevice(n)
            d.enable(self.dt)
            self.sens[n] = d
        self.tx = {s: self.getDevice("to_%s" % s.lower()) for s in SIDES}
        self.rx = {s: self.getDevice("from_%s" % s.lower()) for s in SIDES}
        for r in self.rx.values():
            r.enable(self.dt)
        self.tx_op = self.getDevice("to_operator")
        self.kb = self.getKeyboard()
        self.kb.enable(self.dt)
        self.me = self.getSelf()
        names = ["PART_%d" % i for i in range(L.N_PARTS)]
        names += [L.tray_def(s, i) for s in SIDES for i in range(L.MAG_COUNT + 1)]
        names += ["DOOR", "OPERATOR"]
        self.node = {d: self.getFromDef(d) for d in names}
        self.lamp = {k: self.getFromDef("LAMP_" + k).getField("emissiveColor") for k in LAMP}
        self.pool = ["PART_%d" % i for i in range(L.N_PARTS)]
        self.belt = []
        self.magazine = {s: [L.tray_def(s, i) for i in range(L.MAG_COUNT)] for s in SIDES}
        self.station = {s: {"tray": L.tray_def(s, L.MAG_COUNT), "parts": []} for s in SIDES}
        self.transfer = {s: {"state": "idle", "t": 0.0, "tray": None, "parts": []} for s in SIDES}
        self.last_feed = -10.0
        self.door_open, self.door_angle = False, 0.0
        self.arm = {s: arm_state() for s in SIDES}
        self.zone = None
        self.estop = False
        self.fault = None
        self.cycle_stop = False
        self.scan_stop, self.scan_warn, self.scan_clear_t = False, False, 0.0
        self.held = False
        self.override = None
        self.events = []
        self.stats = {"parts": 0, "trays": 0, "errors": []}
        self.fk_check = {}
        self.smoke = None
        for a in sys.argv[1:]:
            if a.startswith("--smoke="):
                self.smoke = float(a.split("=", 1)[1])
        self.op = self.node["OPERATOR"]

    def log(self, msg):
        t = self.getTime()
        self.events.append("%6.1f  %s" % (t, msg))
        self.events = self.events[-8:]
        print("[PLC %6.1f] %s" % (t, msg))

    def put(self, d, p, yaw=0.0):
        n = self.node[d]
        n.getField("translation").setSFVec3f(list(p))
        n.getField("rotation").setSFRotation([0, 0, 1, yaw])

    def sees(self, name, limit=0.12):
        return self.sens[name].getValue() < limit

    def send(self, side, **m):
        self.tx[side].send(json.dumps(m))

    def gripped(self):
        return {self.arm[s]["part"] for s in SIDES if self.arm[s]["holding"] and self.arm[s]["part"]}

    def cell_ok(self):
        return not self.estop and self.fault is None and not self.door_open

    # -------------------------------------------------------------- belt and trays
    def run_belt(self, dt):
        run = self.cell_ok() and not self.cycle_stop
        held = self.gripped()
        if run:
            limit = L.PICK[1]
            for item in self.belt:
                if item[0] not in held:
                    item[1] = min(item[1] + L.BELT_SPEED * dt, limit)
                limit = item[1] - L.PART_GAP
            entry_free = all(y > L.SPAWN_Y + L.PART_GAP for _, y in self.belt)
            if (self.getTime() - self.last_feed >= L.FEED_PERIOD and entry_free and not self.sees("B5")
                    and self.pool):
                d = self.pool.pop(0)
                self.belt.append([d, L.SPAWN_Y])
                self.last_feed = self.getTime()
        z = L.CONV_Z + L.PART_H / 2
        for d, y in self.belt:
            if d not in held:
                self.put(d, (L.CONV_X, y, z))

    def part_at_stop(self):
        if not self.belt:
            return None
        d, y = self.belt[0]
        if y < L.PICK[1] - 0.004 or d in self.gripped():
            return None
        for s in SIDES:
            if self.arm[s]["part"] == d and self.arm[s]["busy"]:
                return None
        return d

    def run_transfer(self, side, dt):
        tr = self.transfer[side]
        if tr["state"] == "idle":
            return
        if tr["state"] in ("leaving", "loading") and (not self.cell_ok() or self.scan_stop):
            return
        tr["t"] += dt
        sx, sy = L.STATION[side]
        ox, oy = L.OUT[side]
        if tr["state"] == "leaving":
            u = min(1.0, tr["t"] / LEAVE_T)
            self._slide(side, tr, sx + (ox - sx) * u, sy + (oy - sy) * u, L.TABLE_Z)
            if u >= 1.0:
                tr["state"], tr["t"] = "out", 0.0
                self.stats["trays"] += 1
                self.log("Tray %s is out" % side)
        elif tr["state"] == "out" and tr["t"] >= OUT_DWELL:
            self._ship(side, tr)
        elif tr["state"] == "loading":
            u = min(1.0, tr["t"] / LOAD_T)
            mx, my = L.MAG[side]
            self._slide(side, tr, mx + (sx - mx) * u, my + (sy - my) * u, L.TABLE_Z)
            if u >= 1.0:
                self.station[side] = {"tray": tr["tray"], "parts": []}
                self.arm[side]["tray"] = True
                self.arm[side]["pockets_left"] = L.PARTS_PER_TRAY
                self.arm[side]["pocket"] = 0
                tr.update(state="idle", t=0.0, tray=None, parts=[])
                self.log("Empty tray at station %s" % side)

    def _slide(self, side, tr, x, y, z):
        self.put(tr["tray"], (x, y, z))
        for j, pd in enumerate(tr["parts"]):
            dx, dy = L.POCKETS[j]
            self.put(pd, (x + dx, y + dy, z + L.TRAY_FLOOR + L.PART_H / 2))

    def _ship(self, side, tr):
        for pd in tr["parts"]:
            i = int(pd.split("_")[1])
            col, row = i % 12, i // 12
            self.put(pd, (-3.0 + 0.05 * col, -3.6 - 0.05 * row, -0.4))
            self.pool.append(pd)
        # restack: the shipped tray goes to the bottom, the others move up one slot
        stack = [tr["tray"]] + self.magazine[side]
        self.magazine[side] = stack[:-1]
        for i, name in enumerate(self.magazine[side]):
            self.put(name, (L.MAG[side][0], L.MAG[side][1], L.TABLE_Z + i * L.TRAY_H))
        nxt = stack[-1]
        tr.update(state="loading", t=0.0, tray=nxt, parts=[])
        self.log("Tray %s shipped, next tray loading" % side)

    def start_transfer(self, side):
        st = self.station[side]
        self.transfer[side] = {"state": "leaving", "t": 0.0, "tray": st["tray"], "parts": list(st["parts"])}
        self.station[side] = {"tray": None, "parts": []}
        self.arm[side]["tray"] = False
        self.log("Tray %s full, leaving" % side)

    # -------------------------------------------------------------- safety
    def operator_distance(self):
        p = self.op.getPosition()
        if p[1] < L.SCANNER[1] - 0.02:
            return float("inf")
        return math.hypot(p[0] - L.SCANNER[0], p[1] - L.SCANNER[1])

    def safety(self):
        if self.estop and self.fault is None:
            self.fault = "EMERGENCY STOP"
            self.log("Emergency stop pressed")
        if self.door_open and self.fault is None:
            self.fault = "GUARD DOOR OPEN"
            self.log("Guard door opened: cell stopped")
        d = self.operator_distance()
        inside_stop, inside_warn = d < L.STOP_R, d < L.WARN_R
        if inside_stop:
            if not self.scan_stop:
                self.log("Scanner protective field: robots hold (cat. 2)")
            self.scan_stop, self.scan_clear_t = True, self.getTime()
        elif self.scan_stop and self.getTime() - self.scan_clear_t > L.SCAN_CLEAR_T:
            self.scan_stop = False
            self.log("Protective field clear: automatic restart")
        if inside_warn != self.scan_warn:
            self.log("Scanner warning field %s" % ("entered: reduced speed" if inside_warn else "clear"))
        self.scan_warn = inside_warn
        hold = not self.cell_ok() or self.scan_stop
        if hold != self.held:
            self.held = hold
            for s in SIDES:
                self.send(s, cmd="hold" if hold else "release_hold")
        ov = L.SLOW if self.scan_warn and not hold else 1.0
        if ov != self.override:
            self.override = ov
            for s in SIDES:
                self.send(s, cmd="speed", value=ov)
        blink = int(self.getTime() * 2) % 2 == 0
        state = {"RED": self.fault is not None and blink,
                 "AMBER": self.fault is None and (self.scan_warn or self.scan_stop or self.cycle_stop),
                 "GREEN": self.fault is None and not self.scan_stop}
        for k, on in state.items():
            self.lamp[k].setSFColor([float(v) for v in LAMP[k].split()] if on else [0, 0, 0])

    def reset(self):
        if self.fault is None:
            return
        if self.estop:
            self.log("Reset refused: e-stop still pressed")
        elif self.door_open:
            self.log("Reset refused: door still open")
        else:
            self.log("Reset: %s cleared" % self.fault)
            self.fault = None

    def command(self, c):
        c = c.upper()
        if c == "E":
            self.estop = not self.estop
            if not self.estop:
                self.log("E-stop released (reset required)")
        elif c == "D":
            self.door_open = not self.door_open
            if not self.door_open:
                self.log("Guard door closed (reset required)")
        elif c == "R":
            self.reset()
        elif c == "P":
            self.tx_op.send("walk")
            self.log("Operator walks through the scanner fields")
        elif c == "S":
            self.cycle_stop = not self.cycle_stop
            self.log("End-of-cycle stop" if self.cycle_stop else "Cycle restarted")

    def run_door(self, dt):
        tgt = 1.4 if self.door_open else 0.0
        step = 1.2 * dt
        self.door_angle += max(-step, min(step, tgt - self.door_angle))
        self.node["DOOR"].getField("rotation").setSFRotation([0, 0, 1, self.door_angle])

    # -------------------------------------------------------------- robots
    def on_robot(self, side, m):
        a = self.arm[side]
        evt = m.get("evt")
        if evt == "ready":
            a["ready"] = True
            self.fk_check[side] = float(m.get("fk_err", 1.0))
            self.log("SCARA %s ready, FK error %.2f mm" % (side, self.fk_check[side] * 1000))
        elif evt == "picked":
            a["holding"] = True
        elif evt == "clear":
            if self.zone == side:
                self.zone = None
        elif evt == "placed":
            a["holding"] = False
            part = m.get("part", a["part"])
            self.belt = [it for it in self.belt if it[0] != part]
            if self.station[side]["tray"]:
                self.station[side]["parts"].append(part)
            a["pockets_left"] = max(0, L.PARTS_PER_TRAY - len(self.station[side]["parts"]))
            a["placed"] += 1
            self.stats["parts"] += 1
            a["part"] = None
        elif evt == "done":
            a["busy"] = False
            if self.zone == side:
                self.zone = None
        elif evt == "error":
            a["busy"] = False
            a["holding"] = False
            if self.zone == side:
                self.zone = None
            self.stats["errors"].append("%s: %s" % (side, m.get("msg")))
            self.log("SCARA %s error: %s" % (side, m.get("msg")))

    def dispatch(self):
        if self.cell_ok() and not self.cycle_stop and not self.scan_stop:
            part = self.part_at_stop()
            who = choose(self.arm, part is not None and self.sees("B1"), self.zone)
            if who is not None and self.sees("B2" + who):
                a = self.arm[who]
                place = L.pocket_world(who, a["pocket"])
                a["pocket"] += 1
                a["id"] += 1
                a["busy"] = True
                a["part"] = part
                self.zone = who
                self.send(who, cmd="pick_place", id=a["id"], part=part,
                          pick=list(L.PICK), place=list(place))
                self.log("SCARA %s pick %d -> pocket %d" % (who, a["id"], a["pocket"]))
        for s in SIDES:
            b = self.arm[s]
            if (b["pockets_left"] == 0 and not b["busy"] and not b["holding"]
                    and self.transfer[s]["state"] == "idle" and self.station[s]["tray"]):
                self.start_transfer(s)

    def overlay(self):
        head = "DEFECT: %s (R = reset)" % self.fault if self.fault else (
            "OPRIRE DE PROTECTIE" if self.scan_stop else "VITEZA REDUSA" if self.scan_warn else "AUTOMAT")
        self.setLabel(0, "Doua SCARA Epson T6   |   %s" % head, 0.01, 0.01, 0.07,
                      0xFF5050 if self.fault else 0xFFD040 if (self.scan_stop or self.scan_warn) else 0xFFFFFF, 0)
        self.setLabel(1, "piese %d    tavi %d    |    %s" % (
            self.stats["parts"], self.stats["trays"], self.events[-1][8:] if self.events else ""),
            0.01, 0.05, 0.06, 0xFFE070, 0)
        self.setLabel(2, "taste: E stop urgenta   D usa   R reset   P operator   S oprire de ciclu",
                      0.01, 0.95, 0.05, 0xCCCCCC, 0)

    def smoke_script(self, t):
        plan = getattr(self, "_plan", None)
        if plan is None:
            T = self.smoke
            self._plan = plan = [(T * 0.62, "E"), (T * 0.62 + 2.0, "R"), (T * 0.62 + 3.0, "E"),
                                 (T * 0.62 + 4.0, "R"), (T * 0.72, "P")]
        while plan and t >= plan[0][0]:
            self.command(plan.pop(0)[1])

    def smoke_verdict(self):
        ok, why = True, []
        if self.stats["parts"] < 2:
            ok = False
            why.append("parts=%d < 2" % self.stats["parts"])
        if self.arm["A"]["placed"] < 1 or self.arm["B"]["placed"] < 1:
            ok = False
            why.append("A placed %d, B placed %d" % (self.arm["A"]["placed"], self.arm["B"]["placed"]))
        if self.stats["errors"]:
            ok = False
            why += self.stats["errors"]
        for name, err in self.fk_check.items():
            print("[SMOKE] FK %s vs simulator: %.2f mm" % (name, err * 1000))
            if err > 0.002:
                ok = False
                why.append("FK mismatch %s %.1f mm" % (name, err * 1000))
        if len(self.fk_check) < 2:
            ok = False
            why.append("a robot never became ready")
        for flag, label in [(getattr(self, "_saw_estop_hold", False), "e-stop did not hold the robots"),
                            (getattr(self, "_saw_refused", False), "reset with e-stop pressed was not refused"),
                            (getattr(self, "_saw_scan_stop", False), "scanner stop never triggered"),
                            (getattr(self, "_saw_scan_resume", False), "no automatic restart after the scanner")]:
            if not flag:
                ok = False
                why.append(label)
        print("[SMOKE] stats %s" % json.dumps(self.stats))
        print("[SMOKE] %s %s" % ("PASS" if ok else "FAIL", "; ".join(why)))
        return ok

    def run(self):
        dt = self.dt / 1000.0
        wall0 = time.time()
        self.log("Cell started")
        while self.step(self.dt) != -1:
            t = self.getTime()
            k = self.kb.getKey()
            while k > 0:
                if 0 < k < 128:
                    self.command(chr(k))
                k = self.kb.getKey()
            cd = self.me.getField("customData")
            if cd.getSFString():
                for ch in cd.getSFString():
                    self.command(ch)
                cd.setSFString("")
            if self.smoke:
                self.smoke_script(t)
            for s in SIDES:
                while self.rx[s].getQueueLength() > 0:
                    self.on_robot(s, json.loads(self.rx[s].getString()))
                    self.rx[s].nextPacket()
            self.safety()
            if self.smoke:
                if self.estop and self.held:
                    self._saw_estop_hold = True
                if self.estop and self.events and "refused" in self.events[-1]:
                    self._saw_refused = True
                if self.scan_stop:
                    self._saw_scan_stop = True
                if getattr(self, "_saw_scan_stop", False) and not self.scan_stop and not self.held:
                    self._saw_scan_resume = True
            self.dispatch()
            self.run_belt(dt)
            for s in SIDES:
                self.run_transfer(s, dt)
            self.run_door(dt)
            self.overlay()
            if self.smoke and int(t / dt) % int(5 / dt) == 0:
                print("[SMOKE] t=%.0f s  wall=%.0f s  parts %d  zone %s  B1 %d B5 %d B2A %d B2B %d  A %s  B %s" % (
                    t, time.time() - wall0, self.stats["parts"], self.zone,
                    *(int(self.sees(n)) for n in ("B1", "B5", "B2A", "B2B")),
                    {k: self.arm["A"][k] for k in ("ready", "busy", "placed")},
                    {k: self.arm["B"][k] for k in ("ready", "busy", "placed")}))
            if self.smoke and t >= self.smoke:
                ok = self.smoke_verdict()
                for _ in range(8):
                    self.step(self.dt)
                self.simulationQuit(0 if ok else 1)
                self.step(self.dt)
                break


PLC().run()
