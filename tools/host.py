"""Host CLI for the conveyor actuator (REQ-H02, REQ-H03).

python tools/host.py --port COM5 ping
python tools/host.py --port COM5 shell                      # interactive, heartbeat on
python tools/host.py --port COM5 step-vel --v 50 --reps 3   # scenario, logs CSV to runs/
"""
import argparse
import csv
import os
import shlex
import sys
import threading
import time

import serial

import proto as P

CSV_HEADER = ["t_ms", "state", "fault_bits", "pos_mm", "vel_mms", "vel_ref_mms", "pos_ref_mm",
              "duty", "missed_cycles"]


class LinkError(Exception):
    pass


def telem_row(t: dict) -> list:
    return [t["t_ms"], t["state"], t["fault_bits"], t["pos_um"] / 1000, t["vel_dmms"] / 10,
            t["vel_ref_dmms"] / 10, t["pos_ref_um"] / 1000, t["duty"], t["missed_cycles"]]


class Link:
    """Serial link: reader thread, request/reply with retry (REQ-P08), heartbeat, CSV log."""

    def __init__(self, port: str, baud: int = 115200, verbose: bool = True,
                 on_telem=None, on_event=None):
        self.ser = serial.serial_for_url(port, baud, timeout=0.05)  # COM5 or socket://host:port
        # 5 ms (REQ-P02) is for the MCU. Here bytes arrive in OS-buffered chunks and are
        # handled when the GIL allows, so a frame split across reads can look "late"; CRC still checks
        self.parser = P.Parser(gap_s=0.1)
        self.verbose = verbose
        self.seq = 0
        self.last_exec = None  # (seq, cmd) of the last non-exempt request
        self.pending = {}      # (seq, ack cmd) -> [threading.Event, payload, t]
        self.lock = threading.Lock()
        self.tx_lock = threading.Lock()
        self.events = []       # (t, cmd, payload)
        self.telem = None
        self.csv = None
        self.running = True
        self.heartbeat_on = False
        self.on_telem = on_telem  # called in the reader thread with each telemetry dict
        self.on_event = on_event  # called in the reader thread with (cmd, payload)
        self.last_rx = 0.0        # perf_counter of the last valid frame
        self.ping_ms = None       # last heartbeat round trip
        self.reader = threading.Thread(target=self._read_loop, daemon=True)
        self.reader.start()
        threading.Thread(target=self._heartbeat_loop, daemon=True).start()

    def close(self):
        self.heartbeat_on = False
        self.running = False
        self.reader.join(0.5)
        self.stop_log()
        self.ser.close()

    def _read_loop(self):
        while self.running:
            # read(1) returns on the first byte; read(256) would wait for the whole timeout
            data = self.ser.read(1)
            if not data:
                continue
            data += self.ser.read(self.ser.in_waiting)
            now = time.perf_counter()
            for seq, cmd, pl in self.parser.feed(data, now):
                self.last_rx = now
                self._dispatch(seq, cmd, pl, now)

    def _dispatch(self, seq, cmd, pl, now):
        if cmd == P.TELEM and len(pl) == P.TELEM_LEN:
            self.telem = P.unpack_telem(pl)
            if self.csv:
                self.csv[1].writerow(telem_row(self.telem))
            if self.on_telem:
                self.on_telem(self.telem)
        elif cmd in (P.EVT_MOVE_DONE, P.EVT_FAULT):
            with self.lock:
                self.events.append((now, cmd, pl))
            if self.on_event:
                self.on_event(cmd, pl)
            if self.verbose:
                print("  <-", describe_event(cmd, pl))
        else:
            with self.lock:
                p = self.pending.get((seq, cmd))
                if p:
                    p[1], p[2] = pl, now
                    p[0].set()

    def _heartbeat_loop(self):
        while self.running:
            time.sleep(0.1)
            if self.heartbeat_on:
                try:
                    self.ping_ms = self.request(P.PING)[2] * 1000
                except LinkError:
                    self.ping_ms = None
                    if self.verbose:
                        print("heartbeat: no reply")

    def _next_seq(self, cmd):
        with self.lock:
            self.seq = (self.seq + 1) & 0xFF
            # A new command must not look like a retry of the last executed one (REQ-P07)
            if cmd not in P.EXEMPT and self.last_exec == (self.seq, cmd):
                self.seq = (self.seq + 1) & 0xFF
            return self.seq

    def request(self, cmd, payload=b"", retries=3, timeout=0.05, lose=0, seq=None):
        """Sends cmd, waits for its ACK. Returns (status, data, latency_s).
        lose: number of attempts whose request is not sent (link test)."""
        seq = self._next_seq(cmd) if seq is None else seq
        frame = P.encode(seq, cmd, payload)
        key = (seq, P.ACK_FLAG | cmd)
        entry = [threading.Event(), None, 0.0]
        with self.lock:
            self.pending[key] = entry
        try:
            for attempt in range(1 + retries):
                t0 = time.perf_counter()
                if attempt >= lose:
                    with self.tx_lock:
                        self.ser.write(frame)
                if entry[0].wait(timeout):
                    lat = entry[2] - t0
                    if cmd not in P.EXEMPT:
                        self.last_exec = (seq, cmd)
                    return entry[1][0], entry[1][1:], lat
            raise LinkError(f"{P.CMD_NAME.get(cmd, hex(cmd))}: no reply after {retries} retries")
        finally:
            with self.lock:
                self.pending.pop(key, None)

    def cmd(self, cmd, *args, check=True):
        """request() with payload packing; raises on a non-OK status when check is set."""
        st, data, _ = self.request(cmd, P.pack(cmd, *args))
        if check and st != 0:
            raise LinkError(f"{P.CMD_NAME[cmd]}: {P.STATUS[st]}")
        return st, data

    def status(self) -> dict:
        return P.unpack_telem(self.cmd(P.GET_STATUS)[1])

    def wait_event(self, cmd, t_from, timeout):
        end = time.perf_counter() + timeout
        while time.perf_counter() < end:
            with self.lock:
                for t, c, pl in self.events:
                    if c == cmd and t >= t_from:
                        return pl
            time.sleep(0.01)
        return None

    def start_log(self, path):
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        f = open(path, "w", newline="")
        w = csv.writer(f)
        w.writerow(CSV_HEADER)
        self.csv = (f, w)

    def stop_log(self):
        if self.csv:
            f, self.csv = self.csv[0], None
            f.close()


def describe_event(cmd, pl):
    if cmd == P.EVT_MOVE_DONE:
        return f"EVT_MOVE_DONE seq={pl[0]} err={int.from_bytes(pl[1:5], 'little', signed=True)} um"
    return f"EVT_FAULT {P.fault_names(pl[0])}"


def describe_reply(cmd, st, data):
    s = f"{P.CMD_NAME[cmd]}: {P.STATUS[st] if st < len(P.STATUS) else st}"
    if st == 0 and cmd == P.PING:
        s += f" version={data[0]}"
    elif st == 0 and cmd == P.GET_STATUS:
        t = P.unpack_telem(data)
        s += (f" state={P.STATES[t['state']]} faults={P.fault_names(t['fault_bits'])}"
              f" pos={t['pos_um'] / 1000:.3f} mm vel={t['vel_dmms'] / 10:.1f} mm/s"
              f" duty={t['duty']} missed={t['missed_cycles']}")
    elif st == 0 and cmd == P.GET_DIAG:
        s += " " + " ".join(f"{k}={v}" for k, v in P.unpack_diag(data).items())
    return s


# ---- scenarios (REQ-H03): telemetry 100 Hz, one CSV per run ----

def begin(link, path, gains=None, limits=None):
    st = link.status()
    if st["state"] == 4:
        raise LinkError(f"actuator in FAULT ({P.fault_names(st['fault_bits'])}): clear-fault first")
    link.heartbeat_on = True
    link.cmd(P.ENABLE, 1)
    if gains:
        link.cmd(P.SET_GAINS, *gains)
    if limits:
        link.cmd(P.SET_LIMITS, *limits)
    link.start_log(path)
    link.cmd(P.SET_TELEM, 100)
    time.sleep(0.5)


def end(link, settle=1.0):
    link.cmd(P.STOP, check=False)
    time.sleep(settle)
    link.cmd(P.SET_TELEM, 0, check=False)
    link.stop_log()


def finish(link):
    link.cmd(P.ENABLE, 0, check=False)
    link.heartbeat_on = False


def scn_sysid(link, a):
    for d in a.duty:
        for sign in (1, -1):
            for r in range(a.reps):
                path = os.path.join(a.out, f"sysid_{sign * d:+d}_{r + 1}.csv")
                begin(link, path)
                link.cmd(P.OPENLOOP_DUTY, sign * d)
                time.sleep(a.hold)
                link.cmd(P.OPENLOOP_DUTY, 0)
                end(link)
                print("wrote", path)
    for sign, name in ((1, "pos"), (-1, "neg")):  # deadzone: slow duty ramp (plan 2.4)
        path = os.path.join(a.out, f"sysid_dz_{name}.csv")
        begin(link, path)
        for d in range(0, a.dz_max + 1, 5):
            link.cmd(P.OPENLOOP_DUTY, sign * d)
            time.sleep(0.1)
        link.cmd(P.OPENLOOP_DUTY, 0)
        end(link)
        print("wrote", path)
    finish(link)


def scn_step_vel(link, a):
    for r in range(a.reps):
        path = os.path.join(a.out, f"step_vel_{a.v}_{r + 1}.csv")
        begin(link, path, a.gains, (a.vmax, a.amax))
        link.cmd(P.SET_VEL, a.v)
        time.sleep(a.hold)
        end(link)
        print("wrote", path)
    finish(link)


def scn_move(link, a):
    for r in range(a.reps):
        path = os.path.join(a.out, f"move_{a.dist:g}_{r + 1}.csv")
        begin(link, path, a.gains, (a.vmax, a.amax))
        t0 = time.perf_counter()
        link.cmd(P.MOVE_REL, int(round(a.dist * 1000)))
        t_est = abs(a.dist) / a.vmax + a.vmax / a.amax
        if link.wait_event(P.EVT_MOVE_DONE, t0, 2 * t_est + 3) is None:
            print("no EVT_MOVE_DONE")
        time.sleep(0.5)
        end(link, 0.2)
        print("wrote", path)
    finish(link)


def scn_disturbance(link, a):
    path = os.path.join(a.out, f"disturbance_{a.v}.csv")
    begin(link, path, a.gains, (a.vmax, a.amax))
    link.cmd(P.SET_VEL, a.v)
    time.sleep(3)
    print(">>> add the load now")
    time.sleep(a.hold - 3)
    end(link)
    finish(link)
    print("wrote", path)


def scn_log(link, a):
    link.heartbeat_on = True
    link.start_log(a.file)
    link.cmd(P.SET_TELEM, a.hz)
    time.sleep(a.time)
    link.cmd(P.SET_TELEM, 0, check=False)
    link.stop_log()
    print("wrote", a.file)


# ---- CLI ----

def build_parser():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    sub = ap.add_subparsers(dest="cmd", required=True)

    for name in ("ping", "get-status", "get-diag", "stop", "estop", "clear-fault"):
        sub.add_parser(name)
    sub.add_parser("enable").add_argument("on", type=int, choices=(0, 1))
    sub.add_parser("set-vel", help="mm/s").add_argument("v", type=int)
    sub.add_parser("move-rel", help="mm").add_argument("mm", type=float)
    p = sub.add_parser("set-limits", help="v_max mm/s, a_max mm/s2")
    p.add_argument("vmax", type=int)
    p.add_argument("amax", type=int)
    sub.add_parser("servo-set", help="deg").add_argument("deg", type=float)
    p = sub.add_parser("set-gains", help="Kp_v Ki_v Kp_p u_dz")
    for g in ("kpv", "kiv", "kpp", "udz"):
        p.add_argument(g, type=float)
    sub.add_parser("set-telem").add_argument("hz", type=int)
    sub.add_parser("openloop-duty", help="permille").add_argument("duty", type=int)
    sub.add_parser("shell", help="interactive; heartbeat keeps the watchdog fed")

    p = sub.add_parser("log", help="log telemetry to CSV")
    p.add_argument("file")
    p.add_argument("--time", type=float, default=60)
    p.add_argument("--hz", type=int, default=100)

    def scenario(name, **kw):
        p = sub.add_parser(name, **kw)
        p.add_argument("--out", default="runs")
        p.add_argument("--reps", type=int, default=3)
        p.add_argument("--gains", type=float, nargs=4, metavar=("KPV", "KIV", "KPP", "UDZ"))
        return p

    p = scenario("sysid", help="open-loop duty steps + deadzone ramps")
    p.add_argument("--duty", type=int, nargs="+", default=[300, 500, 700])
    p.add_argument("--hold", type=float, default=2.0)
    p.add_argument("--dz-max", type=int, default=400)
    p = scenario("step-vel", help="PERF-01")
    p.add_argument("--v", type=int, default=50)
    p.add_argument("--hold", type=float, default=3.0)
    p.add_argument("--vmax", type=int, default=150)
    p.add_argument("--amax", type=int, default=2000)
    p = scenario("move", help="PERF-02")
    p.add_argument("--dist", type=float, default=200, help="mm")
    p.add_argument("--vmax", type=int, default=60)
    p.add_argument("--amax", type=int, default=200)
    p = scenario("disturbance", help="PERF-03: add load by hand")
    p.add_argument("--v", type=int, default=50)
    p.add_argument("--hold", type=float, default=20.0)
    p.add_argument("--vmax", type=int, default=150)
    p.add_argument("--amax", type=int, default=200)
    return ap


SIMPLE = {"ping": P.PING, "get-status": P.GET_STATUS, "get-diag": P.GET_DIAG, "stop": P.STOP,
          "estop": P.ESTOP, "clear-fault": P.CLEAR_FAULT}
SCENARIOS = {"sysid": scn_sysid, "step-vel": scn_step_vel, "move": scn_move,
             "disturbance": scn_disturbance, "log": scn_log}


def run_command(link, a):
    if a.cmd in SCENARIOS:
        return SCENARIOS[a.cmd](link, a)
    if a.cmd in SIMPLE:
        cmd, args = SIMPLE[a.cmd], ()
    else:
        # lambdas: each command has its own argument names
        cmd, args = {
            "enable": lambda: (P.ENABLE, (a.on,)),
            "set-vel": lambda: (P.SET_VEL, (a.v,)),
            "move-rel": lambda: (P.MOVE_REL, (int(round(a.mm * 1000)),)),
            "set-limits": lambda: (P.SET_LIMITS, (a.vmax, a.amax)),
            "servo-set": lambda: (P.SERVO_SET, (int(round(a.deg * 10)),)),
            "set-gains": lambda: (P.SET_GAINS, (a.kpv, a.kiv, a.kpp, a.udz)),
            "set-telem": lambda: (P.SET_TELEM, (a.hz,)),
            "openloop-duty": lambda: (P.OPENLOOP_DUTY, (a.duty,)),
        }[a.cmd]()
    st, data, lat = link.request(cmd, P.pack(cmd, *args))
    print(describe_reply(cmd, st, data), f"({lat * 1000:.1f} ms)")
    if st == 2:  # BAD_STATE: say which state refused it
        t = link.status()
        hint = {1: "run 'enable 1' first", 4: "run 'clear-fault', then 'enable 1'"}
        print(f"  state={P.STATES[t['state']]} faults={P.fault_names(t['fault_bits'])}"
              f" -> {hint.get(t['state'], 'see spec §6.4 for allowed states')}")


def shell(link, ap):
    link.heartbeat_on = True
    print("commands as on the command line (without --port); 'quit' to exit")
    try:
        while True:
            try:
                line = input("> ").strip()
            except EOFError:
                break
            if line in ("quit", "exit"):
                break
            if not line:
                continue
            try:
                a = ap.parse_args(["--port", "-"] + shlex.split(line))
                if a.cmd != "shell":
                    run_command(link, a)
            except SystemExit:
                pass
            except LinkError as e:
                print("error:", e)
    finally:
        # Leave the actuator in IDLE: once the heartbeat stops, READY/RUNNING would trip WATCHDOG
        try:
            link.cmd(P.STOP, check=False)
            link.cmd(P.ENABLE, 0, check=False)
        except LinkError:
            pass


def main():
    ap = build_parser()
    a = ap.parse_args()
    link = Link(a.port, a.baud)
    try:
        if a.cmd == "shell":
            shell(link, ap)
        else:
            run_command(link, a)
    except LinkError as e:
        print("error:", e)
        sys.exit(1)
    except KeyboardInterrupt:
        try:
            link.cmd(P.STOP, check=False)
        except LinkError:
            pass
    finally:
        link.close()


if __name__ == "__main__":
    main()
