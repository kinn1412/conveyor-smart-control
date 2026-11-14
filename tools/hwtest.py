"""Hardware tests over the protocol (plan phase 0, 1, 5; spec §10.2).

python tools/hwtest.py --port COM5 <test> [options]
  pwm       duty steps for the multimeter check (plan 0.4)
  encoder   hand-turn count check, ENC_CPR +- 4 per output rev (plan 0.5)
  motor     direction / sign check at low duty (plan 0.6)
  servo     angle sweep (plan 0.7, 6.x)
  link      1000 requests with injected losses + duplicate filter (REQ-P04, P07, P08)
  watchdog  stop the heartbeat, expect FAULT WATCHDOG (plan 1.6)
  estop     ESTOP latency over 100 commands (REQ-S04)
  diag      timing / stack / errors under 100 Hz telemetry + 20 Hz commands (REQ-F01, F04, F07)
"""
import argparse
import math
import os
import random
import re
import sys
import time

import proto as P
from host import Link, LinkError

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def config_value(name):
    with open(os.path.join(ROOT, "App", "config.h")) as f:
        return float(re.search(rf"#define {name}\s+([-\d.]+)", f.read()).group(1))


ENC_CPR = 11 * int(config_value("GEAR_RATIO")) * 4
UM_PER_CNT = math.pi * config_value("ROLLER_D_MM") * 1000.0 / ENC_CPR
BRINGUP_DUTY = int(config_value("BRINGUP_DUTY_MAX_PERMILLE"))

results = []


def check(name, ok, detail=""):
    results.append(ok)
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}")


def counts(t):
    return t["pos_um"] / UM_PER_CNT


def ensure_idle(link):
    st = link.status()
    if st["state"] == 4:
        print("clearing fault:", P.fault_names(st["fault_bits"]))
        link.cmd(P.CLEAR_FAULT)
    link.cmd(P.ENABLE, 0)


def wait_enter(msg):
    input(f"{msg} [Enter] ")


def t_pwm(link, a):
    levels = [0, 250, 500, 1000, -250]
    if not a.motor_disconnected:
        levels = [d for d in levels if abs(d) <= BRINGUP_DUTY]
        print(f"motor connected: duty limited to {BRINGUP_DUTY} permille "
              "(use --motor-disconnected for the full list)")
    ensure_idle(link)
    link.heartbeat_on = True
    link.cmd(P.ENABLE, 1)
    for d in levels:
        link.cmd(P.OPENLOOP_DUTY, d)
        wait_enter(f"duty {d:+5d}: expect {3.3 * abs(d) / 1000:.2f} V on PA6, "
                   f"DIR (PB12) {'high' if d < 0 else 'low'}")
    link.cmd(P.STOP)
    link.cmd(P.ENABLE, 0)


def t_encoder(link, a):
    ensure_idle(link)
    link.heartbeat_on = True
    c0 = counts(link.status())
    print("turn the motor output shaft exactly one revolution forward, then press Enter")
    try:
        while True:
            print(f"\r  {counts(link.status()) - c0:9.1f} counts", end="")
            time.sleep(0.2)
            if os.name == "nt":
                import msvcrt
                if msvcrt.kbhit() and msvcrt.getwch() in "\r\n":
                    break
    except KeyboardInterrupt:
        pass
    print()
    n = counts(link.status()) - c0
    check("one output rev", abs(n - ENC_CPR) <= 4,
          f"{n:.1f} counts ({ENC_CPR} +- 4; negative: flip ENC_SIGN; wrong size: check GEAR_RATIO)")


def t_motor(link, a):
    d = min(a.duty, BRINGUP_DUTY)
    ensure_idle(link)
    link.heartbeat_on = True
    for sign, name in ((1, "forward"), (-1, "reverse")):
        link.cmd(P.ENABLE, 1)
        c0 = counts(link.status())
        link.cmd(P.OPENLOOP_DUTY, sign * d)
        time.sleep(a.time)
        st = link.status()
        link.cmd(P.STOP, check=False)
        time.sleep(0.3)
        if st["state"] != 3:
            check(f"{name} run", False, f"state {P.STATES[st['state']]} "
                  f"faults {P.fault_names(st['fault_bits'])}")
            ensure_idle(link)
            continue
        dn = counts(st) - c0
        print(f"{name}: duty {sign * d:+d}, v = {st['vel_dmms'] / 10:.1f} mm/s, {dn:+.0f} counts")
        check(f"{name}: counter moves with the duty sign", dn * sign > 0,
              "(fix DIR_SIGN or ENC_SIGN in config.h if not)")
        input(f"  did the belt move {name}? (Enter = yes, Ctrl+C = no) ")
    link.cmd(P.ENABLE, 0, check=False)


def t_servo(link, a):
    ensure_idle(link)
    link.heartbeat_on = True
    ang = a.start
    while (ang <= a.end) if a.step > 0 else (ang >= a.end):
        link.cmd(P.SERVO_SET, int(round(ang * 10)))
        if a.dwell:
            print(f"servo {ang:+.1f} deg")
            time.sleep(a.dwell)
        else:
            wait_enter(f"servo {ang:+.1f} deg: measure the angle")
        ang += a.step
    link.cmd(P.SERVO_SET, 0)


def t_link(link, a):
    rng = random.Random(1)
    lat, retries = [], 0
    safe = [(P.PING, ()), (P.GET_STATUS, ()), (P.GET_DIAG, ()), (P.SET_TELEM, (0,)),
            (P.STOP, ()), (P.ENABLE, (0,))]
    ensure_idle(link)
    for i in range(a.n):
        cmd, args = rng.choice(safe)
        lose = 1 if rng.random() < a.loss else 0
        retries += lose
        try:
            st, _, dt = link.request(cmd, P.pack(cmd, *args), lose=lose)
        except LinkError as e:
            check("link", False, f"request {i}: {e}")
            return
        if lose == 0:
            lat.append(dt)
    lat_ms = sorted(x * 1000 for x in lat)
    check(f"{a.n} requests answered", True, f"({retries} injected losses recovered by retry)")
    check("reply latency <= 20 ms (REQ-P04, includes PC/USB latency)", lat_ms[-1] <= 20,
          f"median {lat_ms[len(lat_ms) // 2]:.1f} ms, max {lat_ms[-1]:.1f} ms")

    # Duplicate filter (REQ-P07): a retried CLEAR_FAULT must not execute twice
    link.cmd(P.ESTOP)
    time.sleep(0.05)
    seq = 77 if link.last_exec != (77, P.CLEAR_FAULT) else 78
    st1, _, _ = link.request(P.CLEAR_FAULT, seq=seq)
    st2, _, _ = link.request(P.CLEAR_FAULT, seq=seq)  # lost-reply retry: same SEQ
    st3, _, _ = link.request(P.CLEAR_FAULT)
    check("duplicate filter", (st1, st2, st3) == (0, 0, 2),
          f"statuses {[P.STATUS[s] for s in (st1, st2, st3)]}, expected OK OK BAD_STATE")


def t_watchdog(link, a):
    ensure_idle(link)
    link.heartbeat_on = True
    link.cmd(P.ENABLE, 1)
    time.sleep(0.3)
    link.heartbeat_on = False
    time.sleep(0.6)
    st = link.status()
    check("watchdog", st["state"] == 4 and st["fault_bits"] & 1,
          f"state {P.STATES[st['state']]} faults {P.fault_names(st['fault_bits'])}")
    ensure_idle(link)


def t_estop(link, a):
    ensure_idle(link)
    link.cmd(P.GET_DIAG)  # reset max values
    for _ in range(a.n):
        link.cmd(P.ENABLE, 1)
        link.cmd(P.ESTOP)
        time.sleep(0.03)
        link.cmd(P.CLEAR_FAULT)
    d = P.unpack_diag(link.cmd(P.GET_DIAG)[1])
    check("ESTOP latency <= 2 ms (REQ-S04)", d["estop_latency_max_us"] <= 2000,
          f"max {d['estop_latency_max_us']} us over {a.n}")
    link.cmd(P.ENABLE, 0)


def t_diag(link, a):
    ensure_idle(link)
    link.heartbeat_on = True
    e0 = P.unpack_diag(link.cmd(P.GET_DIAG)[1])
    link.cmd(P.SET_TELEM, 100)
    end = time.monotonic() + a.time
    while time.monotonic() < end:  # 20 Hz commands on top of the 10 Hz heartbeat
        link.cmd(P.GET_STATUS)
        time.sleep(0.05)
    link.cmd(P.SET_TELEM, 0)
    d = P.unpack_diag(link.cmd(P.GET_DIAG)[1])
    t = link.status()
    print(" ".join(f"{k}={v}" for k, v in d.items()))
    jitter = max(10000 - d["period_min_us"], d["period_max_us"] - 10000)
    check("period jitter <= 0.2 ms (REQ-F01)", jitter <= 200,
          f"{d['period_min_us']}..{d['period_max_us']} us")
    check("execution < 1 ms (REQ-F01)", d["exec_max_us"] < 1000, f"{d['exec_max_us']} us")
    check("missed cycles = 0 (REQ-F07)", t["missed_cycles"] == 0, str(t["missed_cycles"]))
    for name, size in (("ctrl", int(config_value("CTRL_STACK_WORDS"))),
                       ("rx", int(config_value("RX_STACK_WORDS"))),
                       ("tx", int(config_value("TX_STACK_WORDS")))):
        free = d[f"stack_free_{name}"]
        check(f"stack {name} >= 25 % free (REQ-F04)", free >= size // 4, f"{free}/{size} words")
    errs = {k: d[k] - e0[k] for k in ("err_crc", "err_len", "err_timeout")}
    check("no frame errors (REQ-F03)", not any(errs.values()), str(errs))


TESTS = {"pwm": t_pwm, "encoder": t_encoder, "motor": t_motor, "servo": t_servo, "link": t_link,
         "watchdog": t_watchdog, "estop": t_estop, "diag": t_diag}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--port", required=True)
    sub = ap.add_subparsers(dest="test", required=True)
    sub.add_parser("pwm").add_argument("--motor-disconnected", action="store_true")
    sub.add_parser("encoder")
    p = sub.add_parser("motor")
    p.add_argument("--duty", type=int, default=200)
    p.add_argument("--time", type=float, default=1.5)
    p = sub.add_parser("servo")
    p.add_argument("--start", type=float, default=-90)
    p.add_argument("--end", type=float, default=90)
    p.add_argument("--step", type=float, default=15)
    p.add_argument("--dwell", type=float, default=0, help="s per angle; 0 = wait for Enter")
    p = sub.add_parser("link")
    p.add_argument("--n", type=int, default=1000)
    p.add_argument("--loss", type=float, default=0.1)
    sub.add_parser("watchdog")
    sub.add_parser("estop").add_argument("--n", type=int, default=100)
    sub.add_parser("diag").add_argument("--time", type=float, default=60)
    a = ap.parse_args()

    link = Link(a.port, verbose=False)
    try:
        TESTS[a.test](link, a)
    except LinkError as e:
        check(a.test, False, str(e))
    except KeyboardInterrupt:
        print("\naborted")
        results.append(False)
    finally:
        try:
            link.cmd(P.STOP, check=False)
        except LinkError:
            pass
        link.close()
    sys.exit(0 if results and all(results) else 1)


if __name__ == "__main__":
    main()
