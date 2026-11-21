"""Run analysis (REQ-H04). Time axis = MCU t_ms.

python tools/analyze.py runs/step_vel_50_1.csv [more.csv ...] [--sim test/sim/out.csv]
python tools/analyze.py --fit runs/sysid_+*.csv runs/sysid_-*.csv
python tools/analyze.py --dz runs/sysid_dz_pos.csv runs/sysid_dz_neg.csv
"""
import argparse
import os

import numpy as np
from scipy.optimize import curve_fit

RUNNING, READY = 3, 2


def load(path):
    d = np.genfromtxt(path, delimiter=",", names=True)
    d = {k: d[k] for k in d.dtype.names}
    d["t"] = (d["t_ms"] - d["t_ms"][0]) / 1000.0
    return d


def is_move(d):
    run = d["state"] == RUNNING
    return bool(np.any(np.abs(d["pos_ref_mm"][run] - d["pos_mm"][run]) > 1e-6))


def step_start(d):
    """Index of the first sample with a non-zero reference (or duty)."""
    ref = d["vel_ref_mms"] if np.any(d["vel_ref_mms"]) else d["duty"]
    nz = np.flatnonzero(ref)
    return nz[0] if len(nz) else 0


def vel_metrics(d):
    """t_r (10-90 %), t_s (2 %), POT, e_ss on the velocity step."""
    i0 = step_start(d)
    ref = d["vel_ref_mms"]
    target = ref[np.argmax(np.abs(ref))]
    hold = np.flatnonzero(ref == target)
    i1 = hold[-1] + 1
    t = d["t"][i0:i1] - d["t"][i0]
    y = d["vel_mms"][i0:i1] / target  # normalized, positive towards the target

    def first(cond):
        idx = np.flatnonzero(cond)
        return t[idx[0]] if len(idx) else np.nan

    out_band = np.flatnonzero(np.abs(y - 1) > 0.02)
    tail = t >= t[-1] - 0.5
    return {
        "target_mms": target,
        "t_r_s": first(y >= 0.9) - first(y >= 0.1),
        "t_s_s": t[out_band[-1] + 1] if len(out_band) and out_band[-1] + 1 < len(t) else
        (np.nan if len(out_band) else 0.0),
        "POT_pct": max(0.0, (y.max() - 1) * 100),
        "e_ss_mms": target - d["vel_mms"][i0:i1][tail].mean(),
    }


def move_metrics(d):
    run = np.flatnonzero(d["state"] == RUNNING)
    e = d["pos_ref_mm"][run] - d["pos_mm"][run]
    final_ref = d["pos_ref_mm"][run[-1]]
    return {
        "track_max_mm": np.abs(e).max(),
        "track_rms_mm": np.sqrt(np.mean(e ** 2)),
        "final_err_mm": final_ref - d["pos_mm"][-1],
        "move_time_s": d["t"][run[-1]] - d["t"][run[0]],
    }


def metrics(d):
    return move_metrics(d) if is_move(d) else vel_metrics(d)


# Spec §10.1 targets: metric -> (limit, compare on the absolute value)
TARGETS = {"POT_pct": (5.0, False), "t_s_s": (0.5, False), "e_ss_mms": (1.0, True),
           "track_max_mm": (3.0, False), "final_err_mm": (1.0, True)}


def passes(key, value):
    """True / False against §10.1, None when the metric has no target."""
    if key not in TARGETS or value != value:  # NaN never passes
        return None if key not in TARGETS else False
    limit, absolute = TARGETS[key]
    return (abs(value) if absolute else value) <= limit


def pi_from_model(k, tau, tau_cl, ts=0.01):
    """Spec §5.2, pole cancellation of G(s) = K / (tau s + 1).
    k in mm/s per permille, tau and tau_cl in s."""
    kp_v = tau / (k * tau_cl)
    return {"kp_v": kp_v, "ki_v": kp_v / tau, "kp_p_max": 1.0 / (5.0 * tau_cl),
            "t_s": 4.0 * tau_cl, "ts_ok": ts <= tau_cl / 10.0}


def print_table(rows, sim_row=None):
    keys = list(rows[0][1].keys())
    print("run".ljust(32) + "".join(k.rjust(14) for k in keys))
    for name, m in rows:
        print(name[-32:].ljust(32) + "".join(f"{m[k]:14.4g}" for k in keys))
    if len(rows) > 1:
        a = np.array([[m[k] for k in keys] for _, m in rows])
        print("mean".ljust(32) + "".join(f"{x:14.4g}" for x in np.nanmean(a, 0)))
        print("std".ljust(32) + "".join(f"{x:14.4g}" for x in np.nanstd(a, 0)))
    if sim_row:
        print("SIL".ljust(32) + "".join(f"{sim_row[k]:14.4g}" for k in keys))


def plot(runs, sim, save):
    import matplotlib.pyplot as plt

    move = is_move(runs[0][1])
    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    sets = [(n, d, "-") for n, d in runs] + ([("SIL", sim, "--")] if sim else [])
    for name, d, ls in sets:
        i0 = step_start(d)
        t = d["t"] - d["t"][i0]
        label = os.path.basename(name)
        ax[0].plot(t, d["vel_mms"], ls, label=label)
        ax[0].plot(t, d["vel_ref_mms"], ":", color="gray")
        if move:
            ax[1].plot(t, d["pos_ref_mm"] - d["pos_mm"], ls, label=label)
        else:
            ax[1].plot(t, d["pos_mm"], ls, label=label)
        ax[2].plot(t, d["duty"], ls, label=label)
    ax[0].set_ylabel("v [mm/s]")
    ax[1].set_ylabel("tracking error [mm]" if move else "pos [mm]")
    ax[2].set_ylabel("duty [permille]")
    ax[2].set_xlabel("t from step start [s]")
    for x in ax:
        x.grid(True)
    ax[0].legend(fontsize=8)
    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=150)
        print("saved", save)
    else:
        plt.show()


def fit_step(d):
    """First-order fit v = v_ss (1 - exp(-(t - t0) / tau)) on one open-loop duty step."""
    duty = d["duty"]
    i0 = np.flatnonzero(duty)[0]
    i1 = np.flatnonzero(duty[i0:] != duty[i0])
    i1 = i0 + (i1[0] if len(i1) else len(duty) - i0)
    t = d["t"][i0:i1] - d["t"][i0]
    v = d["vel_mms"][i0:i1]
    f = lambda t, vss, tau: vss * (1 - np.exp(-t / tau))  # noqa: E731
    (vss, tau), _ = curve_fit(f, t, v, p0=(v[-1], 0.1), bounds=([-1e3, 1e-3], [1e3, 5]))
    return duty[i0], vss, tau


def cmd_fit(files):
    res = []
    print(f"{'file':32}{'u':>8}{'v_ss':>10}{'tau':>10}")
    for path in files:
        u, vss, tau = fit_step(load(path))
        res.append((u, vss, tau))
        print(f"{os.path.basename(path)[-32:]:32}{u:8.0f}{vss:10.2f}{tau:10.4f}")
    r = np.array(res)
    for sign, name in ((1, "forward"), (-1, "reverse")):
        s = r[np.sign(r[:, 0]) == sign]
        if len(s) < 2:
            continue
        k, b = np.polyfit(np.abs(s[:, 0]), np.abs(s[:, 1]), 1)  # |v_ss| = K (|u| - u_dz)
        taus = s[:, 2]
        print(f"{name}: K = {k:.4f} mm/s per permille, u_dz = {-b / k:.1f} permille, "
              f"tau = {taus.mean():.4f} s (spread {np.ptp(taus) / taus.mean() * 100:.1f} %)")
        for u in np.unique(np.abs(s[:, 0])):
            g = np.abs(s[np.abs(s[:, 0]) == u, 1]) / u
            if len(g) > 1:
                print(f"  u={u:.0f}: v_ss/u spread {np.ptp(g) / g.mean() * 100:.1f} %")


def cmd_dz(files):
    for path in files:
        d = load(path)
        moving = np.flatnonzero(np.abs(d["vel_mms"]) > 2.0)
        u = abs(d["duty"][moving[0]]) if len(moving) else np.nan
        print(f"{os.path.basename(path)}: belt starts at |duty| = {u:.0f} permille")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("files", nargs="+")
    ap.add_argument("--sim", help="SIL CSV to overlay")
    ap.add_argument("--fit", action="store_true", help="fit K, tau, u_dz from sysid steps")
    ap.add_argument("--dz", action="store_true", help="deadzone from sysid_dz ramps")
    ap.add_argument("--save", help="save the figure instead of showing it")
    ap.add_argument("--no-plot", action="store_true")
    a = ap.parse_args()

    if a.fit:
        return cmd_fit(a.files)
    if a.dz:
        return cmd_dz(a.files)
    runs = [(f, load(f)) for f in a.files]
    sim = load(a.sim) if a.sim else None
    print_table([(f, metrics(d)) for f, d in runs], metrics(sim) if sim else None)
    if not a.no_plot:
        plot(runs, sim, a.save)


if __name__ == "__main__":
    main()
