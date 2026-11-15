#!/usr/bin/env python3
"""
Графики полёта по тестовому маршруту (tools/mission.py) из лога PX4 (.ulg).

    python3 ~/simulation/tools/plot_mission.py            # последний лог PX4 SITL
    python3 ~/simulation/tools/plot_mission.py путь.ulg   # конкретный лог

Сохраняет docs/mission_<имя лога>.png и печатает сводку:
время маршрута, отклонение от линии маршрута, промах по точкам, высоту.
"""
import glob
import math
import os
import sys
from pathlib import Path

try:
    import numpy as np
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from pyulog import ULog
except ImportError as e:
    sys.exit(f"Не хватает библиотеки ({e.name}). Установите:\n"
             "  pip install --break-system-packages pyulog matplotlib numpy")

sys.path.insert(0, str(Path(__file__).resolve().parent))
from route import LAT0, LON0, ROUTE, ALT, meters_per_degree  # noqa: E402

PX4_DIR = os.environ.get("PX4_DIR", os.path.expanduser("~/PX4-Autopilot"))
LOG_DIR = os.path.join(PX4_DIR, "build", "px4_sitl_default", "rootfs", "log")
OUT_DIR = Path(__file__).resolve().parent.parent / "docs"

NAV_AUTO_MISSION = 3      # vehicle_status.nav_state
ARMING_ARMED = 2          # vehicle_status.arming_state


def latest_log():
    logs = glob.glob(os.path.join(LOG_DIR, "*", "*.ulg"))
    if not logs:
        sys.exit(f"Логов нет в {LOG_DIR}.")
    return max(logs, key=os.path.getmtime)


def ds(ulog, name):
    try:
        return ulog.get_dataset(name).data
    except (KeyError, IndexError, ValueError):
        return None


def route_local(ref_lat, ref_lon):
    """Точки маршрута в локальных координатах PX4 (север, восток), м.
    Путь: старт -> WP1 -> ... -> WP4 -> старт."""
    m_lat, m_lon = meters_per_degree(LAT0)
    m_lat_r, m_lon_r = meters_per_degree(ref_lat)

    def conv(east, north):
        lat = LAT0 + north / m_lat
        lon = LON0 + east / m_lon
        return (lat - ref_lat) * m_lat_r, (lon - ref_lon) * m_lon_r

    home = conv(0.0, 0.0)
    wps = [(name, *conv(e, n)) for name, e, n in ROUTE]
    path = [home] + [(n, e) for _, n, e in wps] + [home]
    return home, wps, np.array(path)


def dist_to_path(pn, pe, path):
    """Расстояние от точек (pn, pe) до ломаной path [[n, e], ...], м."""
    p = np.stack([pn, pe], axis=1)
    best = np.full(len(p), np.inf)
    for a, b in zip(path[:-1], path[1:]):
        ab = b - a
        L2 = float(ab @ ab)
        t = np.zeros(len(p)) if L2 == 0 else np.clip(((p - a) @ ab) / L2, 0, 1)
        proj = a + t[:, None] * ab
        best = np.minimum(best, np.linalg.norm(p - proj, axis=1))
    return best


def analyse(t, n, e, alt, in_mission, path, wps):
    """Сводка по маршруту. alt — высота над точкой взлёта."""
    res = {}
    tm = t[in_mission]
    res["duration"] = float(tm[-1] - tm[0]) if len(tm) > 1 else float("nan")
    # «крейсерский» участок: в режиме Mission и на высоте маршрута (без взлёта и посадки)
    cruise = in_mission & (alt > ALT - 1.0)
    if cruise.sum() < 5:
        cruise = in_mission
    d = dist_to_path(n[cruise], e[cruise], path)
    res["xtrack_max"] = float(d.max()) if len(d) else float("nan")
    res["xtrack_mean"] = float(d.mean()) if len(d) else float("nan")
    res["alt_mean"] = float(alt[cruise].mean()) if cruise.any() else float("nan")
    res["alt_std"] = float(alt[cruise].std()) if cruise.any() else float("nan")
    res["alt_maxdev"] = float(np.abs(alt[cruise] - ALT).max()) if cruise.any() else float("nan")
    res["wp_miss"] = []
    for name, wn, we in wps:
        dd = np.hypot(n[in_mission] - wn, e[in_mission] - we) if in_mission.any() else np.array([np.nan])
        res["wp_miss"].append((name, float(np.nanmin(dd))))
    return res, cruise


def main():
    path_log = sys.argv[1] if len(sys.argv) > 1 else latest_log()
    ulog = ULog(path_log, ["vehicle_local_position", "vehicle_status", "home_position",
                           "vehicle_local_position_setpoint"])
    lp = ds(ulog, "vehicle_local_position")
    st = ds(ulog, "vehicle_status")
    if lp is None:
        sys.exit("В логе нет vehicle_local_position.")
    t0 = np.int64(ulog.start_timestamp)
    sec = lambda d: (d["timestamp"].astype(np.int64) - t0) / 1e6

    t = sec(lp)
    n, e, z = lp["x"], lp["y"], lp["z"]
    vh = np.hypot(lp["vx"], lp["vy"])
    ref_lat = float(lp["ref_lat"][-1]) if "ref_lat" in lp and lp["ref_lat"][-1] != 0 else LAT0
    ref_lon = float(lp["ref_lon"][-1]) if "ref_lon" in lp and lp["ref_lon"][-1] != 0 else LON0
    home, wps, path = route_local(ref_lat, ref_lon)

    # Режим Mission и «вооружение» по vehicle_status
    in_mission = np.zeros(len(t), bool)
    armed = np.ones(len(t), bool)
    if st is not None and len(st["timestamp"]):
        ts = sec(st)
        idx = np.clip(np.searchsorted(ts, t, side="right") - 1, 0, len(ts) - 1)
        in_mission = st["nav_state"][idx] == NAV_AUTO_MISSION
        armed = st["arming_state"][idx] == ARMING_ARMED
    if not in_mission.any():
        print("Режим Mission в логе не найден — считаю по всему полёту.")
        in_mission = armed.copy()

    # Высота над домашней точкой PX4 (от неё отсчитывается высота маршрута).
    # z в PX4 направлен вниз. Если домашней точки в логе нет — от момента взлёта.
    z_home = z[np.argmax(armed)] if armed.any() else z[0]
    home_src = "момент включения моторов"
    hp = ds(ulog, "home_position")
    if hp is not None and len(hp["timestamp"]):
        th = sec(hp)
        ok = hp["valid_lpos"].astype(bool) if "valid_lpos" in hp else np.ones(len(th), bool)
        t_ref = t[in_mission][0] if in_mission.any() else t[0]
        cand = np.where(ok & (th <= t_ref))[0]
        if len(cand) == 0:
            cand = np.where(ok)[0]
        if len(cand):
            z_home = float(hp["z"][cand[-1]])
            home_src = "домашняя точка PX4"
    alt = -(z - z_home)

    res, cruise = analyse(t, n, e, alt, in_mission, path, wps)

    # Высота, которую PX4 задавал сам себе (уставка регулятора положения)
    sp = ds(ulog, "vehicle_local_position_setpoint")
    t_sp = alt_sp = None
    if sp is not None and len(sp["timestamp"]) and "z" in sp:
        t_sp = sec(sp)
        alt_sp = -(sp["z"] - z_home)
        keep = np.isfinite(alt_sp)
        t_sp, alt_sp = t_sp[keep], alt_sp[keep]
        if len(t_sp):
            idx = np.clip(np.searchsorted(t_sp, t, side="right") - 1, 0, len(t_sp) - 1)
            err = (alt - alt_sp[idx])[cruise]
            res["alt_track"] = float(np.abs(err).max()) if err.size else float("nan")
            res["alt_sp_mean"] = float(alt_sp[idx][cruise].mean()) if err.size else float("nan")

    # ------------------------------ графики ---------------------------------
    fig = plt.figure(figsize=(12, 8.5))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.15, 1], height_ratios=[1, 1])
    ax = fig.add_subplot(gs[:, 0])
    ax.plot(path[:, 1], path[:, 0], "--", color="#888", lw=1.5, label="маршрут")
    fl = armed
    ax.plot(e[fl], n[fl], color="#1f77b4", lw=1.6, label="траектория (PX4)")
    ax.plot(home[1], home[0], "s", ms=10, mfc="#666", mec="k", label="старт / посадка")
    for name, wn, we in wps:
        ax.plot(we, wn, "o", ms=12, mfc="#f39c12", mec="k")
        ax.annotate(name, (we, wn), xytext=(8, 8), textcoords="offset points", fontsize=11)
        ax.add_patch(plt.Circle((we, wn), 1.0, fill=False, ls=":", color="#f39c12"))
    ax.set_aspect("equal")
    ax.set_xlabel("восток, м")
    ax.set_ylabel("север, м")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.09), ncol=3, fontsize=9)
    ax.set_title("Вид сверху (пунктирные круги — радиус достижения 1 м)")

    def shade(a):
        if in_mission.any():
            a.axvspan(t[in_mission][0], t[in_mission][-1], color="#2ca02c", alpha=0.08)

    a1 = fig.add_subplot(gs[0, 1])
    a1.plot(t, alt, color="#1f77b4", label="высота (оценка PX4)")
    if t_sp is not None and len(t_sp):
        a1.plot(t_sp, alt_sp, color="#ff7f0e", lw=1.2, ls=":", label="уставка PX4")
    a1.axhline(ALT, ls="--", color="#888", label=f"задано {ALT:g} м")
    shade(a1)
    a1.set_ylabel("высота, м")
    a1.grid(alpha=0.3)
    a1.legend(fontsize=9)
    a1.set_title("Высота (зелёное — режим Mission)")

    a2 = fig.add_subplot(gs[1, 1], sharex=a1)
    a2.plot(t, vh, color="#d62728")
    shade(a2)
    a2.set_ylabel("горизонтальная скорость, м/с")
    a2.set_xlabel("время, с")
    a2.grid(alpha=0.3)
    if armed.any():
        a1.set_xlim(max(0, t[armed][0] - 5), t[armed][-1] + 5)

    name = Path(path_log).stem
    fig.suptitle(f"Тестовый маршрут WP1–WP4, PX4 SITL — {name}.ulg", fontsize=13)
    fig.tight_layout()
    OUT_DIR.mkdir(exist_ok=True)
    out = OUT_DIR / f"mission_{name}.png"
    fig.savefig(out, dpi=110)

    print(f"Лог:      {path_log}")
    print(f"График:   {out}")
    print(f"Высота отсчитывается от: {home_src}")
    print(f"Маршрут (режим Mission): {res['duration']:.1f} с")
    print(f"  отклонение от линии маршрута: макс. {res['xtrack_max']:.2f} м, среднее {res['xtrack_mean']:.2f} м")
    print(f"  высота на маршруте: {res['alt_mean']:.2f} ± {res['alt_std']:.2f} м "
          f"(задано {ALT:g}, макс. отклонение {res['alt_maxdev']:.2f} м)")
    if "alt_track" in res:
        print(f"  уставка высоты PX4 на маршруте: в среднем {res['alt_sp_mean']:.2f} м; "
              f"отставание высоты от уставки: макс. {res['alt_track']:.2f} м")
    print("  промах по точкам (ближайшее расстояние):")
    for wp, d in res["wp_miss"]:
        print(f"    {wp}: {d:.2f} м")


if __name__ == "__main__":
    main()
