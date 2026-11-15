#!/usr/bin/env python3
"""
Графики полёта из лога PX4 (.ulg) — для отчёта о стабилизации.

    python3 ~/simulation/tools/plot_flight.py            # последний лог PX4 SITL
    python3 ~/simulation/tools/plot_flight.py путь.ulg   # конкретный лог

Сохраняет картинку flight_<имя лога>.png в ~/simulation/docs/ и печатает
сводку: высота висения, её разброс, максимальные крен/тангаж, уход в сторону.
Нужны pyulog и matplotlib — их ставит установщик PX4 (Tools/setup/ubuntu.sh).
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

PX4_DIR = os.environ.get("PX4_DIR", os.path.expanduser("~/PX4-Autopilot"))
LOG_DIR = os.path.join(PX4_DIR, "build", "px4_sitl_default", "rootfs", "log")
OUT_DIR = Path(__file__).resolve().parent.parent / "docs"


def latest_log():
    logs = glob.glob(os.path.join(LOG_DIR, "*", "*.ulg"))
    if not logs:
        sys.exit(f"Логов нет в {LOG_DIR}.\n"
                 "Лог пишется, пока дрон «вооружён» (от взлёта до посадки).")
    return max(logs, key=os.path.getmtime)


def ds(ulog, name):
    try:
        return ulog.get_dataset(name).data
    except (KeyError, IndexError, ValueError):
        return None


def quat_to_euler(q0, q1, q2, q3):
    roll = np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 ** 2 + q2 ** 2))
    pitch = np.arcsin(np.clip(2 * (q0 * q2 - q3 * q1), -1, 1))
    yaw = np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 ** 2 + q3 ** 2))
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else latest_log()
    names = ["vehicle_local_position", "vehicle_attitude", "vehicle_attitude_setpoint",
             "vehicle_status", "actuator_motors", "actuator_outputs",
             "trajectory_setpoint", "vehicle_local_position_setpoint"]
    ulog = ULog(path, names)
    t0 = ulog.start_timestamp
    sec = lambda d: (d["timestamp"].astype(np.int64) - np.int64(t0)) / 1e6

    lp = ds(ulog, "vehicle_local_position")
    att = ds(ulog, "vehicle_attitude")
    if lp is None or att is None:
        sys.exit("В логе нет позиции или ориентации — это точно лог полёта PX4?")

    t_lp, alt = sec(lp), -lp["z"]
    t_att = sec(att)
    roll, pitch, _ = quat_to_euler(att["q[0]"], att["q[1]"], att["q[2]"], att["q[3]"])

    # уставки
    sp = ds(ulog, "trajectory_setpoint") or ds(ulog, "vehicle_local_position_setpoint")
    asp = ds(ulog, "vehicle_attitude_setpoint")
    # моторы: нормированные команды 0..1 (новые PX4) или сырые выходы
    mot = ds(ulog, "actuator_motors")
    if mot is not None:
        t_m = sec(mot)
        motors = [mot[f"control[{i}]"] for i in range(4)]
        mot_label = "команда моторам (0…1)"
    else:
        out = ds(ulog, "actuator_outputs")
        t_m = sec(out) if out is not None else None
        motors = [out[f"output[{i}]"] for i in range(4)] if out is not None else None
        mot_label = "выходы на моторы"

    # участок висения: высота > 1 м и вертикальная скорость < 0,2 м/с
    hover = (alt > 1.0) & (np.abs(lp["vz"]) < 0.2)
    st = ds(ulog, "vehicle_status")
    armed_t = None
    if st is not None:
        armed = st["arming_state"] == 2
        armed_t = sec(st)[armed]

    fig, ax = plt.subplots(4, 1, figsize=(11, 12), sharex=True)
    title = f"Полёт project_quad, PX4 SITL — {os.path.basename(path)}"
    fig.suptitle(title, fontsize=13)

    ax[0].plot(t_lp, alt, label="высота (факт)", lw=1.6)
    if sp is not None and "z" in sp:
        z = sp["z"].astype(float)
        ok = np.isfinite(z)
        ax[0].plot(sec(sp)[ok], -z[ok], "--", label="высота (задание)", lw=1.2)
    ax[0].set_ylabel("высота, м")
    ax[0].legend(loc="upper right")

    ax[1].plot(t_lp, lp["x"] - lp["x"][0], label="X (север)")
    ax[1].plot(t_lp, lp["y"] - lp["y"][0], label="Y (восток)")
    ax[1].set_ylabel("смещение, м")
    ax[1].legend(loc="upper right")

    ax[2].plot(t_att, roll, label="крен", lw=1.2)
    ax[2].plot(t_att, pitch, label="тангаж", lw=1.2)
    if asp is not None and "q_d[0]" in asp:
        r_d, p_d, _ = quat_to_euler(asp["q_d[0]"], asp["q_d[1]"], asp["q_d[2]"], asp["q_d[3]"])
        ax[2].plot(sec(asp), r_d, ":", label="крен (задание)")
        ax[2].plot(sec(asp), p_d, ":", label="тангаж (задание)")
    ax[2].set_ylabel("угол, °")
    ax[2].legend(loc="upper right", ncol=2)

    if motors is not None:
        for i, m in enumerate(motors):
            ax[3].plot(t_m, m, label=f"мотор {i + 1}", lw=1)
        ax[3].set_ylabel(mot_label)
        ax[3].legend(loc="upper right", ncol=4)
    ax[3].set_xlabel("время, с")
    ax[3].set_xlim(0, float(np.nanmax(t_lp)))

    for a in ax:
        a.grid(alpha=0.3)
        if hover.any():
            a.axvspan(t_lp[hover][0], t_lp[hover][-1], color="green", alpha=0.06)

    OUT_DIR.mkdir(exist_ok=True)
    out = OUT_DIR / f"flight_{Path(path).stem}.png"
    fig.tight_layout()
    fig.savefig(out, dpi=110)

    print(f"Лог:      {path}")
    print(f"График:   {out}")
    if armed_t is not None and len(armed_t):
        print(f"В воздухе/вооружён: {armed_t[0]:.1f} … {armed_t[-1]:.1f} с")
    if hover.any():
        th = t_lp[hover]
        h = alt[hover]
        in_hover = (t_att >= th[0]) & (t_att <= th[-1])
        xy = np.hypot(lp["x"][hover] - lp["x"][hover][0], lp["y"][hover] - lp["y"][hover][0])
        print("Висение (зелёная зона на графике):")
        print(f"  длительность          {th[-1] - th[0]:.1f} с")
        print(f"  высота                {h.mean():.2f} м, разброс ±{h.std():.2f} м")
        print(f"  макс. крен / тангаж   {np.abs(roll[in_hover]).max():.1f}° / {np.abs(pitch[in_hover]).max():.1f}°")
        print(f"  уход по горизонтали   до {xy.max():.2f} м")
    else:
        print("Участка висения (высота > 1 м) в логе не найдено.")


if __name__ == "__main__":
    main()
