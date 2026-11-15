#!/usr/bin/env python3
"""
Расчётная проверка стабилизации PX4 для project_quad (без Gazebo и без PX4).

На Python повторён каскад стабилизации PX4: регулятор угла (MC_*_P) →
регулятор угловой скорости (PID × MC_*RATE_K, D по отфильтрованному гироскопу)
→ распределение по моторам с нормировкой как в PX4 → моторы с запаздыванием →
тяга ω² → вращение корпуса. Сравниваются:
  * x500 из PX4 с параметрами по умолчанию (эталон: он летает в PX4 SITL);
  * project_quad с K = 1 (стандартные коэффициенты);
  * project_quad с K из нашего airframe.
Плюс поиск K, при котором начинается раскачка (запас устойчивости).

    python3 tools/analysis/px4_stability.py      → docs/px4_stability.png

Это упрощённая модель (нет шумов, ветра, задержек связи PX4 ↔ Gazebo), она
показывает порядок величин, а не заменяет полёт в Gazebo.
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import G, ROOT, load_airframe, load_model  # noqa: E402

# Параметры PX4 по умолчанию (v1.17): MC_ROLLRATE_P/I/D, MC_YAWRATE_P/I, MC_ROLL_P, MC_YAW_P,
# IMU_GYRO_CUTOFF 40 Гц, IMU_DGYRO_CUTOFF 30 Гц, CA_ROTOR*_CT 6.5
RATE_P = np.array([0.15, 0.15, 0.2])
RATE_I = np.array([0.2, 0.2, 0.1])
RATE_D = np.array([0.003, 0.003, 0.0])
ATT_P = np.array([6.5, 6.5, 2.8])
GYRO_LPF, DGYRO_LPF = 40.0, 30.0


def x500():
    pos = [(0.13, -0.22), (-0.13, 0.20), (0.13, 0.22), (-0.13, -0.20)]   # FLU
    return dict(name="x500, PX4 по умолчанию (эталон)", m=2.064, J=np.diag([0.0216667, 0.0216667, 0.04]),
                pos=pos, dirs=[1, 1, -1, -1], k=8.54858e-6, km=0.016, wmin=150.0, wmax=1000.0,
                tau=(0.0125, 0.025), K=(1.0, 1.0, 1.0), ca_km=0.05)


def ours(K=None, label=None):
    m, af = load_model(), load_airframe()
    pos = [(af[f"CA_ROTOR{i}_PX"], -af[f"CA_ROTOR{i}_PY"]) for i in range(4)]   # FRD -> FLU
    K = K or (af["MC_ROLLRATE_K"], af["MC_PITCHRATE_K"], af["MC_YAWRATE_K"])
    return dict(name=label or f"project_quad, K = {K[0]:g}/{K[1]:g}/{K[2]:g} (наш airframe)",
                m=m["mass"], J=np.diag(np.diag(m["J"])), pos=pos,
                dirs=[1 if af[f"CA_ROTOR{i}_KM"] > 0 else -1 for i in range(4)],
                k=m["k"], km=m["km"], wmin=af["SIM_GZ_EC_MIN1"], wmax=af["SIM_GZ_EC_MAX1"],
                tau=(m["tau_up"], m["tau_down"]), K=K, ca_km=abs(af["CA_ROTOR0_KM"]))


def mixer(v):
    """Псевдообратная матрица и нормировка как в PX4 ControlAllocationPseudoInverse."""
    ct = 6.5
    B = np.array([[y * ct for x, y in v["pos"]], [-x * ct for x, y in v["pos"]],
                  [-d * v["ca_km"] * ct for d in v["dirs"]], [ct] * 4])
    mix = np.linalg.pinv(B)
    nz = lambda c: np.sum(np.abs(c) > 1e-3)
    sc = max(math.sqrt(np.sum(mix[:, i] ** 2) / (nz(mix[:, i]) / 2)) for i in (0, 1))
    mix[:, 0] /= sc
    mix[:, 1] /= sc
    mix[:, 2] /= mix[:, 2].max()
    return mix[:, :3]


def simulate(v, T=5.0):
    dt, cdt = 0.0005, 0.004                 # физика 2 кГц, регулятор 250 Гц (как IMU)
    mix, J, k = mixer(v), v["J"], v["k"]
    K = np.array(v["K"])
    wh = math.sqrt(v["m"] * G / 4 / k)
    u0 = (wh - v["wmin"]) / (v["wmax"] - v["wmin"])
    ang, w = np.zeros(3), np.zeros(3)
    wm, cmd = np.full(4, wh), np.full(4, wh)
    gf, dgf, pg, integ = np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3)
    ag = 1 - math.exp(-2 * math.pi * GYRO_LPF * cdt)
    ad = 1 - math.exp(-2 * math.pi * DGYRO_LPF * cdt)
    log, sat = [], 0
    for n in range(int(T / dt)):
        t = n * dt
        sp = np.radians([10 if t > 0.5 else 0, 10 if t > 2.0 else 0, 30 if t > 3.5 else 0])
        if n % int(cdt / dt) == 0:
            gf += ag * (w - gf)
            dg = (gf - pg) / cdt
            pg = gf.copy()
            dgf += ad * (dg - dgf)
            rsp = np.clip(ATT_P * (sp - ang), -math.radians(220), math.radians(220))
            e = rsp - gf
            integ = np.clip(integ + e * cdt, -0.3, 0.3)
            u = u0 + mix @ (K * (RATE_P * e + RATE_I * integ - RATE_D * dgf))
            sat += int(np.any((u < 0) | (u > 1)))
            cmd = v["wmin"] + (v["wmax"] - v["wmin"]) * np.clip(u, 0, 1)
        tc = np.where(cmd > wm, v["tau"][0], v["tau"][1])
        wm += (cmd - wm) * dt / tc
        F = k * wm ** 2
        tq = np.array([sum(y * F[i] for i, (x, y) in enumerate(v["pos"])),
                       sum(-x * F[i] for i, (x, y) in enumerate(v["pos"])),
                       sum(-v["dirs"][i] * v["km"] * F[i] for i in range(4))])
        w = w + np.linalg.solve(J, tq - np.cross(w, J @ w)) * dt
        ang = ang + w * dt
        if n % 8 == 0:
            log.append((t, *np.degrees(ang)))
    L = np.array(log)
    res = {}
    for ax, t0, t1, ref in [(0, 0.5, 2.0, 10), (1, 2.0, 3.5, 10), (2, 3.5, 5.0, 30)]:
        s = (L[:, 0] > t0) & (L[:, 0] < t1)
        y, tt = L[s, 1 + ax], L[s, 0] - t0
        rise = tt[np.argmax(y >= 0.9 * ref)] if (y >= 0.9 * ref).any() else float("nan")
        tail = y[tt > 0.8]
        res[ax] = ((y.max() - ref) / ref * 100, rise, (tail.max() - tail.min()) / 2)
    return L, res, sat


def margin(v):
    """Минимальный множитель к K крена/тангажа, при котором начинается раскачка."""
    for m in [1, 1.5, 2, 2.5, 3, 4, 5, 6, 8, 10, 13, 16, 20]:
        vv = dict(v, K=(v["K"][0] * m, v["K"][1] * m, v["K"][2]))
        _, res, _ = simulate(vv)
        if max(res[0][2], res[1][2]) > 0.5:
            return m
    return None


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    cases = [x500(), ours(K=(1.0, 1.0, 1.0), label="project_quad, K = 1 (стандартные PX4)"), ours()]
    axes = ["крен", "тангаж", "рыскание"]
    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    print("Ступенька по углу: крен 10°, тангаж 10°, рыскание 30°")
    for v, c in zip(cases, ["#777", "#d62728", "#2ca02c"]):
        L, res, sat = simulate(v)
        mg = margin(v)
        print(f"\n{v['name']}")
        for i in range(3):
            o, r, osc = res[i]
            print(f"  {axes[i]:9s} перерегулирование {o:5.1f}%   до 90% за {r * 1000:4.0f} мс   остаточные колебания ±{osc:.2f}°")
        print(f"  запас: раскачка крена/тангажа при K × {mg}" if mg else "  запас: раскачки нет до K × 20")
        for i in range(3):
            ax[i].plot(L[:, 0], L[:, 1 + i], color=c, lw=1.3, label=v["name"])
    for i, (ref, t0) in enumerate([(10, 0.5), (10, 2.0), (30, 3.5)]):
        ax[i].step([0, t0, 5], [0, ref, ref], where="post", color="k", ls=":", lw=1,
                   label="задание" if i == 0 else None)
        ax[i].set_ylabel(f"{axes[i]}, °")
        ax[i].grid(alpha=0.3)
    ax[0].legend(fontsize=8, loc="lower right")
    ax[2].set_xlabel("время, с")
    fig.suptitle("Расчётная проверка стабилизации PX4 (Python, без Gazebo)")
    fig.tight_layout()
    out = ROOT / "docs" / "px4_stability.png"
    fig.savefig(out, dpi=105)
    print(f"\nГрафик: {out}")


if __name__ == "__main__":
    main()
