#!/usr/bin/env python3
"""
Расчётная проверка встроенного стабилизатора Gazebo (MulticopterVelocityControl)
на нашей модели — без Gazebo.

Регулятор повторён по исходникам gz-sim 8 (LeeVelocityController), параметры
берутся из worlds/test_field_stab.sdf, масса/инерция/моторы — из model.sdf.
Сценарий как в tools/stab_flight.sh: старт с наклоном 5°, подъём 0,5 м/с 4 с,
висение 20 с, спуск 0,3 м/с.

    python3 tools/analysis/gz_stabilizer.py      → docs/gz_stabilizer.png
"""
import math
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import G, ROOT, STAB_WORLD, load_model  # noqa: E402


def main():
    m = load_model()
    plug = STAB_WORLD.read_text(encoding="utf-8")
    plug = plug[plug.index("MulticopterVelocityControl"):]
    vec = lambda t: np.array([float(x) for x in re.search(rf"<{t}>([^<]+)</{t}>", plug).group(1).split()])
    Kv, Ka, Kw, amax = vec("velocityGain"), vec("attitudeGain"), vec("angularRateGain"), vec("maximumLinearAcceleration")
    M, J, k, km = m["mass"], m["J"], m["k"], m["km"]

    A = np.zeros((4, 4))
    for i, (r, d) in enumerate(zip(m["rotors"], m["dirs"])):
        ang, arm = math.atan2(r[1], r[0]), math.hypot(r[0], r[1])
        A[:, i] = [math.sin(ang) * arm * k, -math.cos(ang) * arm * k, -d * k * km, k]
    moi = np.eye(4)
    moi[:3, :3] = J
    Ainv = A.T @ np.linalg.inv(A @ A.T) @ moi
    nKa, nKw = Ka @ np.linalg.inv(J), Kw @ np.linalg.inv(J)
    grav = np.array([0, 0, -G])

    def ctrl(R, v, w, vcmd):
        acc = Kv * (v - R @ vcmd) / M
        acc = np.minimum(np.abs(acc), amax) * np.sign(acc) + grav
        b1, b3 = R[:, 0], -acc / np.linalg.norm(acc)
        b2 = np.cross(b3, b1)
        b2 /= np.linalg.norm(b2)
        Rd = np.column_stack([np.cross(b2, b3), b2, b3])
        E = 0.5 * (Rd.T @ R - R.T @ Rd)
        eR = np.array([E[2, 1], E[0, 2], E[1, 0]])
        alpha = -(eR * nKa) - (w * nKw)
        return np.sqrt(np.maximum(Ainv @ np.r_[alpha, -M * acc @ R[:, 2]], 0))

    def vcmd(t):
        return np.array([0, 0, 0.5 if 1 <= t < 5 else (-0.3 if 25 <= t < 32 else 0.0)])

    c, s = math.cos(math.radians(5)), math.sin(math.radians(5))
    R = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    P = [r - np.r_[m["cg"][:2], 0] for r in m["rotors"]]
    dt = 0.001
    p, v, w, wm, cmd = np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(4), np.zeros(4)
    log = []
    for n in range(int(34 / dt)):
        t = n * dt
        if n % 4 == 0:
            cmd = np.minimum(ctrl(R, v, w, vcmd(t)), m["wmax"])
        wm += (cmd - wm) * dt / np.where(cmd > wm, m["tau_up"], m["tau_down"])
        F = k * wm ** 2
        tq = sum(np.cross(P[i], [0, 0, F[i]]) for i in range(4)) + np.array([0, 0, sum(-m["dirs"][i] * km * F[i] for i in range(4))])
        v = v + (grav + R @ np.array([0, 0, F.sum()]) / M) * dt
        p = p + v * dt
        if p[2] < 0:
            p[2], v[2] = 0, max(v[2], 0)
        w = w + np.linalg.solve(J, tq - np.cross(w, J @ w)) * dt
        W = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
        u, _, vt = np.linalg.svd(R @ (np.eye(3) + W * dt))
        R = u @ vt
        if n % 20 == 0:
            log.append((t, p[2], math.degrees(math.atan2(R[2, 1], R[2, 2])),
                        math.degrees(-math.asin(np.clip(R[2, 0], -1, 1))), *wm))
    L = np.array(log)
    hov = (L[:, 0] > 8) & (L[:, 0] < 25)
    print(f"Масса {M:.3f} кг, висение {math.sqrt(M * G / 4 / k):.0f} усл. рад/с")
    print(f"Висение 8–25 с: высота {L[hov, 1].mean():.2f} м (±{L[hov, 1].std():.3f}), "
          f"макс. крен {np.abs(L[hov, 2]).max():.2f}°, тангаж {np.abs(L[hov, 3]).max():.2f}°")
    print(f"Начальный наклон 5° убран (крен < 0,5°) за {L[np.argmax(np.abs(L[:, 2]) < 0.5), 0]:.2f} с")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    ax[0].plot(L[:, 0], L[:, 1])
    ax[0].set_ylabel("высота, м")
    ax[1].plot(L[:, 0], L[:, 2], label="крен")
    ax[1].plot(L[:, 0], L[:, 3], label="тангаж")
    ax[1].set_ylabel("угол, °")
    ax[1].legend()
    for i in range(4):
        ax[2].plot(L[:, 0], L[:, 4 + i], lw=1, label=f"мотор {i + 1}")
    ax[2].set_ylabel("обороты, усл. рад/с")
    ax[2].legend(ncol=4, fontsize=8)
    ax[2].set_xlabel("время, с")
    for a in ax:
        a.grid(alpha=0.3)
    fig.suptitle("Расчёт: стабилизатор Gazebo на модели project_quad (Python, без Gazebo)")
    fig.tight_layout()
    out = ROOT / "docs" / "gz_stabilizer.png"
    fig.savefig(out, dpi=105)
    print(f"График: {out}")


if __name__ == "__main__":
    main()
