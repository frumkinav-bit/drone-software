"""Общие функции для расчётных проверок: чтение модели и airframe."""
import math
import re
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
MODEL = ROOT / "models" / "project_quad" / "model.sdf"
AIRFRAME = next((ROOT / "px4").glob("*_gz_project_quad"))
STAB_WORLD = ROOT / "worlds" / "test_field_stab.sdf"
G = 9.80665


def _shift(I, m, d):
    return I + m * (d @ d * np.eye(3) - np.outer(d, d))


def load_model():
    """Масса, ЦМ, инерция (всего дрона), роторы и константы моторов из model.sdf."""
    sdf = MODEL.read_text(encoding="utf-8")
    parts, rotors = [], {}
    for name, body in re.findall(r'<link name="([^"]+)">(.*?)</link>', sdf, re.S):
        head = body.split("<inertial>")[0]
        lp = re.search(r"<pose>([^<]+)</pose>", head)
        lpose = np.array([float(v) for v in lp.group(1).split()[:3]]) if lp else np.zeros(3)
        ine = re.search(r"<inertial>(.*?)</inertial>", body, re.S).group(1)
        ip = np.array([float(v) for v in re.search(r"<pose>([^<]+)</pose>", ine).group(1).split()[:3]])
        m = float(re.search(r"<mass>([^<]+)</mass>", ine).group(1))
        g = lambda t: float(re.search(rf"<{t}>([^<]+)</{t}>", ine).group(1))
        I = np.array([[g("ixx"), g("ixy"), g("ixz")], [g("ixy"), g("iyy"), g("iyz")], [g("ixz"), g("iyz"), g("izz")]])
        parts.append((m, lpose + ip, I))
        if name.startswith("rotor_"):
            rotors[int(name.split("_")[1])] = lpose
    M = sum(p[0] for p in parts)
    cg = sum(p[0] * p[1] for p in parts) / M
    J = sum(_shift(p[2], p[0], p[1] - cg) for p in parts)
    num = lambda t: float(re.search(rf"<{t}>([^<]+)</{t}>", sdf).group(1))
    dirs = [1 if d == "ccw" else -1 for d in re.findall(r"<turningDirection>(\w+)</turningDirection>", sdf)]
    return dict(mass=M, cg=cg, J=J, rotors=[rotors[i] for i in sorted(rotors)], dirs=dirs,
                k=num("motorConstant"), km=num("momentConstant"), wmax=num("maxRotVelocity"),
                tau_up=num("timeConstantUp"), tau_down=num("timeConstantDown"))


def load_airframe():
    txt = AIRFRAME.read_text()
    return {k: float(v) for k, v in re.findall(r"param set-default (\w+) (\S+)", txt)}
