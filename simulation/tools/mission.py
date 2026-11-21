#!/usr/bin/env python3
"""
Тестовый маршрут в симуляции: WP1 -> WP2 -> WP3 -> WP4 -> домой, посадка.

Точки — маркеры wp1...wp4 в мире test_field (оранжевые столбы с шарами на 5 м).
Скрипт загружает маршрут в PX4 SITL по MAVLink, запускает режим Mission и печатает
ход полёта до посадки.

Окно 1:  cd ~/simulation && gz sim -r worlds/test_field.sdf
Окно 2:  bash ~/simulation/tools/px4_run.sh
         pxh> param set NAV_DLL_ACT 0          (если QGroundControl не подключён)
         дождаться "Ready for takeoff!"
Окно 3:  python3 ~/simulation/tools/mission.py

    python3 tools/mission.py --upload-only   только загрузить (запуск — из QGC
                                             или в pxh: commander mode auto:mission,
                                             затем commander arm)

После посадки — графики: python3 ~/simulation/tools/plot_mission.py
"""
import argparse
import math
import os
import sys
import time

os.environ.setdefault("MAVLINK20", "1")
try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("Нет библиотеки pymavlink. Установите:\n"
             "  pip install --break-system-packages pymavlink")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from route import ROUTE, ALT, SPEED, HOLD, ACC_RADIUS, world_to_latlon  # noqa: E402

# PX4 SITL: канал MAVLink для внешних программ (PX4 слушает 14580, шлёт на 14540)
CONNECTION = "udpin:0.0.0.0:14540"
M = mavutil.mavlink
FRAME = M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT


def build_items():
    """Список пунктов миссии: (название, команда, p1..p4, east, north, alt)."""
    items = [
        ("взлёт", M.MAV_CMD_NAV_TAKEOFF, (0, 0, 0, math.nan), 0.0, 0.0, ALT),
        (f"скорость {SPEED:g} м/с", M.MAV_CMD_DO_CHANGE_SPEED, (1, SPEED, -1, 0), 0.0, 0.0, 0.0),
    ]
    for name, east, north in ROUTE:
        items.append((name, M.MAV_CMD_NAV_WAYPOINT, (HOLD, ACC_RADIUS, 0, math.nan), east, north, ALT))
    items.append(("домой", M.MAV_CMD_NAV_WAYPOINT, (HOLD, ACC_RADIUS, 0, math.nan), 0.0, 0.0, ALT))
    items.append(("посадка", M.MAV_CMD_NAV_LAND, (0, 0, 0, math.nan), 0.0, 0.0, 0.0))
    return items


def send_item(m, seq, item):
    _, cmd, (p1, p2, p3, p4), east, north, alt = item
    lat, lon = world_to_latlon(east, north)
    m.mav.mission_item_int_send(
        m.target_system, m.target_component, seq, M.MAV_FRAME_MISSION if cmd == M.MAV_CMD_DO_CHANGE_SPEED else FRAME, cmd,
        0, 1, p1, p2, p3, p4,
        int(round(lat * 1e7)), int(round(lon * 1e7)), alt,
        M.MAV_MISSION_TYPE_MISSION)


def upload(m, items):
    m.mav.mission_count_send(m.target_system, m.target_component, len(items),
                             M.MAV_MISSION_TYPE_MISSION)
    deadline = time.time() + 15
    while time.time() < deadline:
        msg = m.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                           blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == "MISSION_ACK":
            if msg.type == M.MAV_MISSION_ACCEPTED:
                return True
            sys.exit(f"PX4 не принял маршрут (код {msg.type}). Сообщения PX4 — в окне 2.")
        if 0 <= msg.seq < len(items):
            send_item(m, msg.seq, items[msg.seq])
            deadline = time.time() + 15
    sys.exit("PX4 не ответил при загрузке маршрута.")


def command(m, cmd, *params, wait=5):
    p = list(params) + [0] * (7 - len(params))
    m.mav.command_long_send(m.target_system, m.target_component, cmd, 0, *p)
    deadline = time.time() + wait
    while time.time() < deadline:
        msg = m.recv_match(type=["COMMAND_ACK", "STATUSTEXT"], blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            print("  PX4:", msg.text)
        elif msg.command == cmd:
            return msg.result == M.MAV_RESULT_ACCEPTED
    return False


def heartbeat(m):
    m.mav.heartbeat_send(M.MAV_TYPE_GCS, M.MAV_AUTOPILOT_INVALID, 0, 0, 0)


def monitor(m, items, timeout=600):
    was_armed = False
    reached = set()          # PX4 повторяет MISSION_ITEM_REACHED — печатаем один раз
    max_alt = 0.0
    t0 = time.time()
    last_hb = last_pos = 0.0
    while time.time() - t0 < timeout:
        now = time.time()
        if now - last_hb > 1:
            heartbeat(m)
            last_hb = now
        msg = m.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        typ = msg.get_type()
        if typ == "MISSION_ITEM_REACHED" and 0 <= msg.seq < len(items) \
                and msg.seq not in reached:
            reached.add(msg.seq)
            print(f"[{now - t0:5.1f} с] достигнут пункт {msg.seq}: {items[msg.seq][0]}")
        elif typ == "STATUSTEXT":
            print(f"[{now - t0:5.1f} с] PX4: {msg.text}")
        elif typ == "GLOBAL_POSITION_INT":
            max_alt = max(max_alt, msg.relative_alt / 1000)
        if typ == "GLOBAL_POSITION_INT" and now - last_pos > 3 and was_armed:
            print(f"[{now - t0:5.1f} с]   высота {msg.relative_alt / 1000:4.1f} м")
            last_pos = now
        elif typ == "HEARTBEAT" and msg.get_srcSystem() == m.target_system \
                and msg.type != M.MAV_TYPE_GCS:
            armed = bool(msg.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)
            if armed and not was_armed:
                print(f"[{now - t0:5.1f} с] моторы включены, маршрут начат")
            if was_armed and not armed:
                if max_alt < 1.0:
                    print(f"[{now - t0:5.1f} с] моторы выключены, но дрон НЕ ВЗЛЕТЕЛ "
                          f"(высота не больше {max_alt:.1f} м). Сообщения PX4 — в окне 2.")
                    return False
                print(f"[{now - t0:5.1f} с] посадка, моторы выключены — маршрут выполнен "
                      f"(наибольшая высота {max_alt:.1f} м)")
                return True
            was_armed = armed
    print("Время ожидания вышло.")
    return False


def main():
    ap = argparse.ArgumentParser(description="Тестовый маршрут WP1–WP4 в PX4 SITL")
    ap.add_argument("--upload-only", action="store_true", help="только загрузить маршрут")
    ap.add_argument("--connect", default=CONNECTION, help="адрес MAVLink (по умолчанию %(default)s)")
    args = ap.parse_args()

    items = build_items()
    print("Маршрут (мир test_field, X — восток, Y — север):")
    for i, (name, _, _, e, n, a) in enumerate(items):
        print(f"  {i}: {name:<16} X={e:6.1f}  Y={n:6.1f}  высота {a:.0f} м")

    print(f"\nПодключение к PX4 ({args.connect})...")
    m = mavutil.mavlink_connection(args.connect, source_system=255, source_component=190)
    if m.wait_heartbeat(timeout=30) is None:
        sys.exit("PX4 не найден. Запущен ли px4_run.sh в другом окне?")
    m.target_component = m.target_component or 1
    print(f"PX4 найден (система {m.target_system}).")

    upload(m, items)
    print(f"Маршрут загружен: {len(items)} пунктов.")
    if args.upload_only:
        print("Запуск: в pxh — commander mode auto:mission, затем commander arm.")
        return

    # Начать с первого пункта. PX4 помнит, где закончилась прошлая миссия (даже после
    # перезапуска), и при загрузке такого же маршрута продолжает с того же пункта —
    # например, сразу с посадки. Поэтому явно ставим текущим пункт 0.
    if not command(m, M.MAV_CMD_DO_SET_MISSION_CURRENT, 0, 1):
        m.mav.mission_set_current_send(m.target_system, m.target_component, 0)
        time.sleep(0.5)
    print("Текущий пункт маршрута: 0 (взлёт).")

    # Режим Mission: PX4 custom mode AUTO (4) / MISSION (4), как делает QGroundControl
    if not command(m, M.MAV_CMD_DO_SET_MODE, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 4):
        if not command(m, M.MAV_CMD_MISSION_START, 0, len(items) - 1):
            sys.exit("PX4 не включил режим Mission. Сообщения PX4 — в окне 2.")
    print("Режим Mission включён.")
    if not command(m, M.MAV_CMD_COMPONENT_ARM_DISARM, 1):
        sys.exit("PX4 не включил моторы (не готов к взлёту?). Сообщения — в окне 2.\n"
                 "Если там 'No connection to the GCS' — введите в pxh: param set NAV_DLL_ACT 0")
    monitor(m, items)
    print("\nГрафики: python3 ~/simulation/tools/plot_mission.py")


if __name__ == "__main__":
    main()
