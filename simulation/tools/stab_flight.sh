#!/usr/bin/env bash
# Полёт со ВСТРОЕННЫМ стабилизатором Gazebo (запасной вариант, без PX4).
#
# Окно 1:  cd ~/simulation && gz sim -r worlds/test_field_stab.sdf
# Окно 2:  bash ~/simulation/tools/stab_flight.sh          — весь сценарий:
#              взлёт ~2 м → висение 20 с → посадка
#
# Или по шагам:
#   bash tools/stab_flight.sh up      подъём 0,5 м/с (пока не дадите hover)
#   bash tools/stab_flight.sh hover   висеть на месте
#   bash tools/stab_flight.sh down    спуск 0,3 м/с
#   bash tools/stab_flight.sh off     выключить стабилизатор (моторы стоп!)
NS=project_quad

if ! gz topic -l | grep -q "^/world/test_field/clock"; then
  echo "Gazebo не запущен. Сначала: cd ~/simulation && gz sim -r worlds/test_field_stab.sdf"
  exit 1
fi

vel() {   # вертикальная скорость, м/с
  gz topic -t /$NS/cmd_vel -m gz.msgs.Twist -p "linear: {x: 0, y: 0, z: $1}"
}
enable() {
  gz topic -t /$NS/enable -m gz.msgs.Boolean -p "data: $1"
}

case "$1" in
  up)    enable true; vel 0.5;  echo "Подъём 0,5 м/с" ;;
  hover) enable true; vel 0;    echo "Висение" ;;
  down)  enable true; vel -0.3; echo "Спуск 0,3 м/с" ;;
  off)   enable false;          echo "Стабилизатор выключен" ;;
  "")
    enable true
    echo "[$(date +%T)] Взлёт: подъём 0,5 м/с в течение 4 с";   vel 0.5;  sleep 4
    echo "[$(date +%T)] Висение 20 с";                         vel 0;    sleep 20
    echo "[$(date +%T)] Посадка: спуск 0,3 м/с";                vel -0.3; sleep 9
    vel 0
    echo "[$(date +%T)] Готово. Дрон на земле, стабилизатор включён."
    ;;
  *) echo "Неизвестная команда: $1 (up | hover | down | off)"; exit 1 ;;
esac
