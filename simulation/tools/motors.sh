#!/usr/bin/env bash
# Ручная проверка моторов без автопилота (Gazebo должен быть запущен, ▶ нажата).
#
#   bash tools/motors.sh hover    — обороты висения (дрон почти висит, потом уплывает)
#   bash tools/motors.sh up       — на 10% больше висения: взлёт
#   bash tools/motors.sh stop     — остановить моторы
#   bash tools/motors.sh 1200     — все 4 мотора на 1200 рад/с
#   bash tools/motors.sh 1000,1100,1000,1100 — каждому мотору своё значение
#
# Порядок моторов PX4: 1 передний правый, 2 задний левый, 3 передний левый, 4 задний правый.
# Стабилизации нет, поэтому дрон со временем уходит в сторону — это нормально.

DIR="$(cd "$(dirname "$0")" && pwd)"
TOPIC=$(gz topic -l | grep 'command/motor_speed$' | head -1)
if [ -z "$TOPIC" ]; then
  echo "Не найден топик моторов. Запущен ли Gazebo с моделью project_quad?"
  exit 1
fi

HOVER=$(tr -d ' \n' < "$DIR/hover_speeds.txt")
case "$1" in
  hover) SPEEDS="$HOVER" ;;
  up)    SPEEDS=$(echo "$HOVER" | awk -F, '{printf "%d,%d,%d,%d", $1*1.1, $2*1.1, $3*1.1, $4*1.1}') ;;
  stop)  SPEEDS="0,0,0,0" ;;
  "")    echo "Укажите: hover | up | stop | число | 4 числа через запятую"; exit 1 ;;
  *,*)   SPEEDS="$1" ;;
  *)     SPEEDS="$1,$1,$1,$1" ;;
esac

echo "Топик: $TOPIC"
echo "Обороты моторов 1-4: $SPEEDS рад/с"
gz topic -t "$TOPIC" --msgtype gz.msgs.Actuators -p "velocity:[$SPEEDS]"
