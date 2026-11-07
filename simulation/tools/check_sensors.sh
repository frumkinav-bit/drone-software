#!/usr/bin/env bash
# Проверка тестовых сенсоров project_quad.
# Запускать во втором окне Ubuntu, когда Gazebo открыт и нажата ▶.
#
#   bash tools/check_sensors.sh
#
# Для каждого сенсора ищется топик и ждётся одно сообщение (до 5 с).

check() {
  local name="$1" pattern="$2" show="$3"
  local topic
  topic=$(gz topic -l | grep -E "$pattern" | head -1)
  if [ -z "$topic" ]; then
    printf "  %-14s НЕТ ТОПИКА   (%s)\n" "$name" "$pattern"
    return
  fi
  local msg
  msg=$(timeout 5 gz topic -e -t "$topic" -n 1 2>/dev/null)
  if [ -z "$msg" ]; then
    printf "  %-14s НЕТ ДАННЫХ   %s\n" "$name" "$topic"
  else
    printf "  %-14s OK           %s\n" "$name" "$topic"
    if [ -n "$show" ]; then
      echo "$msg" | grep -E "$show" | head -6 | sed 's/^/                   /'
    fi
  fi
}

echo "Сенсоры project_quad:"
check "IMU"          "imu_sensor/imu$"                     "^  (x|y|z):"
check "Барометр"     "air_pressure_sensor/air_pressure$"   "pressure:"
check "Магнитометр"  "magnetometer_sensor/magnetometer$"   "^  (x|y|z):"
check "GPS"          "navsat_sensor/navsat$"               "(latitude|longitude|altitude)_?(deg)?:"
check "Лидар"        "project_quad/lidar$"                 "^(range_min|range_max|count):"
check "Камера вниз"  "project_quad/camera_down/image$"     "^(width|height):"
check "FPV-камера"   "project_quad/camera_fpv/image$"      "^(width|height):"
echo
echo "FPV-камера по умолчанию выключена (ENABLE_FPV_CAMERA в tools/generate_model.py)."
