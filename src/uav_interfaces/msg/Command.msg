# Команда автопилоту. Тема: /uav/command
# Команду принимает узел autopilot_bridge и передаёт автопилоту (в заглушке только пишет в журнал)
std_msgs/Header header        # метка времени

# Коды команд (поле command)
uint8 ARM=0                   # включить моторы (arm)
uint8 DISARM=1                # выключить моторы (disarm)
uint8 TAKEOFF=2               # взлёт на высоту по умолчанию (задаётся в PX4, MIS_TAKEOFF_ALT)
uint8 LAND=3                  # посадка
uint8 RTL=4                   # возврат на точку взлёта
uint8 SET_MODE=5              # сменить режим полёта (имя режима в поле mode)

uint8 command                 # какая команда: одно из значений выше
string mode                   # для SET_MODE: STABILIZED, ALTITUDE, POSITION, HOLD, MISSION, OFFBOARD, ...
