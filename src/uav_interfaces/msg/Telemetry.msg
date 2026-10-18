# Сообщение телеметрии БПЛА (ТЗ, п. 1.9). Тема: /uav/telemetry, частота 10 Гц
std_msgs/Header header        # метка времени измерения
string uav_id                 # идентификатор БПЛА, например "uav_01"
float64 latitude              # широта, градусы (WGS84)
float64 longitude             # долгота, градусы (WGS84)
float32 altitude_m            # высота над точкой взлёта, метры
float32 speed_mps             # скорость, м/с
float32 roll_deg              # крен, градусы
float32 pitch_deg             # тангаж, градусы
float32 yaw_deg               # курс (рыскание), градусы
float32 battery_voltage_v     # напряжение аккумулятора, В
int8 battery_percent          # заряд 0-100 (-1 если неизвестен)
uint8 link_state              # связь: 0 нет, 1 слабая, 2 нормальная
string flight_mode            # режим: MANUAL, HOLD, AUTO, RTL
string mission_status         # статус: IDLE, RUNNING, DONE, ABORTED
string[] alerts               # аварийные события, пустой список = всё в норме
