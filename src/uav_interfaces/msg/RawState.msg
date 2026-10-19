# Сырое состояние БПЛА от автопилота (или от test_data_player). Тема: /uav/raw_state
# То же, что Telemetry, но без uav_id и mission_status: их добавляет telemetry_node
std_msgs/Header header        # метка времени
float64 latitude              # широта, градусы (WGS84)
float64 longitude             # долгота, градусы (WGS84)
float32 altitude_m            # высота, метры
float32 speed_mps             # скорость, м/с
float32 roll_deg              # крен, градусы
float32 pitch_deg             # тангаж, градусы
float32 yaw_deg               # курс, градусы
float32 battery_voltage_v     # напряжение аккумулятора, В
int8 battery_percent          # заряд 0-100 (-1 если неизвестен)
uint8 link_state              # связь: 0 нет, 1 слабая, 2 нормальная
string flight_mode            # режим: MANUAL, HOLD, AUTO, RTL
string[] alerts               # аварийные события, пустой список = норма
