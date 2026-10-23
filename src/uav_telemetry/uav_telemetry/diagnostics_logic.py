"""Правила диагностики: какие проблемы видны по данным телеметрии.

Здесь нет ROS, поэтому правила можно проверить обычными тестами.
"""

# Уровни серьёзности проблемы
OK = 0     # всё в порядке
WARN = 1   # предупреждение
ERROR = 2  # серьёзная проблема

LOW_BATTERY_WARN = 20   # ниже этого заряда (%) — предупреждение
LOW_BATTERY_ERROR = 10  # ниже этого заряда (%) — серьёзная проблема


def evaluate(battery_percent, link_state, flight_mode, alerts):
    """Проверяет один набор данных телеметрии.

    Возвращает словарь {код_проблемы: (уровень, текст)}.
    Пустой словарь означает, что проблем нет.
    """
    problems = {}  # сюда складываем найденные проблемы

    # Заряд аккумулятора. -1 значит «неизвестен», такой случай пропускаем
    if 0 <= battery_percent < LOW_BATTERY_ERROR:
        problems['LOW_BATTERY'] = (ERROR, f'Критический заряд аккумулятора: {battery_percent}%')
    elif 0 <= battery_percent < LOW_BATTERY_WARN:
        problems['LOW_BATTERY'] = (WARN, f'Низкий заряд аккумулятора: {battery_percent}%')

    # Связь: 0 — нет связи, 1 — слабая, 2 — нормальная
    if link_state == 0:
        problems['LINK_LOST'] = (ERROR, 'Потеряна связь с наземной системой')
    elif link_state == 1:
        problems['LINK_WEAK'] = (WARN, 'Слабый сигнал связи')

    # Режим возврата: дрон летит обратно на точку взлёта
    if flight_mode == 'RTL':
        problems['MODE_RTL'] = (WARN, 'Режим возврата на точку взлёта (RTL)')

    # Аварийные события из самого сообщения; повторы уже найденных не добавляем
    for code in alerts:
        if code not in problems:
            problems[code] = (WARN, f'Аварийное событие: {code}')

    return problems


def worst_level(problems):
    """Самый высокий уровень среди проблем; если проблем нет — OK."""
    levels = [level for level, _text in problems.values()]
    return max(levels) if levels else OK
