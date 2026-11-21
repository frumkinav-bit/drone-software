"""Объединение аварийных событий автопилота и лидара. Здесь нет ROS, поэтому проверяется обычными тестами."""

# Сколько секунд статус лидара считается свежим; если lidar_processor замолчал, его коды не используем
LIDAR_STATUS_MAX_AGE_S = 2.0


def merge_alerts(raw_alerts, lidar_alerts, lidar_age_s, max_age_s=LIDAR_STATUS_MAX_AGE_S):
    """Склеивает коды автопилота и лидара без повторов.

    lidar_age_s — сколько секунд назад пришёл статус лидара (None, если не приходил совсем).
    Устаревший статус не добавляем: иначе после остановки lidar_processor тревога «залипла» бы навсегда.
    Порядок: сначала коды автопилота, потом коды лидара.
    """
    result = list(raw_alerts)
    fresh = lidar_age_s is not None and lidar_age_s <= max_age_s
    if fresh:
        for code in lidar_alerts:
            if code not in result:
                result.append(code)
    return result
