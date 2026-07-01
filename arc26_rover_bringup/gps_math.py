import math


def normalize_heading_deg(angle: float) -> float:
    while angle >= 360.0:
        angle -= 360.0
    while angle < 0.0:
        angle += 360.0
    return angle


def angle_error_deg(target_deg: float, current_deg: float) -> float:
    error = target_deg - current_deg

    while error > 180.0:
        error -= 360.0
    while error < -180.0:
        error += 360.0

    return error


def bearing_between_gps_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    dlon_rad = math.radians(lon2 - lon1)

    y = math.sin(dlon_rad) * math.cos(lat2_rad)
    x = (
        math.cos(lat1_rad) * math.sin(lat2_rad)
        - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon_rad)
    )

    bearing = math.degrees(math.atan2(y, x))
    return normalize_heading_deg(bearing)
