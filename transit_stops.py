# transit_stops.py
import math
import config
from bus_api import fetch_jsonp, extract_stops


def _haversine_m(a, b, c, d):
    r = 6371000.0
    phi1 = math.radians(a)
    phi2 = math.radians(c)
    dphi = math.radians(c - a)
    dl = math.radians(d - b)
    h = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(h))


def fetch_shuttle_stops(center, radius_m, timeout=10):
    if not config.EVAC_BUS_API_URL:
        return []
    try:
        payload = fetch_jsonp(config.EVAC_BUS_API_URL, timeout=timeout)
        stops = extract_stops(payload)
    except Exception:
        return []

    if not stops:
        return []

    clat, clon = center
    filtered = []
    for s in stops:
        lat = s.get("lat")
        lon = s.get("lon")
        if lat is None or lon is None:
            continue
        if _haversine_m(clat, clon, lat, lon) <= radius_m:
            filtered.append(s)
    return filtered

