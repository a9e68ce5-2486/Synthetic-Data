# bus_api.py
import json
import urllib.request


def fetch_jsonp(url, timeout=10):
    with urllib.request.urlopen(url, timeout=timeout) as resp:
        raw = resp.read().decode("utf-8", errors="ignore").strip()
    # JSONP: callback({...})
    if raw.startswith("{") or raw.startswith("["):
        return json.loads(raw)
    start = raw.find("{")
    end = raw.rfind("}")
    if start == -1 or end == -1:
        raise ValueError("Invalid JSONP payload")
    return json.loads(raw[start : end + 1])


def decode_polyline(encoded):
    # Google encoded polyline algorithm
    coords = []
    index = 0
    lat = 0
    lng = 0
    length = len(encoded)
    while index < length:
        shift = 0
        result = 0
        while True:
            b = ord(encoded[index]) - 63
            index += 1
            result |= (b & 0x1F) << shift
            shift += 5
            if b < 0x20:
                break
        dlat = ~(result >> 1) if (result & 1) else (result >> 1)
        lat += dlat

        shift = 0
        result = 0
        while True:
            b = ord(encoded[index]) - 63
            index += 1
            result |= (b & 0x1F) << shift
            shift += 5
            if b < 0x20:
                break
        dlng = ~(result >> 1) if (result & 1) else (result >> 1)
        lng += dlng
        coords.append((lat / 1e5, lng / 1e5))
    return coords


def extract_routes(payload):
    # best-effort extraction of encoded polylines or stop coordinates
    routes = []
    if isinstance(payload, dict):
        for key in ("Routes", "routes", "data"):
            if key in payload and isinstance(payload[key], list):
                routes = payload[key]
                break
    elif isinstance(payload, list):
        routes = payload

    polylines = []
    for r in routes:
        if not isinstance(r, dict):
            continue
        enc = (
            r.get("EncodedPolyline")
            or r.get("EncodedLine")
            or r.get("encodedLine")
            or r.get("Polyline")
            or r.get("polyline")
        )
        if enc:
            try:
                polylines.append(decode_polyline(enc))
                continue
            except Exception:
                pass
        stops = r.get("Stops")
        if isinstance(stops, list) and stops:
            pts = []
            for s in stops:
                if not isinstance(s, dict):
                    continue
                lat = s.get("Latitude")
                lon = s.get("Longitude")
                if lat is not None and lon is not None:
                    pts.append((lat, lon))
            if pts:
                polylines.append(pts)
    return polylines


def extract_stops(payload):
    # best-effort extraction of stop coordinates
    stops = []
    routes = []
    if isinstance(payload, dict):
        for key in ("Routes", "routes", "data"):
            if key in payload and isinstance(payload[key], list):
                routes = payload[key]
                break
    elif isinstance(payload, list):
        routes = payload

    for r in routes:
        if not isinstance(r, dict):
            continue
        r_stops = r.get("Stops")
        if not isinstance(r_stops, list):
            continue
        for s in r_stops:
            if not isinstance(s, dict):
                continue
            lat = s.get("Latitude")
            lon = s.get("Longitude")
            if lat is None or lon is None:
                continue
            name = s.get("Name") or s.get("StopName") or s.get("Description") or ""
            stops.append({"lat": lat, "lon": lon, "name": name})
    return stops
