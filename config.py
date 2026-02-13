# config.py

# Phase 1: CampusResilience (Evacuation POMDP)
EVAC_USE_OSM = True
EVAC_PLACE = "University of Utah, Salt Lake City, Utah, USA"
EVAC_FALLBACK_CENTER = (40.7649368, -111.8421021)  # University of Utah (201 Presidents Circle)
EVAC_RADIUS_M = 1200
EVAC_BUS_ROUTE_RADIUS_M = 1200
EVAC_OSM_TIMEOUT_S = 30
EVAC_OSM_USE_CACHE = True
EVAC_SHOW_STOPS_ONLY = False
EVAC_STOP_SAMPLE_M = 50

EVAC_PED_COUNT = 40
EVAC_CAR_COUNT = 15
EVAC_BUS_COUNT = 8
EVAC_STEP_LIMIT = 600
EVAC_DRAW_EVERY = 5

# Partial observability
EVAC_OBS_RADIUS_M = 80.0
EVAC_OBS_ERROR_WALK = 0.1
EVAC_OBS_ERROR_DRIVE = 0.0

# Hazards
EVAC_BLOCK_PROB = 0.08
EVAC_SNOW_MIN = 0.0
EVAC_SNOW_MAX = 1.0
EVAC_SLOPE_ALPHA = 0.6
EVAC_SNOW_ALPHA = 1.2

# Shelters
EVAC_SHELTER_COUNT = 6

# Speeds (meters per step)
EVAC_SPEED_WALK = 1.4
EVAC_SPEED_CAR = 8.0
EVAC_SPEED_BUS = 6.0

# Bus route
EVAC_BUS_STOPS = 6
EVAC_BUS_API_URL = "https://uofubus.com/Services/JSONPRelay.svc/GetRoutesForMapWithScheduleWithEncodedLine?apiKey=ride1791&isDispatch=false"
EVAC_BUS_SAMPLE_EVERY = 20
EVAC_BUS_SNAP_MAX_M = 250.0
EVAC_SHUTTLE_DWELL_STEPS = 3

# Population composition (Faculty vs Staff) from official figures
EVAC_FACULTY_RATIO = 0.25
EVAC_STAFF_RATIO = 0.75

# Campus bus fleet (estimated for Phase 1)
EVAC_BUS_COUNT = 8
