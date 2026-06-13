"""
config.py
==========================================================
All constants, colors, and settings for the
Drone Management System - PDC Project
==========================================================
"""

# -- Simulation Settings --------------------------------
GRID_SIZE    = 22        # Grid is GRID_SIZE x GRID_SIZE
NUM_DRONES   = 4         # Number of parallel drone processes
NUM_TASKS    = 15        # Total mission tasks
DRONE_SPEED  = 0.22      # Movement speed (grid cells per step)
FPS          = 20        # GUI frames per second
HISTORY_LEN  = 80        # Points shown in rolling charts

# -- Drone Identities -----------------------------------
DRONE_NAMES = ["ALPHA", "BETA", "GAMMA", "DELTA", "EPSILON", "ZETA"]

DRONE_COLORS = [
    "#58A6FF",   # ALPHA   - blue
    "#3FB950",   # BETA    - green
    "#F78166",   # GAMMA   - red/coral
    "#D2A8FF",   # DELTA   - purple
    "#FFA657",   # EPSILON - orange
    "#79C0FF",   # ZETA    - light blue
]

# -- Drone Status Values --------------------------------
class Status:
    IDLE      = "IDLE"
    MOVING    = "MOVING"
    ON_TASK   = "ON TASK"
    CHARGING  = "CHARGING"
    ERROR     = "ERROR"

STATUS_COLOR = {
    Status.IDLE     : "#8B949E",
    Status.MOVING   : "#58A6FF",
    Status.ON_TASK  : "#3FB950",
    Status.CHARGING : "#F78166",
    Status.ERROR    : "#FF4444",
}

# -- GUI Color Palette (Dark Theme) --------------------
C = {
    "bg"        : "#0D1117",   # window background
    "panel"     : "#161B22",   # axis background
    "card"      : "#21262D",   # card / box background
    "border"    : "#30363D",   # subtle borders
    "hi"        : "#F0F6FC",   # high-contrast text
    "lo"        : "#8B949E",   # muted text
    "blue"      : "#58A6FF",
    "green"     : "#3FB950",
    "red"       : "#F78166",
    "purple"    : "#D2A8FF",
    "orange"    : "#FFA657",
    "lblue"     : "#79C0FF",
    "yellow"    : "#E3B341",
    "grid"      : "#1C2128",
    "obs"       : "#2D333B",
    "obs_edge"  : "#444C56",
}

# -- Obstacle Cells on the Grid ------------------------
OBSTACLES = frozenset([
    # Cluster 1 - top-left area
    (4, 4), (4, 5), (5, 4), (5, 5), (5, 6),
    # Cluster 2 - centre
    (10, 9), (10, 10), (11, 9), (11, 10), (11, 11),
    # Cluster 3 - right side
    (16, 4), (16, 5), (17, 4), (17, 5),
    # Cluster 4 - bottom-right
    (15, 15), (15, 16), (16, 15),
    # Cluster 5 - left middle
    (3, 13), (3, 14), (4, 13),
    # Cluster 6 - top-right
    (17, 17), (18, 17), (18, 18),
    # Scattered
    (8, 6), (9, 6), (8, 7),
    (13, 13), (14, 13),
    (6, 17), (7, 17),
])

# -- Task Priority Markers ------------------------------
PRIORITY_MARKER = {1: "v", 2: "o", 3: "*"}
PRIORITY_COLOR  = {
    1: C["lo"],
    2: C["orange"],
    3: C["red"],
}
PRIORITY_LABEL = {1: "LOW", 2: "MED", 3: "HIGH"}
