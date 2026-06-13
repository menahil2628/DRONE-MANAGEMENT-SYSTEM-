"""
drone_process.py
==========================================================
Drone Process Worker + Sensor Sub-Thread
----------------------------------------------------------
PARALLEL COMPUTING CONCEPTS USED HERE:
  1. Multiprocessing  - each drone is a separate OS process
  2. Nested Threading - each drone spawns a sensor thread
  3. Message Passing  - mp.Queue for tasks and results
  4. Shared Memory    - mp.Manager dict for GUI to read
  5. Synchronization  - mp.Lock for mutual exclusion
  6. Event Signalling - mp.Event for clean shutdown
  7. Parallel A*      - path computed in THIS process
==========================================================
"""
 
import multiprocessing as mp
import threading
import math
import time
import random
import os
 
from config     import DRONE_NAMES, DRONE_SPEED, Status
from pathfinder import astar
 
 
def sensor_worker(drone_id, out_queue, stop):
    """Sensor thread - reads temperature, altitude, signal, wind."""
    rng = random.Random(drone_id * 17 + 3)
    alt = 0.0
    while not stop.is_set():
        alt = max(0.0, alt + rng.uniform(-2.0, 3.5))
        reading = {
            "temp"   : round(18.0 + rng.uniform(-3, 12), 1),
            "alt"    : round(alt, 1),
            "signal" : round(max(50, min(100, 87 + rng.uniform(-20, 13))), 1),
            "wind"   : round(rng.uniform(0, 40), 1),
        }
        try:
            out_queue.put_nowait(reading)
        except Exception:
            pass
        time.sleep(0.2 + rng.uniform(0, 0.15))
 
 
def drone_worker(
    drone_id,
    task_queue,
    result_queue,
    shared_state,
    obstacles,
    stop_event,
    state_lock,
):
    """
    DRONE PROCESS - one per drone, runs in true parallel.
    Uses threading.Thread for sensor (avoids daemon child error).
    """
    rng = random.Random(drone_id * 11 + 7)
    sx  = float(rng.randint(0, 3))
    sy  = float((drone_id * 3) % 18 + 1)
 
    loc = {
        "x": sx, "y": sy,
        "status": Status.IDLE,
        "battery": 100.0,
        "done": 0,
        "path": [],
        "temp": 25.0, "alt": 0.0,
        "signal": 95.0, "wind": 0.0,
        "cpu": 0.0,
        "pid": os.getpid(),
        "name": DRONE_NAMES[drone_id],
    }
 
    # Use threading.Thread instead of mp.Process to avoid
    # "daemonic processes are not allowed to have children" error
    sensor_q      = mp.Queue(maxsize=10)
    sensor_stop   = threading.Event()
    sensor_thread = threading.Thread(
        target=sensor_worker,
        args=(drone_id, sensor_q, sensor_stop),
        daemon=True,
        name=f"Sensor-{DRONE_NAMES[drone_id]}",
    )
    sensor_thread.start()
 
    def read_sensors():
        while not sensor_q.empty():
            try:
                r = sensor_q.get_nowait()
                loc["temp"]   = r["temp"]
                loc["alt"]    = r["alt"]
                loc["signal"] = r["signal"]
                loc["wind"]   = r["wind"]
            except Exception:
                pass
 
    def push_state():
        snap = {k: (v[:] if isinstance(v, list) else v) for k, v in loc.items()}
        try:
            with state_lock:
                shared_state[drone_id] = snap
        except Exception:
            pass
 
    current_task = None
    path_idx     = 0
 
    while not stop_event.is_set():
 
        loc["cpu"] = round(rng.uniform(5, 92), 1)
        read_sensors()
 
        # Pick up task
        if current_task is None and loc["status"] == Status.IDLE:
            try:
                current_task  = task_queue.get_nowait()
                loc["status"] = Status.MOVING
                sx_i = int(round(loc["x"]))
                sy_i = int(round(loc["y"]))
                gx   = int(round(current_task["x"]))
                gy   = int(round(current_task["y"]))
                raw  = astar((sx_i, sy_i), (gx, gy), obstacles)
                loc["path"] = [(float(p[0]), float(p[1])) for p in raw]
                path_idx    = 0
                result_queue.put({
                    "event": "ACCEPTED",
                    "drone_id": drone_id,
                    "drone_name": DRONE_NAMES[drone_id],
                    "task_id": current_task["tid"],
                    "path_len": len(loc["path"]),
                })
            except Exception:
                pass
 
        # Move along path
        if current_task and loc["path"] and path_idx < len(loc["path"]):
            wx, wy = loc["path"][path_idx]
            dx, dy = wx - loc["x"], wy - loc["y"]
            dist   = math.hypot(dx, dy)
            if dist < DRONE_SPEED:
                loc["x"], loc["y"] = wx, wy
                path_idx += 1
            else:
                loc["x"] += (dx / dist) * DRONE_SPEED
                loc["y"] += (dy / dist) * DRONE_SPEED
            loc["battery"] = max(0.0, loc["battery"] - 0.010)
 
        # Reached destination
        if current_task and path_idx >= len(loc["path"]) and loc["path"]:
            loc["status"] = Status.ON_TASK
            push_state()
            time.sleep(rng.uniform(0.6, 1.4))
            loc["done"]   += 1
            loc["battery"] = max(0.0, loc["battery"] - 1.8)
            loc["path"]    = []
            loc["status"]  = Status.IDLE
            result_queue.put({
                "event": "DONE",
                "drone_id": drone_id,
                "drone_name": DRONE_NAMES[drone_id],
                "task_id": current_task["tid"],
                "total_done": loc["done"],
            })
            current_task = None
 
        # Battery
        if loc["battery"] < 20.0 and loc["status"] != Status.CHARGING:
            loc["status"] = Status.CHARGING
            loc["path"]   = []
            current_task  = None
            path_idx      = 0
 
        if loc["status"] == Status.CHARGING:
            loc["battery"] = min(100.0, loc["battery"] + 0.9)
            if loc["battery"] >= 96.0:
                loc["status"] = Status.IDLE
 
        push_state()
        time.sleep(0.05)
 
    sensor_stop.set()
 