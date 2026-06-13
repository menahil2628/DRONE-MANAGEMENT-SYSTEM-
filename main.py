"""
+==============================================================+
|          DRONE MANAGEMENT SYSTEM                             |
|          Parallel & Distributed Computing - PDC Project      |
+==============================================================+
|  HOW TO RUN IN VS CODE                                       |
|  ---------------------                                       |
|  1. Install dependencies:                                    |
|       pip install matplotlib numpy                           |
|                                                              |
|  2. Open folder in VS Code:                                  |
|       File -> Open Folder -> DroneManagementSystem            |
|                                                              |
|  3. Run this file:                                           |
|       Press F5   OR   python main.py in terminal            |
|                                                              |
|  Close the window to stop all processes.                     |
+==============================================================+

FILES IN THIS PROJECT
---------------------
  main.py          <- YOU ARE HERE (run this)
  config.py        <- Settings, constants, colors
  pathfinder.py    <- A* parallel path planning algorithm
  drone_process.py <- Drone OS process + sensor sub-process
  load_balancer.py <- Work-stealing load balancer process
  gui.py           <- Full matplotlib GUI dashboard

PARALLEL COMPUTING CONCEPTS
----------------------------
  1.  Multiprocessing       6 drone processes run simultaneously
  2.  Nested Parallelism    Each drone has a sensor sub-process
  3.  Message Passing       mp.Queue for task & result IPC
  4.  Shared Memory         mp.Manager().dict() read by GUI
  5.  Mutual Exclusion      mp.Lock protects shared dict writes
  6.  Event Signalling      mp.Event for clean shutdown
  7.  Parallel A* Planning  Each drone plans its own path
  8.  Load Balancing        Work-stealing master process
  9.  Master-Worker         LB (master) + Drones (workers)
"""

import multiprocessing as mp
import random
import os
import sys
import time

from config          import NUM_DRONES, NUM_TASKS, GRID_SIZE, DRONE_NAMES, OBSTACLES
from drone_process   import drone_worker
from load_balancer   import load_balancer
from gui             import DroneDashboard


# ----------------------------------------------------------
def make_tasks(seed: int = 99) -> list:
    """Generate NUM_TASKS random mission positions on the grid."""
    rng       = random.Random(seed)
    blocked   = set(OBSTACLES)
    positions = []

    while len(positions) < NUM_TASKS:
        x = rng.randint(2, GRID_SIZE - 2)
        y = rng.randint(2, GRID_SIZE - 2)
        if (x, y) not in blocked and (x, y) not in positions:
            positions.append((x, y))

    return [
        {
            "tid"     : i,
            "x"       : float(p[0]),
            "y"       : float(p[1]),
            "priority": rng.randint(1, 3),  # 1=LOW 2=MED 3=HIGH
        }
        for i, p in enumerate(positions)
    ]


# ----------------------------------------------------------
def banner(tasks: list):
    """Print startup info to the VS Code terminal."""
    hi  = sum(1 for t in tasks if t["priority"] == 3)
    med = sum(1 for t in tasks if t["priority"] == 2)
    lo  = sum(1 for t in tasks if t["priority"] == 1)

    print()
    print("+" + "=" * 60 + "+")
    print("|   [DRONE]  DRONE MANAGEMENT SYSTEM                          |")
    print("|       Parallel & Distributed Computing - PDC Project   |")
    print("+" + "=" * 60 + "+")
    print(f"|  Main PID       : {os.getpid():<42}|")
    print(f"|  CPU Cores      : {mp.cpu_count():<42}|")
    print(f"|  Drone Processes: {NUM_DRONES:<42}|")
    print(f"|  Mission Tasks  : {NUM_TASKS}  (HIGH={hi}  MED={med}  LOW={lo})"
          + " " * (42 - len(f"{NUM_TASKS}  (HIGH={hi}  MED={med}  LOW={lo})")) + "|")
    print(f"|  Grid Size      : {GRID_SIZE}x{GRID_SIZE:<41}|")
    print("+" + "=" * 60 + "+")
    print("|  PDC Concepts:                                         |")
    print("||   + Multiprocessing   (one OS process per drone)       ||")
    print("||   + Nested Processes  (sensor sub-process per drone)   ||")
    print("||   + Message Passing   (mp.Queue  IPC)                  ||")
    print("||   + Shared Memory     (mp.Manager dict)                ||")
    print("||   + Synchronization   (mp.Lock + mp.Event)             ||")
    print("||   + Parallel A*       (path planning per process)      ||")
    print("||   + Work-Stealing     (load balancer process)          ||")
    print("||   + Master-Worker     (LB master + drone workers)      ||")
    print("+" + "=" * 60 + "+")
    print("|  Close the GUI window to stop all processes.           |")
    print("+" + "=" * 60 + "+")
    print()


# ----------------------------------------------------------
def main():
    # Required on Windows for multiprocessing
    mp.freeze_support()

    tasks = make_tasks(seed=99)
    banner(tasks)

    # -- Shared Multiprocessing Infrastructure --------------
    manager      = mp.Manager()
    shared_state = manager.dict()    # SHARED MEMORY  - all drone states
    state_lock   = mp.Lock()         # SYNCHRONISATION - protects shared_state
    stop_event   = mp.Event()        # EVENT SIGNALLING - triggers shutdown
    result_queue = mp.Queue()        # MESSAGE PASSING  - drone -> LB results
    log_queue    = mp.Queue(maxsize=500)

    # One task queue per drone (Load Balancer -> Drone, via IPC)
    task_queues = [mp.Queue(maxsize=15) for _ in range(NUM_DRONES)]

    all_procs = []

    # -- Launch one OS process per drone -------------------
    print("  Launching drone processes...")
    for did in range(NUM_DRONES):
        p = mp.Process(
            target = drone_worker,
            args   = (
                did,
                task_queues[did],
                result_queue,
                shared_state,
                set(OBSTACLES),
                stop_event,
                state_lock,
            ),
            name   = f"Drone-{DRONE_NAMES[did]}",
            daemon = True,
        )
        p.start()
        all_procs.append(p)
        print(f"    +  Drone {DRONE_NAMES[did]:<7s}  started  ->  PID {p.pid}")
        log_queue.put(f"[MAIN] + Drone {DRONE_NAMES[did]} process started  (PID {p.pid})")

    # -- Launch Load Balancer process ----------------------
    print("  Launching load balancer...")
    lb = mp.Process(
        target = load_balancer,
        args   = (
            tasks,
            task_queues,
            result_queue,
            shared_state,
            stop_event,
            state_lock,
            log_queue,
        ),
        name   = "LoadBalancer",
        daemon = True,
    )
    lb.start()
    all_procs.append(lb)
    print(f"    [LB]  Load Balancer  started  ->  PID {lb.pid}")
    log_queue.put(f"[MAIN] [LB] Load Balancer process started  (PID {lb.pid})")

    total = len(all_procs) + 1   # +1 for main process
    nested = NUM_DRONES           # sensor sub-processes
    print(f"\n  [OK]  {total} processes running + {nested} sensor sub-processes")
    print(f"  Total parallel workers: {total + nested}\n")

    # Give processes a moment to initialise
    time.sleep(0.5)

    # -- Launch GUI (blocks until window is closed) ---------
    dash = DroneDashboard(
        shared_state = shared_state,
        state_lock   = state_lock,
        log_queue    = log_queue,
        tasks        = tasks,
        stop_event   = stop_event,
    )
    dash.show(all_procs, manager)


# ----------------------------------------------------------
if __name__ == "__main__":
    main()
