"""
load_balancer.py
==========================================================
Load Balancer - Runs as its own OS process
----------------------------------------------------------
PARALLEL COMPUTING CONCEPT:
  Master-Worker Pattern + Work-Stealing Algorithm

  The Load Balancer is the "master".
  Drone processes are the "workers".

  Algorithm each cycle:
    1. Drain result_queue  -> track completions
    2. Read shared_state   -> check each drone's load
    3. Sort by load (ascending)
    4. Assign pending tasks to least-loaded drones
    5. Repeat until all tasks are assigned & done

  Communication:
    - result_queue  - receives DONE/ACCEPTED events (IPC)
    - task_queues[] - sends tasks to each drone (IPC)
    - shared_state  - reads drone status (Shared Memory)
    - log_queue     - sends log messages to GUI
==========================================================
"""

import multiprocessing as mp
import time
import os

from config import DRONE_NAMES, NUM_DRONES, Status


def load_balancer(
    tasks        : list,
    task_queues  : list,      # one mp.Queue per drone
    result_queue : mp.Queue,  # receives events from drones
    shared_state : dict,      # Manager dict - read drone states
    stop_event,
    state_lock,
    log_queue    : mp.Queue,  # sends log lines to GUI
):
    """
    ====================================================
    LOAD BALANCER PROCESS
    ====================================================
    Implements Work-Stealing load distribution.
    Runs continuously as a separate OS process,
    monitoring and re-distributing work across drones.
    ====================================================
    """
    # Sort tasks: HIGH priority (3) first
    pending       = sorted(tasks, key=lambda t: -t["priority"])
    assigned      = {i: 0 for i in range(NUM_DRONES)}
    completed     = 0
    total         = len(tasks)

    log_queue.put(
        f"[LB]  Load Balancer started  |  PID {os.getpid()}"
        f"  |  {total} tasks queued  |  Algorithm: Work-Stealing"
    )

    while not stop_event.is_set():

        # -- Step 1: Drain result events ------------------
        while not result_queue.empty():
            try:
                msg = result_queue.get_nowait()

                if msg["event"] == "DONE":
                    did = msg["drone_id"]
                    assigned[did] = max(0, assigned[did] - 1)
                    completed    += 1
                    log_queue.put(
                        f"[LB OK]  Task #{msg['task_id']:02d} DONE  "
                        f"by Drone {DRONE_NAMES[did]:<7s}  |  "
                        f"Progress: {completed}/{total}"
                    )

                elif msg["event"] == "ACCEPTED":
                    log_queue.put(
                        f"[LB ->]  Task #{msg['task_id']:02d} ACCEPTED "
                        f"by Drone {msg.get('drone_name','-'):<7s}  |  "
                        f"Path: {msg.get('path_len','-')} steps"
                    )
            except Exception:
                pass

        # -- Step 2: Assign pending tasks -----------------
        if pending:
            # Read drone states from shared memory
            try:
                with state_lock:
                    states = {
                        i: dict(shared_state[i])
                        for i in range(NUM_DRONES)
                        if i in shared_state
                    }
            except Exception:
                states = {}

            # Calculate effective load per drone
            load = {}
            for i in range(NUM_DRONES):
                s = states.get(i, {})
                in_flight = 1 if s.get("status") in [
                    Status.MOVING, Status.ON_TASK
                ] else 0
                load[i] = assigned[i] + in_flight

            # Available drones: not charging, load < 2
            available = [
                i for i in range(NUM_DRONES)
                if states.get(i, {}).get("status") not in [
                    Status.CHARGING, Status.ERROR
                ]
                and load.get(i, 0) < 2
            ]

            # Work-Stealing: assign to least-loaded first
            for did in sorted(available, key=lambda i: load.get(i, 0)):
                if not pending:
                    break
                task = pending.pop(0)
                try:
                    task_queues[did].put_nowait(task)   # <- IPC MESSAGE
                    assigned[did] += 1
                    log_queue.put(
                        f"[LB -]  Task #{task['tid']:02d} "
                        f"[{['','LOW','MED','HIGH'][task['priority']]:4s}]  "
                        f"->  Drone {DRONE_NAMES[did]:<7s}  "
                        f"|  Drone load: {load.get(did,0)+1}"
                    )
                except Exception:
                    pending.insert(0, task)   # re-queue on failure

        # -- All tasks assigned and completed -------------
        if not pending and completed >= total:
            log_queue.put(
                f"[LB *]  ALL {total} TASKS COMPLETED!  "
                f"Load Balancer going idle."
            )
            # Wait for shutdown signal
            while not stop_event.is_set():
                time.sleep(0.5)
            break

        time.sleep(0.20)

    log_queue.put("[LB]  Load Balancer terminated.")
