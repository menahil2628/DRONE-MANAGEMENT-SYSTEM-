"""
gui.py
==========================================================
Full GUI Dashboard - Matplotlib Animated Interface
----------------------------------------------------------
Layout:
  -------------------------------------------------------
  -                      -  6 DRONE STATUS CARDS        -
  -   MISSION MAP        --------------------------------
  -   20x20 live grid    - Battery Chart-  CPU Chart    -
  -   drones + paths     --------------------------------
  -   tasks + obstacles  -  EVENT LOG (IPC messages)    -
  -------------------------------------------------------
            STATUS BAR: elapsed | tasks done | battery avg
==========================================================
"""

import time
from collections import deque

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, Circle
from matplotlib.lines import Line2D
from matplotlib.animation import FuncAnimation
from config import (
    GRID_SIZE, NUM_DRONES, NUM_TASKS, HISTORY_LEN,
    DRONE_NAMES, DRONE_COLORS, OBSTACLES,
    PRIORITY_MARKER, PRIORITY_COLOR,
    C, STATUS_COLOR, Status,
)


class DroneDashboard:
    """
    Full Matplotlib GUI dashboard for the Drone Management System.
    Reads drone states from shared memory every frame and redraws.
    """

    def __init__(self, shared_state, state_lock, log_queue, tasks, stop_event):
        self.shared_state = shared_state
        self.state_lock   = state_lock
        self.log_queue    = log_queue
        self.tasks        = tasks
        self.stop_event   = stop_event
        self.start_time   = time.time()

        # Rolling data buffers
        self.batt_hist  = {i: deque([100.0] * HISTORY_LEN, maxlen=HISTORY_LEN)
                           for i in range(NUM_DRONES)}
        self.cpu_hist   = {i: deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN)
                           for i in range(NUM_DRONES)}
        self.trail_buf  = {i: deque(maxlen=35) for i in range(NUM_DRONES)}
        self.log_lines  = deque(
            [">  Drone Management System - PDC Project",
             ">  Parallel processes initialising..."],
            maxlen=22,
        )

        self._theme()
        self._layout()
        self._draw_map()
        self._draw_status_panel()
        self._draw_charts()
        self._draw_log_panel()
        self._draw_title()

    # --------------------------------------------------
    def _theme(self):
        plt.rcParams.update({
            "figure.facecolor": C["bg"],
            "axes.facecolor"  : C["panel"],
            "axes.edgecolor"  : C["border"],
            "text.color"      : C["hi"],
            "axes.labelcolor" : C["hi"],
            "xtick.color"     : C["lo"],
            "ytick.color"     : C["lo"],
            "grid.color"      : C["grid"],
            "grid.linewidth"  : 0.5,
            "font.family"     : "monospace",
            "font.size"       : 8,
        })

    # --------------------------------------------------
    def _layout(self):
        self.fig = plt.figure(figsize=(12, 7), facecolor=C["bg"])
        try:
            self.fig.canvas.manager.set_window_title(
                "Drone Management System  -  Parallel & Distributed Computing (PDC)"
            )
        except Exception:
            pass

        gs = gridspec.GridSpec(
            3, 3,
            figure = self.fig,
            left=0.025, right=0.978,
            top=0.935,  bottom=0.038,
            hspace=0.50, wspace=0.28,
        )
        self.ax_map    = self.fig.add_subplot(gs[:, 0])     # full left column
        self.ax_cards  = self.fig.add_subplot(gs[0, 1:])    # top right
        self.ax_batt   = self.fig.add_subplot(gs[1, 1])     # mid right-left
        self.ax_cpu    = self.fig.add_subplot(gs[1, 2])     # mid right-right
        self.ax_log    = self.fig.add_subplot(gs[2, 1:])    # bottom right

    # --------------------------------------------------
    def _draw_title(self):
        self.fig.text(
            0.5, 0.974,
            "[DRONE]  DRONE MANAGEMENT SYSTEM  -  Parallel & Distributed Computing  (PDC Project)",
            ha="center", fontsize=13.5, fontweight="bold", color=C["blue"],
        )
        self.fig.text(
            0.5, 0.957,
            f"  {NUM_DRONES} Drone Processes  |  {NUM_TASKS} Mission Tasks  "
            f"|  Grid {GRID_SIZE}x{GRID_SIZE}  "
            f"|  Multiprocessing + IPC Queues + Shared Memory + Lock + Event + A*  ",
            ha="center", fontsize=7.8, color=C["lo"],
        )

    # --------------------------------------------------
    def _draw_map(self):
        ax = self.ax_map
        ax.set_facecolor(C["bg"])
        ax.set_xlim(-0.5, GRID_SIZE - 0.5)
        ax.set_ylim(-0.5, GRID_SIZE - 0.5)
        ax.set_aspect("equal")
        ax.set_title(
            "  MISSION MAP  -  Live Parallel Drone Tracking",
            color=C["blue"], fontsize=9.5, pad=7, fontweight="bold",
        )

        # Grid
        for i in range(GRID_SIZE + 1):
            ax.axhline(i - 0.5, color=C["grid"], lw=0.35, alpha=0.8)
            ax.axvline(i - 0.5, color=C["grid"], lw=0.35, alpha=0.8)

        # Obstacles
        for ox, oy in OBSTACLES:
            r = FancyBboxPatch(
                (ox - 0.46, oy - 0.46), 0.92, 0.92,
                boxstyle="round,pad=0.05",
                facecolor=C["obs"], edgecolor=C["obs_edge"], lw=0.7, zorder=3,
            )
            ax.add_patch(r)

        # Base station
        ax.plot(0.5, 0.5, "s", ms=16, color=C["blue"],
                mec=C["hi"], mew=1.8, zorder=7)
        ax.text(0.5, -0.28, "BASE", ha="center", fontsize=7,
                color=C["blue"], fontweight="bold")

        # Task markers
        self.task_art = {}
        for t in self.tasks:
            a, = ax.plot(
                t["x"], t["y"],
                PRIORITY_MARKER[t["priority"]],
                ms=13, color=PRIORITY_COLOR[t["priority"]],
                mec=C["bg"], mew=0.7, zorder=5,
            )
            self.task_art[t["tid"]] = a

        # Drone trails (updated each frame)
        self.trail_lines = {}
        for did in range(NUM_DRONES):
            ln, = ax.plot([], [], "-",
                          color=DRONE_COLORS[did], alpha=0.20, lw=1.5, zorder=4)
            self.trail_lines[did] = ln

        # Drone planned-path dashed lines
        self.path_lines = {}
        for did in range(NUM_DRONES):
            pl, = ax.plot([], [], "--",
                          color=DRONE_COLORS[did], alpha=0.45, lw=1.0, zorder=4)
            self.path_lines[did] = pl

        # Drone body circles + name text
        self.d_circles = {}
        self.d_labels  = {}
        for did in range(NUM_DRONES):
            circ = Circle(
                (1 + did, 1), 0.46,
                facecolor=DRONE_COLORS[did],
                edgecolor="white", lw=1.3, zorder=9, alpha=0.93,
            )
            ax.add_patch(circ)
            self.d_circles[did] = circ

            lbl = ax.text(
                1 + did, 1.65, DRONE_NAMES[did][:3],
                ha="center", fontsize=5.8,
                color="white", fontweight="bold", zorder=10,
            )
            self.d_labels[did] = lbl

        # Map legend
        legend_els = [
            Line2D([0],[0], marker="*", color="w", ms=10,
                   markerfacecolor=C["red"],    label="High Priority Task"),
            Line2D([0],[0], marker="o", color="w", ms=10,
                   markerfacecolor=C["orange"], label="Med Priority Task"),
            Line2D([0],[0], marker="v", color="w", ms=10,
                   markerfacecolor=C["lo"],     label="Low Priority Task"),
            mpatches.Patch(facecolor=C["obs"],  edgecolor=C["obs_edge"],
                           label="Obstacle"),
            mpatches.Patch(facecolor=C["blue"], label="Base Station"),
        ]
        ax.legend(
            handles=legend_els, loc="upper right",
            facecolor=C["card"], edgecolor=C["border"],
            labelcolor=C["hi"], fontsize=6.5, framealpha=0.92,
        )
        ax.tick_params(labelsize=6.5)

    # --------------------------------------------------
    def _draw_status_panel(self):
        ax = self.ax_cards
        ax.set_facecolor(C["panel"])
        ax.set_xlim(0, NUM_DRONES)
        ax.set_ylim(0, 1)
        ax.axis("off")
        ax.set_title(
            "  DRONE STATUS  -  Process Telemetry & Sensor Readings",
            color=C["green"], fontsize=9.5, pad=5, fontweight="bold",
        )

        self.card_bbar  = {}
        self.card_texts = {}
        cw = 1.0

        for did in range(NUM_DRONES):
            cx = did * cw + 0.025

            # Card background
            box = FancyBboxPatch(
                (cx, 0.02), cw - 0.05, 0.96,
                boxstyle="round,pad=0.025",
                facecolor=C["card"],
                edgecolor=DRONE_COLORS[did], lw=1.8, zorder=2,
            )
            ax.add_patch(box)

            # Battery track
            ax.add_patch(FancyBboxPatch(
                (cx + 0.04, 0.055), cw - 0.13, 0.105,
                boxstyle="round,pad=0.01",
                facecolor=C["bg"], zorder=3,
            ))

            # Battery fill bar
            bbar = FancyBboxPatch(
                (cx + 0.04, 0.060), cw - 0.13, 0.095,
                boxstyle="round,pad=0.01",
                facecolor=DRONE_COLORS[did], zorder=4, alpha=0.88,
            )
            ax.add_patch(bbar)
            self.card_bbar[did] = bbar

            # Text placeholders (filled each frame)
            ys   = [0.925, 0.840, 0.750, 0.665, 0.575, 0.490, 0.400, 0.305, 0.215]
            txts = []
            for y in ys:
                t = ax.text(
                    cx + cw / 2, y, "",
                    ha="center", va="center",
                    fontsize=6.1, fontfamily="monospace",
                    color=C["hi"], zorder=5,
                )
                txts.append(t)
            self.card_texts[did] = txts

    # --------------------------------------------------
    def _draw_charts(self):
        xs = list(range(HISTORY_LEN))

        # -- Battery History ---------------------------
        ax = self.ax_batt
        ax.set_facecolor(C["panel"])
        ax.set_title("  Battery Level  (%)", color=C["orange"], fontsize=9, pad=4)
        ax.set_ylim(0, 110)
        ax.set_xlim(0, HISTORY_LEN)
        ax.set_xlabel("Time  ->", fontsize=7)
        ax.grid(True, alpha=0.22)
        ax.axhline(20, color=C["red"], lw=0.9, ls="--", alpha=0.55)
        ax.text(2, 23, "LOW !", color=C["red"], fontsize=6.5)
        self.batt_lines = {}
        for did in range(NUM_DRONES):
            ln, = ax.plot(xs, list(self.batt_hist[did]),
                          color=DRONE_COLORS[did], lw=1.4,
                          label=DRONE_NAMES[did][:3])
            self.batt_lines[did] = ln
        ax.legend(loc="lower left", facecolor=C["card"],
                  edgecolor=C["border"], labelcolor=C["hi"],
                  fontsize=6, ncol=3)

        # -- CPU Load History --------------------------
        ax2 = self.ax_cpu
        ax2.set_facecolor(C["panel"])
        ax2.set_title("  Process CPU Load  (%)", color=C["purple"], fontsize=9, pad=4)
        ax2.set_ylim(0, 110)
        ax2.set_xlim(0, HISTORY_LEN)
        ax2.set_xlabel("Time  ->", fontsize=7)
        ax2.grid(True, alpha=0.22)
        self.cpu_lines = {}
        for did in range(NUM_DRONES):
            ln2, = ax2.plot(xs, list(self.cpu_hist[did]),
                            color=DRONE_COLORS[did], lw=1.4,
                            label=DRONE_NAMES[did][:3])
            self.cpu_lines[did] = ln2
        ax2.legend(loc="upper right", facecolor=C["card"],
                   edgecolor=C["border"], labelcolor=C["hi"],
                   fontsize=6, ncol=3)

    # --------------------------------------------------
    def _draw_log_panel(self):
        ax = self.ax_log
        ax.set_facecolor(C["bg"])
        ax.axis("off")
        ax.set_title(
            "  EVENT LOG  -  IPC Messages  |  Load Balancer  |  Process Events",
            color=C["lblue"], fontsize=9.5, pad=4, fontweight="bold",
        )
        self.log_text = ax.text(
            0.008, 0.97, "",
            transform=ax.transAxes,
            va="top", ha="left",
            fontsize=6.6, fontfamily="monospace",
            color=C["hi"], linespacing=1.6,
        )
        # Bottom status bar
        self.statusbar = self.fig.text(
            0.5, 0.013, "Initialising...",
            ha="center", fontsize=8,
            color=C["lo"],
        )

    # --------------------------------------------------
    #  FRAME UPDATE (called by FuncAnimation every 1/FPS s)
    # --------------------------------------------------
    def _frame(self, _n):
        states  = self._get_states()
        elapsed = time.time() - self.start_time

        # Pull new log lines
        while not self.log_queue.empty():
            try:
                self.log_lines.append(self.log_queue.get_nowait())
            except Exception:
                pass

        self._update_map(states)
        self._update_cards(states)
        self._update_charts(states)
        self._update_log(states, elapsed)

    def _get_states(self):
        out = {}
        try:
            with self.state_lock:
                for did in range(NUM_DRONES):
                    if did in self.shared_state:
                        out[did] = dict(self.shared_state[did])
        except Exception:
            pass
        return out

    # --------------------------------------------------
    def _update_map(self, states):
        for did, s in states.items():
            x      = s.get("x", 0.0)
            y      = s.get("y", 0.0)
            status = s.get("status", Status.IDLE)

            # Trail
            self.trail_buf[did].append((x, y))
            if len(self.trail_buf[did]) > 1:
                self.trail_lines[did].set_data(
                    [p[0] for p in self.trail_buf[did]],
                    [p[1] for p in self.trail_buf[did]],
                )

            # Drone colour by status
            col   = DRONE_COLORS[did]
            alpha = 0.92
            if status == Status.CHARGING:
                col = C["red"];   alpha = 0.70
            elif status == Status.ON_TASK:
                col = C["green"]; alpha = 1.00
            elif status == Status.IDLE:
                alpha = 0.60

            self.d_circles[did].center = (x, y)
            self.d_circles[did].set_facecolor(col)
            self.d_circles[did].set_alpha(alpha)
            self.d_labels[did].set_position((x, y + 0.62))

            # Planned path
            path = s.get("path", [])
            if len(path) > 1:
                self.path_lines[did].set_data(
                    [p[0] for p in path],
                    [p[1] for p in path],
                )
            else:
                self.path_lines[did].set_data([], [])

    # --------------------------------------------------
    def _update_cards(self, states):
        cw = 1.0
        for did in range(NUM_DRONES):
            s    = states.get(did, {})
            txts = self.card_texts[did]
            batt = s.get("battery", 100.0)
            stat = s.get("status", "...")
            cx   = did * cw + 0.025

            scol = STATUS_COLOR.get(stat, C["lo"])
            bcol = (
                C["red"]    if batt < 25 else
                C["orange"] if batt < 50 else
                C["green"]
            )

            txts[0].set_text(DRONE_NAMES[did])
            txts[0].set_color(DRONE_COLORS[did])
            txts[0].set_fontweight("bold")
            txts[0].set_fontsize(9.5)

            txts[1].set_text(stat)
            txts[1].set_color(scol)
            txts[1].set_fontsize(7)

            txts[2].set_text(f"Tasks Done: {s.get('done', 0)}")
            txts[2].set_color(C["green"])

            txts[3].set_text(f"Battery: {batt:.0f}%")
            txts[3].set_color(bcol)

            txts[4].set_text(f"Temp:   {s.get('temp', 0):.1f} -C")
            txts[4].set_color(C["lo"])

            txts[5].set_text(f"Alt:    {s.get('alt', 0):.1f} m")
            txts[5].set_color(C["lo"])

            txts[6].set_text(f"Signal: {s.get('signal', 0):.1f}%")
            txts[6].set_color(C["lo"])

            txts[7].set_text(f"Wind:   {s.get('wind', 0):.1f} km/h")
            txts[7].set_color(C["lo"])

            txts[8].set_text(f"PID: {s.get('pid', '---')}")
            txts[8].set_color(C["purple"])
            txts[8].set_fontsize(5.8)

            # Update battery bar width
            max_w = cw - 0.13
            self.card_bbar[did].set_width(max(0.003, (batt / 100.0) * max_w))
            self.card_bbar[did].set_facecolor(bcol)

    # --------------------------------------------------
    def _update_charts(self, states):
        xs = list(range(HISTORY_LEN))
        for did in range(NUM_DRONES):
            s = states.get(did, {})
            self.batt_hist[did].append(s.get("battery", 100.0))
            self.cpu_hist[did].append(s.get("cpu", 0.0))
            self.batt_lines[did].set_data(xs, list(self.batt_hist[did]))
            self.cpu_lines[did].set_data(xs, list(self.cpu_hist[did]))

    # --------------------------------------------------
    def _update_log(self, states, elapsed):
        self.log_text.set_text("\n".join(list(self.log_lines)[-22:]))

        done    = sum(s.get("done", 0) for s in states.values())
        avg_b   = sum(s.get("battery", 100) for s in states.values()) / max(1, len(states))
        moving  = sum(1 for s in states.values() if s.get("status") == Status.MOVING)
        on_task = sum(1 for s in states.values() if s.get("status") == Status.ON_TASK)
        charge  = sum(1 for s in states.values() if s.get("status") == Status.CHARGING)

        self.statusbar.set_text(
            f"[TIME]  {elapsed:>6.1f}s   |   "
            f"Tasks Done: {done}/{NUM_TASKS}   |   "
            f"Avg Battery: {avg_b:.1f}%   |   "
            f"[DRONE] Moving: {moving}   |   "
            f"[PROC] On-Task: {on_task}   |   "
            f"[BATT] Charging: {charge}"
        )

    # --------------------------------------------------
    def show(self, processes, mp_manager):
        """Start the animation and open the window."""
        self.fig.canvas.mpl_connect(
            "close_event",
            lambda e: self._shutdown(e, processes, mp_manager),
        )
        plt.tight_layout(rect=[0, 0.036, 1, 0.944])

        self._anim = FuncAnimation(
            self.fig,
            self._frame,
            interval=1000 // 20,    # 20 FPS
            blit=False,
            cache_frame_data=False,
        )
        plt.show()

    # --------------------------------------------------
    def _shutdown(self, _event, processes, mp_manager):
        print("\n[GUI] Window closed - shutting down all processes...")
        self.stop_event.set()
        time.sleep(0.4)
        for p in processes:
            if p.is_alive():
                p.terminate()
                print(f"[GUI] Terminated: {p.name}  (PID {p.pid})")
        try:
            mp_manager.shutdown()
        except Exception:
            pass
        print("[GUI] All done. Goodbye!")
