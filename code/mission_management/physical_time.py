"""Physical ROS-time windows with an independent dead-clock watchdog."""
import math
import time

class PhysicalClockError(RuntimeError):
    pass

class PhysicalClock:
    def __init__(self, now, wall=time.monotonic, stall_wall_s=10.0):
        self.now = now
        self.wall = wall
        self.stall_wall_s = stall_wall_s
        self.previous = None
        self.last_advance = wall()

    def read(self):
        sim, wall = self.now(), self.wall()
        if not math.isfinite(sim) or sim < 0:
            raise PhysicalClockError("invalid_simulation_clock")
        if self.previous is not None and sim < self.previous:
            raise PhysicalClockError("simulation_clock_moved_backwards")
        if self.previous is None or sim > self.previous:
            self.last_advance = wall
        elif wall - self.last_advance >= self.stall_wall_s:
            raise PhysicalClockError("simulation_clock_stalled")
        self.previous = sim
        return sim
