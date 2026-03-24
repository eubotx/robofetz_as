import math
from .base_condition import BaseCondition


class ProximityCondition(BaseCondition):
    def __init__(self, threshold: float = 0.15, duration: float = 5.0):
        self.threshold = threshold
        self.duration = duration
        self._time_accumulated = 0.0
        self._last_distance = None

    def update(self, robot_pose=None, opponent_pose=None, dt: float = 0.0, **kwargs) -> None:
        if robot_pose is None or opponent_pose is None:
            return

        dx = robot_pose.position.x - opponent_pose.pose.position.x
        dy = robot_pose.position.y - opponent_pose.pose.position.y
        dist = math.sqrt(dx * dx + dy * dy)
        self._last_distance = dist

        if dist <= self.threshold:
            self._time_accumulated += dt

    def is_triggered(self) -> bool:
        return self._time_accumulated >= self.duration

    def reset(self) -> None:
        self._time_accumulated = 0.0

    @property
    def status(self) -> dict:
        return {
            'distance': self._last_distance,
            'time_accumulated': self._time_accumulated,
            'threshold': self.threshold,
            'duration': self.duration,
            'triggered': self.is_triggered()
        }
