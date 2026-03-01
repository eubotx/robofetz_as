from .base_condition import BaseCondition


class TimeoutCondition(BaseCondition):
    def __init__(self, duration: float = 3.0):
        self.duration = duration
        self._elapsed = 0.0

    def update(self, elapsed: float = 0.0, **kwargs) -> None:
        self._elapsed = elapsed

    def is_triggered(self) -> bool:
        return self._elapsed >= self.duration

    def reset(self) -> None:
        self._elapsed = 0.0

    @property
    def status(self) -> dict:
        return {
            'elapsed': self._elapsed,
            'duration': self.duration,
            'triggered': self.is_triggered()
        }
