from .base_condition import BaseCondition


class NavDoneCondition(BaseCondition):
    def __init__(self):
        self._goal_in_progress = True

    def update(self, goal_in_progress: bool = True, **kwargs) -> None:
        self._goal_in_progress = goal_in_progress

    def is_triggered(self) -> bool:
        return not self._goal_in_progress

    def reset(self) -> None:
        self._goal_in_progress = True

    @property
    def status(self) -> dict:
        return {
            'goal_in_progress': self._goal_in_progress,
            'triggered': self.is_triggered()
        }
