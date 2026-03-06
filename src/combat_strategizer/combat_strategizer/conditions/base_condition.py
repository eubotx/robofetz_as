from abc import ABC, abstractmethod


class BaseCondition(ABC):
    @abstractmethod
    def update(self, **kwargs) -> None:
        pass

    @abstractmethod
    def is_triggered(self) -> bool:
        pass

    @abstractmethod
    def reset(self) -> None:
        pass
