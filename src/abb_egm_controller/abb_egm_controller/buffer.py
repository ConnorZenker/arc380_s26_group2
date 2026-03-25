from __future__ import annotations

from threading import Lock


class Buffer[T]:
    def __init__(self, value: T):
        self._value: T = value
        self._updated: bool = False
        self._lock = Lock()

    def get_value(self) -> T:
        with self._lock:
            if self._updated:
                self._updated = False
            return self._value

    def set_value(self, value: T):
        with self._lock:
            self._value = value
            self._updated = True

    def is_updated(self) -> bool:
        with self._lock:
            return self._updated
