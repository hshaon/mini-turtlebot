#!/usr/bin/env python3
"""CSV metrics logger utilities for ROS-free AI pipeline."""

from __future__ import annotations

import csv
import os
import threading
import time
from datetime import datetime, timezone
from typing import Dict, Iterable, List


def now_mono_us() -> int:
    return int(time.monotonic() * 1_000_000)


def now_wall_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


class CsvMetricLogger:
    """Thread-safe CSV appender with fixed header order."""

    def __init__(self, csv_path: str, fieldnames: Iterable[str]) -> None:
        self.csv_path = os.path.abspath(csv_path)
        self._fieldnames: List[str] = list(fieldnames)
        self._lock = threading.Lock()
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=self._fieldnames)
                writer.writeheader()

    @property
    def path(self) -> str:
        return self.csv_path

    def log(self, row: Dict[str, object]) -> None:
        base = {
            "wall_time_iso": now_wall_iso(),
            "mono_us": now_mono_us(),
        }
        base.update(row)
        out = {k: base.get(k, "") for k in self._fieldnames}
        with self._lock:
            with open(self.csv_path, "a", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=self._fieldnames)
                writer.writerow(out)

