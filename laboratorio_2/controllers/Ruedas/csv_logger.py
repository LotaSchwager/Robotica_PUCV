from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


@dataclass
class CsvLogger:
    """Small CSV logger that can add metadata comment lines."""

    path: Path
    fieldnames: list[str]
    flush_every: int = 50

    def __post_init__(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=self.fieldnames)
        self._header_written = False
        self._count = 0

    def write_metadata(self, metadata: dict[str, Any]) -> None:
        """Write metadata as comment lines at the top of the file.

        Note: Call this BEFORE logging any rows if you want it at the very top.
        """
        if self._header_written:
            # Metadata will no longer be at the very top.
            self._file.write("# WARNING: metadata written after header\n")
        for k, v in metadata.items():
            self._file.write(f"# {k}: {v}\n")
        self._file.flush()

    def write_header(self) -> None:
        if not self._header_written:
            self._writer.writeheader()
            self._header_written = True

    def log(self, row: dict[str, Any]) -> None:
        self.write_header()
        self._writer.writerow(row)
        self._count += 1
        if self.flush_every > 0 and (self._count % self.flush_every) == 0:
            self._file.flush()

    def close(self) -> None:
        try:
            self._file.flush()
        finally:
            self._file.close()

    def __enter__(self) -> "CsvLogger":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
