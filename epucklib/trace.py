"""CSV tracing for Webots controllers."""

import csv
from collections.abc import Sequence
from pathlib import Path


def trace_path(name: str, repo_root) -> Path:
    """Path of the trace file for a controller, creating its directory."""
    directory = Path(repo_root) / "analysis" / "traces"
    directory.mkdir(parents=True, exist_ok=True)
    return directory / f"{name}.csv"


class CsvTrace:
    """A CSV writer that flushes after every row.

    Webots is killed rather than shut down at the end of a `--batch` run, so
    anything still sitting in a buffer is simply lost. Flushing each row costs
    nothing at these rates and means a trace is always complete up to the
    moment the simulation stopped.
    """

    def __init__(self, path, fieldnames: Sequence[str]) -> None:
        self._file = open(path, "w", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=list(fieldnames))
        self._writer.writeheader()
        self._file.flush()
        self.closed = False

    def write(self, **row) -> None:
        if self.closed:
            return
        self._writer.writerow(row)
        self._file.flush()

    def close(self) -> None:
        if not self.closed:
            self._file.close()
            self.closed = True

    def __enter__(self):
        return self

    def __exit__(self, *_) -> None:
        self.close()
