"""
Pipeline run logging service.

Provides a RunLogger that:
  - Tees all stdout to a timestamped log file (captures PDDLStream print output)
  - Configures Python logging with file + console handlers
  - Copies input artefacts (boxel_data.json, problem PDDL) into the log directory
  - Supports three verbosity levels via a single parameter

Usage::

    logger = RunLogger(verbosity='verbose')   # or 'quiet', 'normal'
    ...
    logger.save_artefact('boxel_data.json')
    logger.save_artefact('pddl/problem_debug.pddl')
    ...
    logger.close()

Or as a context manager::

    with RunLogger(verbosity='verbose') as logger:
        ...
"""

import logging
import os
import shutil
import sys
from datetime import datetime
from pathlib import Path

QUIET = logging.WARNING
NORMAL = logging.INFO
VERBOSE = logging.DEBUG

_LEVEL_MAP = {
    'quiet': QUIET,
    'normal': NORMAL,
    'verbose': VERBOSE,
}


class _TeeStream:
    """Duplicates writes to both the original stream and a log file."""

    def __init__(self, original, log_file):
        self.original = original
        self.log_file = log_file

    def write(self, text):
        self.original.write(text)
        self.log_file.write(text)
        self.log_file.flush()

    def flush(self):
        self.original.flush()
        self.log_file.flush()

    def fileno(self):
        return self.original.fileno()

    def isatty(self):
        return self.original.isatty()


class RunLogger:
    """
    Pipeline logging service with verbosity control and persistent output.

    Args:
        verbosity: One of ``'quiet'``, ``'normal'``, ``'verbose'``.
            Controls what appears on the console. The log FILE always
            captures everything (DEBUG and above).
        log_dir: Directory for log files (created if absent).

    Attributes:
        log_path: Path to the current run's log file.
        run_dir:  Per-run subdirectory inside *log_dir* (holds artefacts).
    """

    def __init__(self, verbosity: str = 'normal', log_dir: str = 'logs'):
        self._timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

        self.run_dir = Path(log_dir) / f'run_{self._timestamp}'
        self.run_dir.mkdir(parents=True, exist_ok=True)

        self.log_path = self.run_dir / f'run_{self._timestamp}.log'

        self._log_file = open(self.log_path, 'w', encoding='utf-8')
        self._original_stdout = sys.stdout
        sys.stdout = _TeeStream(self._original_stdout, self._log_file)

        console_level = _LEVEL_MAP.get(verbosity, NORMAL)

        root = logging.getLogger()
        root.setLevel(logging.DEBUG)
        for h in root.handlers[:]:
            root.removeHandler(h)

        fh = logging.StreamHandler(self._log_file)
        fh.setLevel(logging.DEBUG)
        fh.setFormatter(logging.Formatter(
            '%(asctime)s [%(levelname)-7s] %(name)s: %(message)s',
            datefmt='%H:%M:%S'))
        root.addHandler(fh)
        self._file_handler = fh

        ch = logging.StreamHandler(self._original_stdout)
        ch.setLevel(console_level)
        ch.setFormatter(logging.Formatter('[%(levelname)-7s] %(message)s'))
        root.addHandler(ch)
        self._console_handler = ch

        logging.info('Run started  : %s', self._timestamp)
        logging.info('Log file     : %s', self.log_path)
        logging.info('Verbosity    : %s (console), DEBUG (file)', verbosity)

    # ----- artefact saving ---------------------------------------------------

    def save_artefact(self, src_path: str, dest_name: str = None):
        """
        Copy *src_path* into the run directory for reproducibility.

        Args:
            src_path:  Path to the file to copy.
            dest_name: Optional filename override inside the run directory.
        """
        src = Path(src_path)
        if not src.exists():
            logging.warning('Artefact not found, skipping: %s', src)
            return
        dest = self.run_dir / (dest_name or src.name)
        shutil.copy2(src, dest)
        logging.debug('Saved artefact: %s -> %s', src, dest)

    # ----- lifecycle ---------------------------------------------------------

    def close(self):
        """Restore stdout, flush the log file, and remove handlers."""
        logging.info('Run finished : %s',
                     datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))
        logging.info('Full log at  : %s', self.log_path)

        root = logging.getLogger()
        root.removeHandler(self._file_handler)
        root.removeHandler(self._console_handler)

        sys.stdout = self._original_stdout
        self._log_file.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False
