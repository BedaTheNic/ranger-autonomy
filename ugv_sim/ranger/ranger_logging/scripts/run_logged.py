#!/usr/bin/env python3
"""Usable before building the workspace, including for logging build failures."""
from pathlib import Path
import sys

source = Path(__file__).resolve().parent.parent
if (source/'ranger_logging').is_dir():
    sys.path.insert(0, str(source))
from ranger_logging.runner import main

if __name__ == '__main__':
    raise SystemExit(main())
