#!/usr/bin/env python3
import sys
from pathlib import Path
source = Path(__file__).resolve().parents[1]
if (source/'ranger_logging').is_dir():
    sys.path.insert(0, str(source))
from ranger_logging.reader import main
if __name__ == '__main__':
    main()
