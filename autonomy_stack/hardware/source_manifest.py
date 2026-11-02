#!/usr/bin/env python3
"""Record exact package/config inputs in the built image (no credentials/logs)."""
import hashlib
import json
from pathlib import Path
import sys

files = {}
for directory in sys.argv[1:]:
    root = Path(directory)
    for path in sorted(root.rglob('*')):
        if (path.is_file() and '__pycache__' not in path.parts
                and path.suffix not in ('.pyc', '.pyo')):
            files[f'{root.name}/{path.relative_to(root)}'] = hashlib.sha256(path.read_bytes()).hexdigest()
print(json.dumps({'schema': 1, 'files_sha256': files}, indent=2, sort_keys=True))
