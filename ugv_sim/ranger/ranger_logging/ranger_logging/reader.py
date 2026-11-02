"""Read rotated JSONL parts, including records split across part boundaries."""
import argparse
import base64
import hashlib
import json
from pathlib import Path


def records(paths):
    pending = {}
    for path in sorted(paths):
        with Path(path).open(encoding='utf-8') as stream:
            for number, line in enumerate(stream, 1):
                try:
                    row = json.loads(line)
                except ValueError as exc:
                    raise ValueError(f'Invalid JSON: {path}:{number}') from exc
                if row.get('event') != 'record_chunk':
                    yield row
                    continue
                identity = row['record_id']
                state = pending.setdefault(identity, {'sha256': row['sha256'],
                    'count': row['count'], 'parts': {}})
                if state['sha256'] != row['sha256'] or state['count'] != row['count']:
                    raise ValueError(f'Conflicting chunk metadata: {identity}')
                state['parts'][row['index']] = base64.b64decode(row['data'], validate=True)
                if len(state['parts']) == state['count']:
                    payload = b''.join(state['parts'][i] for i in range(state['count']))
                    if hashlib.sha256(payload).hexdigest() != state['sha256']:
                        raise ValueError(f'Checksum mismatch: {identity}')
                    yield json.loads(payload)
                    del pending[identity]
    if pending:
        raise ValueError(f'Incomplete records (interrupted write or missing parts): {list(pending)}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--segment', default='*')
    parser.add_argument('--level', choices=['DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL'])
    parser.add_argument('--event')
    args = parser.parse_args()
    paths = list((args.session/'segments').glob(args.segment+'__*.jsonl'))
    if not paths:
        parser.error('No matching log files in this session')
    try:
        for row in records(paths):
            if args.level and row.get('level') != args.level:
                continue
            if args.event and row.get('event') != args.event:
                continue
            print(json.dumps(row, ensure_ascii=False))
    except BrokenPipeError:
        return
