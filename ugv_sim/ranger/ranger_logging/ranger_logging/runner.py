"""Tee a project command into a dated session; preserve signals and exit status."""
import codecs
import os
from pathlib import Path
import selectors
import signal
import subprocess
import sys
import time

from .storage import Writer, atomic_json, create_session, max_bytes, times


def main():
    action, root, *command = sys.argv[1:]
    session = create_session(root, action)
    writer = Writer(session, 'command', max_bytes())
    env = dict(os.environ, RANGER_LOG_WRAPPED='1', RANGER_LOG_SESSION_DIR=str(session),
               RANGER_LOG_ROOT=str(Path(root).expanduser().resolve()), ROS_LOG_DIR=str(session/'ros_native'))
    manifest = dict(schema=1, **times(), action=action, command=command, cwd=os.getcwd(),
                    domain=env.get('ROS_DOMAIN_ID'), max_bytes=max_bytes(),
                    timezone='Europe/Zagreb', status='running')
    atomic_json(session/'manifest.json', manifest)
    root = Path(root).expanduser().resolve()
    link = root/f'latest-{action}-d{env.get("ROS_DOMAIN_ID", "0")}'
    temporary = root/(link.name+f'.{os.getpid()}.tmp')
    temporary.symlink_to(session)
    os.replace(temporary, link)
    print(f'Ranger logs: {session}', flush=True)
    child = None
    shutdown_at = None
    def forward(signum, _frame):
        nonlocal shutdown_at
        shutdown_at = shutdown_at or time.monotonic()
        writer.emit('system', 'signal', {'signal': signum})
        if child is not None and child.poll() is None:
            try:
                os.killpg(child.pid, signum)
            except ProcessLookupError:
                pass
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, forward)
    result = 1
    try:
        child = subprocess.Popen(command, env=env, stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE, start_new_session=True)
        writer.emit('system', 'process_start', {'pid': child.pid, 'command': command})
        poller = selectors.DefaultSelector()
        for stream, name in [(child.stdout, 'stdout'), (child.stderr, 'stderr')]:
            poller.register(stream, selectors.EVENT_READ, (name, codecs.getincrementaldecoder('utf-8')('replace')))
        while poller.get_map():
            for key, _ in poller.select(timeout=.2):
                name, decoder = key.data
                data = os.read(key.fd, 16384)
                text = decoder.decode(data, final=not data)
                if text:
                    writer.emit('console', 'output', {'stream': name, 'text': text}, source=action)
                if data:
                    try:
                        target = sys.stdout.buffer if name == 'stdout' else sys.stderr.buffer
                        target.write(data)
                        target.flush()
                    except BrokenPipeError:
                        pass
                else:
                    poller.unregister(key.fileobj)
                    key.fileobj.close()
            if shutdown_at and child.poll() is None and time.monotonic()-shutdown_at > 20:
                os.killpg(child.pid, signal.SIGKILL)
                writer.emit('system', 'forced_shutdown', {'after_seconds': 20}, level='ERROR')
        poller.close()
        result = child.wait()
        result = result if result >= 0 else 128-result
    except Exception as exc:
        writer.emit('system', 'runner_error', {'error': repr(exc)}, level='ERROR')
        raise
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGTERM)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        writer.emit('system', 'process_exit', {'exit_code': result}, level='INFO' if result == 0 else 'ERROR')
        manifest.update(status='finished', exit_code=result, finished=times())
        try:
            atomic_json(session/'manifest.json', manifest)
        finally:
            writer.close()
    return result
