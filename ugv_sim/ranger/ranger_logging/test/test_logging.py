import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from ranger_logging.reader import records
from ranger_logging.storage import Parts, Writer, create_session


class LoggingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def test_rotation_keeps_every_record_and_all_previous_parts(self):
        parts = Parts(self.root, 'test', 4096)
        expected = [{'index': i, 'text': 'čćžšđ'*100} for i in range(40)]
        for row in expected:
            parts.write('sensors', row)
        parts.close()
        files = list(self.root.glob('*.jsonl'))
        self.assertGreater(len(files), 4)
        self.assertTrue(all(0 < p.stat().st_size <= 4096 for p in files))
        self.assertEqual(list(records(files)), expected)

    def test_oversized_unicode_record_reassembles_across_parts(self):
        parts = Parts(self.root, 'test', 4096)
        expected = {'large': 'č🌍'*10000, 'values': list(range(4000))}
        parts.write('configuration', expected)
        parts.close()
        files = list(self.root.glob('*.jsonl'))
        self.assertTrue(all(p.stat().st_size <= 4096 for p in files))
        self.assertEqual(list(records(files)), [expected])
        with self.assertRaisesRegex(ValueError, 'Incomplete'):
            list(records(sorted(files)[:-1]))

    def test_reopen_does_not_overwrite_existing_part(self):
        first = Parts(self.root, 'test', 4096)
        first.write('system', {'first': True})
        first.close()
        second = Parts(self.root, 'test', 4096)
        second.stamp = first.stamp
        second.write('system', {'second': True})
        second.close()
        self.assertEqual(list(records(self.root.glob('*.jsonl'))), [{'first': True}, {'second': True}])

    def test_sessions_are_unique_and_grouped_by_date(self):
        a, b = create_session(self.root, 'sim'), create_session(self.root, 'sim')
        self.assertNotEqual(a, b)
        self.assertRegex(a.parent.name, r'^\d{4}-\d{2}-\d{2}$')

    def test_short_writes_are_completed(self):
        parts = Parts(self.root, 'test', 4096)
        parts.write('system', {'index': 0})
        real = parts.files['system']
        class ShortWriter:
            def write(self, data): return real.write(data[:5])
            def close(self): real.close()
        parts.files['system'] = ShortWriter()
        parts.write('system', {'index': 1})
        parts.close()
        self.assertEqual(list(records(self.root.glob('*.jsonl'))), [{'index': 0}, {'index': 1}])

    def test_partial_disk_failure_recovers_into_new_part(self):
        parts = Parts(self.root, 'test', 4096)
        parts.write('system', {'index': 0})
        real = parts.files['system']
        class FailingWriter:
            def write(self, data):
                real.write(data[:3])
                raise OSError('simulated full disk')
            def truncate(self, size): real.truncate(size)
            def close(self): real.close()
        parts.files['system'] = FailingWriter()
        with self.assertRaises(OSError): parts.write('system', {'index': 1})
        parts.write('system', {'index': 2})
        parts.close()
        self.assertEqual(list(records(self.root.glob('*.jsonl'))), [{'index': 0}, {'index': 2}])

    def test_bounded_queue_reports_loss_and_drains(self):
        entered, release = threading.Event(), threading.Event()
        original = Parts.write
        def delayed(instance, *args):
            entered.set()
            release.wait(3)
            return original(instance, *args)
        with patch.object(Parts, 'write', delayed):
            writer = Writer(self.root, 'test', 4096, capacity=1)
            writer.emit('sensors', 'first', {})
            self.assertTrue(entered.wait(1))
            self.assertTrue(writer.emit('sensors', 'queued', {}))
            self.assertFalse(writer.emit('sensors', 'dropped', {}))
            release.set()
            writer.close()
        summary = json.loads(next(self.root.glob('*summary.json')).read_text())
        self.assertTrue(summary['drained'])
        self.assertEqual(summary['dropped'], {'sensors': 1})
        self.assertTrue(any(r.get('event') == 'log_loss' for r in records((self.root/'segments').glob('*.jsonl'))))

    def test_writer_continues_after_io_error(self):
        original = Parts.write
        def fail_once(instance, segment, row):
            if row.get('event') == 'fail': raise OSError('simulated failure')
            return original(instance, segment, row)
        with patch.object(Parts, 'write', fail_once):
            writer = Writer(self.root, 'test', 4096)
            writer.emit('system', 'fail', {})
            writer.emit('system', 'recover', {})
            writer.close()
        summary = json.loads(next(self.root.glob('*summary.json')).read_text())
        self.assertEqual(summary['disk_errors'], 1)
        self.assertTrue(any(r.get('event') == 'recover' for r in records((self.root/'segments').glob('*.jsonl'))))

    def runner(self, script):
        wrapper = Path(__file__).resolve().parents[1]/'scripts/run_logged.py'
        return [sys.executable, str(wrapper), 'test', str(self.root), sys.executable, '-u', '-c', script]

    def test_runner_captures_both_streams_and_preserves_exit(self):
        proc = subprocess.run(self.runner("import sys; print('stdout-marker'); print('stderr-marker',file=sys.stderr); sys.exit(7)"),
                              capture_output=True, text=True, timeout=10)
        self.assertEqual(proc.returncode, 7)
        manifest = next(self.root.glob('*/*/manifest.json'))
        self.assertEqual(json.loads(manifest.read_text())['exit_code'], 7)
        rows = list(records((manifest.parent/'segments').glob('*.jsonl')))
        captured = ''.join(r['data']['text'] for r in rows if r.get('event') == 'output')
        self.assertIn('stdout-marker', captured)
        self.assertIn('stderr-marker', captured)

    def test_runner_forwards_shutdown_and_finishes_manifest(self):
        marker = self.root/'ready'
        code = f"import signal,time,pathlib; signal.signal(signal.SIGTERM, lambda *_: exit(0)); pathlib.Path({str(marker)!r}).touch(); time.sleep(30)"
        proc = subprocess.Popen(self.runner(code), stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        try:
            deadline = time.monotonic()+5
            while not marker.exists() and time.monotonic()<deadline: time.sleep(.02)
            self.assertTrue(marker.exists())
            proc.send_signal(signal.SIGTERM)
            _, stderr = proc.communicate(timeout=8)
            self.assertEqual(proc.returncode, 0, stderr)
        finally:
            if proc.poll() is None: proc.kill(); proc.wait()
        manifest = next(self.root.glob('*/*/manifest.json'))
        self.assertEqual(json.loads(manifest.read_text())['status'], 'finished')


if __name__ == '__main__':
    unittest.main()
