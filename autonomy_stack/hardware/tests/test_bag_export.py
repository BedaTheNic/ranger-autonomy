"""Robot-host export fixtures for MCAP and SQLite rosbag2 directories."""

import hashlib
from pathlib import Path
import subprocess
import sys
import tarfile
from tempfile import TemporaryDirectory
import unittest

import yaml


SCRIPT = Path(__file__).resolve().parents[1] / "bag_export.py"


class BagExportTest(unittest.TestCase):
    def setUp(self):
        temporary = TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        self.bags = root / "bags"
        self.bags.mkdir()
        self.exports = root / "exports"
        self.stage = root / "stage"
        self.stage.mkdir()

    def make_session(self, name, storage="mcap", suffix="mcap", count=2):
        session = self.bags / name
        session.mkdir()
        files = {
            ".recording-complete": b'{"reason":"operator stop"}\n',
            **{f"{name}_{i}.{suffix}": f"storage bytes {i}".encode() for i in range(count)},
        }
        splits = [filename for filename in files if not filename.startswith(".")]
        metadata = {
            "rosbag2_bagfile_information": {
                "storage_identifier": storage,
                "relative_file_paths": splits,
                "files": [{"path": filename} for filename in splits],
            }
        }
        files["metadata.yaml"] = yaml.safe_dump(metadata).encode()
        for filename, content in files.items():
            (session / filename).write_bytes(content)
        return files

    def run_export(self, *args):
        return subprocess.run(
            [
                sys.executable, str(SCRIPT),
                "--bags-dir", str(self.bags), "--export-dir", str(self.exports), *args,
            ],
            check=False, capture_output=True, text=True, timeout=10,
        )

    def assert_rejected(self, session, message):
        result = self.run_export(session)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(message, result.stderr)
        self.assertFalse(list(self.exports.glob("*")))

    def test_multiple_sessions_stage_and_verify_archive(self):
        first = self.make_session("blocked-turn")
        second = self.make_session("test2", "sqlite3", "db3", count=3)
        result = self.run_export(
            "blocked-turn", "test2", "--stage-dir", str(self.stage),
            "--download-base-url", "https://example.test/downloads/",
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        folder, = self.exports.iterdir()
        archive, = folder.glob("*.tar")
        checksum = archive.with_suffix(".tar.sha256")
        expected = {
            f"{folder.name}/{session}/{filename}": content
            for session, files in (("blocked-turn", first), ("test2", second))
            for filename, content in files.items()
        }
        with tarfile.open(archive) as bag:
            self.assertEqual(set(bag.getnames()), set(expected))
            for name, content in expected.items():
                with bag.extractfile(name) as extracted:
                    self.assertEqual(extracted.read(), content)

        digest = hashlib.sha256(archive.read_bytes()).hexdigest()
        self.assertEqual(checksum.read_text(), f"{digest}  {archive.name}\n")
        self.assertEqual((self.stage / folder.name / archive.name).read_bytes(), archive.read_bytes())
        self.assertEqual((self.stage / folder.name / checksum.name).read_text(), checksum.read_text())
        self.assertIn(
            f"Archive URL: https://example.test/downloads/{folder.name}/{archive.name}",
            result.stdout,
        )
        self.assertIn(f"Local archive: {archive}", result.stdout)

    def test_rejects_unfinalized_even_if_metadata_is_present(self):
        self.make_session("active")
        (self.bags / "active" / ".recording-complete").unlink()
        self.assert_rejected("active", "stop and finalize")

    def test_rejects_missing_split_and_removes_partial_output(self):
        self.make_session("missing")
        (self.bags / "missing" / "missing_1.mcap").unlink()
        self.assert_rejected("missing", "missing_1.mcap")

    def test_rejects_symlink_session_and_symlink_split(self):
        self.make_session("original")
        (self.bags / "symlink").symlink_to(self.bags / "original", target_is_directory=True)
        self.assert_rejected("symlink", "symlink")
        split = self.bags / "original" / "original_1.mcap"
        split.unlink()
        split.symlink_to("original_0.mcap")
        self.assert_rejected("original", "original_1.mcap")

    def test_rejects_path_traversal_and_corrupt_metadata(self):
        self.assert_rejected("../blocked-turn", "session directory names")
        self.make_session("traversal")
        metadata = self.bags / "traversal" / "metadata.yaml"
        metadata.write_text(
            "rosbag2_bagfile_information:\n"
            "  storage_identifier: mcap\n"
            "  relative_file_paths: [../hidden.mcap]\n"
        )
        self.assert_rejected("traversal", "invalid rosbag2 storage filename")
        metadata.write_text("rosbag2_bagfile_information: [")
        self.assert_rejected("traversal", "Bag export failed")

    def test_missing_staging_directory_preserves_local_export(self):
        self.make_session("final")
        result = self.run_export("final", "--stage-dir", str(self.stage / "absent"))
        self.assertNotEqual(result.returncode, 0)
        folder, = self.exports.iterdir()
        archive, = folder.glob("*.tar")
        self.assertIn(f"Local archive: {archive}", result.stdout)
        self.assertTrue(archive.with_suffix(".tar.sha256").is_file())
        self.assertFalse(list(self.stage.iterdir()))


if __name__ == "__main__":
    unittest.main()
