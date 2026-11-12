#!/usr/bin/env python3
"""Archive finalized Ranger rosbag2 sessions on the robot host."""

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import os
from pathlib import Path
import re
import shutil
import stat
import sys
import tarfile
from tempfile import TemporaryDirectory
from urllib.parse import urlsplit
from uuid import uuid4

try:
    import yaml
except ImportError:
    yaml = None


HERE = Path(__file__).resolve().parent
COMPLETE = ".recording-complete"
BASENAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]*")
STORAGE_SUFFIXES = {"mcap": ".mcap", "sqlite3": ".db3"}


@contextmanager
def open_directory(path, parent=None):
    fd = os.open(path, os.O_DIRECTORY | os.O_NOFOLLOW | os.O_RDONLY, dir_fd=parent)
    try:
        yield fd
    finally:
        os.close(fd)


@contextmanager
def open_regular(name, directory):
    fd = os.open(
        name, os.O_NOFOLLOW | os.O_NONBLOCK | os.O_RDONLY, dir_fd=directory
    )
    with os.fdopen(fd, "rb") as stream:
        status = os.fstat(stream.fileno())
        if not stat.S_ISREG(status.st_mode):
            raise ValueError(f"{name} must be a regular file")
        yield stream, status


def split_names(metadata):
    if not isinstance(metadata, dict):
        raise ValueError("metadata.yaml must contain a rosbag2 metadata mapping")
    info = metadata.get("rosbag2_bagfile_information")
    if not isinstance(info, dict):
        raise ValueError("missing rosbag2_bagfile_information in metadata.yaml")
    storage = info.get("storage_identifier")
    if not isinstance(storage, str) or storage not in STORAGE_SUFFIXES:
        raise ValueError("unsupported rosbag2 storage format in metadata.yaml")
    suffix = STORAGE_SUFFIXES[storage]
    names = info.get("relative_file_paths")
    if not isinstance(names, list) or not names:
        raise ValueError("metadata.yaml declares no storage files")
    for name in names:
        if (
            not isinstance(name, str)
            or not BASENAME.fullmatch(name)
            or not name.endswith((suffix, suffix + ".zstd"))
        ):
            raise ValueError(f"invalid rosbag2 storage filename {name!r}")
    if len(set(names)) != len(names):
        raise ValueError("metadata.yaml declares a storage file twice")
    files = info.get("files")
    if files is not None and (
        not isinstance(files, list)
        or any(not isinstance(entry, dict) for entry in files)
        or [entry.get("path") for entry in files] != names
    ):
        raise ValueError("rosbag2 metadata storage lists disagree")
    return names


def add_file(archive, prefix, name, stream, status):
    if status.st_size == 0:
        raise ValueError(f"{name} is empty")
    member = tarfile.TarInfo(f"{prefix}/{name}")
    member.size = status.st_size
    member.mtime = int(status.st_mtime)
    member.mode = 0o644
    stream.seek(0)
    archive.addfile(member, stream)
    latest = os.fstat(stream.fileno())
    if (latest.st_size, latest.st_mtime_ns) != (status.st_size, status.st_mtime_ns):
        raise ValueError(f"{name} changed during export")


def add_session(archive, bags, export_name, session):
    try:
        with open_directory(session, bags) as directory:
            prefix = f"{export_name}/{session}"
            try:
                with open_regular(COMPLETE, directory) as (stream, status):
                    add_file(archive, prefix, COMPLETE, stream, status)
            except FileNotFoundError as error:
                raise ValueError(
                    f"missing {COMPLETE}; stop and finalize the recording first"
                ) from error
            with open_regular("metadata.yaml", directory) as (stream, status):
                names = split_names(yaml.safe_load(stream))
                add_file(archive, prefix, "metadata.yaml", stream, status)
            for name in names:
                with open_regular(name, directory) as (stream, status):
                    add_file(archive, prefix, name, stream, status)
    except (OSError, ValueError, yaml.YAMLError) as error:
        raise ValueError(f"{session}: {error}") from error


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def export_sessions(bags_dir, export_dir, sessions):
    if any(not BASENAME.fullmatch(session) for session in sessions):
        raise ValueError("use session directory names without slashes or spaces")
    if len(set(sessions)) != len(sessions):
        raise ValueError("each session may be selected only once")

    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    export_name = f"ranger-bags-{timestamp}-{uuid4().hex[:8]}"
    archive_name = export_name + ".tar"
    export_dir.mkdir(parents=True, exist_ok=True)
    destination = export_dir / export_name
    with TemporaryDirectory(prefix=".bag-export-", dir=export_dir) as temporary:
        work_dir = Path(temporary) / export_name
        work_dir.mkdir(mode=0o700)
        archive_path = work_dir / archive_name
        with open_directory(bags_dir) as bags, tarfile.open(archive_path, "w") as archive:
            for session in sessions:
                add_session(archive, bags, export_name, session)
        checksum = sha256_file(archive_path)
        (work_dir / (archive_name + ".sha256")).write_text(
            f"{checksum}  {archive_name}\n", encoding="ascii"
        )
        work_dir.rename(destination)
    return destination / archive_name


def stage_export(archive, staging_dir):
    checksum = archive.with_suffix(".tar.sha256")
    folder = archive.parent.name
    destination = staging_dir / folder
    if destination.exists() or destination.is_symlink():
        raise ValueError(f"staging destination already exists: {destination}")
    with TemporaryDirectory(prefix=".bag-stage-", dir=staging_dir) as temporary:
        work_dir = Path(temporary) / folder
        work_dir.mkdir()
        shutil.copyfile(archive, work_dir / archive.name)
        shutil.copyfile(checksum, work_dir / checksum.name)
        work_dir.rename(destination)
    return destination / archive.name


def show_paths(archive, label):
    print(f"{label} archive: {archive}", flush=True)
    print(f"{label} checksum: {archive.with_suffix('.tar.sha256')}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sessions", nargs="+", help="completed bag directory names")
    parser.add_argument("--bags-dir", type=Path, default=HERE / "bags")
    parser.add_argument("--export-dir", type=Path, default=HERE / "bag-exports")
    parser.add_argument("--stage-dir", type=Path, help="existing mounted share or web root")
    parser.add_argument("--download-base-url", help="HTTP(S) URL serving the staging root")
    args = parser.parse_args()
    if args.download_base_url:
        try:
            url = urlsplit(args.download_base_url)
            if (
                not args.stage_dir or url.scheme not in ("http", "https") or not url.hostname
                or url.query or url.fragment or url.username is not None
            ):
                raise ValueError
        except ValueError:
            parser.error("--download-base-url requires --stage-dir and an HTTP(S) base URL")
    if yaml is None:
        parser.exit(1, "bag_export.py requires PyYAML (Ubuntu package: python3-yaml)\n")

    try:
        archive = export_sessions(args.bags_dir, args.export_dir.absolute(), args.sessions)
        show_paths(archive, "Local")
        if args.stage_dir:
            staged = stage_export(archive, args.stage_dir.absolute())
            show_paths(staged, "Staged")
            if args.download_base_url:
                url = f"{args.download_base_url.rstrip('/')}/{staged.parent.name}/{staged.name}"
                print(f"Archive URL: {url}\nChecksum URL: {url}.sha256")
    except (OSError, ValueError, tarfile.TarError) as error:
        parser.exit(1, f"Bag export failed: {error}\n")


if __name__ == "__main__":
    main()
