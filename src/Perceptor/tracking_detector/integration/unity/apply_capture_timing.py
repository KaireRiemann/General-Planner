#!/usr/bin/env python3
"""Validate all UnitySensors source hashes before applying the capture patch."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tempfile


def install(project, apply=False, backup=None):
    root = Path(__file__).resolve().parent
    pending = []
    for item in json.loads((root/"manifest.json").read_text()):
        target = project/item["path"]
        original = target.read_bytes()
        replacement = (root/item["replacement"]).read_bytes()
        if original == replacement:
            continue
        if hashlib.sha256(original).hexdigest() != item["original_sha256"]:
            raise ValueError("Unexpected source version; no files changed: "+str(target))
        pending.append((target, original, replacement))
    if not apply:
        print("Validated; %d source files need updating" % len(pending))
        return
    if not pending:
        print("Capture timing patch is already installed")
        return
    if backup is None:
        raise ValueError("--backup-dir is required with --apply")
    backup.mkdir(parents=True, exist_ok=False)
    for target,original,_ in pending:
        saved = backup/target.relative_to(project)
        saved.parent.mkdir(parents=True, exist_ok=True)
        saved.write_bytes(original)
    completed = []
    try:
        for target,original,replacement in pending:
            # Preserve modes and replace whole files, never partially truncate.
            with tempfile.NamedTemporaryFile(dir=target.parent, delete=False) as out:
                staged = Path(out.name)
                out.write(replacement)
            try:
                shutil.copymode(target, staged)
                staged.replace(target)
            finally:
                staged.unlink(missing_ok=True)
            completed.append((target,original))
    except Exception:
        for target,original in completed:
            target.write_bytes(original)
        raise
    print("Installed %d files; originals saved in %s" % (len(pending),backup))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("project", type=Path)
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--backup-dir", type=Path)
    options = parser.parse_args()
    install(options.project.resolve(), options.apply, options.backup_dir)
