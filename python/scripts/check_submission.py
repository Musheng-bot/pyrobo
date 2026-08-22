#!/usr/bin/env python3
"""Reject changes inside the protected Python implementation directory."""

from __future__ import annotations

import argparse
import os
import subprocess
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[2]
PROTECTED_DIRECTORY = "python/"


def run_git(args: list[str]) -> list[str]:
    result = subprocess.run(
        ["git", *args],
        cwd=ROOT_DIR,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return [line.strip().replace("\\", "/") for line in result.stdout.splitlines() if line.strip()]


def is_protected(path: str) -> bool:
    normalized = path.replace("\\", "/").lstrip("./")
    return (
        normalized == PROTECTED_DIRECTORY.rstrip("/")
        or normalized.startswith(PROTECTED_DIRECTORY)
    )


def changed_files(base: str | None) -> set[str]:
    files: set[str] = set()

    if base:
        files.update(run_git(["diff", "--name-only", "--diff-filter=ACMRTUXB", f"{base}...HEAD"]))

    files.update(run_git(["diff", "--name-only", "--diff-filter=ACMRTUXB"]))
    files.update(run_git(["diff", "--cached", "--name-only", "--diff-filter=ACMRTUXB"]))
    files.update(run_git(["ls-files", "--others", "--exclude-standard"]))
    return files


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Ensure the protected Python directory is unchanged."
    )
    parser.add_argument(
        "--base",
        default=os.environ.get("PYROBO_BASE_REF"),
        help=(
            "Starter commit/tag/ref used to detect committed changes. "
            "Can also be set with PYROBO_BASE_REF."
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    illegal = sorted(path for path in changed_files(args.base) if is_protected(path))

    if illegal:
        print("Submission check failed: the Python directory must not be modified:")
        print(f"  - {PROTECTED_DIRECTORY}")
        print("\nIllegal modified files:")
        for path in illegal:
            print(f"  - {path}")
        print(
            "\nRestore those files to the starter version, or run the checker with "
            "--base <starter-ref> during grading to catch committed changes."
        )
        return 1

    print("Submission check passed: the Python directory is unchanged.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
