#!/usr/bin/env python3
"""Reject contestant changes outside the allowed C++ answer file."""

from __future__ import annotations

import argparse
import os
import subprocess
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_ALLOWED_PATHS = ("cpp/src/contestant.cpp",)


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


def normalize_path(path: str) -> str:
    return path.replace("\\", "/").strip("/")


def is_allowed(path: str, allowed_paths: tuple[str, ...]) -> bool:
    normalized = path.replace("\\", "/").strip("/")
    return normalized in allowed_paths


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
        description="Ensure contestant changes are limited to C++ answer files."
    )
    parser.add_argument(
        "--base",
        default=os.environ.get("PYROBO_BASE_REF"),
        help=(
            "Starter commit/tag/ref used to detect committed changes. "
            "Can also be set with PYROBO_BASE_REF."
        ),
    )
    parser.add_argument(
        "--allow",
        action="append",
        default=[],
        help="Allowed file path. Defaults to cpp/src/contestant.cpp. Can be repeated.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    allowed_paths = tuple(
        normalize_path(path)
        for path in (args.allow if args.allow else DEFAULT_ALLOWED_PATHS)
    )
    illegal = sorted(
        path for path in changed_files(args.base) if not is_allowed(path, allowed_paths)
    )

    if illegal:
        print("Submission check failed: only these paths may be modified:")
        for path in allowed_paths:
            print(f"  - {path}")
        print("\nIllegal modified files:")
        for path in illegal:
            print(f"  - {path}")
        print(
            "\nRestore those files to the starter version, or run the checker with "
            "--base <starter-ref> during grading to catch committed changes."
        )
        return 1

    print("Submission check passed: changes are limited to allowed C++ answer files.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
