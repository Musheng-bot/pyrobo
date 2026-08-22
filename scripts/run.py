#!/usr/bin/env python3
"""Build the C++ template if requested, then run the Python simulator."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
BUILD_SCRIPT = ROOT_DIR / "scripts" / "build.py"
CHECK_SCRIPT = ROOT_DIR / "scripts" / "check_submission.py"
MAIN_SCRIPT = ROOT_DIR / "main.py"


def run_command(command: list[str], cwd: Path) -> None:
    print(f"+ {' '.join(command)}", flush=True)
    subprocess.run(command, cwd=cwd, check=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run PyRobo from the repository root."
    )
    parser.add_argument(
        "--skip-build",
        action="store_true",
        help="Start the simulator without compiling the C++ interface first.",
    )
    parser.add_argument(
        "--python-navigation",
        action="store_true",
        help="Use the reference Python navigation instead of the compiled C++ answer.",
    )
    parser.add_argument(
        "--config",
        default="Release",
        choices=("Debug", "Release", "RelWithDebInfo", "MinSizeRel"),
        help="CMake build configuration used when building first.",
    )
    parser.add_argument(
        "--clean",
        action="store_true",
        help="Clean the C++ build directory before building.",
    )
    parser.add_argument(
        "--python",
        default=sys.executable,
        help="Python executable used to run main.py.",
    )
    parser.add_argument(
        "--skip-submission-check",
        action="store_true",
        help="Do not reject changes outside cpp/src/contestant.cpp before running.",
    )
    parser.add_argument(
        "--base",
        help=(
            "Starter commit/tag/ref for the submission check. "
            "Also catches committed non-C++ changes."
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.skip_build and not args.skip_submission_check:
        check_command = [args.python, str(CHECK_SCRIPT)]
        if args.base:
            check_command.extend(["--base", args.base])
        run_command(check_command, ROOT_DIR)

    if not args.skip_build:
        build_command = [args.python, str(BUILD_SCRIPT), "--config", args.config]
        if args.clean:
            build_command.append("--clean")
        if args.skip_submission_check:
            build_command.append("--skip-submission-check")
        if args.base:
            build_command.extend(["--base", args.base])
        run_command(build_command, ROOT_DIR)

    main_command = [args.python, str(MAIN_SCRIPT)]
    if args.python_navigation:
        main_command.append("--python-navigation")
    run_command(main_command, ROOT_DIR)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
