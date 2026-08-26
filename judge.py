#!/usr/bin/env python3
"""Run isolated PyRobo submissions against external case files."""

from __future__ import annotations

import argparse
import csv
import ctypes
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path
from typing import Any

import yaml
import numpy as np


ROOT_DIR = Path(__file__).resolve().parent
PYTHON_DIR = ROOT_DIR / "python"
TRUSTED_CPP_DIR = ROOT_DIR / "cpp"
MAX_SCORES = {1: 20, 2: 20, 3: 30, 4: 30}
GOAL_TOLERANCE = 0.25

sys.path.insert(0, str(PYTHON_DIR))
from sim.cpp_navigation import CppNavigation  # noqa: E402
from sim.map import Map  # noqa: E402
from sim.simulator import Simulator  # noqa: E402


class JudgeError(RuntimeError):
    """A user-facing evaluation error."""


def load_yaml(path: Path) -> dict[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as stream:
            value = yaml.safe_load(stream) or {}
    except OSError as error:
        raise JudgeError(f"cannot read case file {path}: {error}") from error
    if not isinstance(value, dict):
        raise JudgeError(f"case file must contain a mapping: {path}")
    return value


def resolve_case_path(case_file: Path, value: Any, field: str) -> Path:
    if not isinstance(value, str) or not value:
        raise JudgeError(f"{field} must be a relative path in {case_file}")
    relative = Path(value)
    if relative.is_absolute() or ".." in relative.parts:
        raise JudgeError(f"{field} must stay below {case_file.parent}")
    resolved = (case_file.parent / relative).resolve()
    try:
        resolved.relative_to(case_file.parent.resolve())
    except ValueError as error:
        raise JudgeError(f"{field} escapes the case directory") from error
    if not resolved.is_file():
        raise JudgeError(f"{field} does not exist: {resolved}")
    return resolved


def safe_extract(archive: Path, destination: Path) -> None:
    with zipfile.ZipFile(archive) as source:
        root = destination.resolve()
        for member in source.infolist():
            relative = Path(member.filename)
            if relative.is_absolute() or ".." in relative.parts:
                raise JudgeError(f"unsafe path in submission archive: {member.filename}")
            target = (root / relative).resolve()
            try:
                target.relative_to(root)
            except ValueError as error:
                raise JudgeError(f"unsafe path in submission archive: {member.filename}") from error
            if member.is_dir():
                target.mkdir(parents=True, exist_ok=True)
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            with source.open(member) as input_file, target.open("wb") as output_file:
                shutil.copyfileobj(input_file, output_file)


def build_submission(submission_root: Path, question: int, work_root: Path) -> Path:
    candidate_root = submission_root / f"ex{question}"
    source_root = candidate_root / "cpp" / "src"
    for name in ("contestant.cpp", "planner.cpp"):
        if not (source_root / name).is_file():
            raise JudgeError(f"missing {name} in {candidate_root}")

    cpp_root = work_root / "cpp"
    shutil.copytree(TRUSTED_CPP_DIR, cpp_root)
    for name in ("contestant.cpp", "planner.cpp"):
        shutil.copy2(source_root / name, cpp_root / "src" / name)

    build_root = work_root / "build"
    configure = [
        "cmake", "-S", str(cpp_root), "-B", str(build_root),
        "-DCMAKE_BUILD_TYPE=Release",
    ]
    compile = [
        "cmake", "--build", str(build_root), "--config", "Release",
        "--parallel",
    ]
    logs: list[str] = []
    for command in (configure, compile):
        try:
            result = subprocess.run(
                command, cwd=ROOT_DIR, check=False, capture_output=True,
                text=True, encoding="utf-8", errors="replace",
            )
        except OSError as error:
            raise JudgeError(f"cannot run {' '.join(command)}: {error}") from error
        logs.append(f"$ {' '.join(command)}\n{result.stdout}{result.stderr}")
        if result.returncode != 0:
            (work_root / "build.log").write_text("\n".join(logs), encoding="utf-8")
            raise JudgeError(f"submission build failed; see {work_root / 'build.log'}")

    candidates = [
        build_root / "pyrobo_contestant.dll",
        build_root / "libpyrobo_contestant.dll",
        build_root / "Release" / "pyrobo_contestant.dll",
        build_root / "Release" / "libpyrobo_contestant.dll",
        build_root / "libpyrobo_contestant.so",
        build_root / "libpyrobo_contestant.dylib",
    ]
    for library in candidates:
        if library.is_file():
            (work_root / "build.log").write_text("\n".join(logs), encoding="utf-8")
            return library
    (work_root / "build.log").write_text("\n".join(logs), encoding="utf-8")
    raise JudgeError(f"compiled library was not found in {build_root}")


def map_settings(case_file: Path, case: dict[str, Any]) -> tuple[Map, Map, dict[str, Any]]:
    map_config = case.get("map")
    robot_config = case.get("robot")
    if not isinstance(map_config, dict) or not isinstance(robot_config, dict):
        raise JudgeError(f"{case_file} must define map and robot")

    display_file = resolve_case_path(case_file, map_config.get("display"), "map.display")
    real_file = resolve_case_path(
        case_file, map_config.get("real", map_config.get("display")), "map.real"
    )
    resolution = float(map_config.get("resolution", 0.05))
    origin = (
        float(map_config.get("origin_x", 0.0)),
        float(map_config.get("origin_y", 0.0)),
    )
    display_map = Map(display_file, resolution=resolution, origin=origin)
    real_map = Map(real_file, resolution=resolution, origin=origin)

    initial = robot_config.get("initial")
    goal = robot_config.get("goal")
    if not isinstance(initial, dict) or not isinstance(goal, dict):
        raise JudgeError(f"{case_file} must define robot.initial and robot.goal")
    initial_pose = (
        float(initial["x"]), float(initial["y"]), float(initial.get("yaw", 0.0))
    )
    goal_pose = (
        float(goal["x"]), float(goal["y"]), float(goal.get("yaw", 0.0))
    )
    settings = {
        "time_step": float(case.get("time_step", 0.05)),
        "timeout": float(case.get("timeout_seconds", 60.0)),
        "robot_radius": float(robot_config.get("radius", 0.3)),
        "initial": initial_pose,
        "goal": goal_pose,
        "control": case.get("control"),
    }
    return display_map, real_map, settings


class EsdfDistance:
    """Distance from a point to the nearest occupied map cell."""

    def __init__(self, world_map: Map):
        rows, columns = np.nonzero(~world_map.data)
        height, _ = world_map.shape
        self._min_x = world_map.origin[0] + columns * world_map.resolution
        self._max_x = self._min_x + world_map.resolution
        from_bottom = height - rows - 1
        self._min_y = world_map.origin[1] + from_bottom * world_map.resolution
        self._max_y = self._min_y + world_map.resolution

    def distance(self, x: float, y: float) -> float:
        if self._min_x.size == 0:
            return math.inf
        dx = np.maximum(self._min_x - x, 0.0)
        dx = np.maximum(dx, x - self._max_x)
        dy = np.maximum(self._min_y - y, 0.0)
        dy = np.maximum(dy, y - self._max_y)
        return float(np.hypot(dx, dy).min())


def sample_path(path: list[tuple[float, float]], step: float) -> list[tuple[float, float]]:
    if not path:
        return []
    samples = [path[0]]
    for begin, end in zip(path, path[1:]):
        length = math.hypot(end[0] - begin[0], end[1] - begin[1])
        count = max(1, math.ceil(length / step))
        samples.extend(
            (
                begin[0] + (end[0] - begin[0]) * index / count,
                begin[1] + (end[1] - begin[1]) * index / count,
            )
            for index in range(1, count + 1)
        )
    return samples


def path_length(path: list[tuple[float, float]]) -> float:
    return sum(
        math.hypot(end[0] - begin[0], end[1] - begin[1])
        for begin, end in zip(path, path[1:])
    )


def max_turn_angle(path: list[tuple[float, float]]) -> float:
    directions: list[tuple[float, float]] = []
    for begin, end in zip(path, path[1:]):
        dx, dy = end[0] - begin[0], end[1] - begin[1]
        length = math.hypot(dx, dy)
        if length > 1e-9:
            directions.append((dx / length, dy / length))
    angles = []
    for first, second in zip(directions, directions[1:]):
        cosine = max(-1.0, min(1.0, first[0] * second[0] + first[1] * second[1]))
        angles.append(math.acos(cosine))
    return max(angles, default=0.0)


def planning_metrics(
    question: int,
    path: list[tuple[float, float]],
    display_map: Map,
    real_map: Map,
    settings: dict[str, Any],
) -> tuple[dict[str, Any], bool]:
    initial = settings["initial"]
    goal = settings["goal"]
    esdf = EsdfDistance(real_map)
    samples = sample_path(path, real_map.resolution / 2.0)
    min_esdf = min(
        (esdf.distance(x, y) for x, y in samples), default=0.0
    )
    valid = len(path) >= 2
    if valid:
        valid = math.hypot(path[0][0] - initial[0], path[0][1] - initial[1]) <= real_map.resolution * 2
    if valid:
        valid = math.hypot(path[-1][0] - goal[0], path[-1][1] - goal[1]) <= real_map.resolution * 2
    for x, y in samples:
        if not display_map.is_free(x, y):
            valid = False
            break
        if question == 2 and not real_map._is_free_circle(x, y, settings["robot_radius"]):
            valid = False
            break

    metrics = {
        "planning_success": valid,
        "min_esdf": min_esdf,
        "max_turn_angle": max_turn_angle(path),
        "path_length": path_length(path),
    }
    return metrics, valid


def write_trajectory(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def close_navigation(navigation: CppNavigation) -> None:
    library = getattr(navigation, "_library", None)
    handle = getattr(library, "_handle", None)
    navigation.close()
    if os.name == "nt" and handle:
        free_library = ctypes.windll.kernel32.FreeLibrary
        free_library.argtypes = [ctypes.c_void_p]
        free_library.restype = ctypes.c_int
        free_library(ctypes.c_void_p(handle))
    dll_directory = getattr(navigation, "_dll_directory", None)
    if dll_directory is not None:
        dll_directory.close()


def run_case(
    question: int,
    case_file: Path,
    library: Path,
    output_dir: Path,
) -> dict[str, Any]:
    case = load_yaml(case_file)
    if int(case.get("question", question)) != question:
        raise JudgeError(f"question mismatch in {case_file}")
    display_map, real_map, settings = map_settings(case_file, case)
    control = settings["control"] or {
        "mode": "auto",
        "dynamics": {
            "vx_max": 1.5, "vx_min": 0.0, "vy_max": 1.5, "vy_min": 0.0,
            "acc_max": 1.0, "acc_min": -1.0,
        },
    }
    sim = Simulator(
        time_step=settings["time_step"],
        map_data=real_map,
        goal=settings["goal"],
        robot_radius=settings["robot_radius"],
        speed_noise_std=float(case.get("speed_noise_std", 0.01)),
        seed=int(case.get("seed", 42)),
        control_config=control,
        render=False,
        show_lidar=False,
        show_planning_map=False,
    )
    sim.set_display_map(display_map)
    sim.robot.reset(settings["initial"])

    navigation = None
    trajectory: list[dict[str, Any]] = []
    esdf = EsdfDistance(real_map)
    try:
        navigation = CppNavigation(sim, library_path=library, exposed_map=display_map)
        navigation.run()
        planned_path = sim.get_display_path()
        if question in (1, 2):
            metrics, passed = planning_metrics(
                question, planned_path, display_map, real_map, settings
            )
        else:
            goal_time: float | None = None
            dangerous_time = 0.0
            blocked_steps = 0
            opening_discovered = False
            max_steps = max(1, math.ceil(settings["timeout"] / settings["time_step"]))
            for step in range(max_steps):
                command = sim.get_control()
                sim.step()
                after = sim.get_pose()
                feedback = sim.get_feedback()
                elapsed = (step + 1) * settings["time_step"]
                if math.hypot(*feedback) < 1e-9 and math.hypot(*command) > 0.05:
                    blocked_steps += 1
                if esdf.distance(after[0], after[1]) < settings["robot_radius"] + 0.05:
                    dangerous_time += settings["time_step"]
                if not display_map.is_free(after[0], after[1]) and real_map.is_free(after[0], after[1]):
                    opening_discovered = True
                trajectory.append({
                    "step": step + 1,
                    "time": elapsed,
                    "x": after[0],
                    "y": after[1],
                    "yaw": after[2],
                    "command_vx": command[0],
                    "command_vy": command[1],
                    "actual_vx": feedback[0],
                    "actual_vy": feedback[1],
                })
                distance_to_goal = math.hypot(
                    after[0] - settings["goal"][0],
                    after[1] - settings["goal"][1],
                )
                if distance_to_goal <= GOAL_TOLERANCE:
                    goal_time = elapsed
                    break
                navigation.run()
            final_pose = sim.get_pose()
            final_error = math.hypot(
                final_pose[0] - settings["goal"][0],
                final_pose[1] - settings["goal"][1],
            )
            success_key = "control_success" if question == 3 else "bonus_success"
            passed = goal_time is not None
            if question == 4:
                passed = passed and opening_discovered
            metrics = {
                success_key: passed,
                "time_to_goal": goal_time,
                "final_error": final_error,
                "blocked_steps": blocked_steps,
                "dangerous_time": dangerous_time,
            }
            if question == 4:
                metrics["opening_discovered"] = opening_discovered
    finally:
        if navigation is not None:
            close_navigation(navigation)

    write_trajectory(output_dir / "trajectory.csv", trajectory)
    return {
        "case": case_file.parent.name,
        "passed": passed,
        "score": 1 if passed else 0,
        "metrics": metrics,
    }


def find_cases(cases_root: Path, question: int) -> list[Path]:
    cases = []
    for path in sorted(cases_root.rglob("case.yaml")):
        data = load_yaml(path)
        if int(data.get("question", -1)) == question:
            cases.append(path)
    return cases


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate a PyRobo submission.")
    parser.add_argument("--submission", type=Path, required=True, help="submission.zip")
    parser.add_argument("--cases", type=Path, default=ROOT_DIR / "cases")
    parser.add_argument("--results", type=Path, default=ROOT_DIR / "results")
    parser.add_argument("--question", type=int, choices=(1, 2, 3, 4))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    submission = args.submission.resolve()
    cases_root = args.cases.resolve()
    results_root = args.results.resolve()
    if not submission.is_file():
        raise SystemExit(f"submission archive was not found: {submission}")
    if not cases_root.is_dir():
        raise SystemExit(f"cases directory was not found: {cases_root}")

    questions = [args.question] if args.question is not None else [1, 2, 3, 4]
    with tempfile.TemporaryDirectory(
        prefix="pyrobo-judge-", ignore_cleanup_errors=True
    ) as temporary:
        temporary_root = Path(temporary)
        submission_root = temporary_root / "submission"
        safe_extract(submission, submission_root)
        for question in questions:
            case_files = find_cases(cases_root, question)
            if not case_files:
                raise SystemExit(f"no cases found for Question {question}")
            question_output = results_root / f"question{question}"
            question_output.mkdir(parents=True, exist_ok=True)
            try:
                library = build_submission(
                    submission_root, question, temporary_root / f"build-question{question}"
                )
                case_results = []
                for case_file in case_files:
                    case_output = question_output / case_file.parent.name
                    try:
                        case_results.append(run_case(question, case_file, library, case_output))
                    except Exception as error:
                        case_output.mkdir(parents=True, exist_ok=True)
                        message = f"{type(error).__name__}: {error}"
                        (case_output / "error.log").write_text(message + "\n", encoding="utf-8")
                        case_results.append({
                            "case": case_file.parent.name,
                            "passed": False,
                            "score": 0,
                            "error": message,
                        })
            except JudgeError as error:
                case_results = [{
                    "case": "build",
                    "passed": False,
                    "score": 0,
                    "error": str(error),
                }]
            raw_score = MAX_SCORES[question] * sum(
                result["score"] for result in case_results
            ) / len(case_results)
            score = int(raw_score) if raw_score.is_integer() else raw_score
            result = {
                "question": question,
                "max_score": MAX_SCORES[question],
                "score": score,
                "cases": case_results,
            }
            (question_output / "result.json").write_text(
                json.dumps(result, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            print(f"Question {question}: {score:g}/{MAX_SCORES[question]}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except JudgeError as error:
        raise SystemExit(f"judge error: {error}") from None
