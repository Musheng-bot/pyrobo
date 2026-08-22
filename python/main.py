from pathlib import Path
from typing import Any
import argparse

import yaml

from planner.navigation import nav_init, nav_run
from sim.simulator import Simulator
from sim.cpp_navigation import CppNavigation


ROOT_DIR = Path(__file__).resolve().parents[1]
CONFIG_FILE = ROOT_DIR / "config" / "sim.yaml"
MAP_DIR = ROOT_DIR / "map"
SUPPORTED_MAP_SUFFIXES = (".pgm", ".png")


def load_config(path: Path = CONFIG_FILE) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file) or {}
    if not isinstance(config, dict):
        raise ValueError(f"configuration root must be a mapping: {path}")
    return config


def get_mode(config: dict[str, Any]) -> str:
    try:
        mode = config["pyrobo"]["control"]["mode"]
    except (KeyError, TypeError) as error:
        raise ValueError("config must define pyrobo.control.mode as manual or auto") from error
    if mode not in ("manual", "auto"):
        raise ValueError(f"unsupported control mode: {mode!r}; expected manual or auto")
    return mode


def find_map(name: str) -> Path:
    map_path = Path(name)
    candidates = (
        [map_path]
        if map_path.suffix
        else [MAP_DIR / f"{name}{suffix}" for suffix in SUPPORTED_MAP_SUFFIXES]
    )
    if map_path.parent == Path(".") and map_path.suffix:
        candidates = [MAP_DIR / map_path.name]

    for candidate in candidates:
        if candidate.exists() and candidate.is_file():
            return candidate.resolve()
    supported = ", ".join(SUPPORTED_MAP_SUFFIXES)
    raise FileNotFoundError(f"map {name!r} was not found in {MAP_DIR} ({supported})")


def get_robot_config(config: dict[str, Any]) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    pyrobo = config.get("pyrobo")
    robot = pyrobo.get("robot") if isinstance(pyrobo, dict) else None
    if not isinstance(robot, dict):
        raise ValueError("config must define pyrobo.robot")

    initial = robot.get("initial")
    goal = robot.get("goal")
    if not isinstance(initial, dict) or not isinstance(goal, dict):
        raise ValueError("config must define pyrobo.robot.initial and pyrobo.robot.goal")

    try:
        initial_pose = (
            float(initial["x"]),
            float(initial["y"]),
            float(initial["yaw"]),
        )
        goal_pose = (
            float(goal["x"]),
            float(goal["y"]),
            float(goal["yaw"]),
        )
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(
            "robot.initial and robot.goal must define numeric x, y and yaw"
        ) from error
    return initial_pose, goal_pose


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the PyRobo simulator.")
    parser.add_argument(
        "--python-navigation",
        action="store_true",
        help="Use the reference Python navigation instead of the compiled C++ answer.",
    )
    args = parser.parse_args()
    config = load_config()
    pyrobo = config.get("pyrobo")
    if not isinstance(pyrobo, dict):
        raise ValueError("config must define a pyrobo mapping")

    mode = get_mode(config)
    initial_pose, goal = get_robot_config(config)
    control_config = pyrobo.get("control")
    if not isinstance(control_config, dict):
        raise ValueError("config must define pyrobo.control")
    display_config = pyrobo.get("display", {})
    if not isinstance(display_config, dict):
        raise ValueError("pyrobo.display must be a mapping")
    map_config = pyrobo.get("map")
    if not isinstance(map_config, dict) or "name" not in map_config:
        raise ValueError("config must define pyrobo.map.name")

    map_file = find_map(str(map_config["name"]))
    sim = Simulator(
        time_step=float(pyrobo["time_step"]),
        map_data=map_file,
        map_resolution=float(map_config["resolution"]),
        map_origin=(float(map_config["origin_x"]), float(map_config["origin_y"])),
        goal=goal,
        robot_radius=float(pyrobo["robot_radius"]),
        speed_noise_std=0.01,
        seed=42,
        control_config=control_config,
        render=True,
        show_lidar=bool(display_config.get("show_lidar", True)),
        show_planning_map=bool(display_config.get("show_planning_map", False)),
    )
    sim.robot.reset(initial_pose)

    def manual_control(environment: Simulator) -> None:
        vx, vy = environment.get_manual_control()
        environment.set_control(vx, vy)

        # Upper-level software can consume the previous tick's feedback here.
        feedback_vx, feedback_vy = environment.get_feedback()
        _ = feedback_vx, feedback_vy

    if mode == "auto":
        cpp_navigation = None if args.python_navigation else CppNavigation(sim)
        navigation_context = nav_init(sim) if args.python_navigation else None

        def auto_control(environment: Simulator) -> None:
            if cpp_navigation is not None:
                cpp_navigation.run()
            else:
                nav_run(environment, navigation_context)

        callback = auto_control
    else:
        callback = manual_control
    try:
        sim.run(callback=callback)
    finally:
        if mode == "auto" and cpp_navigation is not None:
            cpp_navigation.close()


if __name__ == "__main__":
    main()
