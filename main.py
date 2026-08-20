from pathlib import Path
from typing import Any

import yaml

from sim.simulator import Simulator


ROOT_DIR = Path(__file__).resolve().parent
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


def auto_control(sim: Simulator) -> tuple[float, float]:
    """Example automatic controller; replace with planner/controller logic."""
    return 0.1, 0.0


def main() -> None:
    config = load_config()
    pyrobo = config.get("pyrobo")
    if not isinstance(pyrobo, dict):
        raise ValueError("config must define a pyrobo mapping")

    mode = get_mode(config)
    initial_pose, goal = get_robot_config(config)
    map_config = pyrobo.get("map")
    if not isinstance(map_config, dict) or "name" not in map_config:
        raise ValueError("config must define pyrobo.map.name")

    map_file = find_map(str(map_config["name"]))
    sim = Simulator(
        time_step=float(pyrobo["time_step"]),
        map_data=map_file,
        map_resolution=float(map_config["resolution"]),
        map_origin=(float(map_config["origin_x"]), float(map_config["origin_y"])),
        robot_radius=float(pyrobo["robot_radius"]),
        speed_noise_std=0.01,
        omega_noise_std=0.05,
        seed=42,
        render=True,
    )
    sim.robot.reset(initial_pose)
    sim.set_goal(goal)

    def control_loop(environment: Simulator) -> None:
        if mode == "manual":
            speed, omega = environment.get_manual_control()
        else:
            speed, omega = auto_control(environment)
        environment.set_control(speed, omega)

        # Upper-level software can consume the previous tick's feedback here.
        feedback_speed, feedback_omega = environment.get_feedback()
        _ = feedback_speed, feedback_omega

    sim.run(callback=control_loop)


if __name__ == "__main__":
    main()
