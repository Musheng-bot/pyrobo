import argparse
from pathlib import Path

from sim.simulator import Simulator


MAP_IMAGE = Path(__file__).parent / "examples" / "demo_map.pgm"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the PyRobo simulator")
    parser.add_argument(
        "--mode",
        choices=("manual", "auto"),
        required=True,
        help="control mode; this argument is required",
    )
    return parser.parse_args()


def setup_funcs(sim: Simulator) -> None:
    sim.robot.reset((0.15, 0.15, 0.0))


def main() -> None:
    args = parse_args()
    sim = Simulator(
        time_step=0.05,
        map_data=MAP_IMAGE,
        map_resolution=0.1,
        map_origin=(0.0, 0.0),
        speed_noise_std=0.01,
        omega_noise_std=0.05,
        seed=42,
        robot_radius=0.05,
        render=True,
    )
    setup_funcs(sim)

    def control_loop(environment: Simulator) -> None:
        if args.mode == "manual":
            speed, omega = environment.get_manual_control()
        else:
            speed, omega = auto_control(environment)
        environment.set_control(speed, omega)

        # Upper-level software can consume the previous tick's feedback here.
        feedback_speed, feedback_omega = environment.get_feedback()
        _ = feedback_speed, feedback_omega

    sim.run(callback=control_loop)


def auto_control(sim: Simulator) -> tuple[float, float]:
    """Example automatic controller; replace with planner/controller logic."""
    return 0.1, 0.0


if __name__ == "__main__":
    main()
