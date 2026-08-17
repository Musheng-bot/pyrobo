"""Example upper-level program using Simulator as an environment."""

from pathlib import Path

from sim.simulator import Simulator


MAP_IMAGE = Path(__file__).with_name("demo_map.pgm")


def setup_funcs(sim: Simulator) -> None:
    """Configure the initial state and any upper-level controllers here."""
    sim.robot.reset((0.15, 0.15, 0.0))


def control_loop(sim: Simulator) -> None:
    """Upper-level control callback: command first, then consume feedback."""
    sim.set_control(speed=0.1, omega=0.0)
    feedback_speed, feedback_omega = sim.get_feedback()
    # Replace this with your planner/controller logic.
    _ = feedback_speed, feedback_omega


def main() -> None:
    sim = Simulator(
        time_step=0.05,
        map_data=MAP_IMAGE,
        map_resolution=0.1,
        map_origin=(0.0, 0.0),
        speed_noise_std=0.01,
        omega_noise_std=0.05,
        seed=42,
        render=True,
    )
    setup_funcs(sim)
    sim.run(callback=control_loop)


if __name__ == "__main__":
    main()
