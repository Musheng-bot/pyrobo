from pathlib import Path

from sim.simulator import Simulator


MAP_IMAGE = Path(__file__).parent / "examples" / "demo_map.pgm"


def setup_funcs(sim: Simulator) -> None:
    # Setup planners/controllers and initial robot state here.
    sim.robot.reset((0.15, 0.15, 0.0))


def main():
    sim = Simulator(
        time_step=0.05,
        map_data=MAP_IMAGE,
        map_resolution=0.1,
        map_origin=(0.0, 0.0),
        render=True,
    )
    setup_funcs(sim)

    def control_loop(environment: Simulator) -> None:
        environment.set_control(speed=0.1, omega=0.0)
        speed, omega = environment.get_feedback()
        # Feed speed/omega back into the upper-level controller here.
        _ = speed, omega

    sim.run(callback=control_loop)


if __name__ == '__main__':
    main()
