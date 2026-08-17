import random
from collections.abc import Callable

from sim.robot import Robot


class Controller:
    """Translate expected commands into noisy commands and collect feedback."""

    def __init__(
        self,
        robot: Robot,
        time_step: float,
        speed_noise_std: float = 0.01,
        omega_noise_std: float = 0.05,
        seed: int | None = None,
    ):
        if time_step <= 0:
            raise ValueError("time_step must be greater than zero")
        if speed_noise_std < 0 or omega_noise_std < 0:
            raise ValueError("noise standard deviations must not be negative")

        self.robot = robot
        self.time_step = time_step
        self.speed_noise_std = float(speed_noise_std)
        self.omega_noise_std = float(omega_noise_std)
        self._random = random.Random(seed)
        self._expected_speed = 0.0
        self._expected_omega = 0.0
        self._feedback_speed = 0.0
        self._feedback_omega = 0.0

    def set_control(self, speed: float, omega: float) -> None:
        """Set expected linear and angular velocity for future simulation ticks."""
        self._expected_speed = float(speed)
        self._expected_omega = float(omega)

    def get_control(self) -> tuple[float, float]:
        """Return the currently requested ``(speed, omega)``."""
        return self._expected_speed, self._expected_omega

    def step(self, can_move: Callable[[tuple[float, float, float]], bool] | None = None) -> tuple[float, float]:
        """Apply one noisy command and store the robot's measured feedback."""
        speed = self._expected_speed + self._random.gauss(0.0, self.speed_noise_std)
        omega = self._expected_omega + self._random.gauss(0.0, self.omega_noise_std)
        self._feedback_speed, self._feedback_omega = self.robot.move(
            speed,
            omega,
            self.time_step,
            can_move=can_move,
        )
        return self.get_feedback()

    def get_feedback(self) -> tuple[float, float]:
        """Return measured ``(speed, omega)`` from the last simulation tick."""
        return self._feedback_speed, self._feedback_omega

    # Compatibility aliases for the original API.
    set_target = set_control
    get_fdb = get_feedback
