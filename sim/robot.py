import math
from collections.abc import Callable


class Robot:
    def __init__(
        self,
        pose: tuple[float, float, float] = (0.0, 0.0, 0.0),
        radius: float = 0.05,
    ):
        if radius <= 0:
            raise ValueError("radius must be greater than zero")
        self.radius = float(radius)
        self.reset(pose)

    def move(
        self,
        speed: float,
        omega: float,
        time_step: float,
        can_move: Callable[[tuple[float, float, float]], bool] | None = None,
    ) -> tuple[float, float]:
        """Move and return the actually applied ``(speed, omega)`` feedback."""
        next_pose = self.predict_pose(speed, omega, time_step)
        if can_move is not None and not can_move(next_pose):
            # Keep rotation feedback: a differential-drive robot can turn in place.
            speed = 0.0
            next_pose = self.predict_pose(speed, omega, time_step)
        self.__pose = next_pose
        return speed, omega

    def predict_pose(self, speed: float, omega: float, time_step: float) -> tuple[float, float, float]:
        x, y, yaw = self.__pose
        x += speed * time_step * math.cos(yaw)
        y += speed * time_step * math.sin(yaw)
        yaw += omega * time_step
        return x, y, yaw

    @property
    def pose(self) -> tuple[float, float, float]:
        return self.__pose

    def reset(self, pose: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
        if len(pose) != 3:
            raise ValueError("pose must contain exactly three values")
        self.__pose = tuple(float(value) for value in pose)
