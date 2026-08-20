import math
from collections.abc import Callable


class Robot:
    def __init__(
        self,
        pose: tuple[float, float, float] = (0.0, 0.0, 0.0),
        radius: float = 0.05,
    ):
        """创建机器人。

        ``pose`` 为 ``(x, y, yaw)``，位置单位是米，朝向单位是弧度；
        ``radius`` 是机器人圆形碰撞半径，单位是米。
        """
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
        """按照给定速度移动，并返回实际执行的 ``(speed, omega)``。

        如果 ``can_move`` 判定下一位姿碰撞，机器人不会平移，但仍可以
        原地旋转，因此反馈速度为 0，角速度仍然有效。
        """
        next_pose = self.predict_pose(speed, omega, time_step)
        if can_move is not None and not can_move(next_pose):
            # Keep rotation feedback: a differential-drive robot can turn in place.
            speed = 0.0
            next_pose = self.predict_pose(speed, omega, time_step)
        self.__pose = next_pose
        return speed, omega

    def predict_pose(
        self, speed: float, omega: float, time_step: float
    ) -> tuple[float, float, float]:
        """根据速度预测下一位姿，不修改机器人当前状态。"""
        x, y, yaw = self.__pose
        x += speed * time_step * math.cos(yaw)
        y += speed * time_step * math.sin(yaw)
        yaw += omega * time_step
        return x, y, yaw

    @property
    def pose(self) -> tuple[float, float, float]:
        """当前位姿 ``(x, y, yaw)``，位置单位米，角度单位弧度。"""
        return self.__pose

    def reset(self, pose: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
        """将机器人位姿重置为 ``(x, y, yaw)``。"""
        if len(pose) != 3:
            raise ValueError("pose must contain exactly three values")
        self.__pose = tuple(float(value) for value in pose)
