from __future__ import annotations

import math
import random
from abc import ABC, abstractmethod


Pose = tuple[float, float, float]
Command = tuple[float, float]


class Kinematics(ABC):
    """控制指令到下一位姿的转换模型。"""

    @abstractmethod
    def predict_pose(self, pose: Pose, command: Command, time_step: float) -> Pose:
        """根据控制指令预测下一位姿。"""

    @abstractmethod
    def add_noise(
        self,
        command: Command,
        random_source: random.Random,
        speed_noise_std: float,
        omega_noise_std: float,
    ) -> Command:
        """为控制指令添加符合该模型含义的噪声。"""

    @abstractmethod
    def blocked_command(self, command: Command) -> Command:
        """发生碰撞时返回只允许保留的控制指令。"""

    @abstractmethod
    def feedback(self, command: Command) -> tuple[float, float, float]:
        """将模型指令转换为统一的 ``(vx, vy, omega)`` 反馈。"""


class UnicycleKinematics(Kinematics):
    """单轮/差速模型，指令为 ``(v, omega)``。"""

    def predict_pose(self, pose: Pose, command: Command, time_step: float) -> Pose:
        x, y, yaw = pose
        speed, omega = command
        return (
            x + speed * time_step * math.cos(yaw),
            y + speed * time_step * math.sin(yaw),
            yaw + omega * time_step,
        )

    def add_noise(
        self,
        command: Command,
        random_source: random.Random,
        speed_noise_std: float,
        omega_noise_std: float,
    ) -> Command:
        speed, omega = command
        return (
            speed + random_source.gauss(0.0, speed_noise_std),
            omega + random_source.gauss(0.0, omega_noise_std),
        )

    def blocked_command(self, command: Command) -> Command:
        return 0.0, command[1]

    def feedback(self, command: Command) -> tuple[float, float, float]:
        speed, omega = command
        return speed, 0.0, omega


class HolonomicKinematics(Kinematics):
    """全向轮模型，指令为机器人坐标系下的 ``(vx, vy)``。"""

    def predict_pose(self, pose: Pose, command: Command, time_step: float) -> Pose:
        x, y, yaw = pose
        vx, vy = command
        return (
            x + (vx * math.cos(yaw) - vy * math.sin(yaw)) * time_step,
            y + (vx * math.sin(yaw) + vy * math.cos(yaw)) * time_step,
            yaw,
        )

    def add_noise(
        self,
        command: Command,
        random_source: random.Random,
        speed_noise_std: float,
        omega_noise_std: float,
    ) -> Command:
        vx, vy = command
        return (
            vx + random_source.gauss(0.0, speed_noise_std),
            vy + random_source.gauss(0.0, speed_noise_std),
        )

    def blocked_command(self, command: Command) -> Command:
        return 0.0, 0.0

    def feedback(self, command: Command) -> tuple[float, float, float]:
        vx, vy = command
        return vx, vy, 0.0
