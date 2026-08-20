from __future__ import annotations

import random
from collections.abc import Callable
from typing import TYPE_CHECKING

from sim.kinematics import Kinematics, UnicycleKinematics

if TYPE_CHECKING:
    from sim.robot import Robot


class Controller:
    """控制器：接收期望控制量，加入噪声，并保存机器人反馈。

    速度单位为 m/s，角速度单位为 rad/s。
    """

    def __init__(
        self,
        robot: Robot,
        time_step: float,
        speed_noise_std: float = 0.01,
        omega_noise_std: float = 0.05,
        seed: int | None = None,
        kinematics: Kinematics | None = None,
    ):
        """创建控制器。

        ``speed_noise_std`` 和 ``omega_noise_std`` 分别是线速度、角速度
        高斯噪声的标准差。设置 ``seed`` 后可以复现实验结果。
        """
        if time_step <= 0:
            raise ValueError("time_step must be greater than zero")
        if speed_noise_std < 0 or omega_noise_std < 0:
            raise ValueError("noise standard deviations must not be negative")

        self.robot = robot
        self.time_step = time_step
        self.speed_noise_std = float(speed_noise_std)
        self.omega_noise_std = float(omega_noise_std)
        self._random = random.Random(seed)
        self.kinematics = kinematics or UnicycleKinematics()
        self._expected_speed = 0.0
        self._expected_omega = 0.0
        self._feedback_speed = 0.0
        self._feedback_omega = 0.0
        self._feedback_vx = 0.0
        self._feedback_vy = 0.0

    def set_control(self, speed: float, omega: float) -> None:
        """设置下一次仿真周期使用的期望线速度和角速度。

        该函数只保存期望值，不会立即移动机器人；机器人会在
        :meth:`step` 或 ``Simulator.step`` 时移动。
        """
        self._expected_speed = float(speed)
        self._expected_omega = float(omega)

    def set_kinematics(self, kinematics: Kinematics) -> None:
        """替换控制指令所使用的运动学模型。"""
        self.kinematics = kinematics

    def get_control(self) -> tuple[float, float]:
        """获取当前期望控制量，返回 ``(speed, omega)``。"""
        return self._expected_speed, self._expected_omega

    def step(
        self, can_move: Callable[[tuple[float, float, float]], bool] | None = None
    ) -> tuple[float, float]:
        """执行一个仿真周期，并返回机器人实际反馈 ``(speed, omega)``。

        ``can_move`` 是可选的碰撞检查函数，接收预测位姿并返回是否允许移动。
        """
        command = self.kinematics.add_noise(
            self.get_control(),
            self._random,
            self.speed_noise_std,
            self.omega_noise_std,
        )
        next_pose = self.kinematics.predict_pose(
            self.robot.pose, command, self.time_step
        )
        if can_move is not None and not can_move(next_pose):
            command = self.kinematics.blocked_command(command)
            next_pose = self.kinematics.predict_pose(
                self.robot.pose, command, self.time_step
            )

        self._feedback_speed, self._feedback_omega = command
        self._feedback_vx, self._feedback_vy, self._feedback_omega = (
            self.kinematics.feedback(command)
        )
        self.robot._commit_pose(next_pose)
        return self.get_feedback()

    def get_feedback(self) -> tuple[float, float]:
        """获取最近一个仿真周期的实际反馈 ``(speed, omega)``。"""
        return self._feedback_speed, self._feedback_omega

    def get_vector_feedback(self) -> tuple[float, float, float]:
        """获取最近周期的 ``(vx, vy, omega)`` 反馈。"""
        return self._feedback_vx, self._feedback_vy, self._feedback_omega

    # Compatibility aliases for the original API.
    set_target = set_control
    get_fdb = get_feedback
