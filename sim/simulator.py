import math
import time
from collections.abc import Callable, Iterable

from sim.controller import Controller
from sim.map import Map, MapInput
from sim.robot import Robot


ControlCallback = Callable[["Simulator"], None]
WorldPoint = tuple[float, float]


class Simulator:
    """提供给上层控制软件使用的仿真环境。

    pygame 窗口、地图、机器人和控制器均由该类管理。上层软件主要通过
    ``set_control``、``get_feedback``、``get_pose`` 和 ``step`` 交互。
    """

    def __init__(
        self,
        time_step: float,
        map_data: MapInput | Map | None = None,
        map_resolution: float = 1.0,
        map_origin: tuple[float, float] = (0.0, 0.0),
        speed_noise_std: float = 0.01,
        omega_noise_std: float = 0.05,
        seed: int | None = None,
        robot_radius: float = 0.05,
        render: bool = True,
        window_scale: int = 48,
    ):
        """创建仿真环境。

        ``map_data`` 可以是 ``Map``、``np.ndarray`` 或地图文件路径；
        ``map_resolution`` 为米/像素，``map_origin`` 为地图左下角世界坐标。
        """
        if time_step <= 0:
            raise ValueError("time_step must be greater than zero")
        if window_scale <= 0:
            raise ValueError("window_scale must be greater than zero")

        self.time_step = float(time_step)
        self.map = (
            map_data
            if isinstance(map_data, Map)
            else Map(map_data, resolution=map_resolution, origin=map_origin)
            if map_data is not None
            else None
        )
        self.robot = Robot(radius=robot_radius)
        self.controller = Controller(
            self.robot,
            self.time_step,
            speed_noise_std=speed_noise_std,
            omega_noise_std=omega_noise_std,
            seed=seed,
        )
        self.render_enabled = render
        self.window_scale = window_scale
        self.is_running = False
        self._pygame = None
        self._screen = None
        self._font = None
        self._clock = None
        self._display_path: list[WorldPoint] = []
        self._goal: tuple[float, float, float] | None = None
        self._manual_keys: set[int] = set()

    def set_control(self, speed: float, omega: float) -> None:
        """设置期望线速度和角速度，单位分别为 m/s 和 rad/s。"""
        self.controller.set_control(speed, omega)

    def get_control(self) -> tuple[float, float]:
        """获取当前设置的期望 ``(speed, omega)``。"""
        return self.controller.get_control()

    def get_feedback(self) -> tuple[float, float]:
        """获取最近一个仿真周期的实际 ``(speed, omega)`` 反馈。"""
        return self.controller.get_feedback()

    def get_pose(self) -> tuple[float, float, float]:
        """获取机器人当前世界位姿 ``(x, y, yaw)``。"""
        return self.robot.pose

    def set_goal(self, goal: tuple[float, float] | tuple[float, float, float]) -> None:
        """设置米制世界坐标目标点，pygame 单击地图也会调用此接口。

        目标点只保存在 Simulator 中，由上层导航回调通过 ``get_goal``
        主动读取和处理。
        """
        if len(goal) not in (2, 3):
            raise ValueError("goal must contain (x, y) or (x, y, yaw)")
        yaw = float(goal[2]) if len(goal) == 3 else 0.0
        self._goal = (float(goal[0]), float(goal[1]), yaw)

    def get_goal(self) -> tuple[float, float, float] | None:
        """获取当前目标点；没有设置目标时返回 ``None``。"""
        return self._goal

    def clear_goal(self) -> None:
        """清除目标点和当前显示路径。"""
        self._goal = None
        self._display_path = []

    def set_display_path(self, path: Iterable[WorldPoint]) -> None:
        """设置要在 pygame 中显示的世界坐标路径。

        路径可以来自任意导航算法，不要求继承或绑定 PathPlanner；
        每个点格式为 ``(x, y)``，单位为米。
        """
        self._display_path = [(float(point[0]), float(point[1])) for point in path]

    def get_display_path(self) -> list[WorldPoint]:
        """获取当前显示的世界坐标路径副本。"""
        return list(self._display_path)

    def get_manual_control(
        self,
        speed: float = 0.1,
        omega: float = 1.5,
    ) -> tuple[float, float]:
        """读取 pygame 键盘输入并返回 ``(speed, omega)``。

        该函数只读取输入，不会自动设置控制量；调用方仍需把返回值传给
        :meth:`set_control`。方向键对应前进、后退、左转和右转。
        """
        if self._pygame is None:
            raise RuntimeError("manual control requires the pygame renderer")
        linear_speed = speed if self._pygame.K_UP in self._manual_keys else -speed if self._pygame.K_DOWN in self._manual_keys else 0.0
        angular_speed = -omega if self._pygame.K_LEFT in self._manual_keys else omega if self._pygame.K_RIGHT in self._manual_keys else 0.0
        return linear_speed, angular_speed

    def step(self) -> tuple[float, float]:
        """推进一个仿真周期，并返回实际反馈 ``(speed, omega)``。"""
        can_move = None if self.map is None else lambda pose: self.map.is_free_circle(
            pose[0], pose[1], self.robot.radius
        )
        return self.controller.step(can_move)

    def run(self, callback: ControlCallback | None = None, duration: float | None = None) -> None:
        """在当前线程运行仿真环境。

        每个周期结束后调用一次 ``callback(simulator)``。回调中可以读取本周期
        反馈，并设置下一周期控制量。开启渲染时，关闭 pygame 窗口会停止仿真。
        """
        if duration is not None and duration < 0:
            raise ValueError("duration must be non-negative or None")
        if self.map is None and self.render_enabled:
            raise ValueError("rendering requires map_data")

        if self.render_enabled:
            self._init_renderer()
        self.is_running = True
        elapsed = 0.0
        try:
            while self.is_running and (duration is None or elapsed < duration):
                if self.render_enabled:
                    self._handle_events()
                if not self.is_running:
                    break

                self.step()
                elapsed += self.time_step
                if callback is not None:
                    callback(self)
                if self.render_enabled:
                    self._draw()
                    self._clock.tick(max(1, round(1.0 / self.time_step)))
                else:
                    time.sleep(self.time_step)
        finally:
            self.is_running = False
            if self.render_enabled:
                self._close_renderer()

    def stop(self) -> None:
        """停止持续运行的仿真循环。"""
        self.is_running = False

    def _init_renderer(self) -> None:
        import warnings

        with warnings.catch_warnings():
            # pygame 2.6.1 imports pkg_resources internally; this warning is
            # from pygame itself, not from the simulator code.
            warnings.filterwarnings(
                "ignore",
                message="pkg_resources is deprecated as an API.*",
                category=UserWarning,
                module="pygame.pkgdata",
            )
            import pygame

        if self.map is None:
            raise ValueError("rendering requires map_data")
        pygame.init()
        height, width = self.map.shape
        panel_width = 280
        self._pygame = pygame
        self._screen = pygame.display.set_mode(
            (width * self.window_scale + panel_width, height * self.window_scale)
        )
        pygame.display.set_caption("PyRobo Simulator")
        self._font = pygame.font.Font(None, 24)
        self._clock = pygame.time.Clock()

    def _handle_events(self) -> None:
        for event in self._pygame.event.get():
            if event.type == self._pygame.QUIT:
                self.stop()
            elif event.type == self._pygame.KEYDOWN:
                self._manual_keys.add(event.key)
                if event.key == self._pygame.K_ESCAPE:
                    self.stop()
                elif event.key == self._pygame.K_g:
                    self._set_goal_from_screen(self._pygame.mouse.get_pos())
            elif event.type == self._pygame.KEYUP:
                self._manual_keys.discard(event.key)
            elif event.type == self._pygame.MOUSEBUTTONDOWN and event.button == 1:
                self._set_goal_from_screen(event.pos)

    def _set_goal_from_screen(self, position: tuple[int, int]) -> None:
        """将地图窗口像素坐标转换为米制世界目标点。"""
        if self.map is None:
            return
        height, width = self.map.shape
        cell = self.window_scale
        screen_x, screen_y = position
        if not 0 <= screen_x < width * cell or not 0 <= screen_y < height * cell:
            return
        x = self.map.origin[0] + screen_x / cell * self.map.resolution
        y = self.map.origin[1] + (height - screen_y / cell) * self.map.resolution
        self.set_goal((x, y, 0.0))

    def _world_to_screen(self, point: WorldPoint) -> tuple[int, int]:
        """将米制世界坐标转换为 pygame 窗口像素坐标。"""
        x, y = point
        height, _ = self.map.shape
        return (
            round((x - self.map.origin[0]) / self.map.resolution * self.window_scale),
            round((height - (y - self.map.origin[1]) / self.map.resolution) * self.window_scale),
        )

    def _draw(self) -> None:
        pygame = self._pygame
        world_map = self.map
        screen = self._screen
        cell = self.window_scale
        height, width = world_map.shape
        screen.fill((35, 40, 48))

        for row in range(height):
            for column in range(width):
                color = (242, 242, 242) if world_map.data[row, column] else (20, 20, 22)
                pygame.draw.rect(screen, color, (column * cell, row * cell, cell, cell))

        if len(self._display_path) >= 2:
            pygame.draw.lines(
                screen,
                (40, 120, 230),
                False,
                [self._world_to_screen(point) for point in self._display_path],
                4,
            )
        for point in self._display_path:
            pygame.draw.circle(screen, (40, 120, 230), self._world_to_screen(point), 4)

        if self._goal is not None:
            pygame.draw.circle(screen, (245, 190, 40), self._world_to_screen(self._goal[:2]), 9, 3)

        x, y, yaw = self.robot.pose
        map_width_m, map_height_m = world_map.size_meters
        screen_x = (x - world_map.origin[0]) / map_width_m * width * cell
        screen_y = height * cell - (y - world_map.origin[1]) / map_height_m * height * cell
        if 0 <= screen_x < width * cell and 0 <= screen_y < height * cell:
            center = (round(screen_x), round(screen_y))
            radius_pixels = max(1, round(self.robot.radius / world_map.resolution * cell))
            pygame.draw.circle(screen, (220, 55, 55), center, radius_pixels)
            heading_length = cell * 0.45
            heading_end = (
                round(center[0] + heading_length * math.cos(yaw)),
                round(center[1] - heading_length * math.sin(yaw)),
            )
            pygame.draw.line(screen, (120, 20, 20), center, heading_end, 3)

        # Keep grid lines as a low-contrast visual aid instead of the main map.
        grid_layer = pygame.Surface((width * cell, height * cell), pygame.SRCALPHA)
        for column in range(width + 1):
            pygame.draw.line(
                grid_layer,
                (150, 150, 150, 55),
                (column * cell, 0),
                (column * cell, height * cell),
            )
        for row in range(height + 1):
            pygame.draw.line(
                grid_layer,
                (150, 150, 150, 55),
                (0, row * cell),
                (width * cell, row * cell),
            )
        screen.blit(grid_layer, (0, 0))

        panel_x = width * cell
        pygame.draw.rect(screen, (35, 40, 48), (panel_x, 0, 280, screen.get_height()))
        expected_speed, expected_omega = self.get_control()
        feedback_speed, feedback_omega = self.get_feedback()
        lines = [
            "PYROBO SIMULATOR",
            "",
            f"resolution: {world_map.resolution:.3f} m/pixel",
            f"map size: {world_map.size_meters[0]:.2f} x {world_map.size_meters[1]:.2f} m",
            "",
            f"pose x: {x:.3f} m",
            f"pose y: {y:.3f} m",
            f"pose yaw: {self.robot.pose[2]:.3f} rad",
            "",
            f"command v: {expected_speed:.3f} m/s",
            f"command w: {expected_omega:.3f} rad/s",
            f"feedback v: {feedback_speed:.3f} m/s",
            f"feedback w: {feedback_omega:.3f} rad/s",
            f"path points: {len(self._display_path)}",
            "",
            "left click / G: set goal",
            "ESC  close",
        ]
        for index, line in enumerate(lines):
            color = (255, 220, 120) if index == 0 else (235, 235, 235)
            screen.blit(self._font.render(line, True, color), (panel_x + 18, 18 + index * 24))

        pygame.display.flip()

    def _close_renderer(self) -> None:
        self._pygame.quit()
        self._pygame = None
        self._screen = None
        self._font = None
        self._clock = None
        self._manual_keys.clear()
