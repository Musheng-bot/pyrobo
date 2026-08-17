import math
import time
from collections.abc import Callable

from sim.controller import Controller
from sim.map import Map, MapInput
from sim.robot import Robot


ControlCallback = Callable[["Simulator"], None]


class Simulator:
    """Simulation environment exposed to upper-level control software.

    The optional pygame renderer is owned by this class. Upper-level code only
    sets controls and reads feedback through the public methods.
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
        render: bool = True,
        window_scale: int = 48,
    ):
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
        self.robot = Robot()
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

    def set_control(self, speed: float, omega: float) -> None:
        """Set expected linear and angular velocity."""
        self.controller.set_control(speed, omega)

    def get_control(self) -> tuple[float, float]:
        return self.controller.get_control()

    def get_feedback(self) -> tuple[float, float]:
        """Get measured ``(speed, omega)`` from the last simulation tick."""
        return self.controller.get_feedback()

    def get_pose(self) -> tuple[float, float, float]:
        return self.robot.pose

    def step(self) -> tuple[float, float]:
        """Advance one simulation tick and return measured velocity."""
        can_move = None if self.map is None else lambda pose: self.map.is_free(pose[0], pose[1])
        return self.controller.step(can_move)

    def run(self, callback: ControlCallback | None = None, duration: float | None = None) -> None:
        """Run the environment in the current thread.

        ``callback`` is called after every tick. It can read the latest
        feedback and set the command for the next tick. With rendering enabled,
        closing the pygame window stops the environment.
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
        self.is_running = False

    def _init_renderer(self) -> None:
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
            elif event.type == self._pygame.KEYDOWN and event.key == self._pygame.K_ESCAPE:
                self.stop()

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

        x, y, yaw = self.robot.pose
        map_width_m, map_height_m = world_map.size_meters
        screen_x = (x - world_map.origin[0]) / map_width_m * width * cell
        screen_y = height * cell - (y - world_map.origin[1]) / map_height_m * height * cell
        if 0 <= screen_x < width * cell and 0 <= screen_y < height * cell:
            center = (round(screen_x), round(screen_y))
            pygame.draw.circle(screen, (220, 55, 55), center, max(6, cell // 4))
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
            "",
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
