import numpy as np
from pathlib import Path
from typing import TypeAlias


MapInput: TypeAlias = np.ndarray | str | Path


class Map:
    """A 2D occupancy map with a real-world scale.

    ``resolution`` is metres per pixel. ``origin`` is the world coordinate of
    the bottom-left pixel. World coordinates use x-right/y-up; image rows are
    flipped vertically when converting to world coordinates.
    """

    def __init__(
        self,
        source: MapInput,
        resolution: float = 1.0,
        origin: tuple[float, float] = (0.0, 0.0),
        threshold: int = 128,
    ):
        if resolution <= 0:
            raise ValueError("resolution must be greater than zero")
        if not 0 <= threshold <= 255:
            raise ValueError("threshold must be between 0 and 255")
        if len(origin) != 2:
            raise ValueError("origin must contain exactly two values")

        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))

        data = self._read(source)
        if data.ndim == 3:
            if data.shape[2] not in (3, 4):
                raise ValueError("map image must have 3 or 4 channels")
            data = data[..., :3].mean(axis=2)
        if data.ndim != 2:
            raise ValueError("map must be a 2D grayscale array or an image")
        if data.size == 0:
            raise ValueError("map must not be empty")

        self.data = data.astype(bool, copy=True) if data.dtype == bool else np.asarray(data >= threshold, dtype=bool)

    @staticmethod
    def _read(source: MapInput) -> np.ndarray:
        if isinstance(source, np.ndarray):
            return source
        path = Path(source)
        if path.suffix.lower() == ".pgm":
            return Map._read_pgm(path)
        try:
            from PIL import Image
        except ImportError as error:
            raise ImportError("reading map images requires Pillow") from error
        with Image.open(source) as image:
            return np.asarray(image.convert("L"))

    @staticmethod
    def _read_pgm(path: Path) -> np.ndarray:
        """Read an ASCII PGM image without requiring an image package."""
        tokens = []
        for line in path.read_text(encoding="ascii").splitlines():
            tokens.extend(line.split("#", 1)[0].split())
        if len(tokens) < 4 or tokens[0] != "P2":
            raise ValueError("only ASCII PGM (P2) images are supported without Pillow")
        width, height, max_value = map(int, tokens[1:4])
        values = np.asarray([int(value) for value in tokens[4:]], dtype=np.float32)
        if width <= 0 or height <= 0 or max_value <= 0 or values.size != width * height:
            raise ValueError("invalid PGM image dimensions or pixel data")
        return np.rint(values * (255.0 / max_value)).astype(np.uint8).reshape(height, width)

    @property
    def shape(self) -> tuple[int, int]:
        return self.data.shape

    @property
    def size_meters(self) -> tuple[float, float]:
        """Return map size as ``(width, height)`` in metres."""
        height, width = self.data.shape
        return width * self.resolution, height * self.resolution

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        """Convert world metres to ``(column, row)`` image indices."""
        height, _ = self.data.shape
        column = int(np.floor((x - self.origin[0]) / self.resolution))
        from_bottom = int(np.floor((y - self.origin[1]) / self.resolution))
        row = height - 1 - from_bottom
        return column, row

    def grid_to_world(self, column: int, row: int) -> tuple[float, float]:
        """Return the world coordinate of a grid cell's center."""
        height, width = self.data.shape
        if not 0 <= row < height or not 0 <= column < width:
            raise IndexError("grid coordinate is outside the map")
        x = self.origin[0] + (column + 0.5) * self.resolution
        y = self.origin[1] + (height - row - 0.5) * self.resolution
        return x, y

    def is_free(self, x: float, y: float) -> bool:
        """Return whether the world coordinate ``(x, y)`` is free."""
        column, row = self.world_to_grid(x, y)
        return self._is_free_cell(column, row)

    def is_free_circle(self, x: float, y: float, radius: float) -> bool:
        """Return whether a circle at ``(x, y)`` fits entirely in free cells."""
        if radius < 0:
            raise ValueError("radius must not be negative")

        height, width = self.data.shape
        map_width, map_height = self.size_meters
        min_x, max_x = self.origin[0], self.origin[0] + map_width
        min_y, max_y = self.origin[1], self.origin[1] + map_height
        if x - radius < min_x or x + radius > max_x or y - radius < min_y or y + radius > max_y:
            return False

        min_column = int(np.floor((x - radius - self.origin[0]) / self.resolution))
        max_column = int(
            np.floor(np.nextafter((x + radius - self.origin[0]) / self.resolution, -np.inf))
        )
        min_row_from_bottom = int(np.floor((y - radius - self.origin[1]) / self.resolution))
        max_row_from_bottom = int(
            np.floor(np.nextafter((y + radius - self.origin[1]) / self.resolution, -np.inf))
        )

        for from_bottom in range(min_row_from_bottom, max_row_from_bottom + 1):
            row = height - 1 - from_bottom
            for column in range(min_column, max_column + 1):
                cell_min_x = self.origin[0] + column * self.resolution
                cell_max_x = cell_min_x + self.resolution
                cell_min_y = self.origin[1] + from_bottom * self.resolution
                cell_max_y = cell_min_y + self.resolution
                closest_x = min(max(x, cell_min_x), cell_max_x)
                closest_y = min(max(y, cell_min_y), cell_max_y)
                touches_cell = (closest_x - x) ** 2 + (closest_y - y) ** 2 <= radius**2
                if touches_cell and not self._is_free_cell(column, row):
                    return False
        return True

    def _is_free_cell(self, column: int, row: int) -> bool:
        height, width = self.data.shape
        return 0 <= row < height and 0 <= column < width and bool(self.data[row, column])
