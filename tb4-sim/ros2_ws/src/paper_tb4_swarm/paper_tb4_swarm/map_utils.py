from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import yaml


@dataclass(frozen=True)
class GridPath:
    cells: list[tuple[int, int]]
    waypoints: list[tuple[float, float]]
    length_m: float


class OccupancyMap:
    """Small ROS occupancy-map helper for lab-map simulation planning."""

    def __init__(
        self,
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        data: list[int],
    ) -> None:
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.data = data

    @classmethod
    def from_yaml(cls, yaml_path: str | Path) -> "OccupancyMap":
        path = Path(yaml_path).expanduser()
        if not path.is_absolute():
            path = Path.cwd() / path
        with path.open("r", encoding="utf-8") as file:
            meta = yaml.safe_load(file)

        image_path = Path(str(meta["image"]))
        if not image_path.is_absolute():
            image_path = path.parent / image_path
        width, height, pixels = _read_pgm(image_path)

        resolution = float(meta["resolution"])
        origin_x, origin_y, _ = [float(value) for value in meta["origin"]]
        negate = int(meta.get("negate", 0))
        occupied_thresh = float(meta.get("occupied_thresh", 0.65))
        free_thresh = float(meta.get("free_thresh", 0.196))

        data: list[int] = []
        for map_y in range(height):
            pgm_y = height - 1 - map_y
            row_start = pgm_y * width
            for x in range(width):
                pixel = pixels[row_start + x]
                normalized = pixel / 255.0
                occ = normalized if negate else 1.0 - normalized
                if occ > occupied_thresh:
                    data.append(100)
                elif occ < free_thresh:
                    data.append(0)
                else:
                    data.append(-1)
        return cls(width, height, resolution, origin_x, origin_y, data)

    def grid_to_world(self, cell: tuple[int, int]) -> tuple[float, float]:
        x, y = cell
        return (
            self.origin_x + (x + 0.5) * self.resolution,
            self.origin_y + (y + 0.5) * self.resolution,
        )

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        return (
            int(math.floor((x - self.origin_x) / self.resolution)),
            int(math.floor((y - self.origin_y) / self.resolution)),
        )

    def is_inside(self, cell: tuple[int, int]) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def occupancy(self, cell: tuple[int, int]) -> int:
        if not self.is_inside(cell):
            return 100
        x, y = cell
        return int(self.data[y * self.width + x])

    def is_blocked(
        self, cell: tuple[int, int], unknown_is_obstacle: bool = True
    ) -> bool:
        value = self.occupancy(cell)
        if value < 0:
            return unknown_is_obstacle
        return value >= 65

    def has_clearance(
        self,
        cell: tuple[int, int],
        clearance_m: float,
        unknown_is_obstacle: bool = True,
    ) -> bool:
        if self.is_blocked(cell, unknown_is_obstacle):
            return False

        radius_cells = int(math.ceil(clearance_m / self.resolution))
        wx, wy = self.grid_to_world(cell)
        cx, cy = cell
        for y in range(cy - radius_cells, cy + radius_cells + 1):
            for x in range(cx - radius_cells, cx + radius_cells + 1):
                check = (x, y)
                if not self.is_inside(check):
                    return False
                px, py = self.grid_to_world(check)
                if math.hypot(px - wx, py - wy) > clearance_m:
                    continue
                if self.is_blocked(check, unknown_is_obstacle):
                    return False
        return True

    def nearest_safe_cell(
        self,
        x: float,
        y: float,
        clearance_m: float,
        unknown_is_obstacle: bool = True,
    ) -> tuple[int, int] | None:
        target = self.world_to_grid(x, y)
        if self.has_clearance(target, clearance_m, unknown_is_obstacle):
            return target

        max_radius = max(self.width, self.height)
        best: tuple[float, tuple[int, int]] | None = None
        for radius in range(1, max_radius + 1):
            found = False
            for cell in _ring(target, radius):
                if not self.has_clearance(cell, clearance_m, unknown_is_obstacle):
                    continue
                wx, wy = self.grid_to_world(cell)
                dist = math.hypot(wx - x, wy - y)
                if best is None or dist < best[0]:
                    best = (dist, cell)
                found = True
            if found and best is not None:
                return best[1]
        return None

    def plan(
        self,
        start_xy: tuple[float, float],
        goal_xy: tuple[float, float],
        clearance_m: float,
        unknown_is_obstacle: bool = True,
    ) -> GridPath | None:
        start = self.nearest_safe_cell(
            start_xy[0], start_xy[1], clearance_m, unknown_is_obstacle
        )
        goal = self.nearest_safe_cell(
            goal_xy[0], goal_xy[1], clearance_m, unknown_is_obstacle
        )
        if start is None or goal is None:
            return None
        if start == goal:
            point = self.grid_to_world(goal)
            return GridPath(cells=[goal], waypoints=[point], length_m=0.0)

        came_from: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
        g_score: dict[tuple[int, int], float] = {start: 0.0}
        queue: list[tuple[float, tuple[int, int]]] = [
            (self._heuristic(start, goal), start)
        ]

        while queue:
            _, current = heapq.heappop(queue)
            if current == goal:
                break
            for neighbor, step_cost in self._neighbors(
                current, clearance_m, unknown_is_obstacle
            ):
                score = g_score[current] + step_cost
                if score >= g_score.get(neighbor, math.inf):
                    continue
                came_from[neighbor] = current
                g_score[neighbor] = score
                priority = score + self._heuristic(neighbor, goal)
                heapq.heappush(queue, (priority, neighbor))

        if goal not in came_from:
            return None

        cells: list[tuple[int, int]] = []
        current: tuple[int, int] | None = goal
        while current is not None:
            cells.append(current)
            current = came_from[current]
        cells.reverse()
        cells = self._simplify(cells, clearance_m, unknown_is_obstacle)
        waypoints = [self.grid_to_world(cell) for cell in cells]
        length = 0.0
        for previous, current_cell in zip(cells, cells[1:]):
            length += self._distance(previous, current_cell)
        return GridPath(cells=cells, waypoints=waypoints, length_m=length)

    def _neighbors(
        self,
        cell: tuple[int, int],
        clearance_m: float,
        unknown_is_obstacle: bool,
    ) -> Iterable[tuple[tuple[int, int], float]]:
        x, y = cell
        for dx, dy in (
            (1, 0),
            (-1, 0),
            (0, 1),
            (0, -1),
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1),
        ):
            neighbor = (x + dx, y + dy)
            if not self.has_clearance(neighbor, clearance_m, unknown_is_obstacle):
                continue
            if dx and dy:
                side_a = (x + dx, y)
                side_b = (x, y + dy)
                if not self.has_clearance(
                    side_a, clearance_m, unknown_is_obstacle
                ) or not self.has_clearance(side_b, clearance_m, unknown_is_obstacle):
                    continue
            yield neighbor, self._distance(cell, neighbor)

    def _distance(self, a: tuple[int, int], b: tuple[int, int]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1]) * self.resolution

    def _heuristic(self, a: tuple[int, int], b: tuple[int, int]) -> float:
        return self._distance(a, b)

    def _simplify(
        self,
        cells: list[tuple[int, int]],
        clearance_m: float,
        unknown_is_obstacle: bool,
    ) -> list[tuple[int, int]]:
        if len(cells) <= 2:
            return cells
        simplified = [cells[0]]
        anchor = 0
        probe = 2
        while probe < len(cells):
            if self._line_is_safe(
                cells[anchor], cells[probe], clearance_m, unknown_is_obstacle
            ):
                probe += 1
                continue
            simplified.append(cells[probe - 1])
            anchor = probe - 1
            probe = anchor + 2
        simplified.append(cells[-1])
        return simplified

    def _line_is_safe(
        self,
        a: tuple[int, int],
        b: tuple[int, int],
        clearance_m: float,
        unknown_is_obstacle: bool,
    ) -> bool:
        for cell in _bresenham(a, b):
            if not self.has_clearance(cell, clearance_m, unknown_is_obstacle):
                return False
        return True


def _ring(center: tuple[int, int], radius: int) -> Iterable[tuple[int, int]]:
    cx, cy = center
    for x in range(cx - radius, cx + radius + 1):
        yield x, cy - radius
        yield x, cy + radius
    for y in range(cy - radius + 1, cy + radius):
        yield cx - radius, y
        yield cx + radius, y


def _bresenham(
    a: tuple[int, int], b: tuple[int, int]
) -> Iterable[tuple[int, int]]:
    x0, y0 = a
    x1, y1 = b
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            return
        twice_error = 2 * error
        if twice_error >= dy:
            error += dy
            x0 += sx
        if twice_error <= dx:
            error += dx
            y0 += sy


def _read_pgm(path: Path) -> tuple[int, int, bytes]:
    content = path.read_bytes()
    index = 0

    def next_token() -> bytes:
        nonlocal index
        while index < len(content) and chr(content[index]).isspace():
            index += 1
        if index < len(content) and content[index : index + 1] == b"#":
            while index < len(content) and content[index : index + 1] != b"\n":
                index += 1
            return next_token()
        start = index
        while index < len(content) and not chr(content[index]).isspace():
            index += 1
        return content[start:index]

    magic = next_token()
    if magic != b"P5":
        raise ValueError(f"Only binary PGM (P5) maps are supported: {path}")
    width = int(next_token())
    height = int(next_token())
    max_value = int(next_token())
    if max_value != 255:
        raise ValueError(f"Only 8-bit PGM maps are supported: {path}")
    while index < len(content) and chr(content[index]).isspace():
        index += 1
    pixels = content[index : index + width * height]
    if len(pixels) != width * height:
        raise ValueError(f"PGM pixel count mismatch: {path}")
    return width, height, pixels
