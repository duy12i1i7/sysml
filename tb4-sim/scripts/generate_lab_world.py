#!/usr/bin/env python3
"""Generate the stable Gazebo world from the saved lab occupancy map."""

from __future__ import annotations

import re
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
MAP_YAML = ROOT / "maps" / "lab.yaml"
BASE_WORLD = ROOT / "ros2_ws" / "src" / "paper_tb4_swarm" / "worlds" / "two_tb4_stable.sdf"
OUT_WORLD = ROOT / "ros2_ws" / "src" / "paper_tb4_swarm" / "worlds" / "two_tb4_lab.sdf"

MAP_TO_WORLD_OFFSET = (1.322, 1.317)
ROBOT1_MAP_POSE = (1.322, 1.317)
ROBOT2_MAP_POSE = (2.122, 1.317)
TARGET_MAP_POSE = (1.722, 0.717)
ROBOT1_POSE = (0.0, 0.0, 0.08, 0.0, 0.0, 0.0)
ROBOT2_POSE = (
    ROBOT2_MAP_POSE[0] - MAP_TO_WORLD_OFFSET[0],
    ROBOT2_MAP_POSE[1] - MAP_TO_WORLD_OFFSET[1],
    0.08,
    0.0,
    0.0,
    0.0,
)
TARGET_POSE = (
    TARGET_MAP_POSE[0] - MAP_TO_WORLD_OFFSET[0],
    TARGET_MAP_POSE[1] - MAP_TO_WORLD_OFFSET[1],
    0.02,
    0.0,
    0.0,
    0.0,
)


def read_pgm(path: Path) -> tuple[int, int, bytes]:
    content = path.read_bytes()
    index = 0

    def token() -> bytes:
        nonlocal index
        while index < len(content) and chr(content[index]).isspace():
            index += 1
        if index < len(content) and content[index : index + 1] == b"#":
            while index < len(content) and content[index : index + 1] != b"\n":
                index += 1
            return token()
        start = index
        while index < len(content) and not chr(content[index]).isspace():
            index += 1
        return content[start:index]

    if token() != b"P5":
        raise ValueError("Only P5 PGM maps are supported")
    width = int(token())
    height = int(token())
    max_value = int(token())
    if max_value != 255:
        raise ValueError("Only 8-bit PGM maps are supported")
    while index < len(content) and chr(content[index]).isspace():
        index += 1
    pixels = content[index : index + width * height]
    if len(pixels) != width * height:
        raise ValueError("PGM pixel count mismatch")
    return width, height, pixels


def occupancy_masks() -> tuple[dict, list[list[bool]], list[list[bool]]]:
    with MAP_YAML.open("r", encoding="utf-8") as file:
        meta = yaml.safe_load(file)
    image = Path(meta["image"])
    if not image.is_absolute():
        image = MAP_YAML.parent / image
    width, height, pixels = read_pgm(image)
    meta["width"] = width
    meta["height"] = height
    negate = int(meta.get("negate", 0))
    occupied_thresh = float(meta.get("occupied_thresh", 0.65))
    free_thresh = float(meta.get("free_thresh", 0.196))
    occupied = [[False for _ in range(width)] for _ in range(height)]
    unknown = [[False for _ in range(width)] for _ in range(height)]
    for map_y in range(height):
        pgm_y = height - 1 - map_y
        for x in range(width):
            pixel = pixels[pgm_y * width + x]
            normalized = pixel / 255.0
            occ = normalized if negate else 1.0 - normalized
            if occ > occupied_thresh:
                occupied[map_y][x] = True
            elif occ >= free_thresh:
                unknown[map_y][x] = True
    return meta, occupied, unknown


def rectangles(mask: list[list[bool]]) -> list[tuple[int, int, int, int]]:
    height = len(mask)
    width = len(mask[0]) if height else 0
    active: dict[tuple[int, int], tuple[int, int, int, int]] = {}
    output: list[tuple[int, int, int, int]] = []
    for y in range(height):
        runs: list[tuple[int, int]] = []
        x = 0
        while x < width:
            if not mask[y][x]:
                x += 1
                continue
            start = x
            while x + 1 < width and mask[y][x + 1]:
                x += 1
            runs.append((start, x))
            x += 1

        current = set(runs)
        for key, rect in list(active.items()):
            if key not in current:
                output.append(rect)
                del active[key]
        for start, end in runs:
            if (start, end) in active:
                x0, x1, y0, _ = active[(start, end)]
                active[(start, end)] = (x0, x1, y0, y)
            else:
                active[(start, end)] = (start, end, y, y)
    output.extend(active.values())
    return output


def pose_text(values: tuple[float, float, float, float, float, float]) -> str:
    return " ".join(f"{value:.3f}" for value in values)


def obstacle_model(
    name: str,
    rects: list[tuple[int, int, int, int]],
    meta: dict,
    color: str,
    z: float,
) -> str:
    resolution = float(meta["resolution"])
    ox, oy, _ = [float(value) for value in meta["origin"]]
    links = []
    for index, (x0, x1, y0, y1) in enumerate(rects):
        sx = (x1 - x0 + 1) * resolution
        sy = (y1 - y0 + 1) * resolution
        cx = ox + (x0 + x1 + 1) * resolution / 2.0 - MAP_TO_WORLD_OFFSET[0]
        cy = oy + (y0 + y1 + 1) * resolution / 2.0 - MAP_TO_WORLD_OFFSET[1]
        links.append(
            f"""      <link name="{name}_{index:03d}">
        <pose>{cx:.3f} {cy:.3f} {z / 2.0:.3f} 0 0 0</pose>
        <collision name="collision">
          <geometry><box><size>{sx:.3f} {sy:.3f} {z:.3f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.3f} {sy:.3f} {z:.3f}</size></box></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
      </link>"""
        )
    return f"""    <model name="{name}">
      <static>true</static>
{chr(10).join(links)}
    </model>"""


def map_floor(meta: dict) -> str:
    resolution = float(meta["resolution"])
    ox, oy, _ = [float(value) for value in meta["origin"]]
    sx = int(meta["width"]) * resolution
    sy = int(meta["height"]) * resolution
    cx = ox + sx / 2.0 - MAP_TO_WORLD_OFFSET[0]
    cy = oy + sy / 2.0 - MAP_TO_WORLD_OFFSET[1]
    return f"""    <model name="lab_map_floor">
      <static>true</static>
      <pose>{cx:.3f} {cy:.3f} 0.003 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>{sx:.3f} {sy:.3f} 0.006</size></box></geometry>
          <material><ambient>0.86 0.88 0.84 1</ambient><diffuse>0.86 0.88 0.84 1</diffuse></material>
        </visual>
      </link>
    </model>"""


def main() -> None:
    meta, occupied, unknown = occupancy_masks()
    occupied_rects = rectangles(occupied)
    unknown_rects = rectangles(unknown)
    insert = "\n\n".join(
        [
            map_floor(meta),
            obstacle_model("lab_occupied_cells", occupied_rects, meta, "0.05 0.05 0.05 1", 0.50),
            obstacle_model("lab_unknown_cells", unknown_rects, meta, "0.48 0.50 0.52 1", 0.50),
        ]
    )

    text = BASE_WORLD.read_text(encoding="utf-8")
    text = text.replace('<world name="two_tb4_stable">', '<world name="two_tb4_lab">')
    text = text.replace('      <size>10 10</size>', '      <size>5 5</size>', 2)
    text = text.replace(
        '    <model name="target_marker">\n      <static>true</static>\n      <pose>1.5 0 0.02 0 0 0</pose>',
        f'    <model name="target_marker">\n      <static>true</static>\n      <pose>{pose_text(TARGET_POSE)}</pose>',
    )
    text = re.sub(
        r'(<model name="robot1">\s*<pose>)[^<]+(</pose>)',
        rf"\g<1>{pose_text(ROBOT1_POSE)}\g<2>",
        text,
        count=1,
    )
    text = re.sub(
        r'(<model name="robot2">\s*<pose>)[^<]+(</pose>)',
        rf"\g<1>{pose_text(ROBOT2_POSE)}\g<2>",
        text,
        count=1,
    )
    text = text.replace('    <model name="target_marker">', f"{insert}\n\n    <model name=\"target_marker\">", 1)
    OUT_WORLD.write_text(text, encoding="utf-8")
    print(
        f"Wrote {OUT_WORLD.relative_to(ROOT)} with "
        f"{len(occupied_rects)} occupied rectangles and {len(unknown_rects)} unknown rectangles"
    )


if __name__ == "__main__":
    main()
