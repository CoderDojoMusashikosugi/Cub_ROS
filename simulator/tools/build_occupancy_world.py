#!/usr/bin/env python3
"""Build a Godot occupancy extrusion world package from a PGM map and metadata.

Uses only Python's standard library. Preserves all original pixel values in
source.png without dilation, smoothing, hole-filling, or cropping. Black pixels
(value 0) become 0.5 m walls; white and gray pixels (> 0) become flat floor.
An optional, aligned PNG floor texture is copied unchanged for nearest-filtered display.
"""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
import math
from pathlib import Path
import struct
from typing import Any
import zlib


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MAP_PGM = ROOT / "docker/home/maps/1017_02/map_sim.pgm"
DEFAULT_MAP_YAML = ROOT / "docker/home/maps/1017_02/map.yaml"
DEFAULT_WORLD_ID = "gnss2026_mid360_1017_02"
DEFAULT_DISPLAY_NAME = "GNSS 2026 Mid360 (1017_02)"
CAPSULE_CLEARANCE_RADIUS_M = 0.28


def ros_to_godot(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Convert ROS (x, y, z) to Godot (-y, z, -x)."""
    return (-y, z, -x)


def parse_pgm(data: bytes) -> tuple[int, int, int, bytes]:
    """Parse an 8-bit P5 (binary) or P2 (ASCII) PGM image."""
    idx = 0
    length = len(data)

    def next_token() -> bytes:
        nonlocal idx
        while idx < length:
            b = data[idx]
            if b == ord("#"):
                while idx < length and data[idx] not in (10, 13):
                    idx += 1
            elif b in (9, 10, 11, 12, 13, 32):
                idx += 1
            else:
                break
        if idx >= length:
            raise ValueError("Unexpected end of PGM header")
        start = idx
        while idx < length and data[idx] not in (9, 10, 11, 12, 13, 32, ord("#")):
            idx += 1
        return data[start:idx]

    magic = next_token()
    if magic not in (b"P5", b"P2"):
        raise ValueError(f"Unsupported PGM magic {magic!r}; expected P5 or P2")
    width = int(next_token())
    height = int(next_token())
    maxval = int(next_token())
    if width <= 0 or height <= 0:
        raise ValueError(f"Invalid PGM dimensions: {width}x{height}")
    if maxval != 255:
        raise ValueError(f"Unsupported PGM maxval {maxval}; only 8-bit (255) PGM is supported")

    expected_len = width * height
    if magic == b"P5":
        # Exactly one whitespace byte separates maxval from raw pixel bytes.
        if idx >= length or data[idx] not in (9, 10, 11, 12, 13, 32):
            raise ValueError("Missing whitespace after PGM maxval")
        idx += 1
        pixels = data[idx : idx + expected_len]
        if len(pixels) != expected_len:
            raise ValueError(f"PGM pixel data truncated: got {len(pixels)}, expected {expected_len}")
        return width, height, maxval, pixels

    values = bytearray(expected_len)
    for i in range(expected_len):
        val = int(next_token())
        if val < 0 or val > 255:
            raise ValueError(f"PGM pixel value out of 8-bit range: {val}")
        values[i] = val
    return width, height, maxval, bytes(values)


def encode_grayscale_png(width: int, height: int, pixels: bytes) -> bytes:
    """Encode an 8-bit grayscale image losslessly as a standard PNG."""
    if len(pixels) != width * height:
        raise ValueError("Pixel buffer size does not match width * height")

    def chunk(tag: bytes, body: bytes) -> bytes:
        crc = zlib.crc32(tag + body) & 0xFFFFFFFF
        return struct.pack(">I", len(body)) + tag + body + struct.pack(">I", crc)

    raw_rows = bytearray((width + 1) * height)
    stride = width + 1
    for y in range(height):
        row_start = y * stride
        raw_rows[row_start] = 0  # Filter type 0 (None)
        src_start = y * width
        raw_rows[row_start + 1 : row_start + stride] = pixels[src_start : src_start + width]

    ihdr = struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)
    compressed = zlib.compress(bytes(raw_rows), level=9)
    return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr) + chunk(b"IDAT", compressed) + chunk(b"IEND", b"")


def parse_map_yaml(yaml_path: Path) -> dict[str, Any]:
    """Parse minimal ROS map.yaml fields (image, resolution, origin) without PyYAML."""
    result: dict[str, Any] = {}
    for raw_line in yaml_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        key = key.strip()
        value = value.strip()
        if key == "image":
            result["image"] = value.strip("'\"")
        elif key == "resolution":
            result["resolution"] = float(value)
        elif key == "origin":
            if not (value.startswith("[") and value.endswith("]")):
                raise ValueError(f"Invalid origin format in {yaml_path}: {value}")
            parts = [float(p.strip()) for p in value[1:-1].split(",")]
            if len(parts) != 3:
                raise ValueError(f"Expected 3 values in origin in {yaml_path}, got {len(parts)}")
            result["origin"] = parts
        elif key == "negate":
            result["negate"] = int(value)
        elif key == "occupied_thresh":
            result["occupied_thresh"] = float(value)
        elif key == "free_thresh":
            result["free_thresh"] = float(value)
    if "resolution" not in result or "origin" not in result:
        raise ValueError(f"Missing resolution or origin in {yaml_path}")
    return result


def read_png_dimensions(data: bytes) -> tuple[int, int]:
    """Read a PNG's canvas size without resampling or re-encoding its pixels."""
    if (
        len(data) < 33
        or data[:8] != b"\x89PNG\r\n\x1a\n"
        or data[8:16] != b"\x00\x00\x00\x0dIHDR"
    ):
        raise ValueError("Floor texture must be a PNG image")
    return struct.unpack(">II", data[16:24])


def cell_center_to_ros(
    u: float,
    v: float,
    width: int,
    height: int,
    resolution: float,
    origin: list[float] | tuple[float, float, float],
) -> tuple[float, float]:
    """Convert pixel (u, v) center (top-left = (0, 0)) to ROS map (x, y)."""
    local_x = (u + 0.5) * resolution
    local_y = (height - v - 0.5) * resolution
    return local_to_ros(local_x, local_y, origin)


def cell_corner_to_ros(
    u_edge: float,
    v_edge: float,
    width: int,
    height: int,
    resolution: float,
    origin: list[float] | tuple[float, float, float],
) -> tuple[float, float]:
    """Convert grid corner (u_edge, v_edge) to ROS map (x, y)."""
    local_x = u_edge * resolution
    local_y = (height - v_edge) * resolution
    return local_to_ros(local_x, local_y, origin)


def local_to_ros(
    local_x: float,
    local_y: float,
    origin: list[float] | tuple[float, float, float],
) -> tuple[float, float]:
    ox, oy, oyaw = float(origin[0]), float(origin[1]), float(origin[2])
    c = math.cos(oyaw)
    s = math.sin(oyaw)
    return (ox + c * local_x - s * local_y, oy + s * local_x + c * local_y)


def ros_to_local(
    ros_x: float,
    ros_y: float,
    origin: list[float] | tuple[float, float, float],
) -> tuple[float, float]:
    ox, oy, oyaw = float(origin[0]), float(origin[1]), float(origin[2])
    dx = ros_x - ox
    dy = ros_y - oy
    c = math.cos(oyaw)
    s = math.sin(oyaw)
    return (c * dx + s * dy, -s * dx + c * dy)


def ros_to_cell(
    ros_x: float,
    ros_y: float,
    width: int,
    height: int,
    resolution: float,
    origin: list[float] | tuple[float, float, float],
) -> tuple[float, float]:
    """Convert ROS map (x, y) to continuous pixel coordinates (u, v)."""
    local_x, local_y = ros_to_local(ros_x, ros_y, origin)
    u = local_x / resolution
    v = height - (local_y / resolution)
    return (u, v)


def check_spawn_clearance(
    ros_x: float,
    ros_y: float,
    width: int,
    height: int,
    resolution: float,
    origin: list[float] | tuple[float, float, float],
    pixels: bytes,
    clearance_radius: float = CAPSULE_CLEARANCE_RADIUS_M,
) -> tuple[bool, str]:
    """Verify (ros_x, ros_y) is inside floor bounds and has capsule clearance from walls."""
    local_x, local_y = ros_to_local(ros_x, ros_y, origin)
    map_w = width * resolution
    map_h = height * resolution
    if (
        local_x < clearance_radius
        or local_y < clearance_radius
        or local_x > map_w - clearance_radius
        or local_y > map_h - clearance_radius
    ):
        return False, f"Spawn ({ros_x:.3f}, {ros_y:.3f}) is outside world floor bounds or too close to border"

    u_center = local_x / resolution
    v_center = height - (local_y / resolution)
    cell_radius = math.ceil(clearance_radius / resolution) + 1
    u_min = max(0, int(math.floor(u_center)) - cell_radius)
    u_max = min(width - 1, int(math.floor(u_center)) + cell_radius)
    v_min = max(0, int(math.floor(v_center)) - cell_radius)
    v_max = min(height - 1, int(math.floor(v_center)) + cell_radius)

    for v in range(v_min, v_max + 1):
        cell_y0 = (height - (v + 1)) * resolution
        cell_y1 = (height - v) * resolution
        for u in range(u_min, u_max + 1):
            if pixels[v * width + u] != 0:
                continue
            cell_x0 = u * resolution
            cell_x1 = (u + 1) * resolution
            nearest_x = min(max(local_x, cell_x0), cell_x1)
            nearest_y = min(max(local_y, cell_y0), cell_y1)
            dist_sq = (local_x - nearest_x) ** 2 + (local_y - nearest_y) ** 2
            if dist_sq < (clearance_radius - 1e-6) ** 2:
                return False, f"Spawn ({ros_x:.3f}, {ros_y:.3f}) collides with occupied wall cell ({u}, {v})"
    return True, ""


def build_wall_chunks(width: int, height: int, pixels: bytes, chunk_cells: int = 64) -> dict[str, Any]:
    """Merge occupied cells into rectangles and exterior wall segments per chunk."""

    def is_occ(u: int, v: int) -> bool:
        if u < 0 or u >= width or v < 0 or v >= height:
            return False
        return pixels[v * width + u] == 0

    # Also count global un-chunked horizontal runs and merged rectangles for stats.
    global_runs = 0
    global_rects = 0
    active_global: dict[tuple[int, int], int] = {}
    for v in range(height):
        row = pixels[v * width : (v + 1) * width]
        runs: list[tuple[int, int]] = []
        u = 0
        while u < width:
            if row[u] == 0:
                u0 = u
                while u < width and row[u] == 0:
                    u += 1
                runs.append((u0, u))
            else:
                u += 1
        global_runs += len(runs)
        next_global: dict[tuple[int, int], int] = {}
        for run in runs:
            if run in active_global:
                next_global[run] = active_global.pop(run)
            else:
                next_global[run] = v
        global_rects += len(active_global)
        active_global = next_global
    global_rects += len(active_global)

    chunks: list[dict[str, Any]] = []
    chunk_rect_total = 0
    h_wall_total = 0
    v_wall_total = 0

    for cy in range(0, height, chunk_cells):
        for cx in range(0, width, chunk_cells):
            cw = min(chunk_cells, width - cx)
            ch = min(chunk_cells, height - cy)
            rects: list[list[int]] = []
            active: dict[tuple[int, int], int] = {}
            for dv in range(ch):
                v = cy + dv
                row_offset = v * width + cx
                row = pixels[row_offset : row_offset + cw]
                runs = []
                u = 0
                while u < cw:
                    if row[u] == 0:
                        u0 = u
                        while u < cw and row[u] == 0:
                            u += 1
                        runs.append((u0, u))
                    else:
                        u += 1
                next_active: dict[tuple[int, int], int] = {}
                for run in runs:
                    if run in active:
                        next_active[run] = active.pop(run)
                    else:
                        next_active[run] = dv
                for (u0, u1), v0 in active.items():
                    rects.append([cx + u0, cy + v0, u1 - u0, dv - v0])
                active = next_active
            for (u0, u1), v0 in active.items():
                rects.append([cx + u0, cy + v0, u1 - u0, ch - v0])

            if not rects:
                continue

            # Exterior horizontal wall segments within this chunk:
            # Each segment is [u_start, v_grid, length_cells, normal_dir]
            # where normal_dir = -1 means wall faces North (v-1 is free, v is occupied)
            # and normal_dir = +1 means wall faces South (v-1 is occupied, v is free).
            h_walls: list[list[int]] = []
            for dv in range(ch + 1):
                v_grid = cy + dv
                # North-facing edges belong to occupied cell at v_grid (inside chunk: dv < ch)
                if dv < ch:
                    du = 0
                    while du < cw:
                        u_cell = cx + du
                        if is_occ(u_cell, v_grid) and not is_occ(u_cell, v_grid - 1):
                            du0 = du
                            while du < cw and is_occ(cx + du, v_grid) and not is_occ(cx + du, v_grid - 1):
                                du += 1
                            h_walls.append([cx + du0, v_grid, du - du0, -1])
                        else:
                            du += 1
                # South-facing edges belong to occupied cell at v_grid - 1 (inside chunk: dv > 0)
                if dv > 0:
                    du = 0
                    while du < cw:
                        u_cell = cx + du
                        if is_occ(u_cell, v_grid - 1) and not is_occ(u_cell, v_grid):
                            du0 = du
                            while du < cw and is_occ(cx + du, v_grid - 1) and not is_occ(cx + du, v_grid):
                                du += 1
                            h_walls.append([cx + du0, v_grid, du - du0, 1])
                        else:
                            du += 1

            # Exterior vertical wall segments within this chunk:
            # Each segment is [u_grid, v_start, length_cells, normal_dir]
            # where normal_dir = -1 means wall faces West (u-1 is free, u is occupied)
            # and normal_dir = +1 means wall faces East (u-1 is occupied, u is free).
            v_walls: list[list[int]] = []
            for du in range(cw + 1):
                u_grid = cx + du
                # West-facing edges belong to occupied cell at u_grid (inside chunk: du < cw)
                if du < cw:
                    dv = 0
                    while dv < ch:
                        v_cell = cy + dv
                        if is_occ(u_grid, v_cell) and not is_occ(u_grid - 1, v_cell):
                            dv0 = dv
                            while dv < ch and is_occ(u_grid, cy + dv) and not is_occ(u_grid - 1, cy + dv):
                                dv += 1
                            v_walls.append([u_grid, cy + dv0, dv - dv0, -1])
                        else:
                            dv += 1
                # East-facing edges belong to occupied cell at u_grid - 1 (inside chunk: du > 0)
                if du > 0:
                    dv = 0
                    while dv < ch:
                        v_cell = cy + dv
                        if is_occ(u_grid - 1, v_cell) and not is_occ(u_grid, v_cell):
                            dv0 = dv
                            while dv < ch and is_occ(u_grid - 1, cy + dv) and not is_occ(u_grid, cy + dv):
                                dv += 1
                            v_walls.append([u_grid, cy + dv0, dv - dv0, 1])
                        else:
                            dv += 1

            chunk_rect_total += len(rects)
            h_wall_total += len(h_walls)
            v_wall_total += len(v_walls)
            chunks.append(
                {
                    "cx": cx // chunk_cells,
                    "cy": cy // chunk_cells,
                    "u0": cx,
                    "v0": cy,
                    "w": cw,
                    "h": ch,
                    "rects": rects,
                    "h_walls": h_walls,
                    "v_walls": v_walls,
                }
            )

    return {
        "schema_version": 1,
        "chunk_cells": chunk_cells,
        "image_width": width,
        "image_height": height,
        "stats": {
            "non_empty_chunks": len(chunks),
            "global_runs": global_runs,
            "global_merged_rects": global_rects,
            "chunk_merged_rects": chunk_rect_total,
            "h_wall_segments": h_wall_total,
            "v_wall_segments": v_wall_total,
        },
        "chunks": chunks,
    }


def compute_bounds(
    width: int,
    height: int,
    resolution: float,
    origin: list[float] | tuple[float, float, float],
    floor_height: float,
    wall_height: float,
) -> dict[str, Any]:
    corners_local = [
        (0.0, 0.0),
        (width * resolution, 0.0),
        (width * resolution, height * resolution),
        (0.0, height * resolution),
    ]
    ros_corners = [local_to_ros(lx, ly, origin) for lx, ly in corners_local]
    ros_xs = [p[0] for p in ros_corners]
    ros_ys = [p[1] for p in ros_corners]
    ros_min = [round(min(ros_xs), 6), round(min(ros_ys), 6), round(floor_height, 6)]
    ros_max = [round(max(ros_xs), 6), round(max(ros_ys), 6), round(floor_height + wall_height, 6)]

    godot_corners = [ros_to_godot(rx, ry, floor_height) for rx, ry in ros_corners] + [
        ros_to_godot(rx, ry, floor_height + wall_height) for rx, ry in ros_corners
    ]
    gx = [p[0] for p in godot_corners]
    gy = [p[1] for p in godot_corners]
    gz = [p[2] for p in godot_corners]
    godot_min = [round(min(gx), 6), round(min(gy), 6), round(min(gz), 6)]
    godot_max = [round(max(gx), 6), round(max(gy), 6), round(max(gz), 6)]
    return {
        "ros_min": ros_min,
        "ros_max": ros_max,
        "godot_min": godot_min,
        "godot_max": godot_max,
        "size_m": [round(width * resolution, 6), round(height * resolution, 6), round(wall_height, 6)],
    }


def compute_world_hash(definition_without_hash: dict[str, Any]) -> str:
    payload = dict(definition_without_hash)
    payload.pop("world_hash", None)
    canonical = json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    return sha256(canonical).hexdigest()


def build_occupancy_world(
    pgm_path: Path,
    output_dir: Path,
    world_id: str,
    display_name: str,
    resolution: float,
    origin: list[float],
    start_pixel: tuple[int, int] = (780, 1469),
    start_yaw_deg: float = 180.0,
    chunk_cells: int = 64,
    wall_height: float = 0.5,
    floor_height: float = 0.0,
    floor_texture_path: Path | None = None,
) -> dict[str, Any]:
    pgm_bytes = pgm_path.read_bytes()
    pgm_sha256 = sha256(pgm_bytes).hexdigest()
    width, height, _maxval, pixels = parse_pgm(pgm_bytes)

    floor_texture_bytes = None
    if floor_texture_path is not None:
        floor_texture_bytes = floor_texture_path.read_bytes()
        texture_width, texture_height = read_png_dimensions(floor_texture_bytes)
        if (texture_width, texture_height) != (width, height):
            raise ValueError(
                f"Floor texture dimensions ({texture_width}x{texture_height}) "
                f"must match PGM dimensions ({width}x{height}); no resizing is performed"
            )

    occupied_count = pixels.count(0)
    white_count = pixels.count(254)
    gray_205_count = pixels.count(205)
    other_free_count = len(pixels) - occupied_count - white_count - gray_205_count

    png_bytes = encode_grayscale_png(width, height, pixels)
    png_sha256 = sha256(png_bytes).hexdigest()

    chunks_data = build_wall_chunks(width, height, pixels, chunk_cells=chunk_cells)
    chunks_bytes = (json.dumps(chunks_data, separators=(",", ":"), ensure_ascii=False) + "\n").encode("utf-8")
    chunks_sha256 = sha256(chunks_bytes).hexdigest()

    # Resolve primary start position and 3 client positions from start_pixel.
    u0, v0 = start_pixel
    rx, ry = cell_center_to_ros(u0, v0, width, height, resolution, origin)
    ok, err = check_spawn_clearance(rx, ry, width, height, resolution, origin, pixels)
    if not ok:
        raise ValueError(f"Primary start pixel ({u0}, {v0}) failed clearance check: {err}")

    # Place 3 default client spawns in the open corridor (+2m, +4m, +6m along +u / behind left-facing robot)
    # and fall back to nearby open offsets if needed for small synthetic maps.
    client_spawns: list[dict[str, Any]] = []
    candidate_offsets_m = [
        (2.0, 0.0),
        (4.0, 0.0),
        (6.0, 0.0),
        (-2.0, 0.0),
        (0.0, 1.5),
        (0.0, -1.5),
        (1.0, 0.0),
        (-1.0, 0.0),
        (0.0, 0.0),
    ]
    for dx_local, dy_local in candidate_offsets_m:
        if len(client_spawns) >= 3:
            break
        du = dx_local / resolution
        dv = -dy_local / resolution
        cx, cy = cell_center_to_ros(u0 + du, v0 + dv, width, height, resolution, origin)
        c_ok, _ = check_spawn_clearance(cx, cy, width, height, resolution, origin, pixels)
        if c_ok:
            client_spawns.append(
                {
                    "x": round(cx, 6),
                    "y": round(cy, 6),
                    "z": round(floor_height, 6),
                    "yaw_deg": round(start_yaw_deg, 6),
                }
            )
    while len(client_spawns) < 3:
        client_spawns.append(
            {
                "x": round(rx, 6),
                "y": round(ry, 6),
                "z": round(floor_height, 6),
                "yaw_deg": round(start_yaw_deg, 6),
            }
        )

    primary_spawn = {
        "x": round(rx, 6),
        "y": round(ry, 6),
        "z": round(floor_height, 6),
        "yaw_deg": round(start_yaw_deg, 6),
    }

    spawns = {
        "start_a": {
            "display_name": f"Start A ({u0}, {v0})",
            "pixel": [u0, v0],
            "robot": primary_spawn,
            "clients": client_spawns,
        },
        "default": {
            "display_name": f"Default ({u0}, {v0})",
            "pixel": [u0, v0],
            "robot": primary_spawn,
            "clients": client_spawns,
        },
    }

    bounds = compute_bounds(width, height, resolution, origin, floor_height, wall_height)
    world_def: dict[str, Any] = {
        "schema_version": 1,
        "kind": "occupancy_extrusion",
        "world_id": world_id,
        "display_name": display_name,
        "bounds": bounds,
        "default_spawn": "start_a",
        "spawns": spawns,
        "source": {
            "image": "source.png",
            "original_pgm_name": pgm_path.name,
            "original_pgm_sha256": pgm_sha256,
            "source_png_sha256": png_sha256,
            "width": width,
            "height": height,
            "resolution": round(float(resolution), 6),
            "origin": [round(float(v), 6) for v in origin],
            "occupied_values": [0],
            "free_gray_policy": "free",
            "pixel_counts": {
                "occupied_0": occupied_count,
                "free_white_254": white_count,
                "free_gray_205": gray_205_count,
                "free_other": other_free_count,
            },
        },
        "extrusion": {
            "generator_version": 1,
            "floor_height": round(floor_height, 6),
            "floor_thickness": 0.3,
            "wall_height": round(wall_height, 6),
            "chunk_cells": chunk_cells,
            "chunks_file": "wall_chunks.json",
            "chunks_sha256": chunks_sha256,
            "capsule_clearance_radius_m": CAPSULE_CLEARANCE_RADIUS_M,
        },
    }
    if floor_texture_bytes is not None:
        world_def["appearance"] = {
            "floor_texture": "floor_texture.png",
            "floor_texture_sha256": sha256(floor_texture_bytes).hexdigest(),
        }
    world_def["world_hash"] = compute_world_hash(world_def)

    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "source.png").write_bytes(png_bytes)
    (output_dir / "wall_chunks.json").write_bytes(chunks_bytes)
    if floor_texture_bytes is not None:
        (output_dir / "floor_texture.png").write_bytes(floor_texture_bytes)
    (output_dir / "world.json").write_text(
        json.dumps(world_def, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    return world_def


def build_playground_world_definition(output_dir: Path) -> dict[str, Any]:
    """Write world.json for the built-in test world cub_playground_v1."""
    # In cub_playground_v1:
    # Robot starts at Godot (0, 0.1, 4), yaw 0 -> ROS (-4.0, 0.0, 0.0), yaw 0
    # Clients start at Godot (-2, 0.1, 6), (0, 0.1, 6), (2, 0.1, 6) -> ROS (-6.0, 2.0, 0.0), (-6.0, 0.0, 0.0), (-6.0, -2.0, 0.0)
    spawns = {
        "start_a": {
            "display_name": "Default Playground Start",
            "robot": {"x": -4.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0},
            "clients": [
                {"x": -6.0, "y": 2.0, "z": 0.0, "yaw_deg": 0.0},
                {"x": -6.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0},
                {"x": -6.0, "y": -2.0, "z": 0.0, "yaw_deg": 0.0},
            ],
        },
        "default": {
            "display_name": "Default Playground Start",
            "robot": {"x": -4.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0},
            "clients": [
                {"x": -6.0, "y": 2.0, "z": 0.0, "yaw_deg": 0.0},
                {"x": -6.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0},
                {"x": -6.0, "y": -2.0, "z": 0.0, "yaw_deg": 0.0},
            ],
        },
    }
    world_def: dict[str, Any] = {
        "schema_version": 1,
        "kind": "playground",
        "world_id": "cub_playground_v1",
        "display_name": "Cub Playground",
        "bounds": {
            "ros_min": [-25.0, -25.0, 0.0],
            "ros_max": [25.0, 25.0, 2.0],
            "godot_min": [-25.0, 0.0, -25.0],
            "godot_max": [25.0, 2.0, 25.0],
            "size_m": [50.0, 50.0, 2.0],
        },
        "default_spawn": "start_a",
        "spawns": spawns,
        "playground": {
            "generator_version": 1,
            "capsule_clearance_radius_m": CAPSULE_CLEARANCE_RADIUS_M,
        },
    }
    world_def["world_hash"] = compute_world_hash(world_def)
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "world.json").write_text(
        json.dumps(world_def, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    return world_def


def main() -> None:
    parser = argparse.ArgumentParser(description="Build Godot occupancy world assets from PGM and YAML.")
    parser.add_argument("--pgm", type=Path, default=DEFAULT_MAP_PGM, help="Input 8-bit PGM file path")
    parser.add_argument("--yaml", type=Path, default=DEFAULT_MAP_YAML, help="Optional ROS map.yaml for resolution/origin")
    parser.add_argument("--resolution", type=float, default=None, help="Map resolution in m/cell (overrides YAML)")
    parser.add_argument(
        "--origin",
        type=float,
        nargs=3,
        default=None,
        metavar=("X", "Y", "YAW"),
        help="Map origin [x, y, yaw_rad] in ROS map coordinates (overrides YAML)",
    )
    parser.add_argument("--world-id", type=str, default=DEFAULT_WORLD_ID, help="Stable world identifier")
    parser.add_argument("--display-name", type=str, default=DEFAULT_DISPLAY_NAME, help="Human-readable world name")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Output world directory (defaults to simulator/worlds/<world_id>)",
    )
    parser.add_argument(
        "--spawn-pixel",
        type=int,
        nargs=2,
        default=[780, 1469],
        metavar=("U", "V"),
        help="Primary robot start pixel (u, v) with top-left = (0, 0)",
    )
    parser.add_argument(
        "--spawn-yaw-deg",
        type=float,
        default=180.0,
        help="Primary robot start yaw in degrees (ROS +X = 0, left/CCW positive; 180 = image left)",
    )
    parser.add_argument("--chunk-cells", type=int, default=64, help="Chunk size in cells (default: 64)")
    parser.add_argument("--wall-height", type=float, default=0.5, help="Extruded wall height in meters (default: 0.5)")
    parser.add_argument(
        "--floor-texture",
        type=Path,
        default=None,
        help="Optional PNG floor image with the same canvas/coordinates as the PGM (nearest filtering, no resizing)",
    )
    parser.add_argument(
        "--skip-playground",
        action="store_true",
        help="Do not update simulator/worlds/cub_playground_v1/world.json",
    )
    args = parser.parse_args()

    yaml_meta: dict[str, Any] = {}
    if args.yaml is not None and args.yaml.exists():
        yaml_meta = parse_map_yaml(args.yaml)

    resolution = args.resolution if args.resolution is not None else yaml_meta.get("resolution")
    origin = list(args.origin) if args.origin is not None else yaml_meta.get("origin")
    if resolution is None or origin is None:
        raise SystemExit("Error: resolution and origin must be provided via --yaml or --resolution/--origin")

    output_dir = args.output_dir or (ROOT / "simulator/worlds" / args.world_id)
    world_def = build_occupancy_world(
        pgm_path=args.pgm,
        output_dir=output_dir,
        world_id=args.world_id,
        display_name=args.display_name,
        resolution=float(resolution),
        origin=[float(v) for v in origin],
        start_pixel=(int(args.spawn_pixel[0]), int(args.spawn_pixel[1])),
        start_yaw_deg=float(args.spawn_yaw_deg),
        chunk_cells=int(args.chunk_cells),
        wall_height=float(args.wall_height),
        floor_texture_path=args.floor_texture,
    )

    if not args.skip_playground:
        playground_dir = ROOT / "simulator/worlds/cub_playground_v1"
        build_playground_world_definition(playground_dir)

    robot_spawn = world_def["spawns"]["start_a"]["robot"]
    print(
        f"Built world '{world_def['world_id']}' ({world_def['display_name']}) -> {output_dir}\n"
        f"  hash: {world_def['world_hash']}\n"
        f"  image: {world_def['source']['width']}x{world_def['source']['height']} @ {world_def['source']['resolution']} m/cell\n"
        f"  robot start (ROS): x={robot_spawn['x']}, y={robot_spawn['y']}, yaw_deg={robot_spawn['yaw_deg']}"
    )


if __name__ == "__main__":
    main()

