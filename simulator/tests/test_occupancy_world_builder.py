#!/usr/bin/env python3
"""Unit tests for simulator/tools/build_occupancy_world.py."""

import json
from hashlib import sha256
import math
from pathlib import Path
import struct
import sys
import tempfile
import unittest
import zlib

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "simulator/tools"))

import build_occupancy_world as bow  # noqa: E402


def _decode_grayscale_png(png_bytes: bytes) -> tuple[int, int, bytes]:
    assert png_bytes[:8] == b"\x89PNG\r\n\x1a\n"
    idx = 8
    width = height = 0
    idat = bytearray()
    while idx < len(png_bytes):
        length = struct.unpack(">I", png_bytes[idx : idx + 4])[0]
        tag = png_bytes[idx + 4 : idx + 8]
        body = png_bytes[idx + 8 : idx + 8 + length]
        idx += 12 + length
        if tag == b"IHDR":
            width, height, bit_depth, color_type, _, _, _ = struct.unpack(">IIBBBBB", body)
            assert bit_depth == 8 and color_type == 0
        elif tag == b"IDAT":
            idat.extend(body)
        elif tag == b"IEND":
            break
    raw = zlib.decompress(bytes(idat))
    stride = width + 1
    assert len(raw) == stride * height
    out = bytearray(width * height)
    for y in range(height):
        assert raw[y * stride] == 0
        out[y * width : (y + 1) * width] = raw[y * stride + 1 : (y + 1) * stride]
    return width, height, bytes(out)


class OccupancyWorldBuilderTest(unittest.TestCase):
    def test_pgm_parsing_and_lossless_png_roundtrip(self) -> None:
        w, h = 6, 4
        pixels = bytes([
            0, 0, 205, 254, 128, 1,
            0, 254, 254, 254, 205, 254,
            254, 254, 0, 254, 254, 254,
            205, 205, 205, 254, 254, 0,
        ])
        p5_data = f"P5\n# comment line\n{w} {h}\n255\n".encode("ascii") + pixels
        pw, ph, pmax, parsed = bow.parse_pgm(p5_data)
        self.assertEqual((pw, ph, pmax, parsed), (w, h, 255, pixels))

        p2_body = " ".join(str(b) for b in pixels)
        p2_data = f"P2\n{w} {h}\n255\n{p2_body}\n".encode("ascii")
        self.assertEqual(bow.parse_pgm(p2_data), (w, h, 255, pixels))

        png_bytes = bow.encode_grayscale_png(w, h, pixels)
        dw, dh, decoded = _decode_grayscale_png(png_bytes)
        self.assertEqual((dw, dh, decoded), (w, h, pixels))

        with self.assertRaises(ValueError):
            bow.parse_pgm(b"P5\n2 2\n65535\n\x00\x00\x00\x00\x00\x00\x00\x00")

    def test_coordinates_rotated_origin_and_gray_clearance(self) -> None:
        w, h = 10, 8
        r = 0.2
        origin = [10.0, -5.0, math.pi / 2.0]
        # Top-left cell (u=0, v=0) center in map-local is (0.1, 1.5).
        # With yaw = +90 deg: ros_x = 10.0 - 1.5 = 8.5, ros_y = -5.0 + 0.1 = -4.9.
        rx, ry = bow.cell_center_to_ros(0, 0, w, h, r, origin)
        self.assertAlmostEqual(rx, 8.5, places=6)
        self.assertAlmostEqual(ry, -4.9, places=6)
        u_back, v_back = bow.ros_to_cell(rx, ry, w, h, r, origin)
        self.assertAlmostEqual(u_back, 0.5, places=6)
        self.assertAlmostEqual(v_back, 0.5, places=6)

        grid = bytearray([205] * (w * h))
        for u in range(w):
            for v in range(h):
                if u < 5 and (u + v) % 2 == 0:
                    grid[v * w + u] = 180
        for dv in (-1, 0, 1):
            for du in (-1, 0, 1):
                if (du, dv) != (0, 0):
                    grid[(4 + dv) * w + (7 + du)] = 0
                else:
                    grid[(4 + dv) * w + (7 + du)] = 254

        gray_rx, gray_ry = bow.cell_center_to_ros(2, 4, w, h, r, origin)
        ok_gray, err_gray = bow.check_spawn_clearance(gray_rx, gray_ry, w, h, r, origin, bytes(grid))
        self.assertTrue(ok_gray, err_gray)

        hole_rx, hole_ry = bow.cell_center_to_ros(7, 4, w, h, r, origin)
        ok_hole, err_hole = bow.check_spawn_clearance(hole_rx, hole_ry, w, h, r, origin, bytes(grid))
        self.assertFalse(ok_hole)
        self.assertIn("occupied wall cell", err_hole)

    def test_deterministic_world_generation_in_tempdir(self) -> None:
        w, h = 12, 10
        r = 0.2
        origin = [-1.0, -1.0, 0.0]
        grid = bytearray([254] * (w * h))
        for u in range(w):
            grid[0 * w + u] = 0
        grid[5 * w + 5] = 205
        pgm_bytes = f"P5\n{w} {h}\n255\n".encode("ascii") + bytes(grid)

        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            pgm_path = tmp_path / "custom.pgm"
            pgm_path.write_bytes(pgm_bytes)
            out1 = tmp_path / "world1"
            out2 = tmp_path / "world2"
            def1 = bow.build_occupancy_world(
                pgm_path=pgm_path,
                output_dir=out1,
                world_id="test_world",
                display_name="Test World",
                resolution=r,
                origin=origin,
                start_pixel=(5, 5),
                start_yaw_deg=90.0,
            )
            def2 = bow.build_occupancy_world(
                pgm_path=pgm_path,
                output_dir=out2,
                world_id="test_world",
                display_name="Test World",
                resolution=r,
                origin=origin,
                start_pixel=(5, 5),
                start_yaw_deg=90.0,
            )
            self.assertEqual(def1["world_hash"], def2["world_hash"])
            self.assertEqual((out1 / "world.json").read_bytes(), (out2 / "world.json").read_bytes())
            self.assertEqual((out1 / "source.png").read_bytes(), (out2 / "source.png").read_bytes())
            self.assertEqual((out1 / "wall_chunks.json").read_bytes(), (out2 / "wall_chunks.json").read_bytes())

    def test_generated_gnss2026_world_matches_plan_specifications(self) -> None:
        world_dir = ROOT / "simulator/worlds/gnss2026_mid360_1017_02"
        world_json = world_dir / "world.json"
        source_png = world_dir / "source.png"
        chunks_json = world_dir / "wall_chunks.json"
        self.assertTrue(world_json.exists() and source_png.exists() and chunks_json.exists())

        meta = json.loads(world_json.read_text(encoding="utf-8"))
        self.assertEqual(meta["world_id"], "gnss2026_mid360_1017_02")
        self.assertEqual(meta["kind"], "occupancy_extrusion")
        self.assertEqual(meta["schema_version"], 1)

        src = meta["source"]
        self.assertEqual((src["width"], src["height"]), (3580, 2122))
        self.assertAlmostEqual(src["resolution"], 0.2)
        self.assertEqual(src["origin"], [-92.4275, -100.706, 0.0])
        appearance = meta["appearance"]
        texture_bytes = (world_dir / appearance["floor_texture"]).read_bytes()
        self.assertEqual(bow.read_png_dimensions(texture_bytes), (3580, 2122))
        self.assertEqual(sha256(texture_bytes).hexdigest(), appearance["floor_texture_sha256"])
        self.assertEqual(bow.compute_world_hash(meta), meta["world_hash"])
        self.assertEqual(src["original_pgm_name"], "map_sim.pgm")
        self.assertEqual(src["original_pgm_sha256"],
                         "51a84e586ffac121234e7b678f22cb2100b37ef37d428aded3f43bee8b3cb95f")
        self.assertEqual(sha256(source_png.read_bytes()).hexdigest(), src["source_png_sha256"])
        self.assertEqual(sha256(chunks_json.read_bytes()).hexdigest(), meta["extrusion"]["chunks_sha256"])
        counts = src["pixel_counts"]
        self.assertEqual(counts["occupied_0"], 55246)
        self.assertEqual(counts["free_white_254"], 7541126)
        self.assertEqual(counts["free_gray_205"], 388)
        self.assertEqual(counts["free_other"], 0)

        ext = meta["extrusion"]
        self.assertAlmostEqual(ext["wall_height"], 0.5)
        self.assertAlmostEqual(ext["floor_height"], 0.0)
        self.assertEqual(ext["chunk_cells"], 64)

        chunks = json.loads(chunks_json.read_text(encoding="utf-8"))
        self.assertEqual(chunks["stats"]["non_empty_chunks"], 304)
        self.assertEqual(chunks["stats"]["global_runs"], 17329)
        self.assertEqual(chunks["stats"]["global_merged_rects"], 12704)

        robot_start = meta["spawns"]["start_a"]["robot"]
        self.assertAlmostEqual(robot_start["x"], 63.6725, places=4)
        self.assertAlmostEqual(robot_start["y"], 29.794, places=4)
        self.assertAlmostEqual(robot_start["yaw_deg"], 180.0, places=4)

    def test_optional_floor_texture_preserves_pixels_and_geometry(self) -> None:
        w, h = 12, 10
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            pgm = tmp_path / "map.pgm"
            pgm.write_bytes(f"P5\n{w} {h}\n255\n".encode("ascii") + bytes([254] * w * h))
            texture = tmp_path / "intensity.png"
            texture_bytes = bow.encode_grayscale_png(w, h, bytes(range(w * h)))
            texture.write_bytes(texture_bytes)
            params = dict(
                pgm_path=pgm, world_id="test_world", display_name="Test",
                resolution=0.2, origin=[10.0, -5.0, math.pi / 2], start_pixel=(5, 5),
            )
            plain_dir, textured_dir = tmp_path / "plain", tmp_path / "textured"
            plain = bow.build_occupancy_world(output_dir=plain_dir, **params)
            textured = bow.build_occupancy_world(
                output_dir=textured_dir, floor_texture_path=texture, **params,
            )
            self.assertNotIn("appearance", plain)
            self.assertEqual((textured_dir / "floor_texture.png").read_bytes(), texture_bytes)
            self.assertEqual(textured["appearance"]["floor_texture_sha256"], sha256(texture_bytes).hexdigest())
            for key in ("source", "extrusion", "bounds", "spawns"):
                self.assertEqual(plain[key], textured[key])
            self.assertEqual((plain_dir / "wall_chunks.json").read_bytes(), (textured_dir / "wall_chunks.json").read_bytes())
            self.assertNotEqual(plain["world_hash"], textured["world_hash"])
            texture.write_bytes(bow.encode_grayscale_png(w, h, bytes([128] * w * h)))
            changed = bow.build_occupancy_world(
                output_dir=textured_dir, floor_texture_path=texture, **params,
            )
            self.assertNotEqual(changed["world_hash"], textured["world_hash"])
            texture.write_bytes(bow.encode_grayscale_png(w - 1, h, bytes([0] * (w - 1) * h)))
            with self.assertRaisesRegex(ValueError, "must match PGM dimensions"):
                bow.build_occupancy_world(output_dir=textured_dir, floor_texture_path=texture, **params)
            texture.write_bytes(b"not a PNG")
            with self.assertRaisesRegex(ValueError, "must be a PNG"):
                bow.build_occupancy_world(output_dir=textured_dir, floor_texture_path=texture, **params)


if __name__ == "__main__":
    unittest.main()

