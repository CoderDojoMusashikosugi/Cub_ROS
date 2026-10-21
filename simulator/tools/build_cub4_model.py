#!/usr/bin/env python3
"""Build Cub4's display-only GLB from cub_description using Python's stdlib.

The URDF remains the source of scale/origin; this is intentionally a Cub4
asset build step rather than a runtime URDF loader. No ROS or Blender needed.
"""

from array import array
from hashlib import sha256
import json
import math
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]
URDF = ROOT / "ros/cub_description/urdf/cub4.urdf"
OUTPUT = ROOT / "simulator/assets/cub4/cub4.glb"


def rotate_rpy(vector, angles):
    """URDF fixed-axis roll, pitch, yaw: Rz(yaw) Ry(pitch) Rx(roll)."""
    x, y, z = vector
    roll, pitch, yaw = angles
    y, z = math.cos(roll) * y - math.sin(roll) * z, math.sin(roll) * y + math.cos(roll) * z
    x, z = math.cos(pitch) * x + math.sin(pitch) * z, -math.sin(pitch) * x + math.cos(pitch) * z
    return math.cos(yaw) * x - math.sin(yaw) * y, math.sin(yaw) * x + math.cos(yaw) * y, z


def ros_to_godot(vector):
    # ROS +X forward, +Y left, +Z up -> Godot -Z forward, -X left, +Y up.
    x, y, z = vector
    return -y, z, -x


def read_materials(path):
    colors = {}
    name = None
    for line in path.read_text(encoding="utf-8").splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "newmtl":
            name = " ".join(parts[1:])
        elif parts[0] == "Kd":
            colors[name] = tuple(round(float(channel) * 255) for channel in parts[1:4]) + (255,)
        elif parts[0].startswith("map_") or (parts[0] == "d" and float(parts[1]) != 1.0):
            raise ValueError("Cub4's single-material export supports opaque colors without textures")
    return colors


def little_endian(values):
    if sys.byteorder != "little" and values.itemsize > 1:
        values = values[:]
        values.byteswap()
    return values.tobytes()


def build():
    visuals = ET.parse(URDF).findall("./link/visual")
    if len(visuals) != 1:
        raise ValueError("Expected Cub4's single base_link visual; review the asset builder if the URDF changes")
    visual = visuals[0]
    mesh = visual.find("geometry/mesh")
    package = ROOT / "ros/cub_description"
    filename = mesh.attrib["filename"].replace("file://$(find cub_description)/", "").replace("package://cub_description/", "")
    source = package / filename
    scale = tuple(map(float, mesh.get("scale", "1 1 1").split()))
    origin = visual.find("origin")
    xyz = tuple(map(float, origin.get("xyz", "0 0 0").split())) if origin is not None else (0, 0, 0)
    rpy = tuple(map(float, origin.get("rpy", "0 0 0").split())) if origin is not None else (0, 0, 0)
    positions, normals = [], []
    vertices, vertex_normals, vertex_colors, indices = array("f"), array("f"), array("B"), array("I")
    assert vertices.itemsize == 4 and indices.itemsize == 4
    unique = {}
    materials, material_path, color = {}, None, None

    def vertex_index(reference):
        parts = reference.split("/")
        position_index = int(parts[0])
        normal_index = int(parts[2])
        position_index = position_index - 1 if position_index > 0 else len(positions) + position_index
        normal_index = normal_index - 1 if normal_index > 0 else len(normals) + normal_index
        key = position_index, normal_index, color
        if key not in unique:
            unique[key] = len(unique)
            vertices.extend(positions[position_index])
            vertex_normals.extend(normals[normal_index])
            vertex_colors.extend(color)
        return unique[key]

    for line in source.read_text(encoding="utf-8").splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "mtllib":
            material_path = source.parent / " ".join(parts[1:])
            materials = read_materials(material_path)
        elif parts[0] == "usemtl":
            color = materials[" ".join(parts[1:])]
        elif parts[0] == "v":
            point = rotate_rpy(tuple(float(value) * factor for value, factor in zip(parts[1:4], scale)), rpy)
            positions.append(ros_to_godot(tuple(value + offset for value, offset in zip(point, xyz))))
        elif parts[0] == "vn":
            normal = ros_to_godot(rotate_rpy(tuple(float(value) / factor for value, factor in zip(parts[1:4], scale)), rpy))
            length = math.sqrt(sum(value * value for value in normal))
            normals.append(tuple(value / length for value in normal))
        elif parts[0] == "f":
            face = [vertex_index(reference) for reference in parts[1:]]
            for index in range(1, len(face) - 1):
                indices.extend((face[0], face[index], face[index + 1]))

    minimum = [min(vertices[axis::3]) for axis in range(3)]
    maximum = [max(vertices[axis::3]) for axis in range(3)]
    metadata = {
        "source_sha256": {str(path.relative_to(ROOT)): sha256(path.read_bytes()).hexdigest() for path in (URDF, source, material_path)},
        "urdf_visual": {"scale": scale, "xyz": xyz, "rpy": rpy},
        "ros_to_godot": "(-y, z, -x)",
        "triangles": len(indices) // 3,
        "vertices": len(unique),
        "bounds_m": {"min": minimum, "max": maximum},
    }
    binary = bytearray()
    views = []
    for values, target in ((vertices, 34962), (vertex_normals, 34962), (vertex_colors, 34962), (indices, 34963)):
        binary.extend(b"\0" * (-len(binary) % 4))
        data = little_endian(values)
        views.append({"buffer": 0, "byteOffset": len(binary), "byteLength": len(data), "target": target})
        binary.extend(data)
    document = {
        "asset": {"version": "2.0", "generator": "Cub_ROS/build_cub4_model.py", "extras": metadata},
        "scene": 0,
        "scenes": [{"name": "Cub4", "nodes": [0]}],
        "nodes": [{"name": "Cub4", "mesh": 0}],
        "meshes": [{"name": "Cub4", "primitives": [{"attributes": {"POSITION": 0, "NORMAL": 1, "COLOR_0": 2}, "indices": 3, "material": 0}]}],
        # All source materials are opaque matte colors. Store their diffuse
        # colors per vertex so the entire CAD model needs only one draw call.
        "materials": [{"name": "Cub4 colors", "pbrMetallicRoughness": {"baseColorFactor": [1, 1, 1, 1], "metallicFactor": 0, "roughnessFactor": 1}}],
        "buffers": [{"byteLength": len(binary)}],
        "bufferViews": views,
        "accessors": [
            {"bufferView": 0, "componentType": 5126, "count": len(unique), "type": "VEC3", "min": minimum, "max": maximum},
            {"bufferView": 1, "componentType": 5126, "count": len(unique), "type": "VEC3"},
            {"bufferView": 2, "componentType": 5121, "normalized": True, "count": len(unique), "type": "VEC4"},
            {"bufferView": 3, "componentType": 5125, "count": len(indices), "type": "SCALAR"},
        ],
    }
    json_data = json.dumps(document, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    json_data += b" " * (-len(json_data) % 4)
    binary.extend(b"\0" * (-len(binary) % 4))
    glb = (struct.pack("<III", 0x46546C67, 2, 28 + len(json_data) + len(binary))
           + struct.pack("<II", len(json_data), 0x4E4F534A) + json_data
           + struct.pack("<II", len(binary), 0x004E4942) + binary)
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_bytes(glb)
    OUTPUT.with_suffix(".json").write_text(json.dumps(metadata, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"{OUTPUT.relative_to(ROOT)}: {len(indices) // 3:,} triangles, {len(unique):,} vertices, one surface, {len(glb):,} bytes")
    print("Godot dimensions (m):", [round(high - low, 6) for low, high in zip(minimum, maximum)])


if __name__ == "__main__":
    build()
