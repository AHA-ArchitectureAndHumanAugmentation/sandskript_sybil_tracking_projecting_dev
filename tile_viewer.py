"""Shared compas_viewer display logic -- mirrors Charlotte's view_utils.py
exactly (same camera framing math, same colors, same structure).

Runs as a SEPARATE PROCESS per window, launched via subprocess from
main.py -- not called directly in-process. compas_viewer's OpenGL
context is thread-affine; two windows in the same process (even on
different threads) crash with a GL context error. A fresh process per
window sidesteps that entirely.

Usage (called by main.py, not run by hand normally):
    python tile_viewer.py flat <data.json>
    python tile_viewer.py projected <data.json>
"""

import json
import statistics
import sys

import numpy as np
from compas.colors import Color
from compas.datastructures import Mesh
from compas.geometry import Point, Polyline, Vector
from compas_viewer import Viewer

VIEW_MODE = "perspective"

AXIS_LENGTH_SCALE = 0.8
Z_AXIS_LENGTH_SCALE = 1
SHOW_EVERY_NTH_FRAME = 1
SHOW_NORMALS = True
SHOW_SURFACE = True

AXIS_LINEWIDTH = 0.8
Z_AXIS_LINEWIDTH = 1
POLYLINE_WIDTH = 1
FRAME_BOX_WIDTH = 0.5

TRAIL_POINT_SIZE = 8
START_POINT_SIZE = 10
END_POINT_SIZE = 10
ORIGIN_POINT_SIZE = 10

SURFACE_OPACITY = 0.25

# Colors -- identical values to Charlotte's view_utils.py, for direct comparison
COLOR_BEFORE = Color(0.88, 0.88, 0.86)
COLOR_AFTER = Color(0.15, 0.15, 0.18)
COLOR_START = Color(0.40, 0.78, 0.62)
COLOR_END = Color(0.85, 0.40, 0.40)
COLOR_ORIGIN = Color(0.95, 0.65, 0.25)
COLOR_SURFACE = Color(0.75, 0.75, 0.78)
COLOR_FRAME_BOX = Color(0.6, 0.6, 0.65)

WORLD_ORIGIN = Point(0.0, 0.0, 0.0)


def _rotvec_to_axes(rx, ry, rz):
    r = np.array([rx, ry, rz])
    theta = np.linalg.norm(r)
    if theta < 1e-9:
        return (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)
    k = r / theta
    K = np.array([
        [0, -k[2], k[1]],
        [k[2], 0, -k[0]],
        [-k[1], k[0], 0],
    ])
    R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)
    return tuple(R @ [1.0, 0.0, 0.0]), tuple(R @ [0.0, 1.0, 0.0]), tuple(R @ [0.0, 0.0, 1.0])


def show_flat_stroke_preview(pixel_strokes, frame_width, frame_height, mm_per_px):
    m_per_px = mm_per_px / 1000.0
    frame_w_m = frame_width * m_per_px
    frame_h_m = frame_height * m_per_px

    print(f"[tile_viewer] Camera frame: {frame_w_m:.3f} x {frame_h_m:.3f} m ({mm_per_px:.3f} mm/px)")
    total_points = sum(len(s) for s in pixel_strokes)
    print(f"[tile_viewer] Raw stroke: {total_points} points, flat, before projection")

    viewer = Viewer(show_grid=False, viewmode=VIEW_MODE)

    camera = viewer.renderer.camera
    camera.target = [frame_w_m / 2, frame_h_m / 2, 0]
    camera.position = [frame_w_m / 2 - frame_w_m, frame_h_m / 2 - frame_h_m, frame_w_m * 0.75]
    camera.near = max(frame_w_m, frame_h_m) * 0.001
    camera.far = max(frame_w_m, frame_h_m) * 10

    frame_box = Polyline([
        Point(0, 0, 0), Point(frame_w_m, 0, 0),
        Point(frame_w_m, frame_h_m, 0), Point(0, frame_h_m, 0),
        Point(0, 0, 0),
    ])
    viewer.scene.add(frame_box, linecolor=COLOR_FRAME_BOX, linewidth=FRAME_BOX_WIDTH, show_points=False, name="camera frame")

    for s_i, stroke in enumerate(pixel_strokes):
        points = [Point(x * m_per_px, y * m_per_px, 0) for x, y in stroke]
        viewer.scene.add(Polyline(points), linecolor=COLOR_BEFORE, linewidth=POLYLINE_WIDTH, show_points=False, name=f"stroke {s_i}: raw (pre-projection)")
        for p in points:
            viewer.scene.add(p, pointcolor=Color(0.53, 0.53, 0.5), pointsize=TRAIL_POINT_SIZE, name="raw point")

    viewer.scene.add(WORLD_ORIGIN, pointcolor=COLOR_ORIGIN, pointsize=ORIGIN_POINT_SIZE, name="origin (0,0,0)")

    print("[tile_viewer] Showing RAW stroke.")
    viewer.show()


def show_projection_preview(
    mesh_vertices,
    mesh_faces,
    robot_strokes,
    tile_id,
    show_every_nth_frame=SHOW_EVERY_NTH_FRAME,
    show_normals=SHOW_NORMALS,
    show_surface=SHOW_SURFACE,
):
    print(f"World origin (m): ({WORLD_ORIGIN.x:.3f}, {WORLD_ORIGIN.y:.3f}, {WORLD_ORIGIN.z:.3f})")

    all_points = [pose[:3] for stroke in robot_strokes for pose in stroke]
    print(f"[tile_viewer] Projected stroke: {len(all_points)} points after raycasting")

    xs = [p[0] for p in all_points]
    ys = [p[1] for p in all_points]
    zs = [p[2] for p in all_points]
    path_span = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs), 0.001)

    if len(all_points) > 1:
        gaps = [
            ((all_points[i + 1][0] - all_points[i][0]) ** 2
             + (all_points[i + 1][1] - all_points[i][1]) ** 2
             + (all_points[i + 1][2] - all_points[i][2]) ** 2) ** 0.5
            for i in range(len(all_points) - 1)
        ]
        spacing = statistics.median(gaps)
        print(f"Point spacing (m) -- min: {min(gaps):.4f}  median: {spacing:.4f}  max: {max(gaps):.4f}")
    else:
        spacing = path_span

    axis_length = spacing * AXIS_LENGTH_SCALE
    z_axis_length = spacing * Z_AXIS_LENGTH_SCALE
    print(f"Path span (m): {path_span:.3f}   axis length (m): {axis_length:.4f}   z-axis length (m): {z_axis_length:.4f}")

    surface_mesh = Mesh.from_vertices_and_faces(mesh_vertices, mesh_faces) if show_surface else None

    framing_xs, framing_ys, framing_zs = list(xs), list(ys), list(zs)
    if surface_mesh is not None:
        surface_points = [surface_mesh.vertex_coordinates(vkey) for vkey in surface_mesh.vertices()]
        framing_xs += [p[0] for p in surface_points]
        framing_ys += [p[1] for p in surface_points]
        framing_zs += [p[2] for p in surface_points]
        bbox_min = [min(p[i] for p in surface_points) for i in range(3)]
        bbox_max = [max(p[i] for p in surface_points) for i in range(3)]
        print(f"[tile_viewer] Tile bounding box (m): "
              f"{bbox_max[0]-bbox_min[0]:.3f} x {bbox_max[1]-bbox_min[1]:.3f} x {bbox_max[2]-bbox_min[2]:.3f}")

    center = Point(
        (max(framing_xs) + min(framing_xs)) / 2,
        (max(framing_ys) + min(framing_ys)) / 2,
        (max(framing_zs) + min(framing_zs)) / 2,
    )
    framing_span = max(
        max(framing_xs) - min(framing_xs),
        max(framing_ys) - min(framing_ys),
        max(framing_zs) - min(framing_zs),
        0.001,
    )

    viewer = Viewer(show_grid=False, viewmode=VIEW_MODE)

    camera = viewer.renderer.camera
    camera.target = [center.x, center.y, center.z]
    camera.position = [center.x - framing_span, center.y - framing_span, center.z + framing_span * 0.75]
    camera.near = framing_span * 0.001
    camera.far = framing_span * 10

    if surface_mesh is not None:
        viewer.scene.add(surface_mesh, facecolor=COLOR_SURFACE, opacity=SURFACE_OPACITY, name=f"tile_{tile_id:03d}")

    for s_i, stroke in enumerate(robot_strokes):
        points = [Point(*pose[:3]) for pose in stroke]
        viewer.scene.add(Polyline(points), linecolor=COLOR_AFTER, linewidth=POLYLINE_WIDTH, show_points=False, name=f"stroke {s_i}: projected path")

    point_counter = 0
    for stroke in robot_strokes:
        for i, pose in enumerate(stroke):
            point = Point(*pose[:3])
            viewer.scene.add(point, pointcolor=Color(0.53, 0.53, 0.5), pointsize=TRAIL_POINT_SIZE, name=f"{point_counter}: point")

            if show_normals and i % show_every_nth_frame == 0:
                xaxis, yaxis, zaxis = _rotvec_to_axes(pose[3], pose[4], pose[5])
                viewer.scene.add(Vector(*xaxis).scaled(axis_length), anchor=point, linecolor=Color.red(), linewidth=AXIS_LINEWIDTH, name=f"{point_counter}: x-axis")
                viewer.scene.add(Vector(*yaxis).scaled(axis_length), anchor=point, linecolor=Color.green(), linewidth=AXIS_LINEWIDTH, name=f"{point_counter}: y-axis")
                viewer.scene.add(Vector(*zaxis).scaled(z_axis_length), anchor=point, linecolor=Color.blue(), linewidth=Z_AXIS_LINEWIDTH, name=f"{point_counter}: z-axis")

            point_counter += 1

    if robot_strokes and robot_strokes[0]:
        viewer.scene.add(Point(*robot_strokes[0][0][:3]), pointcolor=COLOR_START, pointsize=START_POINT_SIZE, name="start")
        viewer.scene.add(Point(*robot_strokes[-1][-1][:3]), pointcolor=COLOR_END, pointsize=END_POINT_SIZE, name="end")
    viewer.scene.add(WORLD_ORIGIN, pointcolor=COLOR_ORIGIN, pointsize=ORIGIN_POINT_SIZE, name="world origin")

    print("[tile_viewer] Showing PROJECTED result.")
    viewer.show()


if __name__ == "__main__":
    mode = sys.argv[1]
    with open(sys.argv[2], "r", encoding="utf-8") as f:
        data = json.load(f)

    if mode == "flat":
        show_flat_stroke_preview(data["pixel_strokes"], data["frame_width"], data["frame_height"], data["mm_per_px"])
    elif mode == "projected":
        show_projection_preview(data["mesh_vertices"], data["mesh_faces"], data["robot_strokes"], data["tile_id"])
    else:
        raise ValueError(f"Unknown mode: {mode!r}")