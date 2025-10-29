#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Open3D GUI tool: Convert gigantic outdoor PLY (≈3 km scale) to Nav2-ready occupancy grid (PGM + YAML),
with support for sloped road regions via per-region plane-band filtering.

Open3D 0.19 対応:
- 2点ピック（レガシー）: 「1-alt) Pick 2 corners (legacy)」でサブプロセス経由の VisualizerWithEditing を使用。
- Band 可視化: 下限/上限オフセット平面 + 表示用点群のハイライト。
- PGM プレビュー: 生成中の OccGrid を GUI 内の別ウィンドウでグレースケール表示。
- remove_geometry は 0.19 仕様に合わせて1引数呼び出し。

Controls (left panel)
- Voxel size (display only), Resolution, Inflate radius
- Band lower/upper, Min/Max Z
- Preview toggle (band), Refresh button
- Buttons: New Region, Pick 2 corners (legacy), Estimate Plane, Apply Remove, Apply Rasterize, Undo Last, Save PGM+YAML, Open PGM Preview
"""

import math
import os
import sys
import tempfile
import subprocess
import numpy as np
import open3d as o3d
from dataclasses import dataclass

# ------------------------------
# Open3D 0.19 compatibility helpers
# ------------------------------

def _set_tooltip_safe(widget, text: str):
    try:
        if hasattr(widget, "set_tooltip"):
            widget.set_tooltip(text)
    except Exception:
        pass

def _sep():
    import open3d.visualization.gui as gui
    return gui.Label("")

def _legacy_pick_points_via_subprocess(pcd: o3d.geometry.PointCloud, num_points: int = 2):
    """サブプロセスで VisualizerWithEditing を開いて点をピック（GLFW衝突回避）。"""
    if not isinstance(pcd, o3d.geometry.PointCloud):
        raise TypeError("Only PointCloud is supported in this app.")

    with tempfile.TemporaryDirectory() as td:
        ply_path = os.path.join(td, "tmp_input_cloud.ply")
        out_path = os.path.join(td, "picked_points.npy")
        o3d.io.write_point_cloud(ply_path, pcd, write_ascii=False, compressed=False)

        child_code = f"""
import sys, numpy as np, open3d as o3d
ply_path = r'''{ply_path}'''
out_path = r'''{out_path}'''
num_points = {int(num_points)}
pcd = o3d.io.read_point_cloud(ply_path)
vis = o3d.visualization.VisualizerWithEditing()
vis.create_window(window_name="Pick points (Shift+LeftClick; Q/ESC to finish)", width=1280, height=800)
vis.add_geometry(pcd)
try:
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0.05, 0.05, 0.05])
    opt.point_size = 3.0
except Exception:
    pass
vis.run()
vis.destroy_window()
idx = vis.get_picked_points()
if len(idx) == 0:
    np.save(out_path, np.empty((0,3), dtype=np.float64))
else:
    idx = idx[:num_points]
    pts = np.asarray(pcd.points, dtype=np.float64)[idx, :] 
    np.save(out_path, pts)
"""
        proc = subprocess.run([sys.executable, "-c", child_code])
        if proc.returncode != 0:
            raise RuntimeError("Legacy picker subprocess failed.")
        if not os.path.isfile(out_path):
            return np.empty((0, 3))
        pts = np.load(out_path)
        return pts

# ------------------------------
# Utility: plane fit and distance
# ------------------------------

def fit_plane_svd(points: np.ndarray):
    if points.shape[0] < 3:
        raise ValueError("Not enough points to fit a plane")
    centroid = points.mean(axis=0)
    X = points - centroid
    _, _, Vt = np.linalg.svd(X, full_matrices=False)
    normal = Vt[-1, :]
    normal /= np.linalg.norm(normal)
    d = -np.dot(normal, centroid)
    return normal, d

def point_plane_signed_distance(pts: np.ndarray, n: np.ndarray, d: float):
    return pts @ n + d

# ------------------------------
# Occupancy grid accumulator
# ------------------------------

@dataclass
class GridSpec:
    res: float
    x0: float
    y0: float
    w: int
    h: int

class OccGrid:
    def __init__(self, spec: GridSpec):
        self.spec = spec
        self.grid = np.zeros((spec.h, spec.w), dtype=np.uint8)  # 0 free, 255 occupied

    def world_to_ij(self, x: np.ndarray, y: np.ndarray):
        i = np.floor((y - self.spec.y0) / self.spec.res).astype(np.int64)
        j = np.floor((x - self.spec.x0) / self.spec.res).astype(np.int64)
        return i, j

    def mark_points(self, pts_xy: np.ndarray, inflate_radius_m: float = 0.0):
        if pts_xy.size == 0:
            return 0
        i, j = self.world_to_ij(pts_xy[:, 0], pts_xy[:, 1])
        mask = (i >= 0) & (i < self.spec.h) & (j >= 0) & (j < self.spec.w)
        i, j = i[mask], j[mask]
        self.grid[i, j] = 255
        if inflate_radius_m > 0:
            r = max(1, int(np.ceil(inflate_radius_m / self.spec.res)))
            for di in range(-r, r + 1):
                ii = i + di
                valid_i = (ii >= 0) & (ii < self.spec.h)
                for dj in range(-r, r + 1):
                    jj = j + dj
                    valid = valid_i & (jj >= 0) & (jj < self.spec.w)
                    self.grid[ii[valid], jj[valid]] = 255
        return i.size

    def save_pgm(self, pgm_path: str):
        with open(pgm_path, 'wb') as f:
            header = f"P5\n{self.spec.w} {self.spec.h}\n255\n".encode('ascii')
            f.write(header)
            f.write(self.grid[::-1, :].tobytes())  # ROS 表示用に上下反転

    def save_yaml(self, yaml_path: str, pgm_filename_only: str, occupancy_threshold: float = 0.65,
                  free_threshold: float = 0.196, negate: int = 0):
        origin_x = float(self.spec.x0)
        origin_y = float(self.spec.y0)
        yaw = 0.0
        content = f"""
image: {pgm_filename_only}
resolution: {self.spec.res:.6f}
origin: [{origin_x:.6f}, {origin_y:.6f}, {yaw:.6f}]
negate: {negate}
occupied_thresh: {occupancy_threshold}
free_thresh: {free_threshold}
"""
        with open(yaml_path, 'w') as f:
            f.write(content)

# ------------------------------
# GUI App
# ------------------------------

import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering

class PLY2PGMApp:
    def __init__(self, ply_path: str):
        self.app = gui.Application.instance
        self.app.initialize()

        self.window = gui.Application.instance.create_window(
            title="PLY → PGM (sloped road bands)", width=1500, height=900)

        # 3D Scene
        self.scene = gui.SceneWidget()
        self.scene.scene = rendering.Open3DScene(self.window.renderer)
        if hasattr(self.scene, "set_on_point_picked"):
            try:
                self.scene.set_on_point_picked(self._on_point_picked)
            except Exception:
                # _toast 定義前でも落ちないように print でフォールバック
                print("[INFO] Point picking not available in this build (use legacy)")
        else:
            print("[INFO] Point picking not available in this build (use legacy)")
        self.window.add_child(self.scene)

        # Panel
        em = self.window.theme.font_size
        margin = 0.5 * em
        self.panel = gui.Vert(0, gui.Margins(margin, margin, margin, margin))
        self.panel_frame = gui.CollapsableVert("Controls", 0)
        self.panel.add_child(self.panel_frame)
        self._build_controls()
        self.panel.add_fixed(0.25 * em)
        self.window.add_child(self.panel)

        # Layout
        self.window.set_on_layout(self._on_layout)

        # Data
        self.original_pcd = None
        self.display_pcd = None
        self.o3d_display = None
        self.sel_first = None
        self.sel_second = None
        self.mode_new_region = False
        self.mode_pick_on_plane = False
        self._mouse_cb_installed = False
        self.current_plane = None
        self.original_plane = None
        self.plane_center = None
        self.removed_mask = None
        self.undo_stack = []
        self.grid = None

        # PGM preview
        self.pgm_win = None
        self.pgm_img_widget = None
        self.pgm_info_label = None

        # Load
        self._load_ply(ply_path)

    # -------------------------- UI --------------------------
    def _build_controls(self):
        def label(text):
            lbl = gui.Label(text)
            try:
                lbl.text_color = gui.Color(0.9, 0.9, 0.9)
            except Exception:
                pass
            return lbl
        em = self.window.theme.font_size

        self.voxel_edit = gui.NumberEdit(gui.NumberEdit.DOUBLE); self.voxel_edit.set_value(0.20)
        _set_tooltip_safe(self.voxel_edit, "Display voxel size (m). Affects visualization only.")
        self.res_edit = gui.NumberEdit(gui.NumberEdit.DOUBLE); self.res_edit.set_value(0.10)
        _set_tooltip_safe(self.res_edit, "Grid resolution (m/pixel) for PGM export.")
        self.inflate_edit = gui.NumberEdit(gui.NumberEdit.DOUBLE); self.inflate_edit.set_value(0.20)
        _set_tooltip_safe(self.inflate_edit, "Inflate occupied cells by this radius (m). 0 = no dilation.")

        # Manual region override
        self.chk_manual_region = gui.Checkbox("Override region manually")
        self.bbox_xmin_edit = gui.NumberEdit(gui.NumberEdit.DOUBLE)
        self.bbox_ymin_edit = gui.NumberEdit(gui.NumberEdit.DOUBLE)
        self.bbox_xmax_edit = gui.NumberEdit(gui.NumberEdit.DOUBLE)
        self.bbox_ymax_edit = gui.NumberEdit(gui.NumberEdit.DOUBLE)
        self.btn_get_region_from_selection = gui.Button("Get from Selection")
        self.btn_get_region_from_selection.set_on_clicked(self._on_get_region_from_selection)

        manual_region_v = gui.CollapsableVert("Manual Region Override", 0.25 * em)
        manual_region_v.set_is_open(False)
        manual_region_v.add_child(self.chk_manual_region)
        h_min = gui.Horiz(0.25 * em)
        h_min.add_child(gui.Label("X min:"))
        h_min.add_child(self.bbox_xmin_edit)
        h_min.add_child(gui.Label("Y min:"))
        h_min.add_child(self.bbox_ymin_edit)
        manual_region_v.add_child(h_min)
        h_max = gui.Horiz(0.25 * em)
        h_max.add_child(gui.Label("X max:"))
        h_max.add_child(self.bbox_xmax_edit)
        h_max.add_child(gui.Label("Y max:"))
        h_max.add_child(self.bbox_ymax_edit)
        manual_region_v.add_child(h_max)
        manual_region_v.add_child(self.btn_get_region_from_selection)

        self.lower_edit = gui.NumberEdit(gui.NumberEdit.DOUBLE); self.lower_edit.set_value(-0.05)
        _set_tooltip_safe(self.lower_edit, "Lower signed distance (m) to plane band.")
        self.upper_edit = gui.NumberEdit(gui.NumberEdit.DOUBLE); self.upper_edit.set_value(0.20)
        _set_tooltip_safe(self.upper_edit, "Upper signed distance (m) to plane band.")
        self.minz_edit = gui.NumberEdit(gui.NumberEdit.DOUBLE); self.minz_edit.set_value(-1e9)
        _set_tooltip_safe(self.minz_edit, "Min Z (world). Use -1e9 to disable.")
        self.maxz_edit = gui.NumberEdit(gui.NumberEdit.DOUBLE); self.maxz_edit.set_value(1e9)
        _set_tooltip_safe(self.maxz_edit, "Max Z (world). Use 1e9 to disable.")

        # Band preview controls
        self.chk_visualize_band = gui.Checkbox("Preview: visualize band (display only)")
        self.chk_visualize_band.checked = True
        self.chk_visualize_band.set_on_checked(lambda _: self._update_band_visual())

        self.btn_preview_band = gui.Button("Refresh Band Preview")
        self.btn_preview_band.set_on_clicked(self._update_band_visual)

        # PGM preview button
        self.btn_open_pgm_preview = gui.Button("Open PGM Preview window")
        self.btn_open_pgm_preview.set_on_clicked(self._open_pgm_preview)

        self.btn_pick_legacy = gui.Button("1) Pick 2 corners")
        self.btn_pick_legacy.set_on_clicked(self._on_pick_legacy_two_corners)

        self.btn_est_plane = gui.Button("2) Estimate Plane from Region"); self.btn_est_plane.set_on_clicked(self._on_estimate_plane)

        adj_panel = gui.CollapsableVert("2a) Fine-tune Plane Angle (deg)", 0.25 * em)
        adj_panel.set_is_open(True)
        self.adj_roll_edit = gui.NumberEdit(gui.NumberEdit.DOUBLE); self.adj_roll_edit.set_value(0.0)
        _set_tooltip_safe(self.adj_roll_edit, "Adjust plane Roll (X-axis rotation) in degrees.")
        self.adj_pitch_edit = gui.NumberEdit(gui.NumberEdit.DOUBLE); self.adj_pitch_edit.set_value(0.0)
        _set_tooltip_safe(self.adj_pitch_edit, "Adjust plane Pitch (Y-axis rotation) in degrees.")
        self.adj_yaw_edit = gui.NumberEdit(gui.NumberEdit.DOUBLE); self.adj_yaw_edit.set_value(0.0)
        _set_tooltip_safe(self.adj_yaw_edit, "Adjust plane Yaw (Z-axis rotation) in degrees.")
        h_roll = gui.Horiz(0.25*em); h_roll.add_child(gui.Label("Roll (X)")); h_roll.add_child(self.adj_roll_edit)
        adj_panel.add_child(h_roll)
        h_pitch = gui.Horiz(0.25*em); h_pitch.add_child(gui.Label("Pitch (Y)")); h_pitch.add_child(self.adj_pitch_edit)
        adj_panel.add_child(h_pitch)
        h_yaw = gui.Horiz(0.25*em); h_yaw.add_child(gui.Label("Yaw (Z)")); h_yaw.add_child(self.adj_yaw_edit)
        adj_panel.add_child(h_yaw)
        self.btn_reset_adj = gui.Button("Reset Angles")
        self.btn_reset_adj.set_on_clicked(self._on_reset_angles)
        adj_panel.add_child(self.btn_reset_adj)

        self.btn_pick_on_plane = gui.Button("2b) Pick 2 corners on plane (click anywhere)")
        self.btn_pick_on_plane.set_on_clicked(self._enable_plane_click_mode)
        self.btn_apply_remove = gui.Button("3A) Apply Remove (band)"); self.btn_apply_remove.set_on_clicked(self._on_apply_remove)
        self.btn_apply_raster = gui.Button("3B) Apply Rasterize → PGM (band)"); self.btn_apply_raster.set_on_clicked(self._on_apply_raster)
        self.btn_undo = gui.Button("Undo Last Apply"); self.btn_undo.set_on_clicked(self._on_undo)
        self.btn_save = gui.Button("Save PGM + YAML"); self.btn_save.set_on_clicked(self._on_save)

        self.status_label = gui.Label("")

        # Layouting
        for w in [label("Display"), label("Voxel size (m)"), self.voxel_edit,
                  _sep(),
                  label("PGM Export"), label("Resolution (m/px)"), self.res_edit,
                  label("Inflate radius (m)"), self.inflate_edit]:
            self.panel_frame.add_child(w)
        
        self.panel_frame.add_child(manual_region_v)

        for w in [_sep(),
                  label("Band to Plane (m)"), label("Lower"), self.lower_edit, label("Upper"), self.upper_edit,
                  _sep(),
                  label("Optional world Z filter (m)"), label("Min Z"), self.minz_edit, label("Max Z"), self.maxz_edit,
                  _sep(),
                  self.chk_visualize_band, self.btn_preview_band, self.btn_open_pgm_preview,
                  self.btn_pick_legacy, self.btn_est_plane, adj_panel, self.btn_pick_on_plane,
                  self.btn_apply_remove, self.btn_apply_raster,
                  self.btn_undo, _sep(), self.btn_save]:
            self.panel_frame.add_child(w)
        self.panel_frame.add_child(_sep())
        self.panel_frame.add_child(self.status_label)

        # Callbacks
        for ne in (self.lower_edit, self.upper_edit, self.minz_edit, self.maxz_edit):
            try:
                ne.set_on_value_changed(lambda _=None: self._update_band_visual())
            except Exception:
                pass
        
        self.chk_manual_region.set_on_checked(lambda _: self._on_manual_region_modified())
        for editor in [self.bbox_xmin_edit, self.bbox_ymin_edit, self.bbox_xmax_edit, self.bbox_ymax_edit]:
            try:
                editor.set_on_value_changed(lambda _=None: self._on_manual_region_modified())
            except Exception:
                pass

        for editor in [self.adj_roll_edit, self.adj_pitch_edit, self.adj_yaw_edit]:
            try:
                editor.set_on_value_changed(lambda _=None: self._on_angle_adjusted())
            except Exception:
                pass

    def _on_get_region_from_selection(self):
        bbox = self._get_current_bbox(ignore_manual=True)
        if bbox is None:
            self._toast("Select a region with the picker first.")
            return
        
        xmin, xmax, ymin, ymax = bbox
        self.bbox_xmin_edit.set_value(xmin)
        self.bbox_ymin_edit.set_value(ymin)
        self.bbox_xmax_edit.set_value(xmax)
        self.bbox_ymax_edit.set_value(ymax)
        self._toast("Populated manual region from current selection.")

    def _on_manual_region_modified(self):
        self._draw_region_bbox()
        self._update_band_visual()

    def _get_current_bbox(self, ignore_manual=False):
        if not ignore_manual and getattr(self, 'chk_manual_region', None) and self.chk_manual_region.checked:
            try:
                x1 = self.bbox_xmin_edit.double_value
                y1 = self.bbox_ymin_edit.double_value
                x2 = self.bbox_xmax_edit.double_value
                y2 = self.bbox_ymax_edit.double_value
                xmin, xmax = sorted([x1, x2])
                ymin, ymax = sorted([y1, y2])
                return (xmin, xmax, ymin, ymax)
            except Exception:
                return None # Should not happen
        else:
            if self.sel_first is None or self.sel_second is None:
                return None
            a = self.sel_first
            b = self.sel_second
            xmin, xmax = sorted([a[0], b[0]])
            ymin, ymax = sorted([a[1], b[1]])
            return (xmin, xmax, ymin, ymax)

    # --- Plane-click picking mode (arbitrary coords on estimated plane) ---
    def _enable_plane_click_mode(self):
        if self.current_plane is None:
            self._toast("Estimate a plane first.")
            return
        self.mode_pick_on_plane = True
        try:
            self.scene.set_mouse_mode(gui.SceneWidget.MouseMode.ROTATE_CAMERA)
        except Exception:
            pass
        # Install mouse callback only once
        if not getattr(self, "_mouse_cb_installed", False):
            try:
                self.scene.set_on_mouse(self._on_mouse_plane_pick)
                self._mouse_cb_installed = True
            except Exception:
                self._toast("Mouse callback not available in this Open3D build.")
                self.mode_pick_on_plane = False
                return
        self._toast("Plane-pick mode: left-click on the scene to choose 2 corners on the plane.")
        self._enable_plane_pick_mode()

    
    def _on_mouse_plane_pick(self, event):
        try:
            if not getattr(self, "mode_pick_on_plane", False):
                return gui.Widget.EventCallbackResult.IGNORED

            # マウスダウンのみ処理
            if event.type != gui.MouseEvent.Type.BUTTON_DOWN:
                return gui.Widget.EventCallbackResult.IGNORED

            # CTRL 押下は無視（API があれば）
            try:
                if hasattr(gui, "KeyModifier") and event.is_modifier_down(gui.KeyModifier.CTRL):
                    return gui.Widget.EventCallbackResult.IGNORED
            except Exception:
                pass

            # 左クリック限定（API があれば）
            try:
                if hasattr(event, "is_button_down") and hasattr(gui, "MouseButton"):
                    if not event.is_button_down(gui.MouseButton.LEFT):
                        return gui.Widget.EventCallbackResult.IGNORED
            except Exception:
                pass

            if self.current_plane is None:
                self._toast("Estimate a plane first.")
                return gui.Widget.EventCallbackResult.CONSUMED

            cam = self.scene.scene.camera
            w = float(self.scene.frame.width)
            h = float(self.scene.frame.height)
            if w <= 0 or h <= 0:
                return gui.Widget.EventCallbackResult.CONSUMED

            # 画面座標 → NDC
            x_ndc = 2.0 * (event.x / w) - 1.0
            y_ndc = 1.0 - 2.0 * (event.y / h)

            import numpy as np
            from numpy.linalg import inv, norm

            P = np.asarray(cam.get_projection_matrix(), dtype=np.float64).T
            V = np.asarray(cam.get_view_matrix(), dtype=np.float64).T
            M = inv(P @ V)

            def unproject(xn, yn, zn):
                p = M @ np.array([xn, yn, zn, 1.0], dtype=np.float64)
                if abs(p[3]) > 1e-12:
                    p = p / p[3]
                return p[:3]

            # カメラ位置（world）
            Vinv = inv(V)
            cam_pos = Vinv[:3, 3]

            p_far = unproject(x_ndc, y_ndc, 1.0)
            ray_dir = p_far - cam_pos
            L = float(norm(ray_dir))
            if L < 1e-9:
                return gui.Widget.EventCallbackResult.CONSUMED
            ray_dir = ray_dir / L
            ray_org = cam_pos

            n, d = self.current_plane
            n = np.asarray(n, dtype=np.float64)

            def intersect(n, d, ray_org, ray_dir):
                denom = float(np.dot(n, ray_dir))
                if abs(denom) < 1e-9:
                    return None
                t = -(float(np.dot(n, ray_org)) + float(d)) / denom
                return t

            t1 = intersect(n, d, ray_org, ray_dir)
            t2 = intersect(-n, -d, ray_org, ray_dir)

            t_candidates = [t for t in [t1, t2] if t is not None]

            # 数値誤差で微小負になることがあるので小さく許容
            EPS = 1e-5
            t_pos = [t for t in t_candidates if t is not None and t >= -EPS]

            if not t_pos:
                self._toast("plane is behind the camera.")
                return gui.Widget.EventCallbackResult.CONSUMED

            t = max(0.0, min(t_pos))  # 微小負なら 0 に丸め、最も近い交点を採用
            hit = ray_org + t * ray_dir            

            # 2点記録 → 矩形確定
            if hasattr(self, "_record_click"):
                self._record_click(hit)
            else:
                if self.sel_first is None:
                    self.sel_first = hit; self.sel_second = None
                    self._toast(f"First corner set: ({hit[0]:.2f}, {hit[1]:.2f}, {hit[2]:.2f})")
                else:
                    self.sel_second = hit
                    self._toast(f"Second corner set: ({hit[0]:.2f}, {hit[1]:.2f}, {hit[2]:.2f})")
                    self.mode_new_region = False
                    self.mode_pick_on_plane = False
                    try:
                        self.scene.set_mouse_mode(gui.SceneWidget.MouseMode.ROTATE_CAMERA)
                    except Exception:
                        pass
                    self._draw_region_bbox()
                    self._update_band_visual()
                    self._disable_plane_pick_mode()

            return gui.Widget.EventCallbackResult.CONSUMED

        except Exception as e:
            self._toast(f"plane-pick error: {e}")
            return gui.Widget.EventCallbackResult.HANDLED
    
    def _on_layout(self, layout_context):
        r = self.window.content_rect
        panel_width = 500
        self.panel.frame = gui.Rect(r.get_right() - panel_width, r.y, panel_width, r.height)
        self.scene.frame = gui.Rect(r.x, r.y, r.width - panel_width, r.height)

    # ------------------------ Interaction ------------------------
    def _on_point_picked(self, pick_result):
        # 0.20+ のみで有効
        try:
            if not self.mode_new_region:
                return
            if pick_result.is_valid:
                world = np.array([pick_result.world_coordinate[0],
                                  pick_result.world_coordinate[1],
                                  pick_result.world_coordinate[2]])
                self._record_click(world)
        except Exception as e:
            self._fatal(f"Pick failed: {e}")

    def _record_click(self, world_xyz):
        if self.sel_first is None:
            self.sel_first = world_xyz
            self._toast("First corner set.")
        else:
            self.sel_second = world_xyz
            self._toast("Second corner set.")
            self.mode_new_region = False
            self.mode_pick_on_plane = False
            self._mouse_cb_installed = False
            try:
                self.scene.set_mouse_mode(gui.SceneWidget.MouseMode.ROTATE_CAMERA)
            except Exception:
                pass
            self._draw_region_bbox()
            self._update_band_visual()

    def _on_pick_legacy_two_corners(self):
        """VisualizerWithEditing（レガシー）で2点を取得して矩形を確定"""
        try:
            # レガシーピック実行（外部サブプロセスなど。既存の関数呼び出しを利用）
            pts = _legacy_pick_points_via_subprocess(self.original_pcd, num_points=2)

            if pts is None or getattr(pts, "shape", (0, 0))[0] == 0:
                self._toast("点が選択されませんでした。")
                return

            if pts.shape[0] == 1:
                self.sel_first = pts[0]
                self.sel_second = None
                self._toast("1点目を取得。もう一度押して2点目を取得してください。")
                return

            # 2点取得できた
            self.sel_first = pts[0]
            self.sel_second = pts[1]
            self.mode_new_region = False
            self.mode_pick_on_plane = False
            self._mouse_cb_installed = False
            try:
                self.scene.set_mouse_mode(gui.SceneWidget.MouseMode.ROTATE_CAMERA)
            except Exception:
                pass

            self._draw_region_bbox()
            self._update_band_visual()
            self._toast("2点を取得しました。矩形を描画しました。")

        except Exception as e:
            self._fatal(f"Legacy pick failed: {e}")

    def _enable_plane_pick_mode(self):
        # 他モードを排他
        self.mode_new_region = False
        self.mode_pick_on_plane = True

        # 角の初期化（いきなり Second を防止）
        self.sel_first = None
        self.sel_second = None

        # デバウンス/再入防止の初期化（あれば）
        self._click_busy = False
        self._last_click_xy = None
        self._last_click_time = 0.0

        # マウスコールバックの二重登録防止
        if not getattr(self, "_mouse_cb_installed", False):
            self.scene.set_on_mouse(self._on_mouse_plane_pick)
            self._mouse_cb_installed = True

        # カメラ操作モード→ピックしやすいモードに（任意）
        try:
            self.scene.set_mouse_mode(gui.SceneWidget.MouseMode.ROTATE_CAMERA)
        except Exception:
            pass

        self._toast("Plane-pick mode: left-click on the scene to choose 2 corners on the plane.")

    def _disable_plane_pick_mode(self):
        self.mode_pick_on_plane = False
        if getattr(self, "_mouse_cb_installed", False):
            # 以後は通常操作に戻る
            self.scene.set_on_mouse(None)
            self._mouse_cb_installed = False
    
    def _on_estimate_plane(self):
        """Estimate a plane inside the selected XY region.
        Pipeline:
        - XY crop from original cloud (respecting 'removed_mask')
        - Optional random downsample (to <= 100k points)
        - Try Open3D RANSAC (segment_plane); on failure or too few inliers, fallback to SVD.
        - Flip normal to face the camera (toward camera position).
        """
        # XY領域抽出
        region_pts = self._get_region_points()
        if region_pts.size == 0:
            self._toast("No points in region.")
            return
        import numpy as np
        pts = region_pts
        # ダウンサンプル（高速化）
        if pts.shape[0] > 100_000:
            idx = np.random.choice(pts.shape[0], size=100_000, replace=False)
            pts_ds = pts[idx]
        else:
            pts_ds = pts

        n = None; d = None; inlier_count = 0

        # RANSAC
        try:
            pcd_tmp = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts_ds))
            xs, ys = pts_ds[:,0], pts_ds[:,1]
            span = max(float(xs.max()-xs.min()), float(ys.max()-ys.min()))
            thresh = max(0.01, min(0.30, span * 0.01))
            plane_model, inliers = pcd_tmp.segment_plane(distance_threshold=thresh, ransac_n=3, num_iterations=2000)
            if plane_model and len(plane_model) == 4 and len(inliers) >= 50:
                a,b,c,d0 = map(float, plane_model)
                n = np.array([a,b,c], dtype=float)
                n_norm = np.linalg.norm(n)
                if n_norm > 1e-12:
                    n = n / n_norm
                    d = float(d0) / n_norm
                inlier_count = len(inliers)
        except Exception:
            n = None; d = None

        # フォールバック：SVD
        if n is None or d is None:
            n, d = fit_plane_svd(pts_ds)
            inlier_count = 0

        # カメラから見て表向きに反転
        try:
            cam = self.scene.scene.camera
            import numpy.linalg as LA
            v = np.asarray(cam.get_view_matrix(), dtype=np.float64).T
            cam_pos = LA.inv(v)[:3, 3]
            center = pts_ds.mean(axis=0)
            to_cam = cam_pos - center
            if float(np.dot(n, to_cam)) < 0.0:
                n = -n; d = -d
        except Exception:
            pass

        center = pts_ds.mean(axis=0)
        d_recalc = -np.dot(n, center)

        self.plane_center = center
        self.original_plane = (n, d_recalc)
        self.current_plane = (n, d_recalc)
        
        if hasattr(self, "adj_roll_edit"):
            self.adj_roll_edit.set_value(0.0)
            self.adj_pitch_edit.set_value(0.0)
            self.adj_yaw_edit.set_value(0.0)

        self._draw_plane(n, d_recalc, pts_ds)
        msg = f"Plane estimated. |n|=1, n=({n[0]:.3f},{n[1]:.3f},{n[2]:.3f}), d={d_recalc:.3f}"
        if inlier_count > 0:
            msg += f"  [RANSAC inliers: {inlier_count}]"
        self._toast(msg)
        self._update_band_visual()

    def _on_angle_adjusted(self):
        if self.original_plane is None or self.plane_center is None:
            return

        n_orig, _ = self.original_plane
        center = self.plane_center

        roll = np.deg2rad(self.adj_roll_edit.double_value)
        pitch = np.deg2rad(self.adj_pitch_edit.double_value)
        yaw = np.deg2rad(self.adj_yaw_edit.double_value)

        Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        R = Rz @ Ry @ Rx

        n_new = R @ n_orig
        n_new /= np.linalg.norm(n_new)
        d_new = -np.dot(n_new, center)

        self.current_plane = (n_new, d_new)

        # Redraw plane visualization. We need some points to get the size of the visualization quad.
        # _get_region_points() is the most straightforward way.
        region_pts = self._get_region_points()
        if region_pts.size > 0:
            self._draw_plane(n_new, d_new, region_pts)
        
        self._update_band_visual()

    def _on_reset_angles(self):
        if not hasattr(self, "adj_roll_edit"):
            return
        # Setting value will trigger _on_angle_adjusted through the callback
        self.adj_roll_edit.set_value(0.0)
        self.adj_pitch_edit.set_value(0.0)
        self.adj_yaw_edit.set_value(0.0)
        self._toast("Reset angle adjustments.")


    def _on_apply_remove(self):
            if self.current_plane is None:
                self._toast("Estimate a plane first.")
                return
            lower = self.lower_edit.double_value
            upper = self.upper_edit.double_value
            zmin = self.minz_edit.double_value
            zmax = self.maxz_edit.double_value
            n, d = self.current_plane

            pts = np.asarray(self.original_pcd.points)
            region_mask = self._region_mask(pts)
            if region_mask.sum() == 0:
                self._toast("Region empty.")
                return
            dists = point_plane_signed_distance(pts[region_mask], n, d)
            band_mask = (dists >= lower) & (dists < upper)
            z = pts[region_mask, 2]
            band_mask &= (z >= zmin) & (z <= zmax)

            idx_region = np.where(region_mask)[0]
            idx_remove = idx_region[band_mask]
            if idx_remove.size == 0:
                self._toast("No points within band.")
                return

            if self.removed_mask is None:
                self.removed_mask = np.zeros(len(pts), dtype=bool)

            self.removed_mask[idx_remove] = True
            self.undo_stack.append(("remove", idx_remove))
            self._update_display()
            self._toast(f"Removed {idx_remove.size} points in band.")

    def _on_apply_raster(self):
        if self.current_plane is None:
            self._toast("Estimate a plane first.")
            return
        res = self.res_edit.double_value
        inf_r = self.inflate_edit.double_value
        lower = self.lower_edit.double_value
        upper = self.upper_edit.double_value
        zmin = self.minz_edit.double_value
        zmax = self.maxz_edit.double_value
        n, d = self.current_plane

        pts = np.asarray(self.original_pcd.points)
        region_mask = self._region_mask(pts)
        if region_mask.sum() == 0:
            self._toast("Region empty.")
            return
        pts_r = pts[region_mask]
        dists = point_plane_signed_distance(pts_r, n, d)
        band_mask = (dists >= lower) & (dists < upper)
        z = pts_r[:, 2]
        band_mask &= (z >= zmin) & (z <= zmax)
        pts_band = pts_r[band_mask]

        if pts_band.size == 0:
            self._toast("No points within band.")
            return

        if self.grid is None:
            self.grid = self._make_grid_spec(pts)
        count = self.grid.mark_points(pts_band[:, [0, 1]], inflate_radius_m=inf_r)
        self.undo_stack.append(("raster", pts_band[:, [0, 1]]))
        self._toast(f"Rasterized {count} cells from {pts_band.shape[0]} points.")
        self._update_pgm_preview()  # プレビュー更新

    def _on_undo(self):
        if not self.undo_stack:
            self._toast("Nothing to undo.")
            return
        action, payload = self.undo_stack.pop()
        if action == "remove":
            idx = payload
            self.removed_mask[idx] = False
            self._update_display()
            self._toast(f"Undo remove: {idx.size} points restored.")
        elif action == "raster" and self.grid is not None:
            all_pts = []
            for a, p in self.undo_stack:
                if a == "raster":
                    all_pts.append(p)
            self.grid = self._make_grid_spec(np.asarray(self.original_pcd.points))
            if all_pts:
                for p in all_pts:
                    self.grid.mark_points(p, inflate_radius_m=self.inflate_edit.double_value)
            self._toast("Undo raster: reverted last accumulation.")
            self._update_pgm_preview()
        else:
            self._toast("Unknown undo action.")

    def _on_save(self):
        if self.grid is None:
            self._toast("Nothing rasterized yet.")
            return
        dlg = gui.FileDialog(gui.FileDialog.SAVE, "Save PGM + YAML", self.window.theme)
        dlg.set_on_cancel(lambda: None)
        def on_ok(path):
            if not path.lower().endswith('.pgm'):
                path = path + '.pgm'
            yaml_path = os.path.splitext(path)[0] + '.yaml'
            self.grid.save_pgm(path)
            self.grid.save_yaml(yaml_path, os.path.basename(path))
            self._toast(f"Saved: {path} and {yaml_path}")
        dlg.set_on_done(on_ok)
        self.window.show_dialog(dlg)

    # ------------------------ Helpers ------------------------
    def _load_ply(self, ply_path: str):
        if not os.path.isfile(ply_path):
            self._fatal(f"PLY not found: {ply_path}")
            return
        self._toast("Loading PLY ... (this may take a while)")
        pcd = o3d.io.read_point_cloud(ply_path)
        if len(pcd.points) == 0:
            self._fatal("Empty PLY.")
            return
        self.original_pcd = pcd
        vox = self.voxel_edit.double_value
        dsp = pcd.voxel_down_sample(vox) if vox > 0 else pcd
        self.display_pcd = dsp
        self.o3d_display = o3d.geometry.PointCloud(dsp)
        mat = rendering.MaterialRecord(); mat.shader = "defaultUnlit"; mat.point_size = 1.5
        self.scene.scene.add_geometry("pcd", dsp, mat)
        bounds = dsp.get_axis_aligned_bounding_box()
        self.scene.setup_camera(60.0, bounds, bounds.get_center())
        try:
            self.scene.scene.set_sun_light([1,1,1], [0,-1,0], 65000, True)
            self.scene.scene.enable_sun_light(True)
        except Exception:
            pass
        self.scene.scene.show_axes(True)
        self._toast(f"Loaded {len(pcd.points)} points. Displaying {len(dsp.points)} points.")

    def _update_display(self):
        if self.removed_mask is None:
            return
        pts = np.asarray(self.original_pcd.points)
        keep = ~self.removed_mask
        pcd_keep = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts[keep]))
        vox = self.voxel_edit.double_value
        dsp = pcd_keep.voxel_down_sample(vox) if vox > 0 else pcd_keep
        self.scene.scene.clear_geometry()
        mat = rendering.MaterialRecord(); mat.shader = "defaultUnlit"; mat.point_size = 1.5
        self.scene.scene.add_geometry("pcd", dsp, mat)
        self.scene.scene.show_axes(True)

    def _get_region_points(self):
        bbox = self._get_current_bbox()
        if bbox is None:
            self._toast("Set two corners first.")
            return np.empty((0,3))
        
        xmin, xmax, ymin, ymax = bbox
        pts = np.asarray(self.original_pcd.points)
        mask = (pts[:,0] >= xmin) & (pts[:,0] <= xmax) & (pts[:,1] >= ymin) & (pts[:,1] <= ymax)
        if self.removed_mask is not None:
            mask &= (~self.removed_mask)
        return pts[mask]

    def _region_mask(self, pts):
        bbox = self._get_current_bbox()
        if bbox is None:
            return np.zeros(len(pts), dtype=bool)

        xmin, xmax, ymin, ymax = bbox
        mask = (pts[:,0] >= xmin) & (pts[:,0] <= xmax) & (pts[:,1] >= ymin) & (pts[:,1] <= ymax)
        if self.removed_mask is not None and len(pts) == len(self.removed_mask):
            mask &= (~self.removed_mask)
        return mask

    def _draw_region_bbox(self):
        bbox = self._get_current_bbox()
        if bbox is None:
            try:
                self.scene.scene.remove_geometry("region")
            except Exception:
                pass
            return

        xmin, xmax, ymin, ymax = bbox
        zmin, zmax = -1e3, 1e3
        corners = [
            [xmin, ymin, zmin], [xmax, ymin, zmin], [xmax, ymax, zmin], [xmin, ymax, zmin],
            [xmin, ymin, zmax], [xmax, ymin, zmax], [xmax, ymax, zmax], [xmin, ymax, zmax],
        ]
        lines = [[0,1],[1,2],[2,3],[3,0],[4,5],[5,6],[6,7],[7,4],[0,4],[1,5],[2,6],[3,7]]
        line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(np.array(corners)),
            lines=o3d.utility.Vector2iVector(np.array(lines)))
        colors = np.tile(np.array([[0.2, 0.9, 0.2]]), (len(lines), 1))
        line_set.colors = o3d.utility.Vector3dVector(colors)
        try:
            self.scene.scene.remove_geometry("region")
        except Exception:
            pass
        mat = rendering.MaterialRecord(); mat.shader = "defaultUnlitLine"
        self.scene.scene.add_geometry("region", line_set, mat)

    def _draw_plane(self, n, d, region_pts):
        if region_pts.size == 0:
            return
        center = region_pts.mean(axis=0)
        n = n / np.linalg.norm(n)
        v = np.array([1,0,0])
        if abs(np.dot(v, n)) > 0.9:
            v = np.array([0,1,0])
        u1 = v - np.dot(v, n) * n; u1 /= np.linalg.norm(u1)
        u2 = np.cross(n, u1)
        a = self.sel_first; b = self.sel_second
        w = abs(a[0]-b[0]); h = abs(a[1]-b[1])
        s = max(w, h); sz = max(1.0, s)
        quad = np.array([
            center + (-sz)*u1 + (-sz)*u2,
            center + ( sz)*u1 + (-sz)*u2,
            center + ( sz)*u1 + ( sz)*u2,
            center + (-sz)*u1 + ( sz)*u2,
        ])
        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(quad)
        mesh.triangles = o3d.utility.Vector3iVector(np.array([[0,1,2],[0,2,3]]))
        mesh.compute_vertex_normals()
        mesh.paint_uniform_color([0.1,0.6,0.9])
        try:
            self.scene.scene.remove_geometry("plane")
        except Exception:
            pass
        mat = rendering.MaterialRecord(); mat.shader = "defaultLit"
        self.scene.scene.add_geometry("plane", mesh, mat)

    def _draw_band_planes(self, n, d, lower, upper):
        if self.sel_first is None or self.sel_second is None:
            return
        n = n / np.linalg.norm(n)
        v = np.array([1, 0, 0])
        if abs(np.dot(v, n)) > 0.9:
            v = np.array([0, 1, 0])
        u1 = v - np.dot(v, n) * n; u1 /= np.linalg.norm(u1)
        u2 = np.cross(n, u1)

        a = self.sel_first; b = self.sel_second
        w = abs(a[0] - b[0]); h = abs(a[1] - b[1])
        s = max(w, h); sz = max(1.0, s)
        center = np.array([(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5])

        def make_quad(center_on_plane):
            return np.array([
                center_on_plane + (-sz)*u1 + (-sz)*u2,
                center_on_plane + ( sz)*u1 + (-sz)*u2,
                center_on_plane + ( sz)*u1 + ( sz)*u2,
                center_on_plane + (-sz)*u1 + ( sz)*u2,
            ])

        center_lower = center + n * lower
        center_upper = center + n * upper

        for name, c, col in [("band_lower", center_lower, [0.2, 0.8, 0.2]),
                             ("band_upper", center_upper, [0.9, 0.5, 0.1])]:
            quad = make_quad(c)
            mesh = o3d.geometry.TriangleMesh()
            mesh.vertices = o3d.utility.Vector3dVector(quad)
            mesh.triangles = o3d.utility.Vector3iVector(np.array([[0,1,2],[0,2,3]]))
            mesh.compute_vertex_normals()
            mesh.paint_uniform_color(col)
            mat = rendering.MaterialRecord()
            try:
                mat.shader = "defaultLitTransparency"
                mat.base_color = (*col, 0.35)
            except Exception:
                mat.shader = "defaultLit"
            try:
                self.scene.scene.remove_geometry(name)
            except Exception:
                pass
            self.scene.scene.add_geometry(name, mesh, mat)

    def _update_band_visual(self):
        # remove previous
        for name in ("band_pts", "band_lower", "band_upper"):
            try:
                self.scene.scene.remove_geometry(name)
            except Exception:
                pass

        if not getattr(self, "chk_visualize_band", None):
            return
        if not self.chk_visualize_band.checked:
            return
        if self.current_plane is None:
            self._toast("Plane not estimated yet.")
            return
        
        bbox = self._get_current_bbox()
        if bbox is None:
            self._toast("Select region first.")
            return

        if self.display_pcd is None:
            return

        n, d = self.current_plane
        lower = self.lower_edit.double_value
        upper = self.upper_edit.double_value
        zmin = self.minz_edit.double_value
        zmax = self.maxz_edit.double_value

        # draw offset planes
        self._draw_band_planes(n, d, lower, upper)

        # highlight points within band in display cloud
        dsp_pts = np.asarray(self.display_pcd.points)
        if dsp_pts.size == 0:
            return
        mask_region = self._region_mask(dsp_pts)
        if not np.any(mask_region):
            self._toast("Region has no display points.")
            return
        pts_r = dsp_pts[mask_region]
        dists = point_plane_signed_distance(pts_r, n, d)
        mask_band = (dists >= lower) & (dists < upper)
        z = pts_r[:, 2]
        mask_band &= (z >= zmin) & (z <= zmax)
        if not np.any(mask_band):
            self._toast("No display points in band.")
            return

        band_pts = pts_r[mask_band]
        pcd_band = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(band_pts))
        pcd_band.paint_uniform_color([0.0, 0.9, 0.9])
        mat = rendering.MaterialRecord(); mat.shader = "defaultUnlit"; mat.point_size = 3.0
        self.scene.scene.add_geometry("band_pts", pcd_band, mat)

    # ------------------------ PGM Preview ------------------------
    def _open_pgm_preview(self):
        if self.pgm_win is None:
            self.pgm_win = gui.Application.instance.create_window("PGM Preview", 720, 720)
            layout = gui.Vert(4, gui.Margins(6, 6, 6, 6))
            self.pgm_img_widget = gui.ImageWidget()
            self.pgm_info_label = gui.Label("No grid yet")
            layout.add_child(self.pgm_img_widget)
            layout.add_child(self.pgm_info_label)
            self.pgm_win.add_child(layout)
            # on_close cleanup to prevent use-after-free on 2nd open
            def _on_close():
                try:
                    self.pgm_win = None
                    self.pgm_img_widget = None
                    self.pgm_info_label = None
                except Exception:
                    pass
                return True
            try:
                self.pgm_win.set_on_close(_on_close)
            except Exception:
                pass
        # Always (re)draw current buffer
        self._update_pgm_preview()

    def _update_pgm_preview(self):
        if self.pgm_win is None or self.pgm_img_widget is None:
            return
        if self.grid is None:
            self.pgm_info_label.text = "No grid yet (run Rasterize first)"
            return
        # 表示は保存時と同じ上下反転でユーザー認知を合わせる
        img = self.grid.grid[::-1, :].astype(np.uint8)  # (H, W)
        # Open3DのImageWidgetは1chでも表示可能だが、環境によってはRGBを要求するため3chに拡張
        rgb = np.repeat(img[:, :, None], 3, axis=2)     # (H, W, 3)
        o3d_img = o3d.geometry.Image(rgb)
        try:
            self.pgm_img_widget.update_image(o3d_img)   # 新API（存在しない場合は set_image を使用）
        except Exception:
            try:
                self.pgm_img_widget.set_image(o3d_img)
            except Exception as e:
                self._toast(f"Failed to update PGM preview: {e}")
                return
        spec = self.grid.spec
        self.pgm_info_label.text = f"size: {spec.w}x{spec.h} px  |  res: {spec.res:.3f} m/px  |  origin: ({spec.x0:.2f}, {spec.y0:.2f})"

    def _make_grid_spec(self, pts_all: np.ndarray) -> 'OccGrid':
        x = pts_all[:,0]; y = pts_all[:,1]
        res = self.res_edit.double_value
        pad = 2 * res
        xmin = math.floor((x.min() - pad) / res) * res
        ymin = math.floor((y.min() - pad) / res) * res
        xmax = math.ceil((x.max() + pad) / res) * res
        ymax = math.ceil((y.max() + pad) / res) * res
        w = int(round((xmax - xmin) / res))
        h = int(round((ymax - ymin) / res))
        spec = GridSpec(res=res, x0=float(xmin), y0=float(ymin), w=w, h=h)
        return OccGrid(spec)

    def _toast(self, text):
        try:
            if hasattr(self.window, 'show_message_box'):
                gui.Application.instance.post_to_main_thread(
                    self.window, lambda: self.window.show_message_box('Info', str(text)))
                print(f"[INFO] {text}")
                return
        except Exception:
            pass
        try:
            gui.Application.instance.post_to_main_thread(
                self.window, lambda: setattr(self.status_label, 'text', str(text)))
        except Exception:
            try:
                self.status_label.text = str(text)
            except Exception:
                pass
        print(f"[INFO] {text}")

    def _fatal(self, text):
        try:
            if hasattr(self.window, 'show_message_box'):
                gui.Application.instance.post_to_main_thread(
                    self.window, lambda: self.window.show_message_box('Error', str(text)))
                print(f"[ERROR] {text}")
                return
        except Exception:
            pass
        try:
            gui.Application.instance.post_to_main_thread(
                self.window, lambda: setattr(self.status_label, 'text', f"ERROR: {text}"))
        except Exception:
            try:
                self.status_label.text = f"ERROR: {text}"
            except Exception:
                pass
        print(f"[ERROR] {text}")

    # ------------------------ Run ------------------------
    def run(self):
        self.app.run()

def main():
    if len(sys.argv) < 2:
        print("Usage: o3d_ply_to_pgm.py <path_to_large.ply>")
        return
    app = PLY2PGMApp(sys.argv[1])
    app.run()

if __name__ == "__main__":
    main()
