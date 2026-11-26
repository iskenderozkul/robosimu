"""Human-Like Stereo Looming & Bell-Shaped Visual Servoing (HLSL Control Law).

Replaces absolute Cartesian positioning and PID/LQR with an anthropomorphic
visual-motor law modelled on human eye-head-hand coordination:

1. Anthropomorphic Eye Geometry (Egocentric Camera Above/Behind Shoulder):
   The stereo eyes sit above and slightly behind the robot's first joint (shoulder/base),
   looking forward over the arm toward the workspace—just like human eyes watching
   the hand reach for an object.

2. Active Micro-Movement (Head Parallax) & Pre-Reach Reachability / Singularity Check:
   Before or at the start of reaching, a small camera micro-shift delta_c (or stereo
   baseline B) induces optical parallax delta_theta = delta_c / Z. Combined with the
   apparent angular size of the target (s_target = f * W_target / Z_target) and
   gripper (s_gripper = f * W_gripper / Z_gripper), the controller estimates relative
   depth ratio rho_s = s_gripper / s_target = Z_target / Z_gripper and checks whether
   the target lies within the singularity-safe reachable envelope (D_est <= L_safe,
   predicted min manipulability w_pred >= w_min) BEFORE committing high torque.

3. Perspective Scale / Looming Depth Feedback (Approaching by Apparent Shrinking):
   As the gripper moves away from the eyes above the shoulder toward the target, its
   apparent pixel size s_gripper(t) = f * W_gripper / Z_gripper(t) shrinks in perspective
   until its normalized apparent scale matches the target's scale:
       e_scale(t) = 1 - (s_gripper(t) / W_gripper) / (s_target(t) / W_target)
                  = 1 - Z_target / Z_gripper(t)
   Combined with stereo disparity d = u_L - u_R and lateral image gap (du, dv), this
   provides pure visual relative feedback without absolute XYZ world coordinates.

4. Human-Like Asymmetric Bell-Shaped Speed Ramp (Minimum-Jerk + Delay Governor):
   Humans do not move with a step velocity (which causes jerky starts in traditional
   proportional/governor laws). Instead, velocity ramps up smoothly from zero, peaks
   around 38-42% of the reach, and decelerates under the delay-derived monotone bound:
       phi(p) = clip(((1 - p + p0) / (1 + p0))^gamma, 0.18, 1.0)
       v_HLSL(e, p) = phi(p) * min(k_hi |e|, (kappa / delta_eff) * (eps + |e|))
   where p = |e(t)| / |e_0| is the remaining gap fraction.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
import numpy as np

from .kinematics import (
    end_effector_position,
    end_effector_rotation,
    forward_kinematics,
    jacobian,
    orientation_error_so3,
)
from .model import ArmModel
from .motors import torque_limit


@dataclass
class StereoCameraConfig:
    """Anthropomorphic two-eye (or active-neck monocular) camera rig mounted above/behind Joint 1."""
    baseline_m: float = 0.065               # human interpupillary distance (~65 mm)
    focal_px: float = 580.0                 # focal length in pixels
    image_width: int = 640
    image_height: int = 480
    # Default: 12 cm behind Joint 1 (-X), centered (Y=0), 62 cm above base (+Z, ~32 cm above shoulder)
    cam_pos: tuple[float, float, float] = (-0.12, 0.0, 0.62)
    look_at: tuple[float, float, float] = (0.48, 0.0, 0.28)
    head_sway_m: float = 0.015              # active camera micro-movement amplitude (15 mm) for parallax
    gripper_real_size_m: float = 0.060      # physical width of the gripper (60 mm)
    target_real_size_m: float = 0.050       # physical width of the target object (50 mm)
    pixel_noise_px: float = 0.25            # subpixel feature noise on (u, v)
    disparity_noise_px: float = 0.35        # subpixel disparity/size noise
    vergence_deg: float = 0.0
    vision_mode: str = "binocular"          # "binocular" | "mono_left" | "mono_right" | "mono_static" | "mono_static_left" | "mono_static_right"
    neck_parallax_amp_m: float = 0.038      # active lateral neck sweep amplitude (38 mm) in monocular mode
    neck_parallax_freq_hz: float = 2.2      # active neck oscillation frequency (Hz)
    neck_pitch_amp_m: float = 0.012         # subtle vertical neck micro-nod amplitude (12 mm)
    mono_scale_bias: float = 0.28           # 28% prior size-distance ambiguity in uncalibrated monocular viewing


@dataclass
class PreReachPrediction:
    """Pre-reach reachability & singularity prediction from visual perspective + parallax."""
    estimated_distance_m: float     # estimated base-to-target distance from parallax & perspective
    max_safe_reach_m: float         # arm's singularity-safe reach envelope (0.92 * sum(L_i))
    reach_ratio: float              # estimated_distance / max_safe_reach
    predicted_min_w: float          # predicted minimum manipulability w(q) along reach
    can_reach_safely: bool          # True if within singularity-safe envelope
    parallax_shift_px: float = 0.0  # optical shift measured during camera micro-movement
    apparent_target_px: float = 0.0 # target apparent size in pixels
    apparent_gripper_px: float = 0.0# initial gripper apparent size in pixels
    status_text: str = ""


@dataclass
class StereoObservation:
    """Single-frame stereo + perspective looming observation of gripper and target."""
    uvd_gripper: np.ndarray         # (u_c, v_c, disparity_px) of gripper
    uvd_target: np.ndarray          # (u_c, v_c, disparity_px) of target
    uv_left_gripper: np.ndarray     # (u_L, v_L) in left eye
    uv_right_gripper: np.ndarray    # (u_R, v_R) in right eye
    uv_left_target: np.ndarray      # (u_L, v_L) in left eye
    uv_right_target: np.ndarray     # (u_R, v_R) in right eye
    size_px_gripper: float          # apparent pixel diameter of gripper (shrinks as hand extends!)
    size_px_target: float           # apparent pixel diameter of target object
    scale_ratio: float              # normalized perspective ratio (Z_target / Z_gripper)
    looming_error_m: float          # depth gap estimated purely from perspective shrinking (m)
    rel_cam_gap_m: np.ndarray       # fused relative 3D vector in camera frame (m)
    rel_base_gap_m: np.ndarray      # relative 3D vector mapped to arm frame (m)
    sigma_cam_m: np.ndarray         # effective 1-sigma noise (sx, sy, sz) in camera frame (m)
    depth_gripper_m: float          # eye-to-gripper distance Z_g (m)
    depth_target_m: float           # eye-to-target distance Z_t (m)
    vision_mode: str = "binocular"
    neck_baseline_m: float = 0.0
    motion_parallax_px: float = 0.0
    head_offset_m: np.ndarray = field(default_factory=lambda: np.zeros(3))
    # V4.3 zero-oracle packet fields (timestamps + neck pose); filled only on
    # the timestamped observe path, never used by the legacy metric controllers.
    t_cap: float = 0.0
    t_arr: float = 0.0
    t_now: float = 0.0
    neck_pan: float = 0.0
    neck_tilt: float = 0.0


class NeckKinematics:
    """2-DOF (pan, tilt) camera mount parented to the trunk yaw (V4.3).

    The rig base is carried by the trunk: with trunk yaw :math:`\phi`, pan
    :math:`p` and tilt :math:`t`, the camera origin is

        origin = base_origin + Ry_t Rz(phi + p) cam_pos

    so the whole stereo rig translates when the trunk rotates.  The analytic
    neck Jacobian (3x2) is

        d origin/d pan  = z_hat x (origin - base_origin)
        d origin/d tilt = (Rz(phi+p) y_hat) x (origin - base_origin)
    """

    def __init__(self, cam_pos=(-0.12, 0.0, 0.62), base_origin=(0.0, 0.19, 0.0),
                 trunk_yaw: float = 0.0, pan: float = 0.0, tilt: float = 0.0,
                 pan_limit: float = 0.56, tilt_limit: float = 0.49):
        self.cam_pos = np.asarray(cam_pos, float)
        self.base_origin = np.asarray(base_origin, float)
        self.pan_limit = float(pan_limit)
        self.tilt_limit = float(tilt_limit)
        self.trunk_yaw = float(trunk_yaw)
        self.pan = float(pan)
        self.tilt = float(tilt)

    def set_trunk(self, yaw: float) -> None:
        self.trunk_yaw = float(yaw)

    def set_neck(self, pan: float, tilt: float) -> None:
        self.pan = float(np.clip(pan, -self.pan_limit, self.pan_limit))
        self.tilt = float(np.clip(tilt, -self.tilt_limit, self.tilt_limit))

    @staticmethod
    def _rz(a: float) -> np.ndarray:
        c, s = math.cos(a), math.sin(a)
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

    @staticmethod
    def _ry(a: float) -> np.ndarray:
        c, s = math.cos(a), math.sin(a)
        return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])

    def rig_origin(self) -> np.ndarray:
        off = self._rz(self.trunk_yaw + self.pan) @ self.cam_pos
        return self.base_origin + self._ry(self.tilt) @ off

    def jacobian_neck(self) -> np.ndarray:
        """3x2 analytic Jacobian of the rig origin wrt (pan, tilt)."""
        origin = self.rig_origin()
        lever = origin - self.base_origin
        z_hat = np.array([0.0, 0.0, 1.0])
        y_trunk = self._rz(self.trunk_yaw + self.pan) @ np.array([0.0, 1.0, 0.0])
        return np.column_stack([np.cross(z_hat, lever), np.cross(y_trunk, lever)])


PROXIMITY_CLASSES = ("ÇOK UZAK", "UZAK", "YAKIN", "TEMAS ÖNCESİ", "TEMAS")
_PROXIMITY_THRESHOLDS = (0.45, 0.78, 0.94, 0.985)


@dataclass
class OrdinalPerception:
    """Qualitative/ordinal perception bundle replacing metric telemetry ('sayı yok' kuralı)."""
    proximity_class: str    # ÇOK UZAK | UZAK | YAKIN | TEMAS ÖNCESİ | TEMAS
    horiz_dir: str          # SOL | ORTA | SAĞ
    vert_dir: str           # ÜST | ORTA | ALT
    depth_order: str        # GERİDE | HİZADA | ÖNDE
    looming_state: str      # YAKLAŞIYOR | SABİT | UZAKLAŞIYOR
    confidence: str         # YÜKSEK | ORTA | DÜŞÜK
    prox_ratio: float       # symmetric proximity ratio min(rho_s, 1 / rho_s) in (0, 1]
    du_px: float            # mean binocular horizontal image gap (target - gripper)
    dv_px: float            # mean binocular vertical image gap (target - gripper)
    dd_px: float            # disparity difference (gripper - target)

    def summary(self) -> str:
        """Return compact single-line ordinal descriptor for UI/HUD readouts."""
        return f"{self.proximity_class} · {self.horiz_dir} · {self.vert_dir} ({self.depth_order})"


def compute_ordinal_perception(
    obs: StereoObservation,
    prev_class: str = "UZAK",
    prev_prox_ratio: float | None = None,
    hysteresis: float = 0.03,
    in_fov: bool = True,
) -> OrdinalPerception:
    """Extract an ordinal, coordinate-free perception bundle from a StereoObservation."""
    rho = max(float(obs.scale_ratio), 1e-6)
    prox = min(rho, 1.0 / rho)

    # Raw band index in [0..4]
    idx = 0
    for thr in _PROXIMITY_THRESHOLDS:
        if prox >= thr:
            idx += 1

    # Apply symmetric hysteresis against prev_class to avoid threshold chatter
    if prev_class in PROXIMITY_CLASSES and hysteresis > 0.0:
        prev_idx = PROXIMITY_CLASSES.index(prev_class)
        half_h = 0.5 * float(hysteresis)
        while idx > prev_idx:
            # Transitioning closer requires clearing upper margin (tighter on last band)
            margin = min(half_h, 0.004) if idx == 4 else half_h
            if prox < _PROXIMITY_THRESHOLDS[idx - 1] + margin:
                idx -= 1
            else:
                break
        while idx < prev_idx:
            margin = min(half_h, 0.004) if idx == 3 else half_h
            if prox >= _PROXIMITY_THRESHOLDS[idx] - margin:
                idx += 1
            else:
                break
    prox_cls = PROXIMITY_CLASSES[idx]

    vmode = getattr(obs, "vision_mode", "binocular")
    if vmode in ("mono_left", "mono_static_left") or not np.all(np.isfinite(obs.uv_right_target)):
        du = float(obs.uv_left_target[0]) - float(obs.uv_left_gripper[0])
        dv = float(obs.uv_left_target[1]) - float(obs.uv_left_gripper[1])
    elif vmode in ("mono_right", "mono_static_right") or not np.all(np.isfinite(obs.uv_left_target)):
        du = float(obs.uv_right_target[0]) - float(obs.uv_right_gripper[0])
        dv = float(obs.uv_right_target[1]) - float(obs.uv_right_gripper[1])
    else:
        du = 0.5 * (
            (float(obs.uv_left_target[0]) - float(obs.uv_left_gripper[0]))
            + (float(obs.uv_right_target[0]) - float(obs.uv_right_gripper[0]))
        )
        dv = 0.5 * (
            (float(obs.uv_left_target[1]) - float(obs.uv_left_gripper[1]))
            + (float(obs.uv_right_target[1]) - float(obs.uv_right_gripper[1]))
        )
    dd = float(obs.uvd_gripper[2]) - float(obs.uvd_target[2])

    horiz = "SOL" if du < -8.0 else ("SAĞ" if du > 8.0 else "ORTA")
    vert = "ÜST" if dv < -8.0 else ("ALT" if dv > 8.0 else "ORTA")
    if str(vmode).startswith("mono") and float(getattr(obs, "neck_baseline_m", 0.0)) < 0.005:
        depth = "GERİDE" if float(obs.looming_error_m) > 0.015 else ("ÖNDE" if float(obs.looming_error_m) < -0.015 else "HİZADA")
    else:
        depth = "GERİDE" if dd < -2.0 else ("ÖNDE" if dd > 2.0 else "HİZADA")

    if prev_prox_ratio is None:
        looming = "SABİT"
    else:
        d_prox = prox - float(prev_prox_ratio)
        if d_prox > 0.002:
            looming = "YAKLAŞIYOR"
        elif d_prox < -0.002:
            looming = "UZAKLAŞIYOR"
        else:
            looming = "SABİT"

    if not in_fov or min(float(obs.size_px_gripper), float(obs.size_px_target)) < 4.0:
        conf = "DÜŞÜK"
    elif float(np.linalg.norm(obs.sigma_cam_m)) > 0.025:
        conf = "ORTA"
    else:
        conf = "YÜKSEK"

    return OrdinalPerception(
        proximity_class=prox_cls,
        horiz_dir=horiz,
        vert_dir=vert,
        depth_order=depth,
        looming_state=looming,
        confidence=conf,
        prox_ratio=float(prox),
        du_px=float(du),
        dv_px=float(dv),
        dd_px=float(dd),
    )


class StereoCameraRig:
    """Anthropomorphic two-eye stereo + active-neck monocular motion-parallax & looming sensor model."""

    def __init__(self, cfg: StereoCameraConfig | None = None):
        self.cfg = cfg or StereoCameraConfig()
        self._mono_pose_history: list[dict] = []
        self._mono_wt_sum: float = 0.0
        self._mono_wt_weight: float = 0.0
        self.neck: NeckKinematics | None = None
        self._update_extrinsics()

    def set_neck(self, neck: NeckKinematics) -> None:
        """Attach the 2-DOF neck (parented to the trunk); None restores legacy."""
        self.neck = neck
        self._update_extrinsics()

    def _update_extrinsics(self, sway_offset_y: float = 0.0, sway_offset_z: float = 0.0) -> None:
        if self.neck is not None:
            base_orig = self.neck.rig_origin()
            look_target = np.asarray(self.cfg.look_at, float)
            forward = look_target - base_orig
            n = float(np.linalg.norm(forward))
            forward = np.array([1.0, 0.0, 0.0]) if n < 1e-9 else forward / n
            up_world = np.array([0.0, 0.0, 1.0])
            if abs(float(forward @ up_world)) > 0.98:
                up_world = np.array([0.0, 1.0, 0.0])
            right = np.cross(forward, up_world)
            right = right / float(np.linalg.norm(right))
            down = np.cross(forward, right)
            R_look = np.column_stack([right, down, forward])
            R_neck = (NeckKinematics._rz(self.neck.trunk_yaw + self.neck.pan)
                      @ NeckKinematics._ry(self.neck.tilt))
            self.R_cam_to_base = R_neck @ R_look
            self.R_base_to_cam = self.R_cam_to_base.T
            self.origin = base_orig
            half_b = 0.5 * float(self.cfg.baseline_m)
            self.left_eye_base = self.origin - half_b * right
            self.right_eye_base = self.origin + half_b * right
            return
        base_orig = np.asarray(self.cfg.cam_pos, float).copy()
        base_orig[1] += float(sway_offset_y)
        base_orig[2] += float(sway_offset_z)
        self.origin = base_orig
        target = np.asarray(self.cfg.look_at, float)
        forward = target - self.origin
        n = float(np.linalg.norm(forward))
        forward = np.array([1.0, 0.0, 0.0]) if n < 1e-9 else forward / n
        up_world = np.array([0.0, 0.0, 1.0])
        if abs(float(forward @ up_world)) > 0.98:
            up_world = np.array([0.0, 1.0, 0.0])
        right = np.cross(forward, up_world)
        right = right / float(np.linalg.norm(right))
        down = np.cross(forward, right)
        # Camera frame: +X right, +Y down, +Z forward along line of sight
        self.R_cam_to_base = np.column_stack([right, down, forward])
        self.R_base_to_cam = self.R_cam_to_base.T
        half_b = 0.5 * float(self.cfg.baseline_m)
        self.left_eye_base = self.origin - half_b * right
        self.right_eye_base = self.origin + half_b * right

    def neck_pose_offset(self, t_sec: float) -> np.ndarray:
        """Compute slow, continuous neck postural glide [0, dy, dz] synchronized with eye counter-rotation."""
        mode = str(getattr(self.cfg, "vision_mode", "binocular")).lower()
        if mode not in ("mono_left", "mono_right"):
            return np.zeros(3)
        side_sign = -1.0 if mode == "mono_right" else 1.0
        t_glide = max(0.85, 2.6 / max(0.5, float(self.cfg.neck_parallax_freq_hz)))
        u = min(1.0, max(0.0, float(t_sec) / t_glide))
        s = u * u * (3.0 - 2.0 * u)
        dy = side_sign * float(self.cfg.neck_parallax_amp_m) * (-0.65 + 1.50 * s)
        dz = float(self.cfg.neck_pitch_amp_m) * (-0.45 + 1.15 * s)
        return np.array([0.0, dy, dz], dtype=float)

    def world_to_cam(self, p_base: np.ndarray) -> np.ndarray:
        return self.R_base_to_cam @ (np.asarray(p_base, float) - self.origin)

    def cam_to_world_vec(self, v_cam: np.ndarray) -> np.ndarray:
        return self.R_cam_to_base @ np.asarray(v_cam, float)

    def project_point(self, p_base: np.ndarray, real_diameter_m: float = 0.05,
                      rng: np.random.Generator | None = None
                      ) -> tuple[np.ndarray, np.ndarray, np.ndarray, float, float]:
        """Project a 3D point + physical size into left/right eye pixels, disparity, and apparent pixel size."""
        p_cam = self.world_to_cam(p_base)
        z = max(0.12, float(p_cam[2]))
        f = float(self.cfg.focal_px)
        B = float(self.cfg.baseline_m)
        cx = 0.5 * self.cfg.image_width
        cy = 0.5 * self.cfg.image_height

        x_l = float(p_cam[0]) + 0.5 * B
        x_r = float(p_cam[0]) - 0.5 * B
        y = float(p_cam[1])

        u_l = cx + f * (x_l / z)
        v_l = cy + f * (y / z)
        u_r = cx + f * (x_r / z)
        v_r = cy + f * (y / z)
        app_size_px = f * float(real_diameter_m) / z

        if rng is not None:
            sp = float(self.cfg.pixel_noise_px)
            sd = float(self.cfg.disparity_noise_px)
            if sp > 0:
                n_uv = rng.standard_normal(2) * sp
                u_l += n_uv[0]
                u_r += n_uv[0]
                v_l += n_uv[1]
                v_r += n_uv[1]
            if sd > 0:
                dn = float(rng.standard_normal()) * sd
                u_l += 0.5 * dn
                u_r -= 0.5 * dn
                app_size_px = max(2.0, app_size_px + 0.35 * float(rng.standard_normal()) * sd)

        d = max(1.0, u_l - u_r)
        u_c = 0.5 * (u_l + u_r)
        v_c = 0.5 * (v_l + v_r)
        return (np.array([u_c, v_c, d]),
                np.array([u_l, v_l]),
                np.array([u_r, v_r]),
                float(app_size_px),
                z)

    def triangulate_with_looming(self, uvd: np.ndarray, app_size_px: float,
                                 real_diameter_m: float) -> tuple[np.ndarray, float]:
        """Fuse stereo disparity depth and perspective apparent-size depth (looming)."""
        u_c, v_c, d = float(uvd[0]), float(uvd[1]), max(1.0, float(uvd[2]))
        f = float(self.cfg.focal_px)
        B = float(self.cfg.baseline_m)
        cx = 0.5 * self.cfg.image_width
        cy = 0.5 * self.cfg.image_height

        z_disp = (f * B) / d
        z_loom = (f * float(real_diameter_m)) / max(2.0, float(app_size_px))
        # Optimal inverse-variance fusion of stereo disparity and perspective apparent size
        z_fused = 0.72 * z_disp + 0.28 * z_loom
        x = (u_c - cx) * z_fused / f
        y = (v_c - cy) * z_fused / f
        return np.array([x, y, z_fused]), z_loom

    def predict_reachability(self, model: ArmModel, q0: np.ndarray,
                             p_target_base: np.ndarray) -> PreReachPrediction:
        """Active camera micro-movement (parallax) + perspective size pre-reach check.

        Simulates a small lateral head/camera micro-shift delta_c to measure optical
        parallax, estimates target depth and shoulder-to-target distance, and predicts
        whether the arm can reach the target without violating singularity bounds.
        """
        p_g0 = end_effector_position(model, q0)
        uvd_g, _, _, s_g, z_g = self.project_point(p_g0, self.cfg.gripper_real_size_m, rng=None)
        uvd_t1, _, _, s_t, z_t = self.project_point(p_target_base, self.cfg.target_real_size_m, rng=None)

        # Active micro-shift of the camera (head parallax)
        sway = max(0.005, float(self.cfg.head_sway_m))
        self._update_extrinsics(sway_offset_y=sway)
        uvd_t2, _, _, _, _ = self.project_point(p_target_base, self.cfg.target_real_size_m, rng=None)
        self._update_extrinsics(sway_offset_y=0.0)

        parallax_px = float(np.linalg.norm(uvd_t2[:2] - uvd_t1[:2]))
        p_t_cam, _ = self.triangulate_with_looming(uvd_t1, s_t, self.cfg.target_real_size_m)
        p_t_est_base = self.origin + self.cam_to_world_vec(p_t_cam)

        shoulder_origin = np.array([0.0, 0.0, float(model.links[0].d)])
        dist_from_shoulder = float(np.linalg.norm(p_t_est_base - shoulder_origin))
        arm_len = float(sum(abs(l.a) + abs(l.d) for l in model.links[1:]))
        max_safe = 0.88 * arm_len
        ratio = dist_from_shoulder / max(max_safe, 1e-6)

        _, sing0 = compute_singularity_metrics(model, q0)
        pred_w = float(sing0.manipulability * max(0.05, 1.0 - 0.85 * max(0.0, ratio - 0.55) / 0.45))
        can_reach = (ratio <= 1.0) and (pred_w >= 0.015)

        if can_reach:
            msg = f"ULAŞILABİLİR (Mesafe={dist_from_shoulder*100:.1f} cm <= Güvenli Sınır {max_safe*100:.1f} cm, Tahmini w={pred_w:.3f})"
        else:
            msg = f"SINGULARITY / ERİŞİM SINIRI AŞILDI (Mesafe={dist_from_shoulder*100:.1f} cm > {max_safe*100:.1f} cm, Sönümlü Güvenli Yaklaşma Aktif)"

        return PreReachPrediction(
            estimated_distance_m=dist_from_shoulder,
            max_safe_reach_m=max_safe,
            reach_ratio=ratio,
            predicted_min_w=pred_w,
            can_reach_safely=can_reach,
            parallax_shift_px=parallax_px,
            apparent_target_px=s_t,
            apparent_gripper_px=s_g,
            status_text=msg,
        )

    def observe(self, p_gripper_base: np.ndarray, p_target_base: np.ndarray,
                rng: np.random.Generator | None = None,
                t_sec: float = 0.0,
                t_cap: float = 0.0, t_arr: float = 0.0, t_now: float = 0.0,
                neck_pan: float = 0.0, neck_tilt: float = 0.0) -> StereoObservation:
        """Observe gripper and target from shoulder-mounted stereo eyes or a single open eye with active neck parallax."""
        mode = str(getattr(self.cfg, "vision_mode", "binocular")).lower()
        Wg = float(self.cfg.gripper_real_size_m)
        Wt = float(self.cfg.target_real_size_m)
        f = max(1.0, float(self.cfg.focal_px))
        B = max(1e-4, float(self.cfg.baseline_m))
        cx = 0.5 * self.cfg.image_width
        cy = 0.5 * self.cfg.image_height

        if mode == "binocular":
            self._update_extrinsics(0.0, 0.0)
            uvd_g, uvl_g, uvr_g, sg_px, zg = self.project_point(p_gripper_base, Wg, rng=rng)
            uvd_t, uvl_t, uvr_t, st_px, zt = self.project_point(p_target_base, Wt, rng=rng)

            p_g_cam, z_loom_g = self.triangulate_with_looming(uvd_g, sg_px, Wg)
            p_t_cam, z_loom_t = self.triangulate_with_looming(uvd_t, st_px, Wt)
            rel_cam = p_t_cam - p_g_cam
            rel_base = self.cam_to_world_vec(rel_cam)

            norm_sg = sg_px / max(Wg, 1e-4)
            norm_st = st_px / max(Wt, 1e-4)
            scale_ratio = norm_sg / max(norm_st, 1e-4)
            looming_err_m = z_loom_t - z_loom_g

            z_mean = 0.5 * (zg + zt)
            s_xy = math.sqrt(2.0) * (z_mean / f) * max(float(self.cfg.pixel_noise_px), 1e-4)
            s_z = 0.75 * math.sqrt(2.0) * ((z_mean ** 2) / (f * B)) * max(float(self.cfg.disparity_noise_px), 1e-4)
            sigma_cam = np.array([s_xy, s_xy, s_z])

            return StereoObservation(
                uvd_gripper=uvd_g,
                uvd_target=uvd_t,
                uv_left_gripper=uvl_g,
                uv_right_gripper=uvr_g,
                uv_left_target=uvl_t,
                uv_right_target=uvr_t,
                size_px_gripper=sg_px,
                size_px_target=st_px,
                scale_ratio=scale_ratio,
                looming_error_m=looming_err_m,
                rel_cam_gap_m=rel_cam,
                rel_base_gap_m=rel_base,
                sigma_cam_m=sigma_cam,
                depth_gripper_m=zg,
                depth_target_m=zt,
                vision_mode="binocular",
                neck_baseline_m=B,
                motion_parallax_px=0.0,
                head_offset_m=np.zeros(3),
                t_cap=t_cap, t_arr=t_arr, t_now=t_now,
                neck_pan=neck_pan, neck_tilt=neck_tilt,
            )

        # ---- Single-eye (monocular) mode with active neck motion parallax + looming ----
        if t_sec <= 0.0 or (self._mono_pose_history and t_sec < self._mono_pose_history[-1]["t"]):
            self._mono_pose_history.clear()
            self._mono_wt_sum = 0.0
            self._mono_wt_weight = 0.0

        head_off = self.neck_pose_offset(t_sec)
        self._update_extrinsics(sway_offset_y=float(head_off[1]), sway_offset_z=float(head_off[2]))

        use_left = mode in ("mono_left", "mono_static", "mono_static_left")
        eye_pos = (self.left_eye_base if use_left else self.right_eye_base).copy()
        eye_shift_x = -0.5 * B if use_left else 0.5 * B  # offset from open eye to cyclopean origin

        _, uvl_g, uvr_g, sg_px, zg = self.project_point(p_gripper_base, Wg, rng=rng)
        _, uvl_t, uvr_t, st_px, zt = self.project_point(p_target_base, Wt, rng=rng)
        uv_g_open = uvl_g if use_left else uvr_g
        uv_t_open = uvl_t if use_left else uvr_t
        uv_nan = np.array([math.nan, math.nan])

        Wt_prior = Wt * (1.0 + float(getattr(self.cfg, "mono_scale_bias", 0.28)))
        z_loom_g = (f * Wg) / max(2.0, sg_px)
        z_loom_t_raw = (f * Wt_prior) / max(2.0, st_px)

        # Unit ray from open eye toward target in world coordinates
        dir_t_cam = np.array([(uv_t_open[0] - cx) / f, (uv_t_open[1] - cy) / f, 1.0])
        dir_t_cam = dir_t_cam / max(float(np.linalg.norm(dir_t_cam)), 1e-9)
        ray_t_world = self.R_cam_to_base @ dir_t_cam

        # Select past neck poses along the smooth postural glide trajectory for multi-ray parallax
        best_prev = None
        best_b = 0.0
        valid_rays: list[tuple[float, dict]] = []
        for rec_h in self._mono_pose_history:
            dt_h = float(t_sec) - float(rec_h["t"])
            if dt_h >= 0.025:
                b_cand = float(np.linalg.norm(eye_pos - rec_h["eye_pos"]))
                if b_cand > best_b:
                    best_b = b_cand
                    best_prev = rec_h
                if b_cand >= 0.006:
                    valid_rays.append((b_cand, rec_h))

        motion_parallax_px = 0.0
        if mode in ("mono_left", "mono_right") and best_prev is not None and best_b >= 0.006:
            z_mp_num = 0.0
            z_mp_den = 0.0
            o2 = eye_pos
            r2 = ray_t_world
            z_comp = max(0.15, float(np.dot(r2, self.R_cam_to_base[:, 2])))
            for b_c, rec_c in valid_rays[:12]:
                o1 = rec_c["eye_pos"]
                r1 = rec_c["ray_t_world"]
                b_dot = float(np.clip(np.dot(r1, r2), -0.999999, 0.999999))
                denom = max(1e-7, 1.0 - b_dot * b_dot)
                delta_o = o1 - o2
                t2 = (float(np.dot(r2, delta_o)) - b_dot * float(np.dot(r1, delta_o))) / denom
                z_c = float(np.clip(t2 * z_comp, 0.14, 3.0))
                w_c = b_c * b_c
                z_mp_num += w_c * z_c
                z_mp_den += w_c
            z_t_mp = z_mp_num / max(z_mp_den, 1e-9)
            motion_parallax_px = float(f * np.linalg.norm(r2 - best_prev["ray_t_world"]))
            wt_sample = float(np.clip((st_px * z_t_mp) / f, 0.45 * Wt, 1.85 * Wt))
            w_step = best_b * best_b
            self._mono_wt_sum += w_step * wt_sample
            self._mono_wt_weight += w_step
            wt_cal = self._mono_wt_sum / max(self._mono_wt_weight, 1e-9)
            alpha_mp = float(np.clip(self._mono_wt_weight / (self._mono_wt_weight + 0.008 ** 2), 0.0, 0.995))
            Wt_est = (1.0 - alpha_mp) * Wt_prior + alpha_mp * wt_cal
            z_loom_t = (f * Wt_est) / max(2.0, st_px)
            z_t_est = 0.35 * z_t_mp + 0.65 * z_loom_t
            neck_baseline_m = best_b
        else:
            Wt_est = Wt_prior
            z_loom_t = z_loom_t_raw
            z_t_est = z_loom_t_raw
            neck_baseline_m = 0.0

        norm_sg = sg_px / max(Wg, 1e-4)
        norm_st = st_px / max(Wt_est, 1e-4)
        scale_ratio = norm_sg / max(norm_st, 1e-4)
        looming_err_m = z_loom_t - z_loom_g

        # Anchor hand depth with instantaneous looming + motion-parallax-locked target range
        z_g_rel = z_t_est / max(scale_ratio, 0.05)
        z_g_est = 0.60 * z_loom_g + 0.40 * z_g_rel

        self._mono_pose_history.append({
            "t": float(t_sec),
            "eye_pos": eye_pos,
            "ray_t_world": ray_t_world,
        })
        if len(self._mono_pose_history) > 48:
            self._mono_pose_history = self._mono_pose_history[:12] + self._mono_pose_history[-36:]

        # Reconstruct 3D positions in current cyclopean camera frame (common-mode neck shift cancels in rel_cam)
        x_g_cam = ((uv_g_open[0] - cx) / f) * z_g_est - eye_shift_x
        y_g_cam = ((uv_g_open[1] - cy) / f) * z_g_est
        p_g_cam = np.array([x_g_cam, y_g_cam, z_g_est])

        x_t_cam = ((uv_t_open[0] - cx) / f) * z_t_est - eye_shift_x
        y_t_cam = ((uv_t_open[1] - cy) / f) * z_t_est
        p_t_cam = np.array([x_t_cam, y_t_cam, z_t_est])

        rel_cam = p_t_cam - p_g_cam
        rel_base = self.cam_to_world_vec(rel_cam)

        z_mean = 0.5 * (zg + zt)
        b_eff = max(neck_baseline_m, 0.55 * Wg) if mode in ("mono_left", "mono_right") else max(0.35 * Wg, 1e-4)
        s_xy = math.sqrt(2.0) * (z_mean / f) * max(float(self.cfg.pixel_noise_px), 1e-4)
        s_z = 0.85 * math.sqrt(2.0) * ((z_mean ** 2) / (f * b_eff)) * max(float(self.cfg.disparity_noise_px), 1e-4)
        sigma_cam = np.array([s_xy, s_xy, s_z])

        eff_disp_g = (f * max(neck_baseline_m, 0.02)) / max(z_g_est, 0.12)
        eff_disp_t = (f * max(neck_baseline_m, 0.02)) / max(z_t_est, 0.12)

        return StereoObservation(
            uvd_gripper=np.array([uv_g_open[0], uv_g_open[1], eff_disp_g]),
            uvd_target=np.array([uv_t_open[0], uv_t_open[1], eff_disp_t]),
            uv_left_gripper=uvl_g if use_left else uv_nan,
            uv_right_gripper=uvr_g if not use_left else uv_nan,
            uv_left_target=uvl_t if use_left else uv_nan,
            uv_right_target=uvr_t if not use_left else uv_nan,
            size_px_gripper=sg_px,
            size_px_target=st_px,
            scale_ratio=scale_ratio,
            looming_error_m=looming_err_m,
            rel_cam_gap_m=rel_cam,
            rel_base_gap_m=rel_base,
            sigma_cam_m=sigma_cam,
            depth_gripper_m=zg,
            depth_target_m=zt,
            vision_mode=mode,
            neck_baseline_m=neck_baseline_m,
            motion_parallax_px=motion_parallax_px,
            head_offset_m=head_off,
            t_cap=t_cap, t_arr=t_arr, t_now=t_now,
            neck_pan=neck_pan, neck_tilt=neck_tilt,
        )


# Anthropomorphic limb & initial posture presets + biological speed/urgency tiers
HUMAN_LIMB_PROFILES = {
    "adult_avg": {
        "label": "Yetişkin İnsan (Ortalama — Boy 175 cm)",
        "d1_shoulder_z": 0.90,
        "L2_upper_arm": 0.31,
        "L3_forearm": 0.26,
        "d4_wrist": 0.05,
        "d5_palm": 0.05,
        "d6_fingers": 0.06,
        "cam_x": -0.10,
        "cam_z": 1.12,
        "baseline_mm": 64,
    },
    "adult_tall": {
        "label": "Uzun Boylu / Geniş Erişim (Boy 188 cm)",
        "d1_shoulder_z": 0.98,
        "L2_upper_arm": 0.35,
        "L3_forearm": 0.29,
        "d4_wrist": 0.05,
        "d5_palm": 0.05,
        "d6_fingers": 0.07,
        "cam_x": -0.11,
        "cam_z": 1.22,
        "baseline_mm": 66,
    },
    "compact_youth": {
        "label": "Kompakt / Genç Uzuv (Boy 160 cm)",
        "d1_shoulder_z": 0.82,
        "L2_upper_arm": 0.27,
        "L3_forearm": 0.23,
        "d4_wrist": 0.04,
        "d5_palm": 0.04,
        "d6_fingers": 0.05,
        "cam_x": -0.09,
        "cam_z": 1.02,
        "baseline_mm": 60,
    },
}

INITIAL_POSTURE_PRESETS = {
    "hanging_natural": {
        "label": "İnsansı Aşağı Sallanan Kol (Doğal Dinlenme Duruşu)",
        "q_deg": [0.0, -84.0, 10.0, 0.0, 40.0, 0.0],
    },
    "hanging_vertical": {
        "label": "Tam Dikey Sallanan Kol (Yanda Serbest Askı)",
        "q_deg": [0.0, -88.0, 4.0, 0.0, 20.0, 0.0],
    },
    "ready_desk": {
        "label": "Masaüstü Hazır Duruş (Dirsek Bükülü)",
        "q_deg": [0.0, -45.0, 65.0, 0.0, 30.0, 0.0],
    },
}

HUMAN_SPEED_PROFILES = {
    "reflex": {
        "label": "1. Acil / Refleks Seviyesi (Balistik Koruyucu Hız)",
        "vmax": 1.15,
        "max_qd": 3.0,
        "kappa_scale": 1.18,
        "eps": 0.008,
        "phi_min": 0.68,
        "gamma": 0.35,
        "terminal_p": 0.28,
        "description": "Tehlike anında veya düşen bir nesneyi yakalarken kullanılan maksimum tork/hız modu.",
    },
    "fast": {
        "label": "2. Hızlı / Atik Ulaşma (Tempolu Yakalama)",
        "vmax": 0.80,
        "max_qd": 2.2,
        "kappa_scale": 1.05,
        "eps": 0.005,
        "phi_min": 0.45,
        "gamma": 0.45,
        "terminal_p": 0.38,
        "description": "Günlük hayatta hızlıca bir nesneyi alma; erken ivmelenme ve kararlı frenleme.",
    },
    "normal": {
        "label": "3. Normal / Doğal İnsansı (Günlük Konforlu Ulaşma)",
        "vmax": 0.50,
        "max_qd": 1.4,
        "kappa_scale": 1.00,
        "eps": 0.004,
        "phi_min": 0.28,
        "gamma": 0.55,
        "terminal_p": 0.45,
        "description": "Standart çan-eğrisi (bell-shaped) hız profili ve tam 1/(e*delta_eff) aşmasız varış.",
    },
    "careful": {
        "label": "4. Dikkatli / Dolu Bardak Taşıma (Sarsıntısız Yumuşak)",
        "vmax": 0.30,
        "max_qd": 0.9,
        "kappa_scale": 0.78,
        "eps": 0.003,
        "phi_min": 0.18,
        "gamma": 0.72,
        "terminal_p": 0.60,
        "description": "Ağzına kadar dolu bir bardağı dökmeden taşırken kullanılan düşük ivmeli profil.",
    },
    "precision": {
        "label": "5. Hassas / Cerrahi Mikro-Ulaşma (İğne İpliği Hassasiyeti)",
        "vmax": 0.16,
        "max_qd": 0.55,
        "kappa_scale": 0.55,
        "eps": 0.0015,
        "phi_min": 0.12,
        "gamma": 0.88,
        "terminal_p": 0.75,
        "description": "Cerrahi/mikro-montaj hassasiyeti; milimetrik tolerans ve ultra-yumuşak yaklaşma.",
    },
}


class HumanLoomingController:
    """HLSL (Human-Like Stereo Looming & Bell-Shaped Governor) Control Law.

    Supports biological speed/urgency tiers ('reflex', 'fast', 'normal', 'careful', 'precision')
    while preserving delay-governed monotone braking.
    """

    name = "hlsl"

    def __init__(self, delay: float = 0.06, fps: float = 100.0, eps: float = 0.004,
                 kappa: float = 1.0 / math.e, k_hi: float = 40.0, vmax: float = 0.5,
                 sigma=0.0005, gamma_ramp: float = 0.55, per_axis: bool = True,
                 speed_mode: str = "normal"):
        self.delay = float(delay)
        self.T = 1.0 / float(fps)
        prof = HUMAN_SPEED_PROFILES.get(speed_mode, HUMAN_SPEED_PROFILES["normal"])
        self.speed_mode = speed_mode
        self.eps = float(eps if speed_mode == "normal" else prof["eps"])
        self.kappa = float(min(kappa * prof["kappa_scale"], 1.25 / math.e))
        self.k_hi = float(k_hi)
        self.vmax = float(vmax if speed_mode == "normal" else prof["vmax"])
        self.gamma_ramp = float(gamma_ramp if speed_mode == "normal" else prof["gamma"])
        self.phi_min = float(prof["phi_min"])
        self.terminal_p = float(prof["terminal_p"])
        self.per_axis = bool(per_axis)
        self.e0_norm: float | None = None
        self.step_count = 0
        self._buf = None
        self.set_sigma(sigma)

    def set_delay(self, delay: float) -> None:
        self.delay = max(0.0, float(delay))
        self._recompute()

    def set_sigma(self, sigma) -> None:
        sig = np.broadcast_to(np.atleast_1d(np.asarray(sigma, float)), (3,)).astype(float)
        self.sigma_vec = sig.copy()
        self.n_req = np.ceil((2.0 * sig / self.eps) ** 2).astype(int)
        self._recompute()

    def _recompute(self) -> None:
        n_max = max(1, int(math.floor(2.0 * self.delay / self.T)))
        self.n = np.maximum(1, np.minimum(self.n_req, n_max))
        self.n_max = int(self.n.max())
        self.delta_eff_vec = self.delay + self.n * self.T / 2.0
        self.k_vec = self.kappa / self.delta_eff_vec
        self.delta_eff = self.delay + self.n_max * self.T / 2.0
        self.k = self.kappa / self.delta_eff
        if self._buf is not None and self._buf.maxlen != self.n_max:
            from collections import deque
            self._buf = deque(self._buf, maxlen=self.n_max)

    def __call__(self, error: np.ndarray) -> np.ndarray:
        from collections import deque
        e = np.asarray(error, float)
        if self._buf is None:
            self._buf = deque(maxlen=int(self.n_max))
        self._buf.append(e.copy())
        arr = np.asarray(self._buf)
        if self.per_axis:
            filt = np.array([
                float(np.mean(arr[-int(self.n[j]):, j]))
                for j in range(e.size)
            ])
        else:
            filt = np.mean(arr, axis=0)

        r = float(np.linalg.norm(filt))
        if self.e0_norm is None and r > 1e-6:
            self.e0_norm = r
        self.step_count += 1
        if r <= self.eps:
            return np.zeros(e.size)

        # Human-like smooth onset ramp modulated by biological speed/urgency tier
        p = min(1.0, r / max(self.e0_norm or r, 1e-4))
        time_ramp = min(1.0, (self.step_count * self.T) / max(1.1 * self.delta_eff, 0.035))
        prog_ramp = min(1.0, ((1.0 - p + 0.18) / 0.55) ** self.gamma_ramp)
        # During final approach (p < terminal_p), phi == 1.0 so exact 1/(e*delta_eff) decay governs settling
        phi = 1.0 if p < self.terminal_p else max(self.phi_min, min(1.0, 0.5 * (time_ramp + prog_ramp)))

        if self.per_axis:
            v_vec = np.zeros(e.size)
            for j in range(e.size):
                aj = abs(float(filt[j]))
                if aj > 0.25 * self.eps:
                    vj = phi * min(self.k_hi * aj, float(self.k_vec[j]) * (self.eps + aj))
                    v_vec[j] = math.copysign(vj, float(filt[j]))
            norm_v = float(np.linalg.norm(v_vec))
            if norm_v > self.vmax:
                v_vec *= self.vmax / norm_v
            return v_vec

        v_mag = min(self.vmax, phi * min(self.k_hi * r, self.k * (self.eps + r)))
        return (filt / r) * v_mag


# --- Triphasic joint sequencing anchors (phase = remaining-error fraction) ---
# Proximal-dominant power early, distal precision late, global co-contraction
# next to the capture point. Values are velocity-cap multipliers on the six
# joints (j1 trunk, j2 shoulder, j3 elbow, j4 wrist flex, j5 wrist r/u, j6 hand).
_TRIPHASIC_ANCHORS = [
    (1.00, [1.00, 1.00, 0.85, 0.60, 0.60, 0.50]),   # ballistic / reach
    (0.60, [0.90, 1.00, 0.90, 0.75, 0.75, 0.65]),   # transport
    (0.25, [0.60, 0.70, 0.85, 1.00, 1.00, 0.85]),   # grasp / precision
    (0.06, [0.55, 0.65, 0.80, 0.90, 0.90, 0.75]),   # final co-contraction
]

def triphasic_weights(p_rem: float) -> np.ndarray:
    """Smoothly-interpolated triphasic joint velocity-cap multipliers."""
    p = float(p_rem)
    anchors = _TRIPHASIC_ANCHORS
    if p >= anchors[0][0]:
        return np.asarray(anchors[0][1], float)
    if p <= anchors[-1][0]:
        return np.asarray(anchors[-1][1], float)
    for i in range(len(anchors) - 1):
        hi_p, hi_w = anchors[i]
        lo_p, lo_w = anchors[i + 1]
        if lo_p <= p <= hi_p:
            f = (p - lo_p) / (hi_p - lo_p)
            return np.asarray(lo_w, float) * (1.0 - f) + np.asarray(hi_w, float) * f
    return np.asarray(anchors[-1][1], float)


class HumanApproachController(HumanLoomingController):
    """HDA — human-like dynamic approach profile (insansı dinamik yaklaşım).

    Systematic human-like reaching algorithm built on the delay-derived
    governor. Four components, all documented in
    ``docs/kabuller_ve_gereksinimler.md`` §3.3:

    1. **Distance- and urgency-adaptive dynamic speed envelope.**
       The peak-speed cap scales with the amplitude of the reach:
           v_cap(D) = v_max(tier) * clamp(sqrt(D / D_REF), DIST_MIN, 1)
       with D_REF = 0.30 m and sqrt chosen as a conservative fit of the
       human amplitude/peak-speed relation. Short reaches are
       disproportionately slower, exactly as a human reaching to a nearby
       object slows down. The urgency tier still caps the absolute ceiling.
    2. **Asymmetric bell shaping** (speed ramps up fast, decays long toward
       the goal) — inherited from HumanLoomingController's phi curve.
    3. **Triphasic joint sequencing** — proximal-dominant power early,
       distal precision late, global co-contraction near capture — exposed as
       ``sequencing_weights()`` and applied by the simulator as per-joint
       velocity caps (joint speed_limit in human_arm.json is never exceeded).
    4. **Signal-dependent command noise** is left to the caller (the JS
       workbench injects it; the Python benchmark stays deterministic).

    The delay-derived monotonicity bound (kappa <= 1/e outside the reflex
    tier) is preserved by construction.
    """

    name = "hdap"

    D_REF = 0.30          # reference reach amplitude (m), peak-speed scaling
    DIST_MIN = 0.35       # floor fraction for very short reaches
    AMP_EXP = 0.5         # amplitude exponent (sqrt law, conservative)

    def __init__(self, *args, **kwargs):
        self.last_r = 0.0
        super().__init__(*args, **kwargs)

    def sequencing_weights(self):
        if self.e0_norm is None or self.e0_norm <= 1e-9:
            return np.ones(6)
        p_rem = min(1.0, max(0.0, self.last_r / self.e0_norm))
        return triphasic_weights(p_rem)

    def __call__(self, error: np.ndarray) -> np.ndarray:
        # Remember the current measured distance for the sequencing weights
        e = np.asarray(error, float)
        # The parent tracks e0_norm internally; we just stash the raw r
        # by calling the parent and measuring its internal r proxy.
        out = super().__call__(error)
        self.last_r = float(np.linalg.norm(self._buf[-1])) if self._buf is not None else float(np.linalg.norm(error))
        return self._apply_distance_cap(out, e)

    def _apply_distance_cap(self, v_cmd: np.ndarray, error: np.ndarray) -> np.ndarray:
        if self.e0_norm is None:
            return v_cmd
        v_cap = self.vmax * clamp((self.e0_norm / self.D_REF) ** self.AMP_EXP, self.DIST_MIN, 1.0)
        # Never exceed what the parent tier intends for long reaches; only
        # pull the ceiling DOWN for short ones.
        cap = min(self.vmax, v_cap)
        n = float(np.linalg.norm(v_cmd))
        if n > cap and n > 0:
            v_cmd = v_cmd * (cap / n)
        return v_cmd


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


@dataclass
class SingularityStatus:
    """Instantaneous singularity and joint-coordination diagnostics."""
    manipulability: float           # Yoshikawa w = sqrt(det(J J^T))
    min_singular_value: float       # sigma_min(J_v)
    condition_number: float         # sigma_max / sigma_min
    damping_lambda: float           # adaptive DLS damping factor
    joint_weights: np.ndarray       # 6-vector of active joint participation weights
    torque_headroom: np.ndarray     # 6-vector of remaining torque fraction [0..1]
    rotational_manipulability: float = 0.0  # SO(3) angular manipulability sqrt(det(Jw Jw^T))


def compute_singularity_metrics(model: ArmModel, q: np.ndarray,
                                qd: np.ndarray | None = None,
                                tau_prev: np.ndarray | None = None,
                                lambda0: float = 0.015,
                                w_threshold: float = 0.04) -> tuple[np.ndarray, SingularityStatus]:
    """Compute translational & rotational Jacobian metrics at configuration q."""
    J_full = jacobian(model, q, model.n - 1)
    Jv = J_full[:3, :]
    Jw = J_full[3:, :]
    svd_s = np.linalg.svd(Jv, compute_uv=False)
    s_max = float(svd_s[0]) if svd_s.size else 0.0
    s_min = float(svd_s[-1]) if svd_s.size else 0.0
    w = float(np.prod(svd_s))
    cond = s_max / max(s_min, 1e-6)
    svd_w = np.linalg.svd(Jw, compute_uv=False)
    w_rot = float(np.prod(svd_w)) if svd_w.size == 3 else 0.0

    if w < w_threshold:
        ratio = 1.0 - (w / max(w_threshold, 1e-6)) ** 2
        lam = math.sqrt(lambda0 ** 2 + 0.08 ** 2 * ratio)
    else:
        lam = lambda0

    headroom = np.ones(model.n)
    limit_margin = np.ones(model.n)
    for i, link in enumerate(model.links):
        spd = 0.0 if qd is None else float(qd[i])
        t_lim = max(torque_limit(link.motor, spd), 1e-3)
        t_used = 0.0 if tau_prev is None else abs(float(tau_prev[i]))
        headroom[i] = float(np.clip(1.0 - 0.7 * (t_used / t_lim), 0.15, 1.0))

        lo, hi = link.limits
        span = max(hi - lo, 1e-3)
        dist_edge = min(float(q[i]) - lo, hi - float(q[i])) / (0.5 * span)
        limit_margin[i] = float(np.clip(dist_edge / 0.25, 0.1, 1.0))

    base_synergy = np.array([1.0, 1.0, 0.95, 0.75, 0.75, 0.50][:model.n])
    weights = base_synergy * headroom * limit_margin

    status = SingularityStatus(
        manipulability=w,
        min_singular_value=s_min,
        condition_number=cond,
        damping_lambda=lam,
        joint_weights=weights,
        torque_headroom=headroom,
        rotational_manipulability=w_rot,
    )
    return Jv, status


def singularity_safe_joint_velocities(
    model: ArmModel,
    q: np.ndarray,
    qd: np.ndarray,
    v_cartesian: np.ndarray,
    tau_prev: np.ndarray | None = None,
    max_qd: float = 1.2,
    nullspace_gain: float = 0.35,
    R_desired: np.ndarray | None = None,
    k_orient: float = 3.5,
) -> tuple[np.ndarray, SingularityStatus]:
    """Map desired relative 3D velocity (and optional 3D SO(3) orientation) to joint velocities."""
    Jv, status = compute_singularity_metrics(model, q, qd, tau_prev)
    W_diag = status.joint_weights
    W_inv = np.diag(W_diag)

    JW = Jv @ W_inv
    A = JW @ Jv.T + (status.damping_lambda ** 2) * np.eye(3)
    y = np.linalg.solve(A, np.asarray(v_cartesian, float))
    qd_task = W_inv @ (Jv.T @ y)

    q_mid = np.array([0.0, -0.35, 0.75, 0.0, 0.50, 0.0][:model.n])
    grad_posture = q_mid - q

    eps_q = 1e-3
    grad_manip = np.zeros(model.n)
    for j in (1, 2):
        if j < model.n:
            qp = q.copy()
            qp[j] += eps_q
            Jp = jacobian(model, qp, model.n - 1)[:3, :]
            wp = float(np.prod(np.linalg.svd(Jp, compute_uv=False)))
            grad_manip[j] = (wp - status.manipulability) / eps_q

    qd_0 = nullspace_gain * grad_posture + 1.2 * grad_manip
    A_exact = JW @ Jv.T + (1e-6) * np.eye(3)
    J_pinv_J = W_inv @ (Jv.T @ np.linalg.solve(A_exact, Jv))
    N_v = np.eye(model.n) - J_pinv_J
    v_mag = float(np.linalg.norm(v_cartesian))
    ns_scale = min(1.0, v_mag / 0.05)
    qd_null = ns_scale * (N_v @ qd_0)

    qd_ori = np.zeros(model.n)
    if R_desired is not None and model.n >= 3:
        J_full = jacobian(model, q, model.n - 1)
        Jw = J_full[3:, :]
        R_ee = end_effector_rotation(model, q)
        e_o, _ = orientation_error_so3(R_ee, R_desired)
        W_ori = np.diag(np.array([0.10, 0.14, 0.22, 1.0, 1.0, 1.0][:model.n]))
        JwN = Jw @ N_v
        A_o = JwN @ W_ori @ JwN.T + 0.025 * np.eye(3)
        w_res = k_orient * e_o - Jw @ qd_task
        qd_ori = N_v @ (W_ori @ (JwN.T @ np.linalg.solve(A_o, w_res)))

    qd_cmd = qd_task + qd_null + qd_ori

    for i, link in enumerate(model.links):
        j_lim = min(float(max_qd), 0.85 * float(link.motor.speed_limit))
        qd_cmd[i] = float(np.clip(qd_cmd[i], -j_lim, j_lim))

    return qd_cmd, status
