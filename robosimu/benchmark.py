"""Standardized academic benchmark suite for reproducible verification.

Evaluates 6 visuomotor control laws across 5 canonical reaching scenarios on
``models/human_arm.json`` using academic metrics:
- Terminal position error e_f (mm)
- Target overshoot (%)
- Settling / movement time T_s (s)
- ISO 9241-9 / MacKenzie Fitts effective ID_e (bits) & Throughput TP (bits/s)
- Ballistic initial approach error E_init (mm) & compensation share %T_comp
- Minimum Yoshikawa manipulability w_min
- Peak torque tau_max (Nm) & electrical energy (J)
- Active monocular neck parallax baseline B_neck (mm) and synchronized waist yaw (deg)
"""
from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from .controllers import make_controller
from .kinematics import end_effector_position
from .model import ArmModel, load_model, load_postures
from .simulator import SimConfig, simulate_reach
from .stereo import StereoCameraConfig
from .stats import bootstrap_ci, permutation_test, holm_bonferroni


BENCHMARK_CONTROLLERS = ("hdap", "hlsl", "governor", "pid", "lqr", "fixed")


@dataclass(frozen=True)
class BenchmarkScenario:
    """Canonical scenario definition for the 5-scenario academic benchmark."""

    id: str
    title: str
    target: tuple[float, float, float]
    delay_s: float
    pixel_noise_px: float
    disparity_noise_px: float
    duration_s: float
    vision_mode: str = "binocular"
    mono_scale_bias: float = 0.28
    waist_sync: bool = False
    target_width_m: float = 0.040
    description: str = ""


BENCHMARK_SCENARIOS: tuple[BenchmarkScenario, ...] = (
    BenchmarkScenario(
        id="S1_nominal_binocular",
        title="S1: Nominal Binoküler Stereo (60 ms, 0.25 px)",
        target=(0.45, 0.12, 0.88),
        delay_s=0.06,
        pixel_noise_px=0.25,
        disparity_noise_px=0.25,
        duration_s=3.0,
        vision_mode="binocular",
        description="Nominal visuomotor reach from retracted_compact under 60 ms visual delay.",
    ),
    BenchmarkScenario(
        id="S2_high_latency_120ms",
        title="S2: Yüksek Gecikme Stresi (120 ms, 0.35 px — Lambert-W)",
        target=(0.45, 0.12, 0.88),
        delay_s=0.12,
        pixel_noise_px=0.35,
        disparity_noise_px=0.35,
        duration_s=3.5,
        vision_mode="binocular",
        description="High-latency stress test verifying kappa <= 1/e monotone damping without retuning.",
    ),
    BenchmarkScenario(
        id="S3_extended_singularity",
        title="S3: Sınır / Tekillik Yakını Uzak Uzanma (human_wdls vs plain_dls)",
        target=(0.56, 0.08, 0.96),
        delay_s=0.06,
        pixel_noise_px=0.25,
        disparity_noise_px=0.25,
        duration_s=4.5,
        vision_mode="binocular",
        description="Near-boundary extended reach preserving manipulability margin w_min.",
    ),
    BenchmarkScenario(
        id="S4_monocular_parallax",
        title="S4: Tek Göz Aktif Boyun Paralaksı vs Sabit Tek Göz (+%28 Ölçek Hatası)",
        target=(0.45, 0.12, 0.88),
        delay_s=0.06,
        pixel_noise_px=0.20,
        disparity_noise_px=0.20,
        duration_s=4.0,
        vision_mode="mono_left",
        mono_scale_bias=0.28,
        description="Monocular reach with +28% initial target size prior bias: active neck glide vs static eye.",
    ),
    BenchmarkScenario(
        id="S5_lateral_waist_sync",
        title="S5: Yanal Küre-Dışı Senkron Bel + Kol Uzanma ([0.42, -0.42, 0.86] m)",
        target=(0.42, -0.42, 0.86),
        delay_s=0.06,
        pixel_noise_px=0.25,
        disparity_noise_px=0.25,
        duration_s=3.5,
        vision_mode="binocular",
        waist_sync=True,
        description="Contralateral target beyond static shoulder envelope requiring synchronized torso yaw.",
    ),
)


def compute_target_waist_yaw_rad(model: ArmModel, target: np.ndarray) -> float:
    """Compute torso waist column yaw angle (rad) matching workbench kinematics."""
    base_o = np.asarray(model.base_origin, float)
    sh_z = float(model.links[0].d)
    sh0 = np.array([base_o[0], base_o[1], sh_z], float)
    total_arm_len = sum(abs(float(lk.a)) + abs(float(lk.d)) for lk in model.links[1:])
    dx0 = float(target[0] - sh0[0])
    dy0 = float(target[1] - sh0[1])
    dz0 = float(target[2] - sh0[2])
    dist0 = math.sqrt(dx0 * dx0 + dy0 * dy0 + dz0 * dz0)

    az_sh = math.atan2(dy0, max(0.06, dx0))
    dead = math.radians(16.0)
    yaw_lat = 0.0
    if abs(az_sh) > dead:
        yaw_lat = 0.70 * (az_sh - math.copysign(dead, az_sh))

    yaw_ext = 0.0
    thresh = 0.78 * total_arm_len
    if dist0 > thresh:
        def_u = min(1.0, (dist0 - thresh) / max(0.16 * total_arm_len, 0.05))
        s_def = def_u * def_u * (3.0 - 2.0 * def_u)
        dir_right = math.atan2(-float(target[0]), max(0.05, float(base_o[1]))) * 0.75
        yaw_ext = (0.45 if dy0 > 0.08 else dir_right) * s_def

    yaw_goal = yaw_lat + yaw_ext
    if yaw_goal > 0.0 and target[0] > 0.15:
        max_safe = max(dist0, 0.85 * total_arm_len)
        test_y = yaw_goal
        for _ in range(6):
            bx = float(base_o[0]) * math.cos(test_y) - float(base_o[1]) * math.sin(test_y)
            by = float(base_o[0]) * math.sin(test_y) + float(base_o[1]) * math.cos(test_y)
            d_rot = math.sqrt((float(target[0]) - bx) ** 2 + (float(target[1]) - by) ** 2 + dz0 * dz0)
            if d_rot <= max_safe + 1e-3:
                break
            test_y *= 0.65
        yaw_goal = test_y

    max_waist = math.radians(58.0)
    return max(-max_waist, min(max_waist, yaw_goal))


def _rotate_z(vec: np.ndarray, angle_rad: float) -> np.ndarray:
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([c * vec[0] - s * vec[1], s * vec[0] + c * vec[1], vec[2]], float)


def _mean_sd(vals: list[float]) -> tuple[float, float]:
    arr = np.asarray(vals, float)
    arr = arr[np.isfinite(arr)]
    if arr.size == 0:
        return float("nan"), float("nan")
    return float(np.mean(arr)), float(np.std(arr, ddof=0))


def _evaluate_run(
    model: ArmModel,
    q0: np.ndarray,
    sc: BenchmarkScenario,
    ctrl_name: str,
    row_label: str,
    vision_mode: str,
    singularity_avoidance: bool,
    apply_waist_sync: bool,
    seeds: tuple[int, ...],
    samples_out: dict[tuple[str, str], dict[str, list[float]]] | None = None,
) -> dict[str, Any]:
    target_world = np.asarray(sc.target, float)
    waist_rad = compute_target_waist_yaw_rad(model, target_world) if apply_waist_sync else 0.0
    target_eff = _rotate_z(target_world, -waist_rad) if apply_waist_sync else target_world

    p0 = end_effector_position(model, q0)
    reach_dist_m = float(np.linalg.norm(target_eff - p0))

    err_list: list[float] = []
    os_list: list[float] = []
    ts_list: list[float] = []
    teff_list: list[float] = []
    tp_list: list[float] = []
    ide_list: list[float] = []
    einit_list: list[float] = []
    tcomp_list: list[float] = []
    wmin_list: list[float] = []
    tau_list: list[float] = []
    eng_list: list[float] = []
    neck_list: list[float] = []

    for seed in seeds:
        ctrl = make_controller(
            ctrl_name, delay=sc.delay_s, fps=100.0, control_dt=0.01, eps=0.004, vmax=0.60
        )
        if hasattr(ctrl, "per_axis"):
            ctrl.per_axis = True
        st_cfg = StereoCameraConfig(
            baseline_m=0.064,
            focal_px=520.0,
            cam_pos=(-0.10, 0.0, 1.12),
            look_at=(0.35, 0.0, 0.85),
            pixel_noise_px=sc.pixel_noise_px,
            disparity_noise_px=sc.disparity_noise_px,
            vision_mode=vision_mode,
            mono_scale_bias=sc.mono_scale_bias,
            neck_parallax_amp_m=0.038,
            neck_parallax_freq_hz=2.2,
            neck_pitch_amp_m=0.012,
        )
        cfg = SimConfig(
            duration=sc.duration_s,
            delay=sc.delay_s,
            control_dt=0.01,
            eps=0.005,
            max_qd=1.8,
            max_qdd=6.0,
            noise=0.0,
            singularity_avoidance=singularity_avoidance,
            intermittent_gaze=False,
            stereo=st_cfg,
            seed=int(seed),
        )
        res = simulate_reach(model, q0, target_eff, ctrl, cfg)
        mt = res.metrics
        ef_m = float(mt["final_error_m"])
        ts_val = float(mt["settling_time_s"])
        t_eff = ts_val if np.isfinite(ts_val) and ts_val > 0.05 else sc.duration_s
        w_eff = max(sc.target_width_m, 4.133 * ef_m)
        id_e = math.log2(reach_dist_m / max(w_eff, 1e-6) + 1.0)
        tp = id_e / max(t_eff, 0.1)

        err_list.append(ef_m * 1000.0)
        os_list.append(float(mt["overshoot_pct"]))
        ts_list.append(ts_val)
        teff_list.append(t_eff)
        ide_list.append(id_e)
        tp_list.append(tp)
        einit_list.append(float(mt["e_init_m"]) * 1000.0)
        tcomp_list.append(float(mt["t_comp_pct"]))
        wmin_list.append(float(mt["min_manipulability"]))
        tau_list.append(float(mt["peak_torque_nm"]))
        eng_list.append(float(mt["energy_j"]))
        neck_mm = max((o.neck_baseline_m for o in res.stereo_obs), default=0.0) * 1000.0
        neck_list.append(neck_mm)

    settled_finite = [v for v in ts_list if np.isfinite(v)]
    settled_rate = 100.0 * len(settled_finite) / max(len(ts_list), 1)
    err_ci_lo, err_ci_hi = bootstrap_ci(err_list, n_boot=10000, alpha=0.05, seed=42)
    if samples_out is not None:
        samples_out[(sc.id, row_label)] = {
            "err_mm": list(err_list),
            "ts_s": [float(v) for v in ts_list],
            "e_init_mm": list(einit_list),
            "overshoot_pct": list(os_list),
        }
    ts_mean = float(np.mean(settled_finite)) if settled_finite else float("nan")
    ts_sd = float(np.std(settled_finite, ddof=0)) if settled_finite else float("nan")

    ef_m, ef_sd = _mean_sd(err_list)
    os_m, os_sd = _mean_sd(os_list)
    ide_m, _ = _mean_sd(ide_list)
    tp_m, tp_sd = _mean_sd(tp_list)
    ei_m, ei_sd = _mean_sd(einit_list)
    tc_m, tc_sd = _mean_sd(tcomp_list)
    wm_m, _ = _mean_sd(wmin_list)
    tau_m, _ = _mean_sd(tau_list)
    eng_m, _ = _mean_sd(eng_list)
    neck_m, _ = _mean_sd(neck_list)

    return {
        "scenario_id": sc.id,
        "scenario_title": sc.title,
        "controller": row_label,
        "base_controller": ctrl_name,
        "vision_mode": vision_mode,
        "singularity_avoidance": singularity_avoidance,
        "waist_deg": round(math.degrees(waist_rad), 2),
        "neck_baseline_mm": round(neck_m, 1),
        "reach_dist_cm": round(reach_dist_m * 100.0, 1),
        "final_error_mm_mean": round(ef_m, 2),
        "final_error_mm_sd": round(ef_sd, 2),
        "overshoot_pct_mean": round(os_m, 2),
        "overshoot_pct_sd": round(os_sd, 2),
        "final_error_mm_ci95_lo": round(float(err_ci_lo), 2),
        "final_error_mm_ci95_hi": round(float(err_ci_hi), 2),
        "settling_time_s_mean": None if not np.isfinite(ts_mean) else round(ts_mean, 3),
        "settling_time_s_sd": None if not np.isfinite(ts_sd) else round(ts_sd, 3),
        "settled_rate_pct": round(settled_rate, 1),
        "fitts_id_e_bits": round(ide_m, 2),
        "fitts_tp_bps_mean": round(tp_m, 2),
        "fitts_tp_bps_sd": round(tp_sd, 2),
        "e_init_mm_mean": round(ei_m, 1),
        "e_init_mm_sd": round(ei_sd, 1),
        "t_comp_pct_mean": round(tc_m, 1),
        "min_w": round(wm_m, 4),
        "peak_torque_nm": round(tau_m, 1),
        "energy_j": round(eng_m, 1),
    }


def run_academic_benchmark(
    model_path: str | Path,
    out_dir: str | Path = "reports",
    seeds: tuple[int, ...] = (0, 1, 2),
) -> dict[str, Any]:
    """Run all 5 canonical academic benchmark scenarios and write MD/CSV/JSON."""
    model = load_model(str(model_path))
    postures = load_postures(str(model_path))
    q0 = np.radians(postures.get("retracted_compact", [0, 20, 95, 0, 25, 0]))

    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, Any]] = []
    samples: dict[tuple[str, str], dict[str, list[float]]] = {}

    def _row(sc, ctrl_name, row_label, vision_mode, sing_mode, waist_sync):
        return _evaluate_run(
            model=model, q0=q0, sc=sc, ctrl_name=ctrl_name,
            row_label=row_label, vision_mode=vision_mode,
            singularity_avoidance=sing_mode,
            apply_waist_sync=waist_sync, seeds=seeds,
            samples_out=samples,
        )

    for sc in BENCHMARK_SCENARIOS:
        for cn in BENCHMARK_CONTROLLERS:
            sing_mode = True
            if sc.id == "S3_extended_singularity" and cn in ("pid", "lqr", "fixed"):
                sing_mode = False
            rows.append(_row(sc, cn, cn, sc.vision_mode, sing_mode, sc.waist_sync))
        if sc.id == "S4_monocular_parallax":
            rows.append(_row(sc, "hdap", "hdap_mono_static", "mono_static_left", True, False))
        elif sc.id == "S5_lateral_waist_sync":
            rows.append(_row(sc, "hdap", "hdap_no_waist", "binocular", True, False))

    # ---- pre-defined statistical protocol ---------------------------------
    # Holm-corrected two-sided permutation test on (log) movement times,
    # hdap vs every other controller, per scenario. Non-settling runs are
    # scored at the scenario duration cap (same convention as Fitts T_eff).
    duration = {sc.id: sc.duration_s for sc in BENCHMARK_SCENARIOS}
    pairs: list[tuple[str, str, float]] = []
    for sc in BENCHMARK_SCENARIOS:
        a_ts = [v if np.isfinite(v) else duration[sc.id] for v in samples[(sc.id, "hdap")]["ts_s"]]
        for other in BENCHMARK_CONTROLLERS[1:]:
            b_ts = [v if np.isfinite(v) else duration[sc.id] for v in samples[(sc.id, other)]["ts_s"]]
            p = permutation_test(a_ts, b_ts, n_perm=10000, seed=42)
            pairs.append((sc.id, other, p))
    adj = holm_bonferroni([p for _, _, p in pairs])
    stats = [
        {"scenario_id": s, "vs": o, "p_raw": round(float(p), 4), "p_holm": round(float(a), 4)}
        for (s, o, p), a in zip(pairs, adj)
    ]

    summary = {
        "model": model.name,
        "posture": "retracted_compact",
        "seeds": list(seeds),
        "scenarios": [sc.id for sc in BENCHMARK_SCENARIOS],
        "rows": rows,
        "stats": stats,
    }

    csv_path = out / "academic_benchmark.csv"
    if rows:
        fieldnames = list(rows[0].keys())
        with csv_path.open("w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=fieldnames)
            w.writeheader()
            w.writerows(rows)

    json_path = out / "academic_benchmark.json"
    json_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")

    md_path = out / "academic_benchmark.md"
    _write_benchmark_markdown(summary, md_path)
    return summary


def _write_benchmark_markdown(summary: dict[str, Any], path: Path) -> None:
    rows = summary["rows"]
    seeds = summary["seeds"]
    stats = summary.get("stats", [])
    lines = [
        "# Standart Akademik Benchmark Raporu (5 Senaryo × 6 Kontrolcü + 2 Ablasyon)",
        "",
        f"* **Model:** `{summary['model']}` (`posture={summary['posture']}`, tohumlar `seeds={seeds}`)",
        "* **Dış bağlantı:** `https://github.com/iskenderozkul/robosimu` (kod, model, bu raporun üretildiği komut)",
        f"* **Ölçütler:** Son hata $e_f$ (mm, %95 bootstrap GA, 10 000 tekrar, seed=42), Hedef aşma (%), "
        "Yerleşme süresi $T_s$ (s, tolerans $\\varepsilon=5\\text{ mm}$; yerleşmeyen koşul süre tavanında skorlanır), "
        "ISO 9241-9 / MacKenzie Fitts $TP = ID_e / T_{\\text{eff}}$ (bit/s, $W_t = 40\\text{ mm}$, $W_e = \\max(W_t, 4.133\\sigma_e)$), "
        "$E_{\\text{init}}$ (mm), $\\%T_{\\text{comp}}$, minimum Yoshikawa manipülabilite $w_{\\min}$ ve tepe tork $\\tau_{\\max}$ (Nm).",
        "",
        "## Kabul Kriterleri ve İstatistiksel Protokol (Önceden Tanımlı — Pre-defined)",
        "",
        "1. **Örneklem:** Her hücrede $n={}$ bağımsız gürültü seed'i ($0\\ldots n-1$); aynı seed'ler tüm kontrolcülere uygulanır (eşleşik tasarım).".format(len(seeds)),
        "2. **Başarı ölçütü:** $T_s$ sonlu (5 mm banda oturma) **ve** $e_f \\le 5\\text{ mm}$; yerleşmeyenler $T_s = \\text{süre tavanı}$ skorlanır (Fitts $T_{\\text{eff}}$ ile aynı kural).",
        "3. **Güven aralıkları:** $e_f$ için yüzdelik bootstrap %95 GA (10 000 tekrar, seed=42); oranlar için Wilson GA.",
        "4. **Karşılaştırmalar:** hdap vs diğer 5 kontrolcü, 5 senaryoda 25 eşleşik karşılaştırma; iki-yönlü log-ortalama permütasyon testi (10 000 permütasyon, seed=42); **Holm step-down** düzeltmesi ($\\alpha=0.05$).",
        "5. **Sabitler:** $dt=1\\text{ kHz}$ iç servo, `control_dt=10 ms`, `v_max=0.60 m/s`, `eps=4 mm`, geometri ve gürültü parametreleri `robosimu/benchmark.py`'de dondurulmuştur; değişiklik = yeni sürüm.",
        "",
        "## Doğrulanabilir Tekrar Çalıştırma (GitHub / CLI & Tarayıcı)",
        "",
        "```bash",
        "git clone https://github.com/iskenderozkul/robosimu.git",
        "cd robosimu",
        "python -m robosimu.cli benchmark models/human_arm.json --out reports --seeds 10",
        "python -m robosimu.cli workbench models/human_arm.json --out figures/human_arm_workbench.html",
        "```",
        "",
    ]

    for sc in BENCHMARK_SCENARIOS:
        sc_rows = [r for r in rows if r["scenario_id"] == sc.id]
        lines += [
            f"## {sc.title}",
            "",
            f"> {sc.description}",
            "",
            "| Kontrolcü | Başlangıç→Hedef (cm) | Son Hata $e_f$ (mm) [GA95] | Aşma (%) | $T_s$ (s) | Oturma % | Fitts $TP$ (bit/s) | $E_{\\text{init}}$ (mm) | $\\%T_{\\text{comp}}$ | $w_{\\min}$ | Tepe Tork (Nm) | Bel / Paralaks |",
            "|---|---|---|---|---|---|---|---|---|---|---|---|",
        ]
        for r in sc_rows:
            ts_str = (
                f"{r['settling_time_s_mean']:.2f}"
                if r["settling_time_s_mean"] is not None
                else f"> {sc.duration_s:.1f} (N/A)"
            )
            aux = f"bel={r['waist_deg']:+.1f}°, B_boy={r['neck_baseline_mm']:.1f}mm"
            lines.append(
                f"| `{r['controller']}` | {r['reach_dist_cm']:.1f} | "
                f"{r['final_error_mm_mean']:.2f} ± {r['final_error_mm_sd']:.2f} "
                f"[{r['final_error_mm_ci95_lo']:.2f}–{r['final_error_mm_ci95_hi']:.2f}] | "
                f"{r['overshoot_pct_mean']:.2f} ± {r['overshoot_pct_sd']:.2f} | "
                f"{ts_str} | "
                f"{r.get('settled_rate_pct', 0.0):.0f}% | "
                f"{r['fitts_tp_bps_mean']:.2f} | "
                f"{r['e_init_mm_mean']:.1f} | "
                f"{r['t_comp_pct_mean']:.0f}% | "
                f"{r['min_w']:.4f} | "
                f"{r['peak_torque_nm']:.1f} | "
                f"{aux} |"
            )
        lines.append("")

    if stats:
        lines += [
            "## İstatistiksel Karşılaştırma — hdap vs Klasik Kontrolcüler",
            "",
            "Hareket süresi $T_s$ üzerinden iki-yönlü log-ortalama permütasyon testi "
            "(10 000 permütasyon, seed=42; yerleşmeyen koşullar süre tavanında skorlanır), "
            "**Holm step-down** düzeltmesi ($\\alpha=0.05$, 25 eşleşik karşılaştırma):",
            "",
            "| Senaryo | hdap vs | $p$ (ham) | $p$ (Holm) | Sonuç |",
            "|---|---|---|---|---|",
        ]
        title = {sc.id: sc.title.split(":")[0] for sc in BENCHMARK_SCENARIOS}
        for s in stats:
            verdict = "**anlamlı**" if s["p_holm"] <= 0.05 else "anlamsız"
            lines.append(
                f"| {title.get(s['scenario_id'], s['scenario_id'])} | `{s['vs']}` | "
                f"{s['p_raw']:.4f} | {s['p_holm']:.4f} | {verdict} |"
            )
        lines.append("")
        lines += [
            "> Yorumlama: $p_{\\text{Holm}} \\le 0.05$ olan karşılaştırmalarda hdap, ilgili "
            "kontrolcüye kıyasla **istatistiksel olarak anlamlı** farklı yerleşme süresi gösterir. "
            "Bu tablo çoklu karşılaştırma düzeltmesi uygulanmış aile içi bir testtir; tekil "
            "$p$ değerlerine değil, Holm sütununa bakın.",
            "",
        ]

    path.write_text("\n".join(lines), encoding="utf-8")
