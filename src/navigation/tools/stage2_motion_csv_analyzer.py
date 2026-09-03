#!/usr/bin/env python3
"""Offline analyzer for Stage-2 GNSS straight-motion CSV evidence.

This tool is read-only: it NEVER certifies YAML. Certification remains an explicit
operator action in the GUI after reviewing the same metrics.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Iterable

import yaml


def finite_float(v: object) -> float | None:
    try:
        x = float(v)
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) else None


def truth(v: object) -> bool:
    return str(v).strip().lower() in {"1", "true", "yes", "pass"}


def pctl(values: Iterable[float], q: float) -> float:
    a = sorted(x for x in values if math.isfinite(x))
    if not a:
        return math.nan
    pos = max(0.0, min(1.0, q)) * (len(a) - 1)
    lo = int(math.floor(pos)); hi = int(math.ceil(pos))
    if lo == hi:
        return a[lo]
    t = pos - lo
    return a[lo] * (1.0 - t) + a[hi] * t


def first(row: dict[str, str], names: tuple[str, ...]) -> str:
    for n in names:
        if n in row:
            return row[n]
    return ""


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", type=Path)
    ap.add_argument("--localization-yaml", type=Path,
                    default=Path(__file__).resolve().parents[1] / "config/localization_cpp.yaml")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()
    cfg = (yaml.safe_load(a.localization_yaml.read_text(encoding="utf-8")) or {})["localization_core"]["ros__parameters"]
    with a.csv.expanduser().open(newline="", encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit("CSV kosong")

    n = len(rows)
    vel_ratio = sum(truth(first(r, ("validation_velocity_qualified", "velocity_qualified"))) for r in rows) / n
    cog_ratio = sum(truth(first(r, ("validation_cog_qualified", "cog_qualified"))) for r in rows) / n
    cov_ratio = sum(truth(first(r, ("validation_velocity_covariance_valid", "velocity_covariance_valid"))) for r in rows) / n
    quality_ratio = sum(truth(first(r, ("validation_quality_fresh", "quality_fresh"))) for r in rows) / n

    def abs_values(*names: str) -> list[float]:
        out=[]
        for r in rows:
            x=finite_float(first(r, tuple(names)))
            if x is not None and abs(x) < 900.0:
                out.append(abs(x))
        return out

    sync = abs_values("validation_sync_gap_sec", "sync_gap_sec")
    wheel = abs_values("validation_wheel_minus_gnss_mps", "wheel_minus_gnss_mps")
    lateral = abs_values("base_vy", "vel_base_y_mps")
    cog = abs_values("validation_cog_minus_vel_course_rad", "cog_minus_vel_course_rad")

    sync95=pctl(sync,.95); wheel95=pctl(wheel,.95); lat95=pctl(lateral,.95); cog95=pctl(cog,.95)
    min_vel=int(cfg["stage2_min_velocity_epochs"]); min_cog=int(cfg["stage2_min_cog_epochs"])
    min_vel_ratio=float(cfg["stage2_min_velocity_qualified_ratio"]); min_cog_ratio=float(cfg["stage2_min_cog_qualified_ratio"])
    max_sync=float(cfg["stage2_max_sync_gap_p95_sec"]); max_wheel=float(cfg["stage2_max_wheel_gnss_residual_p95_mps"])
    max_lat=float(cfg["stage2_max_lateral_velocity_p95_mps"]); max_cog=float(cfg["stage2_max_cog_doppler_residual_p95_rad"])

    velocity_pass=(n>=min_vel and vel_ratio>=min_vel_ratio and cov_ratio>=min_vel_ratio and
                   quality_ratio>=min_vel_ratio and math.isfinite(sync95) and sync95<=max_sync and
                   math.isfinite(wheel95) and wheel95<=max_wheel and math.isfinite(lat95) and lat95<=max_lat)
    cog_pass=(velocity_pass and len(cog)>=min_cog and cog_ratio>=min_cog_ratio and
              math.isfinite(cog95) and cog95<=max_cog)

    report={
        "csv":str(a.csv), "total_epochs":n, "cog_residual_epochs":len(cog),
        "velocity_qualified_ratio":vel_ratio, "cog_qualified_ratio":cog_ratio,
        "velocity_covariance_valid_ratio":cov_ratio, "quality_fresh_ratio":quality_ratio,
        "sync_gap_p95_sec":sync95, "wheel_gnss_residual_p95_mps":wheel95,
        "base_lateral_velocity_p95_mps":lat95,
        "cog_vs_doppler_p95_rad":cog95,
        "cog_vs_doppler_p95_deg":math.degrees(cog95) if math.isfinite(cog95) else math.nan,
        "velocity_run_pass":velocity_pass, "cog_run_pass":cog_pass,
    }
    if a.json:
        print(json.dumps(report, indent=2, allow_nan=True))
    else:
        print("STAGE-2 MOTION CSV ANALYSIS")
        for k,v in report.items(): print(f"{k:38s}: {v}")
        print("\nResult: velocity=%s, COG=%s" % ("PASS" if velocity_pass else "FAIL", "PASS" if cog_pass else "FAIL"))
    return 0 if velocity_pass else 2


if __name__ == "__main__":
    raise SystemExit(main())
