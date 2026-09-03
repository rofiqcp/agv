#!/usr/bin/env python3
"""Rank AGV GUI tuning runs from CSV sidecar metadata.

Usage:
  python3 experiment_rank.py ~/.ros/agv_gui_reports

The GUI writes *.meta.json beside every CSV. This tool extracts RMSE/mean-abs
error metrics and produces a compact comparison table plus CSV for BAB IV.
Lower score is better; the raw metrics remain in the output so the final choice
can still prioritize safety or goal accuracy over a single scalar score.
"""
from __future__ import annotations
import argparse, csv, json, math
from pathlib import Path

ERROR_HINTS = {
    "velocity": 1.0,
    "steering": 1.0,
    "yaw_rate": 0.8,
    "cross_track": 1.5,
    "heading": 1.2,
    "goal": 1.5,
    "residual": 0.8,
}

def metric_weight(name: str) -> float:
    low=name.lower()
    for k,w in ERROR_HINTS.items():
        if k in low:
            return w
    return 0.5

def load(meta: Path):
    data=json.loads(meta.read_text(encoding="utf-8"))
    metrics=[]; weighted=[]
    for name,stat in (data.get("numeric_summary") or {}).items():
        if not isinstance(stat,dict): continue
        rmse=stat.get("rmse")
        if isinstance(rmse,(int,float)) and math.isfinite(float(rmse)):
            w=metric_weight(name); metrics.append((name,float(rmse))); weighted.append((w,float(rmse)))
    score=sum(w*v for w,v in weighted)/sum(w for w,_ in weighted) if weighted else float("inf")
    return {
        "meta":meta.name, "csv":data.get("csv",""), "session_id":data.get("session_id",""),
        "category":data.get("experiment_category",""), "variant":data.get("experiment_variant",""),
        "score":score, "rmse_metrics":"; ".join(f"{k}={v:.6g}" for k,v in metrics),
    }

def main():
    ap=argparse.ArgumentParser(); ap.add_argument("folder", nargs="?", default="~/.ros/agv_gui_reports")
    args=ap.parse_args(); root=Path(args.folder).expanduser(); root.mkdir(parents=True, exist_ok=True)
    rows=[load(p) for p in sorted(root.rglob("*.meta.json"))]
    rows.sort(key=lambda r:r["score"])
    out=root/"experiment_ranking.csv"
    with out.open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0].keys()) if rows else ["meta","score"]); w.writeheader(); w.writerows(rows)
    print(f"Ranking: {out}")
    for i,r in enumerate(rows[:20],1):
        score="N/A" if not math.isfinite(r["score"]) else f"{r['score']:.6f}"
        print(f"{i:02d}. {r['category']}/{r['variant']} score={score} csv={r['csv']}")

if __name__=="__main__": main()
