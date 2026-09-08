#!/usr/bin/env python3
"""Export one experiment trial to report-ready XLSX and Matplotlib PNG files."""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from openpyxl import Workbook
from openpyxl.styles import Font, PatternFill, Alignment
from openpyxl.utils import get_column_letter


def load_json(path: str, default):
    p = Path(path)
    if not p.is_file():
        return default
    return json.loads(p.read_text(encoding="utf-8"))


def to_float(value):
    try:
        x = float(value)
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None

def read_rows(path: str):
    with open(path, newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        return list(reader), list(reader.fieldnames or [])


def style_sheet(ws):
    fill = PatternFill("solid", fgColor="D9EAF7")
    for cell in ws[1]:
        cell.font = Font(bold=True)
        cell.fill = fill
        cell.alignment = Alignment(horizontal="center")
    ws.freeze_panes = "A2"
    for idx, col in enumerate(ws.columns, 1):
        width = min(42, max(10, max(len(str(c.value or "")) for c in col) + 2))
        ws.column_dimensions[get_column_letter(idx)].width = width


def write_workbook(rows, columns, summary, trials, output):
    wb = Workbook()
    ws = wb.active
    ws.title = "Raw Data"
    ws.append(columns)
    for row in rows:
        ws.append([row.get(c, "") for c in columns])
    style_sheet(ws)

    sm = wb.create_sheet("Trial Summary")
    sm.append(["Parameter", "Value"])
    for key, value in summary.items():
        sm.append([key, json.dumps(value, ensure_ascii=False) if isinstance(value, (dict, list)) else value])
    style_sheet(sm)

    recap = wb.create_sheet("Trials Recap")
    trial_rows = trials if isinstance(trials, list) else []
    keys = []
    for trial in trial_rows:
        for key in trial.keys():
            if key not in keys:
                keys.append(key)
    if keys:
        recap.append(keys)
        for trial in trial_rows:
            recap.append([trial.get(k, "") for k in keys])
    else:
        recap.append(["No saved trials"])
    style_sheet(recap)
    wb.save(output)


def series_xy(rows, x_key, y_key):
    xs, ys = [], []
    for row in rows:
        x, y = to_float(row.get(x_key)), to_float(row.get(y_key))
        if x is None or y is None:
            continue
        xs.append(x); ys.append(y)
    return xs, ys


def save_graphs(rows, spec, output_prefix):
    live = spec.get("live_series", {})
    graphs = spec.get("graphs", [])
    paths = []
    for index, graph in enumerate(graphs, 1):
        fig, ax = plt.subplots(figsize=(8.0, 4.8), facecolor="white")
        ax.set_facecolor("white")
        kind = graph.get("type", "time_series")
        plotted = False
        if kind == "scatter":
            x_key = graph.get("xSeries", "")
            y_key = graph.get("ySeries", "")
            xs, ys = series_xy(rows, x_key, y_key)
            if xs:
                ax.plot(xs, ys, marker=".", linewidth=1.3, label="Trajectory")
                plotted = True
        else:
            elapsed = [to_float(r.get("elapsed_s")) for r in rows]
            for label in graph.get("series", []):
                key = live.get(label, label)
                xs, ys = [], []
                for t, row in zip(elapsed, rows):
                    y = to_float(row.get(key))
                    if t is None or y is None:
                        continue
                    xs.append(t); ys.append(y)
                if xs:
                    ax.plot(xs, ys, linewidth=1.6, label=label)
                    plotted = True
        ax.set_xlabel(graph.get("xLabel") or ("Time [s]" if kind != "scatter" else "X"))
        ax.set_ylabel(graph.get("yLabel") or "Value")
        ax.grid(True, alpha=0.25)
        if plotted:
            ax.legend(loc="best")
        title = graph.get("title") or f"Trial Graph {index}"
        ax.set_title(title)
        fig.tight_layout()
        path = f"{output_prefix}_G{index:02d}.png"
        fig.savefig(path, dpi=180, facecolor="white", bbox_inches="tight")
        plt.close(fig)
        paths.append(path)
    return paths

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--summary", required=True)
    ap.add_argument("--trials", required=True)
    ap.add_argument("--spec", required=True)
    ap.add_argument("--xlsx", required=True)
    ap.add_argument("--png-prefix", required=True)
    args = ap.parse_args()

    rows, columns = read_rows(args.csv)
    summary = load_json(args.summary, {})
    trials = load_json(args.trials, [])
    spec = load_json(args.spec, {})
    write_workbook(rows, columns, summary, trials, args.xlsx)
    pngs = save_graphs(rows, spec, args.png_prefix)
    print(json.dumps({"xlsx": args.xlsx, "pngs": pngs}, ensure_ascii=False))


if __name__ == "__main__":
    main()
