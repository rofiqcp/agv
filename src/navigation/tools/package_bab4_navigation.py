#!/usr/bin/env python3
"""Build a Windows-safe, report-ready BAB IV 4.2-4.4 package."""
from __future__ import annotations

import argparse
import csv
import json
import math
import re
import shutil
import zipfile
from datetime import datetime
from pathlib import Path

from openpyxl import Workbook
from openpyxl.styles import Alignment, Font, PatternFill
from openpyxl.utils import get_column_letter

SECTION_IDS = [
    "4.2.1","4.2.2","4.2.3","4.2.4","4.2.5",
    "4.3.1","4.3.2","4.3.3","4.3.4","4.3.5",
    "4.4.1","4.4.2","4.4.3","4.4.4","4.4.5",
]
GRAPH_SECTION_ORDER = [
    "4.2.1","4.2.2","4.2.3","4.2.4","4.2.5",
    "4.3.1","4.3.2","4.3.3","4.3.4","4.3.5",
    "4.4.1","4.4.2","4.4.3","4.4.4",
    "4.4.5","4.4.5","4.4.5","4.4.5",
]

REPORT_NAMES = {
    "4.2.1": "Frequency_EKF_Lokal",
    "4.2.2": "Sensor_Timeout_EKF_Lokal",
    "4.2.3": "Measurement_Covariance_EKF_Lokal",
    "4.2.4": "Process_Noise_EKF_Lokal",
    "4.2.5": "Predict_Current_Time_EKF_Lokal",
    "4.3.1": "Frequency_EKF_Global",
    "4.3.2": "Sensor_Timeout_EKF_Global",
    "4.3.3": "Measurement_Covariance_GNSS",
    "4.3.4": "Process_Noise_EKF_Global",
    "4.3.5": "Predict_Current_Time_EKF_Global",
    "4.4.1": "Minimum_Turning_Radius",
    "4.4.2": "Downsampling",
    "4.4.3": "Planner_Tolerance",
    "4.4.4": "Cost_Penalty",
    "4.4.5": "Goal_Checker",
}
TABLE_NAMES = {
    "4.2.1": "Tabel 4.10 Pengujian Frequency EKF Lokal Aktual",
    "4.2.2": "Tabel 4.11 Pengujian sensor_timeout EKF Lokal Aktual",
    "4.2.3": "Tabel 4.12–4.13 Measurement Covariance R EKF Lokal Aktual",
    "4.2.4": "Tabel 4.14 Pengujian Process Noise Q EKF Lokal Aktual",
    "4.2.5": "Tabel 4.15 Pengujian predict_to_current_time EKF Lokal Aktual",
    "4.3.1": "Tabel 4.16 Pengujian Frequency EKF Global Aktual",
    "4.3.2": "Tabel 4.17 Pengujian sensor_timeout EKF Global Aktual",
    "4.3.3": "Tabel 4.18–4.19 Measurement Covariance GNSS Aktual",
    "4.3.4": "Tabel 4.20 Pengujian Process Noise Posisi EKF Global Aktual",
    "4.3.5": "Tabel 4.21 Pengujian predict_to_current_time EKF Global Aktual",
    "4.4.1": "Tabel 4.22 Pengujian minimum_turning_radius Aktual",
    "4.4.2": "Tabel 4.23 Pengujian downsampling_factor Aktual",
    "4.4.3": "Tabel 4.24 Pengujian Planner Tolerance dan Endpoint Error Aktual",
    "4.4.4": "Tabel 4.25 Pengujian cost_penalty Aktual",
    "4.4.5": "Tabel 4.26 Goal Checker dan Akurasi Posisi Aktual",
}

def safe_token(value: object, fallback: str = "unknown") -> str:
    s = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value or "")).strip("._-")
    return s or fallback

def chapter_for(sid: str) -> str:
    return ".".join(sid.split(".")[:2])

def copy_if_file(src: object, dst: Path) -> bool:
    p = Path(str(src or ""))
    if not p.is_file():
        return False
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(p, dst)
    return True

def read_csv_rows(path: Path) -> list[list[str]]:
    with path.open(newline="", encoding="utf-8-sig") as fh:
        return list(csv.reader(fh))

def normalize_gui_table(rows: list[list[str]]) -> list[list[str]]:
    """Keep only columns visible in GUI. Legacy captures also used zero-based seconds."""
    if not rows:
        return rows
    head = rows[0]
    keep = [i for i, h in enumerate(head) if not str(h).startswith("YAML ")]
    out = [[row[i] if i < len(row) else "" for i in keep] for row in rows]
    if out and out[0] and out[0][0] == "Detik [s]" and len(out) > 1:
        try:
            zero_based = int(float(out[1][0])) == 0
        except Exception:
            zero_based = False
        if zero_based:
            for row in out[1:]:
                if not row:
                    continue
                try:
                    row[0] = str(int(float(row[0])) + 1)
                except Exception:
                    pass
    return out

def trim_table_to_terminal(rows: list[list[str]], expected_seconds: int) -> list[list[str]]:
    """Keep exactly Detik 1..terminal from the live mission table; never synthesize rows."""
    rows = normalize_gui_table(rows)
    if not rows or expected_seconds <= 0:
        return rows
    header, data = rows[0], rows[1:]
    by_second: dict[int, list[str]] = {}
    for row in data:
        if not row:
            continue
        try:
            sec = int(float(row[0]))
        except Exception:
            continue
        if 1 <= sec <= expected_seconds and sec not in by_second:
            by_second[sec] = row
    ordered = [by_second[s] for s in range(1, expected_seconds + 1) if s in by_second]
    return [header] + ordered

def write_csv_rows(path: Path, rows: list[list[str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8-sig") as fh:
        csv.writer(fh).writerows(rows)

def style_table_sheet(ws, title: str, col_count: int) -> None:
    title_fill = PatternFill("solid", fgColor="D9EAF7")
    header_fill = PatternFill("solid", fgColor="E2F0D9")
    if col_count:
        ws.merge_cells(start_row=1, start_column=1, end_row=1, end_column=col_count)
    ws["A1"] = title
    ws["A1"].font = Font(bold=True, size=12)
    ws["A1"].fill = title_fill
    ws["A1"].alignment = Alignment(horizontal="left", vertical="center", wrap_text=True)
    for cell in ws[2]:
        cell.font = Font(bold=True)
        cell.fill = header_fill
        cell.alignment = Alignment(horizontal="center", vertical="center", wrap_text=True)
    ws.freeze_panes = "A3"
    for idx, col in enumerate(ws.columns, 1):
        vals = [len(str(c.value or "")) for c in list(col)[:150]]
        ws.column_dimensions[get_column_letter(idx)].width = min(36, max(10, (max(vals) if vals else 8) + 2))

def build_table_xlsx(out: Path, title: str, rows: list[list[str]], meta: dict) -> None:
    wb = Workbook()
    ws = wb.active
    ws.title = "Data"
    header = rows[0] if rows else ["No data"]
    ws.append([title] + [""] * max(0, len(header)-1))
    ws.append(header)
    for row in rows[1:]:
        ws.append(row)
    style_table_sheet(ws, title, len(header))

    info = wb.create_sheet("Info")
    info.append(["Field", "Value"])
    for k, v in [
        ("Section", meta.get("section_id", "")),
        ("Table", title),
        ("Mission", meta.get("mission_id", "")),
        ("Result", meta.get("mission_result", "")),
        ("Candidate", meta.get("candidate", "")),
        ("Cycle", meta.get("cycle", "")),
        ("Trial ID", meta.get("trial_id", "")),
        ("Generated", meta.get("created_at", "")),
    ]:
        info.append([k, v])
    info["A1"].font = info["B1"].font = Font(bold=True)
    info["A1"].fill = info["B1"].fill = PatternFill("solid", fgColor="D9EAF7")
    info.column_dimensions["A"].width = 18
    info.column_dimensions["B"].width = 56
    wb.save(out)

def build_report_tree(content_root: Path, report_root: Path, section_tables: dict, section_graphs: dict, cycle: int) -> None:
    if report_root.exists():
        shutil.rmtree(report_root)
    for sid in SECTION_IDS:
        dst = report_root / chapter_for(sid) / sid
        dst.mkdir(parents=True, exist_ok=True)
        label = REPORT_NAMES[sid]
        tables = section_tables.get(sid, [])
        graphs = section_graphs.get(sid, [])

        # The report ZIP intentionally contains only report-facing files.
        for i, t in enumerate(tables, 1):
            target = dst / (f"Tabel_{label}.xlsx" if len(tables) == 1 else f"Tabel_{i}_{label}.xlsx")
            shutil.copy2(Path(t["xlsx"]), target)

        for i, g in enumerate(graphs, 1):
            target = dst / (f"Grafik_{label}.png" if len(graphs) == 1 else f"Grafik_{i}_{label}.png")
            shutil.copy2(Path(g["png"]), target)

        # Do not fabricate an overlay from a single run.
        # A true 3-profile overlay for 4.2/4.3 is only valid after Baseline,
        # Candidate A, and Candidate B all exist. 4.4 intentionally has no cycling overlay.


def section_from_table_path(path: Path) -> str | None:
    m = re.search(r"(4\.[234]\.[1-5])", path.name)
    return m.group(1) if m and m.group(1) in SECTION_IDS else None

def section_from_graph(path: Path, title: str, index: int) -> str | None:
    m = re.search(r"(4\.[234]\.[1-5])", title or "")
    if m and m.group(1) in SECTION_IDS:
        return m.group(1)
    m = re.search(r"_G(\d{2})(?:_|\.)", path.name)
    if m:
        gi = int(m.group(1)) - 1
        if 0 <= gi < len(GRAPH_SECTION_ORDER):
            return GRAPH_SECTION_ORDER[gi]
    if 0 <= index < len(GRAPH_SECTION_ORDER):
        return GRAPH_SECTION_ORDER[index]
    return None
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--descriptor", required=True)
    args = ap.parse_args()
    desc = json.loads(Path(args.descriptor).read_text(encoding="utf-8"))
    strict_real_field = bool(desc.get("strict_real_field", False))
    browser_graph_indices = {int(x) for x in desc.get("browser_graph_png_indices", []) if str(x).isdigit()}
    expected_graph_count = int(desc.get("expected_graph_count") or 0)
    browser_graph_count = int(desc.get("browser_graph_png_count") or 0)
    synthetic_fill = bool(desc.get("synthetic_fill", False))
    continuity = desc.get("table_continuity") if isinstance(desc.get("table_continuity"), dict) else {}
    expected_seconds = int(continuity.get("expected_seconds") or 0)

    source_folder = Path(desc["source_folder"]).resolve()
    packages_root = Path(desc["packages_root"]).resolve()
    packages_root.mkdir(parents=True, exist_ok=True)

    mission_id = safe_token(desc.get("mission_id"), "MISSION")
    trial_id = safe_token(desc.get("trial_id"), "TRIAL")
    result = safe_token(desc.get("mission_result"), "UNKNOWN").upper()
    candidate = safe_token(desc.get("candidate"), "baseline")
    cycle = int(desc.get("cycle") or 0)
    package_id = safe_token(f"BAB4_{mission_id}_{result}_{trial_id}")
    package_dir = packages_root / package_id
    if package_dir.exists():
        shutil.rmtree(package_dir)
    package_dir.mkdir(parents=True)

    result_code = "OK" if result in {"SUCCEEDED","SUCCESS","ARRIVED"} else ("FAIL" if result in {"FAILED","FAILURE","ABORTED","REJECTED"} else "STOP")
    time_token = (trial_id.replace("_","")[-9:-3] or datetime.now().strftime("%H%M%S"))[-6:]
    root_name = safe_token(f"B4_C{cycle:02d}_{result_code}_{time_token}")
    content_root = package_dir / "content" / root_name
    content_root.mkdir(parents=True)

    meta = {
        "package_id": package_id,
        "mission_id": desc.get("mission_id", ""),
        "mission_result": desc.get("mission_result", ""),
        "candidate": desc.get("candidate", ""),
        "cycle": cycle,
        "trial_id": desc.get("trial_id", ""),
        "source_experiment_id": desc.get("source_experiment_id", "R4.4.5"),
        "source_folder": str(source_folder),
        "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "root_name": root_name,
    }

    copy_if_file(desc.get("primary_csv"), content_root / "RAW.csv")
    copy_if_file(desc.get("master_xlsx"), content_root / "MASTER.xlsx")
    copy_if_file(desc.get("session_manifest"), content_root / "mission.json")

    section_tables: dict[str, list[dict]] = {k: [] for k in SECTION_IDS}
    table_entries = [x for x in desc.get("table_entries", []) if isinstance(x, dict)]
    table_meta = {}
    for ent in table_entries:
        sid = str(ent.get("section_id", ""))
        if sid in SECTION_IDS:
            ti = int(ent.get("table_index", 0)) + 1
            table_meta[(sid, ti)] = ent

    def add_live_table(src: Path, sid: str, ti: int, title: str, source_kind: str) -> bool:
        if sid not in SECTION_IDS or not src.is_file():
            return False
        try:
            resolved = src.resolve()
            if strict_real_field and source_folder not in resolved.parents:
                return False
            rows = trim_table_to_terminal(read_csv_rows(resolved), expected_seconds)
        except Exception:
            return False
        folder = content_root / chapter_for(sid) / sid
        csv_out = folder / f"T{ti}.csv"
        xlsx_out = folder / f"T{ti}.xlsx"
        write_csv_rows(csv_out, rows)
        build_table_xlsx(xlsx_out, title, rows, {**meta, "section_id": sid})
        section_tables[sid].append({"index":ti,"title":title,"csv":csv_out,"xlsx":xlsx_out,
                                    "columns":rows[0] if rows else [],"source_kind":source_kind})
        return True

    # Prefer explicit table entry paths when valid.
    for ent in table_entries:
        src = Path(str(ent.get("path", "")))
        sid = str(ent.get("section_id", ""))
        ti = int(ent.get("table_index", 0)) + 1
        title = str(ent.get("table_name") or TABLE_NAMES.get(sid) or f"Tabel {sid}")
        if any(t["index"] == ti for t in section_tables.get(sid, [])):
            continue
        add_live_table(src, sid, ti, title, "CLIENT_TABLE_ENTRY")

    # Authoritative field-table source: CSV saved during this same mission.
    # This is allowed in strict mode because it is live mission evidence, not legacy reconstruction.
    for raw in desc.get("table_csv_paths", []):
        src = Path(str(raw))
        sid = section_from_table_path(src)
        if not sid:
            continue
        ti = 1
        if any(t["index"] == ti for t in section_tables[sid]):
            continue
        ent = table_meta.get((sid, ti), {})
        title = str(ent.get("table_name") or TABLE_NAMES.get(sid) or f"Tabel {sid}")
        add_live_table(src, sid, ti, title, "LIVE_MISSION_TABLE_CSV")

    section_graphs: dict[str, list[dict]] = {k: [] for k in SECTION_IDS}
    graph_paths = [Path(str(x)) for x in desc.get("graph_png_paths", [])]
    graph_titles = [str(x) for x in desc.get("graph_png_titles", [])]
    for idx, src in enumerate(graph_paths):
        global_graph_index = idx + 1
        if not src.is_file():
            continue
        title = graph_titles[idx] if idx < len(graph_titles) else ""
        sid = section_from_graph(src, title, idx)
        if not sid:
            continue
        folder = content_root / chapter_for(sid) / sid
        folder.mkdir(parents=True, exist_ok=True)
        gi = len(section_graphs[sid]) + 1
        dst = folder / f"G{gi}.png"
        shutil.copy2(src, dst)
        section_graphs[sid].append({"index":gi,"title":title or f"Grafik {gi}","png":dst})
    integrity = {}
    index_sections = {}
    for sid in SECTION_IDS:
        folder = content_root / chapter_for(sid) / sid
        folder.mkdir(parents=True, exist_ok=True)
        tables = section_tables[sid]
        graphs = section_graphs[sid]
        lines = [
            f"SUBBAB {sid}",
            f"Mission: {meta['mission_id']}",
            f"Result: {meta['mission_result']}",
            f"Candidate: {meta['candidate']}",
            f"Cycle: {cycle}",
            "",
            "TABEL SESUAI GUI:",
        ]
        if tables:
            for t in tables:
                lines.append(f"T{t['index']}.csv / T{t['index']}.xlsx = {t['title']}")
                lines.append("  Kolom: " + " | ".join(str(x) for x in t.get("columns", [])))
        else:
            lines.append("(tidak ada)")
        lines += ["", "GRAFIK:"]
        if graphs:
            for g in graphs:
                lines.append(f"G{g['index']}.png = {g['title']}")
        else:
            lines.append("(tidak ada)")
        (folder / "INDEX.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")

        integrity[sid] = {
            "table_csv_count": len(tables),
            "table_xlsx_count": sum(1 for t in tables if Path(t["xlsx"]).is_file()),
            "graph_png_count": len(graphs),
            "ready": bool(tables) and bool(graphs) and all(Path(t["xlsx"]).is_file() for t in tables),
        }
        index_sections[sid] = {
            "tables": [{"file":f"T{t['index']}.csv","xlsx":f"T{t['index']}.xlsx","title":t["title"],"columns":t.get("columns",[])} for t in tables],
            "graphs": [{"file":f"G{g['index']}.png","title":g["title"]} for g in graphs],
        }

    root_master = content_root / "MASTER.xlsx"
    root_raw = content_root / "RAW.csv"
    continuity_sections = continuity.get("sections", []) if isinstance(continuity.get("sections"), list) else []
    no_backfill = all(not (x.get("backfilled_seconds") or []) for x in continuity_sections if isinstance(x, dict))
    continuity_ok = continuity.get("ok") is True and expected_seconds > 0 and no_backfill
    browser_graphs_complete = expected_graph_count > 0 and browser_graph_count == expected_graph_count and len(browser_graph_indices) == expected_graph_count
    field_graph_count = sum(len(section_graphs[sid]) for sid in SECTION_IDS)
    field_graphs_complete = expected_graph_count > 0 and field_graph_count >= expected_graph_count
    server_field_graph_count = max(0, field_graph_count - browser_graph_count)
    raw_sample_count = 0
    raw_second_counts = {}
    raw_missing_seconds = []
    raw_terminal_state = ""
    raw_terminal_ok = False
    raw_first_elapsed_s = None
    raw_last_elapsed_s = None
    if root_raw.is_file():
        try:
            raw_rows = read_csv_rows(root_raw)
            if raw_rows:
                header = raw_rows[0]
                idx_elapsed = header.index("elapsed_s") if "elapsed_s" in header else -1
                idx_goal = header.index("goal_state.state") if "goal_state.state" in header else -1
                raw_sample_count = max(0, len(raw_rows) - 1)
                for row in raw_rows[1:]:
                    if idx_elapsed < 0 or idx_elapsed >= len(row):
                        continue
                    try:
                        elapsed = float(row[idx_elapsed])
                    except Exception:
                        continue
                    if not math.isfinite(elapsed) or elapsed < 0:
                        continue
                    if raw_first_elapsed_s is None:
                        raw_first_elapsed_s = elapsed
                    raw_last_elapsed_s = elapsed
                    sec = int(math.floor(elapsed + 1e-9))
                    raw_second_counts[sec] = raw_second_counts.get(sec, 0) + 1
                    if idx_goal >= 0 and idx_goal < len(row):
                        state = str(row[idx_goal]).strip().upper()
                        if state:
                            raw_terminal_state = state
                if expected_seconds > 0:
                    raw_missing_seconds = [s + 1 for s in range(expected_seconds) if raw_second_counts.get(s, 0) < (1 if s == expected_seconds - 1 else 5)]
                success_states = {"SUCCEEDED", "SUCCESS", "ARRIVED"}
                failed_states = {"FAILED", "FAILURE", "ABORTED", "REJECTED"}
                canceled_states = {"CANCELED", "CANCELLED"}
                mr = str(meta.get("mission_result", "")).upper()
                if mr in success_states:
                    raw_terminal_ok = raw_terminal_state in success_states
                elif mr in failed_states:
                    raw_terminal_ok = raw_terminal_state in failed_states
                elif mr in canceled_states:
                    raw_terminal_ok = raw_terminal_state in canceled_states
                else:
                    raw_terminal_ok = bool(raw_terminal_state)
        except Exception:
            raw_sample_count = 0
            raw_second_counts = {}
            raw_missing_seconds = list(range(1, expected_seconds + 1)) if expected_seconds > 0 else []
            raw_terminal_state = ""
            raw_terminal_ok = False
    raw_recording_ok = (
        raw_sample_count > 0
        and expected_seconds > 0
        and not raw_missing_seconds
        and raw_terminal_ok
        and raw_first_elapsed_s is not None
        and raw_first_elapsed_s <= 0.25
    )
    table_second_checks = {}
    for sid in SECTION_IDS:
        checks = []
        for t in section_tables[sid]:
            rows = read_csv_rows(Path(t["csv"]))
            seconds = []
            for row in rows[1:]:
                try:
                    seconds.append(int(float(row[0])))
                except Exception:
                    pass
            exact = bool(expected_seconds) and seconds == list(range(1, expected_seconds + 1))
            checks.append({"table_index": t["index"], "rows": len(seconds), "first": seconds[0] if seconds else None,
                           "last": seconds[-1] if seconds else None, "continuous_1hz": exact})
        table_second_checks[sid] = checks
        if not checks or not all(x["continuous_1hz"] for x in checks):
            integrity[sid]["ready"] = False
            integrity[sid]["continuous_1hz"] = False
        else:
            integrity[sid]["continuous_1hz"] = True

    tables_complete = all(
        table_second_checks[sid] and all(c.get("continuous_1hz") for c in table_second_checks[sid])
        for sid in SECTION_IDS
    )
    strict_sources_ok = (not strict_real_field) or (
        tables_complete and field_graphs_complete and no_backfill and not synthetic_fill and raw_recording_ok
    )
    ready_sections = sum(1 for v in integrity.values() if v["ready"])
    package_ready = root_master.is_file() and root_raw.is_file() and ready_sections == len(SECTION_IDS) and continuity_ok and strict_sources_ok

    package_manifest = {
        **meta,
        "schema": "agv-bab4-navigation-package-v2",
        "scope": "4.2-4.4",
        "windows_safe_paths": True,
        "strict_real_field": strict_real_field,
        "data_origin": "FIELD_RECORDING" if strict_real_field else "LEGACY_RECOVERY",
        "table_origin": "LIVE_MISSION_TABLE_CSV" if strict_real_field else desc.get("table_origin", ""),
        "graph_origin": ("GUI_BROWSER_CANVAS+SERVER_FIELD_RENDER" if strict_real_field and field_graphs_complete and not browser_graphs_complete else desc.get("graph_origin", "")),
        "synthetic_fill": synthetic_fill,
        "legacy_table_fallback_used": False if strict_real_field else ((not table_entries) and bool(desc.get("table_csv_paths"))),
        "gui_exact_tables": tables_complete,
        "live_table_csv_used": any(t.get("source_kind") == "LIVE_MISSION_TABLE_CSV" for sid in SECTION_IDS for t in section_tables[sid]),
        "browser_graph_png_count": browser_graph_count,
        "browser_graph_png_indices": sorted(browser_graph_indices),
        "expected_graph_count": expected_graph_count,
        "browser_graphs_complete": browser_graphs_complete,
        "field_graph_count": field_graph_count,
        "field_graphs_complete": field_graphs_complete,
        "server_field_graph_count": server_field_graph_count,
        "raw_sample_count": raw_sample_count,
        "raw_first_elapsed_s": raw_first_elapsed_s,
        "raw_last_elapsed_s": raw_last_elapsed_s,
        "raw_second_counts": {str(k + 1): v for k, v in sorted(raw_second_counts.items()) if 0 <= k < expected_seconds},
        "raw_missing_seconds": raw_missing_seconds,
        "raw_terminal_state": raw_terminal_state,
        "raw_terminal_ok": raw_terminal_ok,
        "raw_recording_ok": raw_recording_ok,
        "no_backfill": no_backfill,
        "strict_sources_ok": strict_sources_ok,
        "table_continuity": continuity,
        "expected_seconds": expected_seconds,
        "continuous_1hz": continuity_ok and all(all(c.get("continuous_1hz") for c in table_second_checks[sid]) for sid in SECTION_IDS),
        "table_second_checks": table_second_checks,
        "status": "READY" if package_ready else "INCOMPLETE",
        "ready_sections": ready_sections,
        "total_sections": len(SECTION_IDS),
        "integrity": integrity,
        "sections": index_sections,
    }
    (content_root / "INDEX.json").write_text(json.dumps(package_manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    readme = (
        "AGV BAB IV DATA PACKAGE 4.2-4.4\n"
        "4.1 intentionally separate.\n\n"
        f"Mission : {meta['mission_id']}\n"
        f"Result  : {meta['mission_result']}\n"
        f"Candidate: {meta['candidate']}\n"
        f"Cycle   : {cycle}\n"
        f"Trial   : {meta['trial_id']}\n"
        f"Status  : {package_manifest['status']} ({ready_sections}/{len(SECTION_IDS)})\n\n"
        "Folder/file names are intentionally short for Windows extraction.\n"
        "Open each 4.x/4.x.x/INDEX.txt to see the full GUI table and graph titles.\n"
        "Tn.csv and Tn.xlsx contain the same visible table columns as the GUI.\n"
        "Gn.png contains the graph evidence for that subchapter.\n"
    )
    (content_root / "README.txt").write_text(readme, encoding="utf-8")

    # Build the report-facing tree requested by the thesis workflow.
    # The downloaded ZIP contains ONLY 4.2 / 4.3 / 4.4 and their subchapters.
    report_root = package_dir / "report"
    build_report_tree(content_root, report_root, section_tables, section_graphs, cycle)

    zip_name = safe_token(f"BAB4_Report_{candidate}_C{cycle:02d}.zip")
    zip_path = package_dir / zip_name
    for old_zip in package_dir.glob("*.zip"):
        old_zip.unlink(missing_ok=True)
    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as zf:
        for p in sorted(report_root.rglob("*")):
            if p.is_file():
                zf.write(p, p.relative_to(report_root))

    # Guarantee Windows-friendly internal paths.
    max_internal_path = 0
    with zipfile.ZipFile(zip_path) as zf:
        for n in zf.namelist():
            max_internal_path = max(max_internal_path, len(n))
    package_json = {
        **package_manifest,
        "schema": "agv-bab4-report-package-v3",
        "report_only_zip": True,
        "report_root": str(report_root),
        "zip_path": str(zip_path),
        "zip_name": zip_path.name,
        "package_dir": str(package_dir),
        "bytes": zip_path.stat().st_size if zip_path.exists() else 0,
        "max_internal_path_chars": max_internal_path,
        "download_url": f"/api/navigation/data-package/{package_id}.zip",
        "delete_url": "/api/navigation/data-package/delete",
    }
    (package_dir / "package.json").write_text(json.dumps(package_json, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(package_json, ensure_ascii=False))

if __name__ == "__main__":
    main()
