#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HTML = (ROOT / "web/static/index.html").read_text()
APP = (ROOT / "web/static/app.js").read_text()
VESC = (ROOT / "web/static/vesc_workbench.js").read_text()
REPLAY = (ROOT / "web/static/replay.js").read_text()
CPP = (ROOT / "web/web_server.cpp").read_text()
CAT = (ROOT / "gui/agv_experiment_catalog.hpp").read_text()

checks = {
    "data health": "overviewDataHealthSection" in HTML and "CHANNEL_REGISTRY" in APP,
    "sse sequence": "__event_seq" in CPP and "sseGapCount" in APP,
    "staged config": all(token in APP for token in ("configPending", "validation_id", "configPayload")) and all(token in HTML for token in ("configDiffDrawer", "Apply Validated + Verify")) and all(token in CPP for token in ("/api/config/validate", "/api/config/apply", "TRANSACTION_APPLIED")),
    "replay lock": "REPLAY MODE" in REPLAY and "replay-mode" in REPLAY,
    "test preflight": "testPreflightPanel" in HTML and "testPreflightStatus" in APP,
    "mission dock": "missionDraftDock" in HTML and "renderMissionDraft" in APP,
    "map covariance": "layerCovariance" in HTML and "drawEkfCovariance" in APP,
    "esc compare": "vescCompareSection" in HTML and "v5RenderCompare" in VESC,
}
checks.update({
    "esc fault history": "vescFaultSection" in HTML and "v5RenderFaultHistory" in VESC,
    "perception evidence": "perceptionEvidenceSection" in HTML and "savePerceptionEvidence" in CPP,
    "session manifest": "writeSessionManifest" in CPP and "git_commit" in CPP,
    "candidate compare": "trialComparePanel" in HTML and "renderTrialComparison" in APP,
    "legacy nav deadcode removed": "legacy, unreachable for navigation" not in CAT,
})

failed = [name for name, ok in checks.items() if not ok]
if failed:
    raise SystemExit("FAIL web_engineering_v2_self_check: " + ", ".join(failed))
print(f"PASS web_engineering_v2_self_check ({len(checks)} contracts)")
