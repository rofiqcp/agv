from pathlib import Path


def read_gui_source(navigation_root: Path) -> str:
    """Return the complete modular native GUI source for static regression checks."""
    gui_dir = Path(navigation_root) / "gui"
    parts = []
    for path in sorted(gui_dir.glob("*.hpp")):
        parts.append(path.read_text(encoding="utf-8"))
    parts.append((gui_dir / "agv_gui.cpp").read_text(encoding="utf-8"))
    module_dir = gui_dir / "modules"
    if module_dir.is_dir():
        for path in sorted(module_dir.glob("*.cpp")):
            parts.append(path.read_text(encoding="utf-8"))
    return "\n".join(parts)
