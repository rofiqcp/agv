#!/usr/bin/env python3
"""Prepare official COCO-2017 person/car/motorcycle subset without pycocotools."""
import argparse
import json
import os
import shutil
import urllib.request
import zipfile
from collections import defaultdict
from pathlib import Path

ANNOTATIONS_URL = "http://images.cocodataset.org/annotations/annotations_trainval2017.zip"
COCO_IDS = {1: (0, "person"), 3: (1, "car"), 4: (2, "motorcycle")}
RUNTIME_MAP = {0: 0, 1: 2, 2: 3}


def download(url: str, path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.stat().st_size > 0:
        return
    tmp = path.with_suffix(path.suffix + ".part")
    with urllib.request.urlopen(url, timeout=60) as src, open(tmp, "wb") as dst:
        shutil.copyfileobj(src, dst, length=1024 * 1024)
    tmp.replace(path)

def ensure_annotations(root: Path) -> Path:
    archive = root / "downloads" / "annotations_trainval2017.zip"
    download(ANNOTATIONS_URL, archive)
    ann_dir = root / "annotations"
    target = ann_dir / "instances_train2017.json"
    if not target.exists():
        with zipfile.ZipFile(archive) as zf:
            for name in zf.namelist():
                if name.endswith("instances_train2017.json") or name.endswith("instances_val2017.json"):
                    zf.extract(name, root)
        extracted = root / "annotations"
        if extracted != ann_dir:
            ann_dir.mkdir(parents=True, exist_ok=True)
    return ann_dir


def load_subset(annotation_file: Path):
    data = json.loads(annotation_file.read_text())
    images = {int(x["id"]): x for x in data["images"]}
    annotations = defaultdict(list)
    for ann in data["annotations"]:
        cid = int(ann["category_id"])
        if cid in COCO_IDS and not ann.get("iscrowd", 0):
            annotations[int(ann["image_id"])].append(ann)
    return images, annotations

def yolo_line(ann, image):
    train_id, _ = COCO_IDS[int(ann["category_id"])]
    x, y, w, h = [float(v) for v in ann["bbox"]]
    iw, ih = float(image["width"]), float(image["height"])
    cx = (x + w * 0.5) / iw
    cy = (y + h * 0.5) / ih
    return f"{train_id} {cx:.8f} {cy:.8f} {w/iw:.8f} {h/ih:.8f}"


def prepare_split(root: Path, split: str, download_images: bool, max_images: int):
    ann_file = root / "annotations" / f"instances_{split}.json"
    images, annotations = load_subset(ann_file)
    ids = sorted(annotations)
    if max_images > 0:
        ids = ids[:max_images]
    image_dir = root / "images" / split
    label_dir = root / "labels" / split
    image_dir.mkdir(parents=True, exist_ok=True)
    label_dir.mkdir(parents=True, exist_ok=True)
    manifest = []
    counts = defaultdict(int)
    for image_id in ids:
        image = images[image_id]
        anns = annotations[image_id]
        for ann in anns:
            counts[COCO_IDS[int(ann["category_id"])][1]] += 1
        label_path = label_dir / (Path(image["file_name"]).stem + ".txt")
        label_path.write_text("\n".join(yolo_line(a, image) for a in anns) + "\n")
        image_path = image_dir / image["file_name"]
        if download_images:
            url = image.get("coco_url") or image.get("flickr_url")
            if not url:
                raise RuntimeError(f"COCO URL missing for image {image_id}")
            download(url, image_path)
        manifest.append({
            "image_id": image_id,
            "file_name": image["file_name"],
            "image_path": str(image_path),
            "label_path": str(label_path),
            "downloaded": image_path.exists(),
            "classes": sorted({COCO_IDS[int(a["category_id"])][1] for a in anns}),
        })
    manifest_path = root / f"manifest_{split}.jsonl"
    manifest_path.write_text("\n".join(json.dumps(x, separators=(",", ":")) for x in manifest) + "\n")
    return {"images": len(ids), "instances": dict(counts), "manifest": str(manifest_path)}


def write_config(root: Path):
    config = {
        "train_classes": {"0": "person", "1": "car", "2": "motorcycle"},
        "runtime_internal_mapping": {"0": 0, "1": 2, "2": 3},
        "coco_category_mapping": {"1": 0, "3": 2, "4": 3},
        "safety_note": "Semantic class is not the sole obstacle safety gate.",
    }
    (root / "class_mapping.json").write_text(json.dumps(config, indent=2) + "\n")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default=str(Path(os.environ.get("AGV_ROOT", str(Path.home() / "agv"))) / "data/datasets/coco_obstacles"))
    parser.add_argument("--split", choices=["train2017", "val2017", "both"], default="both")
    parser.add_argument("--download-images", action="store_true")
    parser.add_argument("--max-images", type=int, default=0, help="0 means all matching images")
    args = parser.parse_args()
    root = Path(args.root).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    ensure_annotations(root)
    write_config(root)
    splits = ["train2017", "val2017"] if args.split == "both" else [args.split]
    summary = {}
    for split in splits:
        summary[split] = prepare_split(root, split, args.download_images, args.max_images)
    print(json.dumps({"root": str(root), **summary}, indent=2))
    if not args.download_images:
        print("Manifest/labels ready. Re-run with --download-images to fetch the selected COCO images.")


if __name__ == "__main__":
    main()
