#!/usr/bin/env python3
"""
osm_map.py

SATU PROGRAM untuk:
1. Mengunduh centerline jalan OpenStreetMap.
2. Memotong jalan berdasarkan longitude, latitude, panjang, dan lebar area.
3. Memperlebar centerline berdasarkan lebar total jalan.
4. Membuat PGM dan YAML Nav2.
5. Menjaga scaling FINAL tepat 10 cm per piksel.
6. Membuat datum map terhadap GNSS WGS84/UTM.
7. Memverifikasi ukuran PGM dan scaling setelah file selesai dibuat.

Program langsung dijalankan tanpa argumen:

    /bin/python3 osm_map.py
"""

from __future__ import annotations

# =============================================================================
# KONFIGURASI UTAMA
# =============================================================================

# Titik tengah area, datum WGS84 / EPSG:4326.
CENTER_LONGITUDE = 110.4363513
CENTER_LATITUDE = -7.0500161

# Ukuran area:
# MAP_LENGTH_M = arah barat ↔ timur / sumbu X.
# MAP_WIDTH_M  = arah selatan ↔ utara / sumbu Y.
MAP_LENGTH_M = 620.0
MAP_WIDTH_M = 400.0

# DIKUNCI TEPAT 10 CM PER PIKSEL.
# Jangan diubah menjadi 0.05 jika target Anda 10 cm/piksel.
RESOLUTION_M_PER_PIXEL = 0.1

# Lebar TOTAL koridor jalan.
# 8.0 berarti 4 meter kiri + 4 meter kanan dari centerline OSM.
ROAD_WIDTH_M = 8.0

# Supersampling hanya untuk menghitung batas polygon lebih teliti.
# Resolusi FILE PGM tetap 0.10 m/piksel.
# Faktor 2 berarti polygon diperiksa pada grid internal 5 cm,
# kemudian dikembalikan menjadi piksel final 10 cm.
SUPERSAMPLE_FACTOR = 2

# Piksel final dianggap jalan bila minimal 50% areanya tertutup polygon jalan.
FREE_COVERAGE_THRESHOLD = 0.50

MAP_NAME = "undip_nav2"
OUTPUT_DIRECTORY = "undip_map_output"

# None = otomatis. Semarang akan menjadi EPSG:32749 / UTM zone 49S.
MANUAL_UTM_EPSG = None

CREATE_LOCAL_YAML = True
CREATE_UTM_YAML = True
CREATE_GNSS_DATUM_FILES = True

# Nav2: putih = bebas, hitam = occupied.
FREE_PIXEL_VALUE = 254
OCCUPIED_PIXEL_VALUE = 0

# Jumlah baris final yang diproses per tile.
# Cara ini menjaga penggunaan RAM tetap rendah.
TILE_ROWS = 16

MAX_PIXEL_COUNT = 3_000_000_000
DOWNLOAD_TIMEOUT_SECONDS = 180

# Parameter dasar robot_localization/navsat_transform_node.
NAVSAT_FREQUENCY_HZ = 30.0
NAVSAT_DELAY_SECONDS = 3.0
NAVSAT_ZERO_ALTITUDE = True
MAGNETIC_DECLINATION_RADIANS = 0.0
IMU_YAW_OFFSET_RADIANS = 0.0
USE_ODOMETRY_YAW = False
BROADCAST_UTM_TRANSFORM = True
UTM_AS_PARENT_FRAME = True
PUBLISH_FILTERED_GPS = True

DRIVABLE_HIGHWAYS = {
    "motorway", "motorway_link",
    "trunk", "trunk_link",
    "primary", "primary_link",
    "secondary", "secondary_link",
    "tertiary", "tertiary_link",
    "unclassified",
    "residential",
    "living_street",
    "service",
    "road",
}

# =============================================================================
# AKHIR KONFIGURASI
# =============================================================================

import json
import math
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from decimal import Decimal
from pathlib import Path
from typing import Any, Iterable

try:
    import numpy as np
    import requests
    from PIL import Image
    from pyproj import CRS, Transformer
    from rasterio.features import rasterize
    from rasterio.transform import from_origin
    from shapely.geometry import (
        GeometryCollection,
        LineString,
        MultiLineString,
        box,
        mapping,
    )
    from shapely.ops import transform as shapely_transform
    from shapely.ops import unary_union
except ImportError as exc:
    raise SystemExit(
        "\nLibrary Python belum lengkap.\n\n"
        "Jalankan:\n"
        "  python3 -m pip install numpy Pillow pyproj rasterio requests shapely\n\n"
        f"Library gagal diimpor: {exc}\n"
    ) from exc


OSM_API_URL = "https://api.openstreetmap.org/api/0.6/map"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def validate_configuration() -> None:
    if not -180.0 <= CENTER_LONGITUDE <= 180.0:
        raise ValueError("CENTER_LONGITUDE harus -180 sampai 180.")

    if not -90.0 <= CENTER_LATITUDE <= 90.0:
        raise ValueError("CENTER_LATITUDE harus -90 sampai 90.")

    for name, value in {
        "MAP_LENGTH_M": MAP_LENGTH_M,
        "MAP_WIDTH_M": MAP_WIDTH_M,
        "RESOLUTION_M_PER_PIXEL": RESOLUTION_M_PER_PIXEL,
        "ROAD_WIDTH_M": ROAD_WIDTH_M,
        "TILE_ROWS": TILE_ROWS,
    }.items():
        if value <= 0:
            raise ValueError(f"{name} harus lebih besar dari nol.")

    if Decimal(str(RESOLUTION_M_PER_PIXEL)) != Decimal("0.1"):
        raise ValueError(
            "Program ini dikonfigurasi khusus 10 cm/piksel. "
            "RESOLUTION_M_PER_PIXEL harus tepat 0.10."
        )

    if SUPERSAMPLE_FACTOR not in {1, 2, 4, 5, 8}:
        raise ValueError("SUPERSAMPLE_FACTOR gunakan 1, 2, 4, 5, atau 8.")

    if not 0.0 < FREE_COVERAGE_THRESHOLD <= 1.0:
        raise ValueError("FREE_COVERAGE_THRESHOLD harus > 0 dan <= 1.")

    if FREE_PIXEL_VALUE == OCCUPIED_PIXEL_VALUE:
        raise ValueError("Nilai free dan occupied tidak boleh sama.")

    if not 0 <= FREE_PIXEL_VALUE <= 255:
        raise ValueError("FREE_PIXEL_VALUE harus 0 sampai 255.")

    if not 0 <= OCCUPIED_PIXEL_VALUE <= 255:
        raise ValueError("OCCUPIED_PIXEL_VALUE harus 0 sampai 255.")


def exact_pixel_count(length_m: float, resolution_m: float, name: str) -> int:
    """
    Menghitung jumlah piksel tanpa round() biner.

    Ukuran meter WAJIB habis dibagi resolusi. Hal ini memastikan:
        ukuran_meter = jumlah_piksel × 0.10
    secara tepat, tanpa pemotongan atau penambahan area diam-diam.
    """
    length_dec = Decimal(str(length_m))
    resolution_dec = Decimal(str(resolution_m))
    ratio = length_dec / resolution_dec

    if ratio != ratio.to_integral_value():
        nearest_pixels = int(ratio.to_integral_value())
        nearest_size = Decimal(nearest_pixels) * resolution_dec
        raise ValueError(
            f"{name}={length_m} m tidak habis dibagi "
            f"{resolution_m} m/piksel. "
            f"Gunakan ukuran kelipatan 0.10 m. "
            f"Ukuran terdekat: {nearest_size} m."
        )

    return int(ratio)


def auto_utm_epsg(longitude: float, latitude: float) -> int:
    zone = int(math.floor((longitude + 180.0) / 6.0)) + 1
    zone = max(1, min(zone, 60))
    return (32600 if latitude >= 0.0 else 32700) + zone


def resolve_output_directory() -> Path:
    configured = Path(OUTPUT_DIRECTORY).expanduser()
    if configured.is_absolute():
        return configured.resolve()
    return (Path(__file__).resolve().parent / configured).resolve()


def calculate_exact_grid(utm_epsg: int) -> dict[str, Any]:
    width_px = exact_pixel_count(
        MAP_LENGTH_M,
        RESOLUTION_M_PER_PIXEL,
        "MAP_LENGTH_M",
    )
    height_px = exact_pixel_count(
        MAP_WIDTH_M,
        RESOLUTION_M_PER_PIXEL,
        "MAP_WIDTH_M",
    )

    pixel_count = width_px * height_px
    if pixel_count > MAX_PIXEL_COUNT:
        raise RuntimeError(
            f"Jumlah piksel {pixel_count:,} melebihi "
            f"MAX_PIXEL_COUNT={MAX_PIXEL_COUNT:,}."
        )

    # Hitung kembali ukuran dari jumlah piksel agar grid dan YAML selalu sama.
    actual_length_m = float(
        Decimal(width_px) * Decimal(str(RESOLUTION_M_PER_PIXEL))
    )
    actual_width_m = float(
        Decimal(height_px) * Decimal(str(RESOLUTION_M_PER_PIXEL))
    )

    to_utm = Transformer.from_crs(4326, utm_epsg, always_xy=True)
    to_wgs84 = Transformer.from_crs(utm_epsg, 4326, always_xy=True)

    center_x, center_y = to_utm.transform(
        CENTER_LONGITUDE,
        CENTER_LATITUDE,
    )

    xmin = center_x - actual_length_m / 2.0
    xmax = xmin + actual_length_m
    ymin = center_y - actual_width_m / 2.0
    ymax = ymin + actual_width_m

    points_utm = {
        "southwest": (xmin, ymin),
        "northwest": (xmin, ymax),
        "northeast": (xmax, ymax),
        "southeast": (xmax, ymin),
        "center": (center_x, center_y),
    }

    points_wgs84 = {
        key: to_wgs84.transform(x, y)
        for key, (x, y) in points_utm.items()
    }

    return {
        "utm_epsg": utm_epsg,
        "width_px": width_px,
        "height_px": height_px,
        "pixel_count": pixel_count,
        "resolution_m_per_pixel": RESOLUTION_M_PER_PIXEL,
        "actual_length_m": actual_length_m,
        "actual_width_m": actual_width_m,
        "center_utm": {
            "easting": center_x,
            "northing": center_y,
        },
        "bbox_utm": {
            "xmin": xmin,
            "ymin": ymin,
            "xmax": xmax,
            "ymax": ymax,
        },
        "bbox_wgs84": {
            "west": min(v[0] for v in points_wgs84.values()),
            "south": min(v[1] for v in points_wgs84.values()),
            "east": max(v[0] for v in points_wgs84.values()),
            "north": max(v[1] for v in points_wgs84.values()),
        },
        "control_points_utm": {
            key: {"easting": value[0], "northing": value[1]}
            for key, value in points_utm.items()
        },
        "control_points_wgs84": {
            key: {"longitude": value[0], "latitude": value[1]}
            for key, value in points_wgs84.items()
        },
    }


def download_osm(grid: dict[str, Any]) -> bytes:
    bbox = grid["bbox_wgs84"]
    bbox_text = (
        f'{bbox["west"]:.10f},'
        f'{bbox["south"]:.10f},'
        f'{bbox["east"]:.10f},'
        f'{bbox["north"]:.10f}'
    )

    print("Mengunduh vector OSM...")
    print(f"BBox OSM: {bbox_text}")

    response = requests.get(
        OSM_API_URL,
        params={"bbox": bbox_text},
        headers={
            "User-Agent": "Sirobo-OSM-Nav2-10cm/3.0",
            "Accept": "application/xml",
        },
        timeout=DOWNLOAD_TIMEOUT_SECONDS,
    )
    response.raise_for_status()

    if b"<osm" not in response.content[:1000]:
        raise RuntimeError("Server tidak mengembalikan OSM XML yang valid.")

    return response.content


def parse_osm_roads(osm_xml: bytes) -> list[dict[str, Any]]:
    root = ET.fromstring(osm_xml)
    nodes: dict[str, tuple[float, float]] = {}

    for node in root.findall("node"):
        node_id = node.attrib.get("id")
        longitude = node.attrib.get("lon")
        latitude = node.attrib.get("lat")
        if node_id and longitude and latitude:
            nodes[node_id] = (float(longitude), float(latitude))

    roads: list[dict[str, Any]] = []

    for way in root.findall("way"):
        tags = {
            tag.attrib["k"]: tag.attrib.get("v", "")
            for tag in way.findall("tag")
            if "k" in tag.attrib
        }

        highway = tags.get("highway", "").lower()
        if highway not in DRIVABLE_HIGHWAYS:
            continue

        coordinates = []
        for nd in way.findall("nd"):
            reference = nd.attrib.get("ref")
            if reference in nodes:
                coordinates.append(nodes[reference])

        if len(coordinates) < 2:
            continue

        properties = dict(tags)
        properties["osm_id"] = way.attrib.get("id")
        properties["source"] = "OpenStreetMap API 0.6"

        roads.append({
            "geometry": LineString(coordinates),
            "properties": properties,
        })

    return roads


def line_parts(geometry: Any) -> Iterable[LineString]:
    if geometry.is_empty:
        return

    if isinstance(geometry, LineString):
        yield geometry
    elif isinstance(geometry, MultiLineString):
        yield from geometry.geoms
    elif isinstance(geometry, GeometryCollection):
        for item in geometry.geoms:
            yield from line_parts(item)


def clip_roads_to_grid(
    roads_wgs84: list[dict[str, Any]],
    grid: dict[str, Any],
) -> tuple[list[dict[str, Any]], list[LineString]]:
    utm_epsg = grid["utm_epsg"]
    bbox = grid["bbox_utm"]

    to_utm = Transformer.from_crs(4326, utm_epsg, always_xy=True)
    to_wgs84 = Transformer.from_crs(utm_epsg, 4326, always_xy=True)

    exact_bbox = box(
        bbox["xmin"],
        bbox["ymin"],
        bbox["xmax"],
        bbox["ymax"],
    )

    vector_features: list[dict[str, Any]] = []
    metric_lines: list[LineString] = []

    for road in roads_wgs84:
        metric_geometry = shapely_transform(
            to_utm.transform,
            road["geometry"],
        )
        clipped = metric_geometry.intersection(exact_bbox)

        for part_index, metric_part in enumerate(line_parts(clipped)):
            if metric_part.is_empty or metric_part.length <= 0:
                continue

            metric_lines.append(metric_part)
            wgs84_part = shapely_transform(
                to_wgs84.transform,
                metric_part,
            )

            properties = dict(road["properties"])
            properties["part_index"] = part_index
            properties["length_m"] = round(metric_part.length, 4)

            vector_features.append({
                "type": "Feature",
                "properties": properties,
                "geometry": mapping(wgs84_part),
            })

    return vector_features, metric_lines


def build_road_corridor(
    metric_lines: list[LineString],
    grid: dict[str, Any],
):
    bbox = grid["bbox_utm"]
    exact_bbox = box(
        bbox["xmin"],
        bbox["ymin"],
        bbox["xmax"],
        bbox["ymax"],
    )

    buffer_each_side = ROAD_WIDTH_M / 2.0
    polygons = [
        line.buffer(
            buffer_each_side,
            cap_style=1,
            join_style=1,
        )
        for line in metric_lines
    ]

    corridor = unary_union(polygons).intersection(exact_bbox)
    if corridor.is_empty:
        raise RuntimeError("Buffer jalan menghasilkan polygon kosong.")

    return corridor


def write_geojson(path: Path, features: list[dict[str, Any]]) -> None:
    path.write_text(
        json.dumps(
            {
                "type": "FeatureCollection",
                "features": features,
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )


def create_area_feature(grid: dict[str, Any]) -> dict[str, Any]:
    points = grid["control_points_wgs84"]
    ring = [
        [points["southwest"]["longitude"], points["southwest"]["latitude"]],
        [points["northwest"]["longitude"], points["northwest"]["latitude"]],
        [points["northeast"]["longitude"], points["northeast"]["latitude"]],
        [points["southeast"]["longitude"], points["southeast"]["latitude"]],
        [points["southwest"]["longitude"], points["southwest"]["latitude"]],
    ]

    return {
        "type": "Feature",
        "properties": {
            "name": MAP_NAME,
            "length_m": grid["actual_length_m"],
            "width_m": grid["actual_width_m"],
            "resolution_m_per_pixel": grid["resolution_m_per_pixel"],
            "width_px": grid["width_px"],
            "height_px": grid["height_px"],
            "utm_epsg": grid["utm_epsg"],
        },
        "geometry": {
            "type": "Polygon",
            "coordinates": [ring],
        },
    }


def rasterize_final_tile(
    corridor: Any,
    grid: dict[str, Any],
    start_row: int,
    tile_height: int,
) -> np.ndarray:
    """
    Membuat tile final 10 cm/piksel.

    Raster internal:
        0.10 / SUPERSAMPLE_FACTOR
    tetapi output akhir selalu 0.10 m/piksel.
    """
    factor = SUPERSAMPLE_FACTOR
    final_resolution = grid["resolution_m_per_pixel"]
    internal_resolution = final_resolution / factor
    width_px = grid["width_px"]
    bbox = grid["bbox_utm"]

    high_width = width_px * factor
    high_height = tile_height * factor
    tile_top_y = bbox["ymax"] - start_row * final_resolution

    transform = from_origin(
        bbox["xmin"],
        tile_top_y,
        internal_resolution,
        internal_resolution,
    )

    high_mask = rasterize(
        [(corridor, 1)],
        out_shape=(high_height, high_width),
        transform=transform,
        fill=0,
        all_touched=False,
        dtype=np.uint8,
    )

    if factor == 1:
        coverage = high_mask
        sample_count = 1
    else:
        coverage = high_mask.reshape(
            tile_height,
            factor,
            width_px,
            factor,
        ).sum(axis=(1, 3), dtype=np.uint16)
        sample_count = factor * factor

    required_samples = int(
        math.ceil(FREE_COVERAGE_THRESHOLD * sample_count)
    )

    return np.where(
        coverage >= required_samples,
        FREE_PIXEL_VALUE,
        OCCUPIED_PIXEL_VALUE,
    ).astype(np.uint8)


def write_pgm_streaming(
    output_path: Path,
    corridor: Any,
    grid: dict[str, Any],
) -> None:
    width_px = grid["width_px"]
    height_px = grid["height_px"]

    with output_path.open("wb") as handle:
        handle.write(b"P5\n")
        handle.write(
            b"# Exact 0.1 meter per pixel OSM map\n"
        )
        handle.write(
            f"{width_px} {height_px}\n255\n".encode("ascii")
        )

        for start_row in range(0, height_px, TILE_ROWS):
            tile_height = min(TILE_ROWS, height_px - start_row)
            tile = rasterize_final_tile(
                corridor,
                grid,
                start_row,
                tile_height,
            )
            handle.write(tile.tobytes(order="C"))

            completed = start_row + tile_height
            percent = completed * 100.0 / height_px
            print(
                f"\rMenulis PGM: {percent:6.2f}% "
                f"({completed:,}/{height_px:,} baris)",
                end="",
                flush=True,
            )

    print()


def write_nav2_yaml(
    path: Path,
    image_filename: str,
    origin_x: float,
    origin_y: float,
) -> None:
    path.write_text(
        (
            f'image: "{image_filename}"\n'
            "mode: trinary\n"
            "resolution: 0.10000000000\n"
            f"origin: [{origin_x:.9f}, {origin_y:.9f}, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n"
        ),
        encoding="utf-8",
    )


def write_world_file(path: Path, grid: dict[str, Any]) -> None:
    """
    World file memakai pusat piksel kiri atas.
    """
    resolution = grid["resolution_m_per_pixel"]
    bbox = grid["bbox_utm"]

    upper_left_center_x = bbox["xmin"] + resolution / 2.0
    upper_left_center_y = bbox["ymax"] - resolution / 2.0

    path.write_text(
        (
            f"{resolution:.12f}\n"
            "0.000000000000\n"
            "0.000000000000\n"
            f"{-resolution:.12f}\n"
            f"{upper_left_center_x:.9f}\n"
            f"{upper_left_center_y:.9f}\n"
        ),
        encoding="utf-8",
    )


def write_projection_file(path: Path, utm_epsg: int) -> None:
    path.write_text(
        CRS.from_epsg(utm_epsg).to_wkt(),
        encoding="utf-8",
    )


def write_gnss_datum_files(
    datum_path: Path,
    navsat_path: Path,
    grid: dict[str, Any],
) -> dict[str, Any]:
    """
    Datum map lokal diletakkan pada sudut barat daya raster:
        map(0,0) = southwest UTM/WGS84
        map +X   = timur
        map +Y   = utara
    """
    origin_wgs84 = grid["control_points_wgs84"]["southwest"]
    origin_utm = grid["control_points_utm"]["southwest"]

    datum = {
        "latitude": origin_wgs84["latitude"],
        "longitude": origin_wgs84["longitude"],
        "utm_easting": origin_utm["easting"],
        "utm_northing": origin_utm["northing"],
        "heading_radians": 0.0,
    }

    datum_path.write_text(
        (
            "map_datum:\n"
            "  geodetic_crs: EPSG:4326\n"
            f'  projected_crs: "EPSG:{grid["utm_epsg"]}"\n'
            "  convention: ENU\n"
            "  map_frame: map\n"
            "  utm_frame: utm\n"
            "  map_axes:\n"
            "    x: east\n"
            "    y: north\n"
            "    z: up\n"
            "  map_origin:\n"
            f'    latitude: {datum["latitude"]:.12f}\n'
            f'    longitude: {datum["longitude"]:.12f}\n'
            f'    utm_easting: {datum["utm_easting"]:.6f}\n'
            f'    utm_northing: {datum["utm_northing"]:.6f}\n'
            "    altitude_m: 0.0\n"
            "    heading_radians: 0.0\n"
            "  transform_map_to_utm:\n"
            f'    translation_x: {datum["utm_easting"]:.6f}\n'
            f'    translation_y: {datum["utm_northing"]:.6f}\n'
            "    translation_z: 0.0\n"
            "    yaw: 0.0\n"
            "  resolution_m_per_pixel: 0.1\n"
            f'  width_px: {grid["width_px"]}\n'
            f'  height_px: {grid["height_px"]}\n'
        ),
        encoding="utf-8",
    )

    bool_text = lambda value: "true" if value else "false"

    navsat_path.write_text(
        (
            "navsat_transform_node:\n"
            "  ros__parameters:\n"
            f"    frequency: {NAVSAT_FREQUENCY_HZ:.6f}\n"
            f"    delay: {NAVSAT_DELAY_SECONDS:.6f}\n"
            f"    magnetic_declination_radians: "
            f"{MAGNETIC_DECLINATION_RADIANS:.12f}\n"
            f"    yaw_offset: {IMU_YAW_OFFSET_RADIANS:.12f}\n"
            f"    zero_altitude: {bool_text(NAVSAT_ZERO_ALTITUDE)}\n"
            f"    broadcast_utm_transform: "
            f"{bool_text(BROADCAST_UTM_TRANSFORM)}\n"
            f"    broadcast_utm_transform_as_parent_frame: "
            f"{bool_text(UTM_AS_PARENT_FRAME)}\n"
            f"    publish_filtered_gps: "
            f"{bool_text(PUBLISH_FILTERED_GPS)}\n"
            f"    use_odometry_yaw: {bool_text(USE_ODOMETRY_YAW)}\n"
            "    wait_for_datum: true\n"
            "    datum: "
            f'[{datum["latitude"]:.12f}, '
            f'{datum["longitude"]:.12f}, 0.0]\n'
        ),
        encoding="utf-8",
    )

    return datum


def create_preview(
    path: Path,
    corridor: Any,
    grid: dict[str, Any],
) -> None:
    max_dimension = 1600
    scale = max(
        grid["width_px"] / max_dimension,
        grid["height_px"] / max_dimension,
        1.0,
    )

    preview_width = max(1, int(math.ceil(grid["width_px"] / scale)))
    preview_height = max(1, int(math.ceil(grid["height_px"] / scale)))

    preview_resolution_x = grid["actual_length_m"] / preview_width
    preview_resolution_y = grid["actual_width_m"] / preview_height
    preview_resolution = max(
        preview_resolution_x,
        preview_resolution_y,
    )

    bbox = grid["bbox_utm"]
    transform = from_origin(
        bbox["xmin"],
        bbox["ymax"],
        preview_resolution,
        preview_resolution,
    )

    preview = rasterize(
        [(corridor, FREE_PIXEL_VALUE)],
        out_shape=(preview_height, preview_width),
        transform=transform,
        fill=OCCUPIED_PIXEL_VALUE,
        all_touched=False,
        dtype=np.uint8,
    )

    Image.fromarray(preview, mode="L").save(path)


def read_pgm_header(path: Path) -> tuple[int, int, int, int]:
    with path.open("rb") as handle:
        if handle.readline().strip() != b"P5":
            raise RuntimeError("PGM bukan format P5.")

        tokens: list[bytes] = []
        while len(tokens) < 3:
            line = handle.readline()
            if not line:
                raise RuntimeError("Header PGM tidak lengkap.")
            stripped = line.strip()
            if not stripped or stripped.startswith(b"#"):
                continue
            tokens.extend(stripped.split())

        width = int(tokens[0])
        height = int(tokens[1])
        maxval = int(tokens[2])
        payload_offset = handle.tell()

    return width, height, maxval, payload_offset


def verify_scaling(
    pgm_path: Path,
    grid: dict[str, Any],
) -> dict[str, Any]:
    width, height, maxval, payload_offset = read_pgm_header(pgm_path)
    payload_bytes = pgm_path.stat().st_size - payload_offset
    expected_payload = grid["width_px"] * grid["height_px"]

    checks = {
        "resolution_exactly_10_cm": (
            Decimal(str(grid["resolution_m_per_pixel"]))
            == Decimal("0.1")
        ),
        "width_pixel_exact": width == grid["width_px"],
        "height_pixel_exact": height == grid["height_px"],
        "width_scale_exact": (
            Decimal(width) * Decimal("0.1")
            == Decimal(str(MAP_LENGTH_M))
        ),
        "height_scale_exact": (
            Decimal(height) * Decimal("0.1")
            == Decimal(str(MAP_WIDTH_M))
        ),
        "pgm_maxval_255": maxval == 255,
        "pgm_payload_exact": payload_bytes == expected_payload,
    }

    return {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "width_px": width,
        "height_px": height,
        "resolution_m_per_pixel": 0.1,
        "calculated_length_m": float(
            Decimal(width) * Decimal("0.1")
        ),
        "calculated_width_m": float(
            Decimal(height) * Decimal("0.1")
        ),
        "payload_bytes": payload_bytes,
        "expected_payload_bytes": expected_payload,
    }


def main() -> None:
    validate_configuration()

    output_directory = resolve_output_directory()
    output_directory.mkdir(parents=True, exist_ok=True)

    utm_epsg = (
        MANUAL_UTM_EPSG
        if MANUAL_UTM_EPSG is not None
        else auto_utm_epsg(
            CENTER_LONGITUDE,
            CENTER_LATITUDE,
        )
    )

    grid = calculate_exact_grid(utm_epsg)

    print("=" * 76)
    print("OSM → PGM/YAML NAV2 — EXACT 1 CM PER PIXEL")
    print("=" * 76)
    print(
        f"Pusat WGS84       : "
        f"{CENTER_LATITUDE:.10f}, {CENTER_LONGITUDE:.10f}"
    )
    print(
        f"Ukuran area       : "
        f"{grid['actual_length_m']:.2f} x "
        f"{grid['actual_width_m']:.2f} meter"
    )
    print(
        f"Resolusi final    : "
        f"{grid['resolution_m_per_pixel']:.2f} meter/piksel"
    )
    print(
        f"Ukuran raster     : "
        f"{grid['width_px']:,} x {grid['height_px']:,} piksel"
    )
    print(
        f"Validasi scaling  : "
        f"{grid['width_px']} × 0.10 = {grid['actual_length_m']} m, "
        f"{grid['height_px']} × 0.10 = {grid['actual_width_m']} m"
    )
    print(
        f"Lebar jalan       : "
        f"{ROAD_WIDTH_M:.2f} m total "
        f"({ROAD_WIDTH_M / 2.0:.2f} m per sisi)"
    )
    print(
        f"Supersampling     : "
        f"{SUPERSAMPLE_FACTOR}x "
        f"(internal {RESOLUTION_M_PER_PIXEL / SUPERSAMPLE_FACTOR:.3f} m)"
    )
    print(f"CRS UTM           : EPSG:{utm_epsg}")
    print(f"Folder hasil      : {output_directory}")
    print("=" * 76)

    osm_xml = download_osm(grid)

    raw_osm_path = output_directory / f"{MAP_NAME}_raw.osm"
    raw_osm_path.write_bytes(osm_xml)

    print("Membaca centerline jalan...")
    roads = parse_osm_roads(osm_xml)
    if not roads:
        raise RuntimeError("Tidak ditemukan jalan kendaraan dalam data OSM.")

    vector_features, metric_lines = clip_roads_to_grid(
        roads,
        grid,
    )
    if not metric_lines:
        raise RuntimeError("Tidak ada centerline jalan di area grid.")

    print(f"Way jalan OSM     : {len(roads):,}")
    print(f"Segmen hasil clip : {len(metric_lines):,}")

    roads_geojson_path = output_directory / f"{MAP_NAME}_roads.geojson"
    write_geojson(roads_geojson_path, vector_features)

    area_geojson_path = output_directory / f"{MAP_NAME}_area.geojson"
    write_geojson(
        area_geojson_path,
        [create_area_feature(grid)],
    )

    print("Membuat polygon koridor jalan...")
    corridor = build_road_corridor(metric_lines, grid)

    to_wgs84 = Transformer.from_crs(utm_epsg, 4326, always_xy=True)
    corridor_wgs84 = shapely_transform(
        to_wgs84.transform,
        corridor,
    )

    corridor_geojson_path = (
        output_directory / f"{MAP_NAME}_road_corridor.geojson"
    )
    write_geojson(
        corridor_geojson_path,
        [{
            "type": "Feature",
            "properties": {
                "road_width_total_m": ROAD_WIDTH_M,
                "buffer_each_side_m": ROAD_WIDTH_M / 2.0,
                "final_resolution_m_per_pixel": 0.10,
                "supersample_factor": SUPERSAMPLE_FACTOR,
                "utm_epsg": utm_epsg,
            },
            "geometry": mapping(corridor_wgs84),
        }],
    )

    pgm_path = output_directory / f"{MAP_NAME}.pgm"
    print("Merasterisasi occupancy map...")
    write_pgm_streaming(pgm_path, corridor, grid)

    bbox = grid["bbox_utm"]
    yaml_files: list[Path] = []

    if CREATE_LOCAL_YAML:
        local_yaml_path = output_directory / f"{MAP_NAME}.yaml"
        write_nav2_yaml(
            local_yaml_path,
            pgm_path.name,
            0.0,
            0.0,
        )
        yaml_files.append(local_yaml_path)

    if CREATE_UTM_YAML:
        utm_yaml_path = output_directory / f"{MAP_NAME}_utm.yaml"
        write_nav2_yaml(
            utm_yaml_path,
            pgm_path.name,
            bbox["xmin"],
            bbox["ymin"],
        )
        yaml_files.append(utm_yaml_path)

    pgw_path = output_directory / f"{MAP_NAME}.pgw"
    prj_path = output_directory / f"{MAP_NAME}.prj"
    write_world_file(pgw_path, grid)
    write_projection_file(prj_path, utm_epsg)

    datum = None
    datum_path = output_directory / f"{MAP_NAME}_map_datum.yaml"
    navsat_path = output_directory / f"{MAP_NAME}_navsat_transform.yaml"

    if CREATE_GNSS_DATUM_FILES:
        datum = write_gnss_datum_files(
            datum_path,
            navsat_path,
            grid,
        )

    preview_path = output_directory / f"{MAP_NAME}_preview.png"
    create_preview(preview_path, corridor, grid)

    verification = verify_scaling(pgm_path, grid)
    verification_path = (
        output_directory / f"{MAP_NAME}_scale_verification.json"
    )
    verification_path.write_text(
        json.dumps(verification, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    metadata = {
        "schema": "sirobo_osm_nav2_exact_10cm_v3",
        "created_utc": utc_now(),
        "configuration": {
            "center_longitude": CENTER_LONGITUDE,
            "center_latitude": CENTER_LATITUDE,
            "map_length_m": MAP_LENGTH_M,
            "map_width_m": MAP_WIDTH_M,
            "resolution_m_per_pixel": 0.1,
            "road_width_total_m": ROAD_WIDTH_M,
            "road_buffer_each_side_m": ROAD_WIDTH_M / 2.0,
            "supersample_factor": SUPERSAMPLE_FACTOR,
            "internal_sampling_resolution_m": (
                RESOLUTION_M_PER_PIXEL / SUPERSAMPLE_FACTOR
            ),
            "utm_epsg": utm_epsg,
        },
        "grid": grid,
        "map_to_utm": {
            "utm_easting": f"map_x + {bbox['xmin']:.9f}",
            "utm_northing": f"map_y + {bbox['ymin']:.9f}",
        },
        "gnss_datum": datum,
        "verification": verification,
        "outputs": {
            "raw_osm": raw_osm_path.name,
            "roads_geojson": roads_geojson_path.name,
            "area_geojson": area_geojson_path.name,
            "road_corridor_geojson": corridor_geojson_path.name,
            "pgm": pgm_path.name,
            "yaml": [path.name for path in yaml_files],
            "pgw": pgw_path.name,
            "prj": prj_path.name,
            "datum": datum_path.name if CREATE_GNSS_DATUM_FILES else None,
            "navsat_transform": (
                navsat_path.name if CREATE_GNSS_DATUM_FILES else None
            ),
            "preview": preview_path.name,
            "scale_verification": verification_path.name,
        },
        "warning": (
            "OpenStreetMap bukan hasil survei presisi. "
            "Tetap gunakan GNSS/IMU/LiDAR, obstacle layer, footprint, "
            "inflation, dan validasi lapangan."
        ),
    }

    metadata_path = output_directory / f"{MAP_NAME}_metadata.json"
    metadata_path.write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    print("\n" + "=" * 76)
    print(f"VERIFIKASI SCALING: {verification['status']}")
    print("=" * 76)
    for check_name, passed in verification["checks"].items():
        print(f"{'PASS' if passed else 'FAIL'}  {check_name}")

    print("\nFile hasil:")
    print(f"PGM Nav2          : {pgm_path}")
    for yaml_path in yaml_files:
        print(f"YAML Nav2         : {yaml_path}")
    print(f"World file        : {pgw_path}")
    print(f"Projection        : {prj_path}")

    if CREATE_GNSS_DATUM_FILES:
        print(f"Datum GNSS        : {datum_path}")
        print(f"NavSat config     : {navsat_path}")

    print(f"Preview           : {preview_path}")
    print(f"Verifikasi        : {verification_path}")
    print(f"Metadata          : {metadata_path}")

    if verification["status"] != "PASS":
        raise RuntimeError(
            "Verifikasi scaling gagal. Jangan gunakan map."
        )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit("\nProgram dihentikan oleh pengguna.")
    except requests.RequestException as exc:
        raise SystemExit(
            "\nGagal mengunduh OpenStreetMap. "
            "Periksa koneksi internet.\n"
            f"Detail: {exc}"
        ) from exc
    except Exception as exc:
        raise SystemExit(f"\nERROR: {exc}") from exc
