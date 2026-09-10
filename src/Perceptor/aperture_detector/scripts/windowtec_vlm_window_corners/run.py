"""Run VLM window four-corner detection and create a visual review report.

The runner accepts a directory, one image, or a zip archive. It sends each
image to an OpenAI-compatible vision endpoint and expects window quadrilaterals
on a fixed integer coordinate scale. The output is deliberately a review
artifact rather than a scored benchmark: raw responses, scaled/normalized/pixel
coordinates, validation errors, and self-contained SVG overlays are retained.
"""

from __future__ import annotations

import argparse
import base64
import html
import json
import mimetypes
import os
import re
import shutil
import stat
import struct
import sys
import zipfile
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from windowtec_core.io_utils import append_jsonl, extract_llamacpp_timings, read_jsonl, read_text, write_json
from windowtec_core.json_validator import parse_model_json
from windowtec_core.model_client import ModelClientError, OpenAICompatibleClient


IMAGE_EXTENSIONS = {".png", ".jpg", ".jpeg"}
CORNER_NAMES = ("top-left", "top-right", "bottom-right", "bottom-left")
COLORS = ("#0ea5e9", "#ef4444", "#22c55e", "#f59e0b", "#a855f7", "#14b8a6")


def resolve_path(value: str | Path) -> Path:
    """Resolve a path from either the repository root or the current directory."""

    path = Path(value).expanduser()
    if path.is_absolute():
        return path
    current_candidate = Path.cwd() / path
    if current_candidate.exists():
        return current_candidate
    return ROOT / path


def repo_relative(path: Path) -> str:
    try:
        return path.resolve().relative_to(ROOT.resolve()).as_posix()
    except ValueError:
        return str(path)


def natural_key(value: str) -> tuple[str, int, str]:
    """Sort image.png before image1.png, then image2.png, and so on."""

    name = Path(value).name
    match = re.fullmatch(r"(.*?)(\d+)?(\.[^.]+)?", name)
    if not match:
        return name.lower(), -1, ""
    prefix, number, suffix = match.groups()
    return prefix.lower(), int(number) if number is not None else -1, (suffix or "").lower()


def is_image(path: Path) -> bool:
    return path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS


def safe_extract_zip(archive_path: Path, destination: Path) -> list[Path]:
    """Extract supported images while rejecting traversal and symlink entries."""

    destination.mkdir(parents=True, exist_ok=True)
    root = destination.resolve()
    extracted: list[Path] = []
    with zipfile.ZipFile(archive_path) as archive:
        for info in archive.infolist():
            if info.is_dir():
                continue
            member = Path(info.filename)
            if member.is_absolute() or ".." in member.parts:
                raise ValueError(f"unsafe zip member path: {info.filename!r}")
            mode = (info.external_attr >> 16) & 0o170000
            if mode == stat.S_IFLNK:
                raise ValueError(f"symlink zip member is not allowed: {info.filename!r}")
            if Path(info.filename).suffix.lower() not in IMAGE_EXTENSIONS:
                continue
            target = (destination / member).resolve()
            try:
                target.relative_to(root)
            except ValueError as exc:
                raise ValueError(f"zip member escapes extraction directory: {info.filename!r}") from exc
            target.parent.mkdir(parents=True, exist_ok=True)
            with archive.open(info) as source, target.open("wb") as output:
                shutil.copyfileobj(source, output)
            extracted.append(target)
    return sorted(extracted, key=lambda path: natural_key(path.as_posix()))


def collect_images(input_path: Path, extraction_dir: Path) -> list[Path]:
    if input_path.is_dir():
        images = [path for path in input_path.rglob("*") if is_image(path)]
    elif input_path.is_file() and input_path.suffix.lower() == ".zip":
        images = safe_extract_zip(input_path, extraction_dir)
    elif is_image(input_path):
        images = [input_path]
    else:
        raise FileNotFoundError(f"input does not contain images: {input_path}")
    return sorted(images, key=lambda path: natural_key(path.name))


def image_identifier(image_path: Path, input_path: Path, extraction_dir: Path) -> str:
    """Use a compact, stable name in reports while retaining the real path separately."""

    bases = [input_path] if input_path.is_dir() else [extraction_dir]
    for base in bases:
        try:
            return image_path.relative_to(base).as_posix()
        except ValueError:
            continue
    return image_path.name


def read_image_size(path: Path) -> tuple[int, int]:
    """Read dimensions without requiring an image package."""

    with path.open("rb") as handle:
        header = handle.read(32)
    if header.startswith(b"\x89PNG\r\n\x1a\n") and len(header) >= 24:
        width, height = struct.unpack(">II", header[16:24])
        return int(width), int(height)
    if header.startswith(b"RIFF") and header[8:12] == b"WEBP":
        if header[12:16] == b"VP8X" and len(header) >= 30:
            width = 1 + int.from_bytes(header[24:27], "little")
            height = 1 + int.from_bytes(header[27:30], "little")
            return width, height
    if header.startswith(b"\xff\xd8"):
        return read_jpeg_size(path)
    raise ValueError(f"unsupported or invalid image format: {path}")


def read_jpeg_size(path: Path) -> tuple[int, int]:
    with path.open("rb") as handle:
        if handle.read(2) != b"\xff\xd8":
            raise ValueError(f"invalid JPEG header: {path}")
        while True:
            byte = handle.read(1)
            if not byte:
                break
            if byte != b"\xff":
                continue
            marker = handle.read(1)
            while marker == b"\xff":
                marker = handle.read(1)
            if not marker:
                break
            marker_value = marker[0]
            if marker_value in {0xD8, 0xD9} or 0xD0 <= marker_value <= 0xD7:
                continue
            length_bytes = handle.read(2)
            if len(length_bytes) != 2:
                break
            length = struct.unpack(">H", length_bytes)[0]
            if length < 2:
                break
            if marker_value in {
                0xC0,
                0xC1,
                0xC2,
                0xC3,
                0xC5,
                0xC6,
                0xC7,
                0xC9,
                0xCA,
                0xCB,
                0xCD,
                0xCE,
                0xCF,
            }:
                payload = handle.read(length - 2)
                if len(payload) >= 5:
                    height, width = struct.unpack(">HH", payload[1:5])
                    return int(width), int(height)
                break
            handle.seek(length - 2, os.SEEK_CUR)
    raise ValueError(f"could not read JPEG dimensions: {path}")


def image_to_data_url(path: Path) -> str:
    mime_type = mimetypes.guess_type(str(path))[0] or "application/octet-stream"
    encoded = base64.b64encode(path.read_bytes()).decode("ascii")
    return f"data:{mime_type};base64,{encoded}"


def response_format(coordinate_scale: int = 1000) -> dict[str, Any]:
    point_schema = {
        "type": "array",
        "items": {
            "type": "integer",
            "minimum": 0,
            "maximum": coordinate_scale,
        },
        "minItems": 2,
        "maxItems": 2,
    }
    schema = {
        "type": "object",
        "properties": {
            "windows": {
                "type": "array",
                "items": {
                    "type": "object",
                    "properties": {
                        "points": {
                            "type": "array",
                            "items": point_schema,
                            "minItems": 4,
                            "maxItems": 4,
                        },
                    },
                    "required": ["points"],
                    "additionalProperties": False,
                },
            }
        },
        "required": ["windows"],
        "additionalProperties": False,
    }
    return {
        "type": "json_schema",
        "json_schema": {
            "name": "window_corner_detection",
            "strict": True,
            "schema": schema,
        },
    }


def build_messages(
    system_prompt: str,
    task_prompt: str,
    image_name: str,
    image_width: int,
    image_height: int,
    image_url: str,
    coordinate_scale: int = 1000,
) -> list[dict[str, Any]]:
    example_points = [
        [round(x * coordinate_scale), round(y * coordinate_scale)]
        for x, y in ((0.10, 0.20), (0.30, 0.21), (0.31, 0.40), (0.09, 0.39))
    ]
    example = {
        "windows": [
            {
                "points": example_points,
            }
        ]
    }
    user_text = (
        "请分析下面这张图片。\n"
        f"图片标识: {image_name}\n"
        f"原图尺寸: {image_width} x {image_height}\n\n"
        f"目标定义:\n{task_prompt.strip()}\n\n"
        "坐标与输出协议:\n"
        "- 没有可靠窗户时输出 {\"windows\": []}；\n"
        "- windows 中每个元素只能包含 points 字段，不要输出其他字段；\n"
        "- points 必须严格包含四个点，顺序为 top-left, top-right, bottom-right, bottom-left；\n"
        f"- 每个点是 [x, y]；x 和 y 必须是 0 到 {coordinate_scale}（含端点）的整数；\n"
        f"- 坐标相对于整张原图归一化：左上角为 [0, 0]，右下角为 [{coordinate_scale}, {coordinate_scale}]，不是像素坐标；\n"
        "- 只返回几何框，不要返回任何解释文字。\n\n"
        f"JSON 格式示例: {json.dumps(example, ensure_ascii=False, separators=(',', ':'))}"
    )
    return [
        {"role": "system", "content": system_prompt},
        {
            "role": "user",
            "content": [
                {"type": "text", "text": user_text},
                {"type": "image_url", "image_url": {"url": image_url}},
            ],
        },
    ]


def point_from_value(value: Any) -> tuple[float, float] | None:
    if isinstance(value, dict):
        raw_x = value.get("x", value.get("x_norm", value.get("px")))
        raw_y = value.get("y", value.get("y_norm", value.get("py")))
    elif isinstance(value, (list, tuple)) and len(value) >= 2:
        raw_x, raw_y = value[0], value[1]
    else:
        return None
    try:
        return float(raw_x), float(raw_y)
    except (TypeError, ValueError):
        return None


def raw_points_from_item(item: dict[str, Any]) -> list[Any] | None:
    for key in ("points", "corners", "polygon", "keypoints"):
        value = item.get(key)
        if isinstance(value, list) and len(value) == 4:
            return value
    named = (
        ("top-left", "top-right", "bottom-right", "bottom-left"),
        ("top_left", "top_right", "bottom_right", "bottom_left"),
        ("topLeft", "topRight", "bottomRight", "bottomLeft"),
        ("tl", "tr", "br", "bl"),
    )
    for keys in named:
        if all(key in item for key in keys):
            return [item[key] for key in keys]
    return None


def window_items(payload: Any) -> list[dict[str, Any]] | None:
    if not isinstance(payload, dict):
        return None
    for key in ("windows", "window_boxes", "oriented_boxes", "boxes", "detections", "objects"):
        value = payload.get(key)
        if isinstance(value, list):
            return [item for item in value if isinstance(item, dict)]
    return None


def normalize_points(
    raw_points: list[Any],
    coordinate_scale: int = 1000,
) -> tuple[list[list[float]], str]:
    if coordinate_scale <= 0:
        raise ValueError("coordinate scale must be positive")
    points = [point_from_value(value) for value in raw_points]
    if any(point is None for point in points):
        raise ValueError("each corner must contain numeric x and y")
    numeric = [point for point in points if point is not None]
    if not all(x.is_integer() and y.is_integer() for x, y in numeric):
        raise ValueError("corner coordinates must be integers")
    if not all(0.0 <= x <= coordinate_scale and 0.0 <= y <= coordinate_scale for x, y in numeric):
        raise ValueError(f"corner coordinates must be within 0..{coordinate_scale}")
    normalized = [(x / coordinate_scale, y / coordinate_scale) for x, y in numeric]
    return [[round(x, 7), round(y, 7)] for x, y in normalized], f"normalized_{coordinate_scale}"


def polygon_area(points: list[list[float]]) -> float:
    return 0.5 * abs(
        sum(
            points[index][0] * points[(index + 1) % len(points)][1]
            - points[(index + 1) % len(points)][0] * points[index][1]
            for index in range(len(points))
        )
    )


def orientation(a: list[float], b: list[float], c: list[float]) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def on_segment(a: list[float], b: list[float], p: list[float]) -> bool:
    return (
        min(a[0], b[0]) <= p[0] <= max(a[0], b[0])
        and min(a[1], b[1]) <= p[1] <= max(a[1], b[1])
    )


def segments_intersect(a: list[float], b: list[float], c: list[float], d: list[float]) -> bool:
    eps = 1e-9
    ab_c = orientation(a, b, c)
    ab_d = orientation(a, b, d)
    cd_a = orientation(c, d, a)
    cd_b = orientation(c, d, b)
    if ((ab_c > eps and ab_d < -eps) or (ab_c < -eps and ab_d > eps)) and (
        (cd_a > eps and cd_b < -eps) or (cd_a < -eps and cd_b > eps)
    ):
        return True
    return (
        abs(ab_c) <= eps and on_segment(a, b, c)
        or abs(ab_d) <= eps and on_segment(a, b, d)
        or abs(cd_a) <= eps and on_segment(c, d, a)
        or abs(cd_b) <= eps and on_segment(c, d, b)
    )


def validate_polygon(points: list[list[float]]) -> list[str]:
    errors: list[str] = []
    if len({(point[0], point[1]) for point in points}) != 4:
        errors.append("duplicate_corner_points")
    if polygon_area(points) < 1e-6:
        errors.append("degenerate_quadrilateral")
    if segments_intersect(points[0], points[1], points[2], points[3]) or segments_intersect(
        points[1], points[2], points[3], points[0]
    ):
        errors.append("self_intersecting_quadrilateral")
    return errors


def parse_windows(
    raw_output: str,
    image_width: int,
    image_height: int,
    coordinate_scale: int = 1000,
) -> tuple[dict[str, Any] | None, list[dict[str, Any]], list[dict[str, Any]], str | None, bool]:
    parsed = parse_model_json(raw_output)
    if not parsed.get("ok"):
        error = parsed.get("error") or "invalid_json"
        return None, [], [{"error": error}], error, False
    payload = parsed.get("data")
    items = window_items(payload)
    if items is None:
        error = "missing_windows_array"
        return payload, [], [{"error": error}], error, False

    windows: list[dict[str, Any]] = []
    errors: list[dict[str, Any]] = []
    for index, item in enumerate(items, start=1):
        raw_points = raw_points_from_item(item)
        if raw_points is None:
            errors.append({"index": index, "error": "missing_four_points", "raw": item})
            continue
        try:
            points, coordinate_mode = normalize_points(raw_points, coordinate_scale)
        except ValueError as exc:
            errors.append({"index": index, "error": str(exc), "raw": item})
            continue
        geometry_errors = validate_polygon(points)
        if geometry_errors:
            errors.append({"index": index, "error": geometry_errors, "raw": item})
            continue
        pixel_points = [
            [round(point[0] * image_width, 2), round(point[1] * image_height, 2)] for point in points
        ]
        points_1000 = [[round(point[0] * 1000), round(point[1] * 1000)] for point in points]
        windows.append(
            {
                "window_id": index,
                "points": points,
                "points_1000": points_1000,
                "pixel_points": pixel_points,
                "coordinate_mode": coordinate_mode,
                "coordinate_scale": coordinate_scale,
                "area_ratio": round(polygon_area(points), 7),
            }
        )
    return payload, windows, errors, None, not errors


def dry_run_output(coordinate_scale: int = 1000) -> str:
    def point(x: float, y: float) -> list[int]:
        return [round(x * coordinate_scale), round(y * coordinate_scale)]

    return json.dumps(
        {
            "windows": [
                {
                    "points": [point(0.20, 0.28), point(0.42, 0.27), point(0.43, 0.52), point(0.19, 0.53)],
                },
                {
                    "points": [point(0.59, 0.31), point(0.77, 0.34), point(0.76, 0.55), point(0.58, 0.52)],
                },
            ]
        },
        ensure_ascii=False,
    )


def safe_file_part(value: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", value).strip("._")
    return cleaned or "image"


def svg_text(value: Any) -> str:
    return html.escape(str(value), quote=True)


def write_annotation_svg(record: dict[str, Any], image_path: Path, output_path: Path) -> None:
    width = int(record["image_width"])
    height = int(record["image_height"])
    image_url = image_to_data_url(image_path)
    elements = [
        f'<image href="{svg_text(image_url)}" x="0" y="0" width="{width}" height="{height}" preserveAspectRatio="none"/>'
    ]
    windows = record.get("windows", [])
    for index, window in enumerate(windows):
        points = window["pixel_points"]
        color = COLORS[index % len(COLORS)]
        points_attr = " ".join(f"{point[0]:.2f},{point[1]:.2f}" for point in points)
        elements.append(
            f'<polygon points="{points_attr}" fill="{color}" fill-opacity="0.16" stroke="{color}" stroke-width="3"/>'
        )
        for corner_index, (x, y) in enumerate(points):
            corner = ("TL", "TR", "BR", "BL")[corner_index]
            elements.append(
                f'<circle cx="{x:.2f}" cy="{y:.2f}" r="5" fill="{color}" stroke="#ffffff" stroke-width="2"/>'
                f'<text x="{x + 7:.2f}" y="{y - 7:.2f}" font-family="Arial,sans-serif" font-size="13" font-weight="bold" fill="#ffffff" stroke="#111827" stroke-width="3" paint-order="stroke">{corner}</text>'
            )
        label = f"W{index + 1}"
        label_x = min(point[0] for point in points)
        label_y = max(18.0, min(point[1] for point in points) - 6.0)
        label_width = max(45.0, len(label) * 7.2 + 10.0)
        elements.append(
            f'<rect x="{label_x:.2f}" y="{label_y - 16:.2f}" width="{label_width:.2f}" height="18" fill="{color}" fill-opacity="0.92"/>'
            f'<text x="{label_x + 5:.2f}" y="{label_y - 3:.2f}" font-family="Arial,sans-serif" font-size="12" fill="#ffffff">{svg_text(label)}</text>'
        )
    if not windows:
        message = "No valid windows"
        if record.get("status") != "ok":
            message = "Request failed"
        elements.append(
            f'<rect x="12" y="12" width="180" height="28" fill="#b91c1c" fill-opacity="0.9"/>'
            f'<text x="22" y="31" font-family="Arial,sans-serif" font-size="14" fill="#ffffff">{message}</text>'
        )
    svg = (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">'
        + "".join(elements)
        + "</svg>\n"
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(svg, encoding="utf-8")


def latest_records(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    latest: dict[str, dict[str, Any]] = {}
    for row in rows:
        image_id = str(row.get("image_id") or row.get("image") or "")
        if image_id:
            latest[image_id] = row
    return sorted(latest.values(), key=lambda row: natural_key(str(row.get("image_id", ""))))


def record_image_path(record: dict[str, Any]) -> Path:
    raw = Path(str(record.get("image_path") or record.get("image") or ""))
    return raw if raw.is_absolute() else ROOT / raw


def write_review_html(records: list[dict[str, Any]], output_dir: Path, annotation_paths: dict[str, Path]) -> Path:
    total_windows = sum(len(record.get("windows", [])) for record in records)
    successful = sum(record.get("status") == "ok" for record in records)
    valid = sum(record.get("format_valid") is True for record in records)
    dry_run = bool(records) and all(record.get("dry_run") is True for record in records)
    coordinate_scale = next((record.get("coordinate_scale") for record in records if record.get("coordinate_scale")), 1000)
    mode_text = "当前为离线合成框，仅验证解析和可视化" if dry_run else "当前包含模型请求结果"
    sections: list[str] = []
    for record in records:
        image_id = str(record.get("image_id", ""))
        annotation_path = annotation_paths.get(image_id)
        if annotation_path is None:
            continue
        relative_annotation = os.path.relpath(annotation_path, output_dir).replace(os.sep, "/")
        windows_json = html.escape(json.dumps(record.get("windows", []), ensure_ascii=False, indent=2))
        raw_output = html.escape(str(record.get("raw_output", "")))
        error_payload: Any = record.get("validation_errors", [])
        if record.get("error"):
            error_payload = {
                "request_error": record.get("error"),
                "validation_errors": record.get("validation_errors", []),
            }
        errors = html.escape(json.dumps(error_payload, ensure_ascii=False, indent=2))
        status = "成功" if record.get("status") == "ok" else "失败"
        status_class = "ok" if record.get("status") == "ok" else "failed"
        format_text = "格式通过" if record.get("format_valid") else "格式有问题"
        sections.append(
            "<article class='sample'>"
            f"<div class='sample-header'><h2>{svg_text(image_id)}</h2>"
            f"<span class='status {status_class}'>{status} · {format_text}</span></div>"
            f"<img class='annotation' src='{svg_text(relative_annotation)}' alt='{svg_text(image_id)} 标注结果'>"
            "<div class='meta'>"
            f"<span>窗户数: {len(record.get('windows', []))}</span>"
            f"<span>延迟: {record.get('latency_ms', '-')} ms</span>"
            f"<span>尺寸: {record.get('image_width', '-')} × {record.get('image_height', '-')}</span>"
            "</div>"
            f"<details><summary>查看四角点 JSON</summary><pre>{windows_json}</pre></details>"
            f"<details><summary>查看校验信息</summary><pre>{errors}</pre></details>"
            f"<details><summary>查看模型原始输出</summary><pre>{raw_output}</pre></details>"
            "</article>"
        )
    page = (
        "<!doctype html><html lang='zh-CN'><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>WindowTEC VLM Window Corner Review</title>"
        "<style>"
        "*{box-sizing:border-box}"
        ":root{color-scheme:light;background:#f3f4f6;color:#111827;font-family:Arial,'Noto Sans SC',sans-serif}"
        "body{margin:0;padding:clamp(12px,2vw,24px)}header{max-width:1400px;margin:0 auto 20px}"
        "h1{font-size:24px;margin:0 0 8px}p{color:#4b5563;margin:0 0 12px}"
        ".summary{display:flex;flex-wrap:wrap;gap:8px;margin:12px 0}"
        ".summary span,.meta span{background:#fff;border:1px solid #d1d5db;padding:6px 9px;border-radius:4px;font-size:13px}"
        ".gallery{max-width:1400px;margin:0 auto;display:grid;grid-template-columns:repeat(auto-fit,minmax(min(420px,100%),1fr));gap:16px}"
        ".sample{min-width:0;background:#fff;border:1px solid #d1d5db;border-radius:6px;padding:12px;box-shadow:0 1px 2px #0000000d}"
        ".sample-header{display:flex;align-items:flex-start;justify-content:space-between;flex-wrap:wrap;gap:8px;margin-bottom:8px}"
        "h2{font-size:16px;margin:0;overflow-wrap:anywhere}.status{font-size:12px;padding:4px 7px;border-radius:4px;white-space:normal}.status.ok{background:#dcfce7;color:#166534}.status.failed{background:#fee2e2;color:#991b1b}"
        ".annotation{display:block;width:100%;height:auto;background:#e5e7eb;border:1px solid #9ca3af}"
        ".meta{display:flex;flex-wrap:wrap;gap:6px;margin:9px 0}details{border-top:1px solid #e5e7eb;padding:8px 0}summary{cursor:pointer;color:#374151;font-size:13px}pre{white-space:pre-wrap;overflow:auto;background:#f9fafb;border:1px solid #e5e7eb;padding:8px;font-size:12px;max-height:260px;margin:8px 0 0}"
        "a{color:#0369a1}"
        "</style></head><body><header>"
        "<h1>WindowTEC · VLM 窗户四角点复查</h1>"
        f"<p>{mode_text}。模型坐标协议为 0..{coordinate_scale} 的整数；每个点的标记顺序为 TL → TR → BR → BL；颜色仅用于区分不同窗户。</p>"
        f"<div class='summary'><span>图片: {len(records)}</span><span>请求成功: {successful}</span><span>格式通过: {valid}</span><span>有效窗户: {total_windows}</span>"
        f"<span><a href='predictions.jsonl'>predictions.jsonl</a></span><span><a href='summary.json'>summary.json</a></span></div>"
        "</header><main class='gallery'>"
        + "".join(sections)
        + "</main></body></html>"
    )
    output_path = output_dir / "review.html"
    output_path.write_text(page, encoding="utf-8")
    return output_path


def render_report(records: list[dict[str, Any]], output_dir: Path) -> dict[str, Path]:
    annotation_dir = output_dir / "annotated"
    annotation_paths: dict[str, Path] = {}
    for record in records:
        image_id = str(record.get("image_id", ""))
        image_path = record_image_path(record)
        if not image_id or not image_path.exists():
            continue
        output_path = annotation_dir / f"{safe_file_part(image_id)}.svg"
        write_annotation_svg(record, image_path, output_path)
        annotation_paths[image_id] = output_path
        record["annotation"] = repo_relative(output_path)
    write_review_html(records, output_dir, annotation_paths)
    return annotation_paths


def build_summary(records: list[dict[str, Any]], args: argparse.Namespace, output_dir: Path) -> dict[str, Any]:
    successful = [record for record in records if record.get("status") == "ok"]
    latencies = [record["latency_ms"] for record in successful if isinstance(record.get("latency_ms"), (int, float))]
    return {
        "phase": "windowtec_vlm_window_corners",
        "mode": "dry_run" if args.dry_run else "model",
        "model": next((record.get("model") for record in records if record.get("model")), None),
        "coordinate_scale": args.coordinate_scale,
        "input": str(args.input),
        "image_count": len(records),
        "successful_images": len(successful),
        "format_valid_images": sum(record.get("format_valid") is True for record in records),
        "window_count": sum(len(record.get("windows", [])) for record in records),
        "latency_ms": {
            "mean": round(sum(latencies) / len(latencies), 2) if latencies else None,
            "max": max(latencies) if latencies else None,
        },
        "artifacts": {
            "predictions": repo_relative(output_dir / "predictions.jsonl"),
            "review_html": repo_relative(output_dir / "review.html"),
            "summary": repo_relative(output_dir / "summary.json"),
            "annotated_dir": repo_relative(output_dir / "annotated"),
        },
        "images": [
            {
                "image_id": record.get("image_id"),
                "status": record.get("status"),
                "format_valid": record.get("format_valid"),
                "window_count": len(record.get("windows", [])),
                "validation_error_count": len(record.get("validation_errors", []))
                + (1 if record.get("error") else 0),
            }
            for record in records
        ],
    }


def completed_ids(rows: list[dict[str, Any]]) -> set[str]:
    return {
        str(row.get("image_id"))
        for row in rows
        if row.get("status") == "ok"
        and row.get("format_valid") is True
        and row.get("image_id") is not None
    }


def make_record(
    image_path: Path,
    image_id: str,
    width: int,
    height: int,
    raw_output: str,
    model_name: str,
    requested_model: str,
    latency_ms: int,
    usage: dict[str, Any],
    timings: dict[str, Any],
    dry_run: bool,
    coordinate_scale: int = 1000,
) -> dict[str, Any]:
    parsed_output, windows, validation_errors, parse_error, format_valid = parse_windows(
        raw_output,
        width,
        height,
        coordinate_scale,
    )
    return {
        "image_id": image_id,
        "image": repo_relative(image_path),
        "image_path": repo_relative(image_path),
        "image_width": width,
        "image_height": height,
        "coordinate_scale": coordinate_scale,
        "model": model_name,
        "requested_model": requested_model,
        "status": "ok",
        "dry_run": dry_run,
        "raw_output": raw_output,
        "parsed_output": parsed_output,
        "windows": windows,
        "validation_errors": validation_errors,
        "parse_error": parse_error,
        "format_valid": format_valid and not validation_errors,
        "window_count": len(windows),
        "latency_ms": latency_ms,
        "usage": usage,
        "timings": timings,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", default="images", help="Image directory, image file, or zip archive.")
    parser.add_argument("--base-url", default="http://127.0.0.1:8000/v1")
    parser.add_argument(
        "--api-key",
        default=os.environ.get("WINDOWTEC_API_KEY", os.environ.get("MODEL_EVA_API_KEY", "EMPTY")),
    )
    parser.add_argument("--model", default="auto")
    parser.add_argument("--system-prompt", default="system_prompt.md")
    parser.add_argument("--task-prompt", default="task_prompt.md")
    parser.add_argument("--out-dir", default="results/vlm_window_corners")
    parser.add_argument("--temperature", type=float, default=0.0)
    parser.add_argument("--top-p", type=float, default=1.0)
    parser.add_argument("--max-tokens", type=int, default=2048)
    parser.add_argument(
        "--coordinate-scale",
        type=int,
        default=1000,
        help="Maximum integer coordinate emitted by the model (default: 1000).",
    )
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--limit", type=int, default=None)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--dry-run", action="store_true", help="Use synthetic corner points to verify parsing and visualization.")
    parser.add_argument("--render-only", action="store_true", help="Render an existing predictions.jsonl without calling a model.")
    parser.add_argument("--no-response-format", action="store_true")
    parser.add_argument("--fail-fast", action="store_true")
    return parser


def run(args: argparse.Namespace) -> int:
    if args.coordinate_scale <= 0:
        raise ValueError("--coordinate-scale must be a positive integer")
    input_path = resolve_path(args.input)
    output_dir = resolve_path(args.out_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    predictions_path = output_dir / "predictions.jsonl"

    if args.render_only:
        if not predictions_path.exists():
            raise FileNotFoundError(f"missing predictions file: {predictions_path}")
        records = latest_records(read_jsonl(predictions_path))
        render_report(records, output_dir)
        write_json(output_dir / "summary.json", build_summary(records, args, output_dir))
        print(f"rendered {len(records)} records -> {output_dir / 'review.html'}")
        return 0

    extraction_dir = output_dir / "_extracted"
    image_paths = collect_images(input_path, extraction_dir)
    if args.limit is not None:
        image_paths = image_paths[: args.limit]
    if not image_paths:
        raise FileNotFoundError(f"no supported images found in {input_path}")

    existing_rows: list[dict[str, Any]] = []
    if args.resume and predictions_path.exists():
        existing_rows = read_jsonl(predictions_path)
    else:
        predictions_path.unlink(missing_ok=True)
    done = completed_ids(existing_rows)

    system_prompt = read_text(resolve_path(args.system_prompt))
    task_prompt = read_text(resolve_path(args.task_prompt))
    client: OpenAICompatibleClient | None = None
    model_name = "dry-run"
    if not args.dry_run:
        client = OpenAICompatibleClient(
            args.base_url,
            args.api_key,
            args.model,
            timeout=args.timeout,
            temperature=args.temperature,
            top_p=args.top_p,
            max_tokens=args.max_tokens,
        )
        model_name = client.model

    for image_path in image_paths:
        image_id = image_identifier(image_path, input_path, extraction_dir)
        if image_id in done:
            print(f"skip {image_id} (resume)")
            continue
        width, height = read_image_size(image_path)
        try:
            if args.dry_run:
                raw_output = dry_run_output(args.coordinate_scale)
                latency_ms = 0
                usage: dict[str, Any] = {}
                timings: dict[str, Any] = {}
            else:
                assert client is not None
                messages = build_messages(
                    system_prompt,
                    task_prompt,
                    image_id,
                    width,
                    height,
                    image_to_data_url(image_path),
                    args.coordinate_scale,
                )
                try:
                    result = client.chat(
                        messages,
                        response_format=None if args.no_response_format else response_format(args.coordinate_scale),
                    )
                except ModelClientError:
                    if args.no_response_format:
                        raise
                    print(f"response_format failed for {image_id}; retrying without schema")
                    result = client.chat(messages, response_format=None)
                raw_output = result.content if isinstance(result.content, str) else json.dumps(result.content, ensure_ascii=False)
                latency_ms = result.latency_ms
                usage = {
                    "prompt_tokens": result.prompt_tokens,
                    "completion_tokens": result.completion_tokens,
                    "total_tokens": result.total_tokens,
                }
                timings = extract_llamacpp_timings(result.raw_response)
            record = make_record(
                image_path,
                image_id,
                width,
                height,
                raw_output,
                model_name,
                args.model,
                latency_ms,
                usage,
                timings,
                args.dry_run,
                args.coordinate_scale,
            )
        except Exception as exc:
            record = {
                "image_id": image_id,
                "image": repo_relative(image_path),
                "image_path": repo_relative(image_path),
                "image_width": width,
                "image_height": height,
                "coordinate_scale": args.coordinate_scale,
                "model": model_name,
                "requested_model": args.model,
                "status": "failed",
                "dry_run": args.dry_run,
                "error": f"{type(exc).__name__}: {exc}",
                "windows": [],
                "validation_errors": [],
                "format_valid": False,
                "window_count": 0,
            }
            if args.fail_fast:
                raise
        append_jsonl(predictions_path, [record])
        print(
            f"{record['status']:7} {image_id}: windows={len(record.get('windows', []))} "
            f"format_valid={record.get('format_valid')}"
        )

    records = latest_records(read_jsonl(predictions_path))
    render_report(records, output_dir)
    summary = build_summary(args=args, records=records, output_dir=output_dir)
    write_json(output_dir / "summary.json", summary)
    print(f"review report: {output_dir / 'review.html'}")
    return 0 if all(record.get("status") == "ok" for record in records) else 1


def main() -> int:
    return run(build_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
