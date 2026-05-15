#!/usr/bin/env python3
"""Filter a dense Fast-LIO PCD into a planner-friendly PointXYZI map.

The script intentionally avoids Open3D/PCL Python bindings.  It reads common
ASCII/binary PCD files with x/y/z fields, applies simple planning-oriented
filters, and writes a binary PointXYZI PCD that PCL can load directly.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
from scipy.spatial import cKDTree


def default_pcd_dir() -> Path:
    repo_root = Path(__file__).resolve().parents[1]
    return repo_root / "mars_uav_sim" / "perfect_drone_sim" / "pcd"


def parse_header(lines: Sequence[str]) -> Dict[str, object]:
    meta: Dict[str, object] = {}
    for raw in lines:
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        key = parts[0].upper()
        vals = parts[1:]
        if key in {"FIELDS", "TYPE"}:
            meta[key] = vals
        elif key in {"SIZE", "COUNT"}:
            meta[key] = [int(v) for v in vals]
        elif key in {"WIDTH", "HEIGHT", "POINTS"}:
            meta[key] = int(vals[0])
        elif key == "DATA":
            meta[key] = vals[0].lower()
    if "FIELDS" not in meta or "SIZE" not in meta or "TYPE" not in meta or "DATA" not in meta:
        raise ValueError("PCD header is missing FIELDS/SIZE/TYPE/DATA")
    fields = meta["FIELDS"]
    if not isinstance(fields, list) or not {"x", "y", "z"}.issubset(fields):
        raise ValueError("PCD file must contain x/y/z fields")
    if "COUNT" not in meta:
        meta["COUNT"] = [1] * len(fields)
    if "POINTS" not in meta:
        meta["POINTS"] = int(meta.get("WIDTH", 0)) * int(meta.get("HEIGHT", 1))
    return meta


def numpy_type(size: int, typ: str) -> str:
    typ = typ.upper()
    if typ == "F":
        return f"<f{size}"
    if typ == "I":
        return f"<i{size}"
    if typ == "U":
        return f"<u{size}"
    raise ValueError(f"Unsupported PCD TYPE {typ!r}")


def pcd_struct_dtype(meta: Dict[str, object]) -> np.dtype:
    fields = meta["FIELDS"]
    sizes = meta["SIZE"]
    types = meta["TYPE"]
    counts = meta["COUNT"]
    assert isinstance(fields, list)
    assert isinstance(sizes, list)
    assert isinstance(types, list)
    assert isinstance(counts, list)
    dtype_fields = []
    for name, size, typ, count in zip(fields, sizes, types, counts):
        base = np.dtype(numpy_type(size, typ))
        if count == 1:
            dtype_fields.append((name, base))
        else:
            dtype_fields.append((name, base, (count,)))
    return np.dtype(dtype_fields)


def read_pcd_xyz(path: Path) -> Tuple[np.ndarray, Dict[str, object]]:
    header_lines: List[str] = []
    with path.open("rb") as f:
        while True:
            line_b = f.readline()
            if not line_b:
                raise ValueError("PCD header ended before DATA line")
            line = line_b.decode("latin1").rstrip("\n")
            header_lines.append(line)
            if line.strip().upper().startswith("DATA"):
                break
        meta = parse_header(header_lines)
        data_type = str(meta["DATA"])
        points = int(meta["POINTS"])
        fields = meta["FIELDS"]
        assert isinstance(fields, list)
        xyz_indices = [fields.index(axis) for axis in ("x", "y", "z")]

        if data_type == "binary":
            arr = np.fromfile(f, dtype=pcd_struct_dtype(meta), count=points)
            xyz = np.column_stack([arr["x"], arr["y"], arr["z"]]).astype(np.float32, copy=False)
        elif data_type == "ascii":
            data = np.loadtxt(f, dtype=np.float32)
            if data.ndim == 1:
                data = data.reshape(1, -1)
            xyz = data[:, xyz_indices].astype(np.float32, copy=False)
        else:
            raise ValueError(f"Unsupported PCD DATA mode {data_type!r}; binary_compressed is not supported")

    return xyz, meta


def write_binary_xyzi_pcd(path: Path, points: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    points = np.asarray(points, dtype=np.float32)
    out = np.empty(points.shape[0], dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")])
    out["x"] = points[:, 0]
    out["y"] = points[:, 1]
    out["z"] = points[:, 2]
    out["intensity"] = 0.0
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {len(out)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(out)}\n"
        "DATA binary\n"
    )
    with path.open("wb") as f:
        f.write(header.encode("ascii"))
        out.tofile(f)


def finite_crop_filter(points: np.ndarray, args: argparse.Namespace) -> Tuple[np.ndarray, Dict[str, int]]:
    mask = np.isfinite(points).all(axis=1)
    before_finite = int(mask.size)
    if args.x_min is not None:
        mask &= points[:, 0] >= args.x_min
    if args.x_max is not None:
        mask &= points[:, 0] <= args.x_max
    if args.y_min is not None:
        mask &= points[:, 1] >= args.y_min
    if args.y_max is not None:
        mask &= points[:, 1] <= args.y_max
    if args.z_min is not None:
        mask &= points[:, 2] >= args.z_min
    if args.z_max is not None:
        mask &= points[:, 2] <= args.z_max
    return points[mask], {"removed_by_crop": before_finite - int(mask.sum())}


def clear_spheres(points: np.ndarray, spheres: Iterable[Tuple[float, float, float, float]]) -> Tuple[np.ndarray, int]:
    if points.size == 0:
        return points, 0
    mask = np.ones(points.shape[0], dtype=bool)
    for x, y, z, r in spheres:
        center = np.array([x, y, z], dtype=np.float32)
        mask &= np.sum((points - center) ** 2, axis=1) > float(r) ** 2
    return points[mask], int(mask.size - mask.sum())


def voxel_downsample(points: np.ndarray, voxel_size: float) -> Tuple[np.ndarray, int]:
    if voxel_size <= 0.0 or points.size == 0:
        return points, 0
    origin = points.min(axis=0)
    keys = np.floor((points - origin) / voxel_size).astype(np.int64)
    order = np.lexsort((keys[:, 2], keys[:, 1], keys[:, 0]))
    sorted_keys = keys[order]
    sorted_points = points[order]
    starts = np.r_[0, np.flatnonzero(np.any(np.diff(sorted_keys, axis=0), axis=1)) + 1]
    counts = np.diff(np.r_[starts, len(sorted_points)])
    sums = np.add.reduceat(sorted_points, starts, axis=0)
    downsampled = (sums / counts[:, None]).astype(np.float32)
    return downsampled, int(points.shape[0] - downsampled.shape[0])


def query_knn(tree: cKDTree, points: np.ndarray, k: int) -> np.ndarray:
    try:
        distances, _ = tree.query(points, k=k, workers=-1)
    except TypeError:
        distances, _ = tree.query(points, k=k)
    return distances


def statistical_outlier_filter(points: np.ndarray, k: int, std_ratio: float) -> Tuple[np.ndarray, Dict[str, float]]:
    if k <= 0 or points.shape[0] <= k + 1:
        return points, {"removed_by_sor": 0, "sor_threshold": math.inf}
    tree = cKDTree(points)
    distances = query_knn(tree, points, k + 1)
    mean_dist = distances[:, 1:].mean(axis=1)
    threshold = float(mean_dist.mean() + std_ratio * mean_dist.std())
    mask = mean_dist <= threshold
    return points[mask], {"removed_by_sor": int(mask.size - mask.sum()), "sor_threshold": threshold}


def radius_outlier_filter(points: np.ndarray, radius: float, min_neighbors: int) -> Tuple[np.ndarray, int]:
    if radius <= 0.0 or min_neighbors <= 0 or points.size == 0:
        return points, 0
    tree = cKDTree(points)
    try:
        counts = tree.query_ball_point(points, radius, return_length=True, workers=-1)
    except TypeError:
        try:
            counts = tree.query_ball_point(points, radius, return_length=True)
        except TypeError:
            counts = np.fromiter((len(v) for v in tree.query_ball_point(points, radius)), dtype=np.int32)
    mask = counts >= (min_neighbors + 1)
    return points[mask], int(mask.size - mask.sum())


def parse_sphere(value: str) -> Tuple[float, float, float, float]:
    parts = [float(v.strip()) for v in value.split(",")]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("clear sphere must be x,y,z,r")
    if parts[3] <= 0:
        raise argparse.ArgumentTypeError("clear sphere radius must be positive")
    return parts[0], parts[1], parts[2], parts[3]


def bounds_dict(points: np.ndarray) -> Dict[str, Optional[List[float]]]:
    if points.size == 0:
        return {"min": None, "max": None}
    return {
        "min": [float(v) for v in points.min(axis=0)],
        "max": [float(v) for v in points.max(axis=0)],
    }


def build_arg_parser() -> argparse.ArgumentParser:
    pcd_dir = default_pcd_dir()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=pcd_dir / "2_1.pcd")
    parser.add_argument("--output", type=Path, default=pcd_dir / "2_1_planning.pcd")
    parser.add_argument("--report", type=Path, default=None, help="Optional path to save the JSON report")
    parser.add_argument("--voxel-size", type=float, default=0.12, help="Voxel centroid size in meters")
    parser.add_argument("--sor-k", type=int, default=24, help="Neighbor count for statistical outlier removal")
    parser.add_argument("--sor-std-ratio", type=float, default=1.4)
    parser.add_argument("--radius", type=float, default=0.32, help="Radius outlier search radius in meters")
    parser.add_argument("--min-neighbors", type=int, default=3, help="Minimum neighbors inside radius, excluding self")
    parser.add_argument("--x-min", type=float, default=None)
    parser.add_argument("--x-max", type=float, default=None)
    parser.add_argument("--y-min", type=float, default=None)
    parser.add_argument("--y-max", type=float, default=None)
    parser.add_argument("--z-min", type=float, default=0.10, help="Default removes the floor plane that inflates into takeoff")
    parser.add_argument("--z-max", type=float, default=3.20, help="Default removes upper-floor/ceiling clutter for this map")
    parser.add_argument(
        "--clear-sphere",
        action="append",
        type=parse_sphere,
        default=[],
        metavar="x,y,z,r",
        help="Remove points inside a sphere; can be repeated for start/landing areas",
    )
    parser.add_argument("--dry-run", action="store_true", help="Run filters but do not write the output PCD")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    src = args.input.expanduser().resolve()
    dst = args.output.expanduser().resolve()
    report_path = args.report.expanduser().resolve() if args.report else None

    points, meta = read_pcd_xyz(src)
    report: Dict[str, object] = {
        "input": str(src),
        "output": str(dst),
        "input_points": int(points.shape[0]),
        "input_bounds": bounds_dict(points),
        "pcd_data": meta.get("DATA"),
        "filters": {
            "voxel_size": args.voxel_size,
            "sor_k": args.sor_k,
            "sor_std_ratio": args.sor_std_ratio,
            "radius": args.radius,
            "min_neighbors": args.min_neighbors,
            "z_min": args.z_min,
            "z_max": args.z_max,
            "clear_spheres": args.clear_sphere,
        },
    }

    points, crop_stats = finite_crop_filter(points, args)
    report.update(crop_stats)
    report["after_crop_points"] = int(points.shape[0])
    report["after_crop_bounds"] = bounds_dict(points)

    points, cleared = clear_spheres(points, args.clear_sphere)
    report["removed_by_clear_spheres"] = cleared
    report["after_clear_spheres_points"] = int(points.shape[0])

    points, voxel_removed = voxel_downsample(points, args.voxel_size)
    report["removed_by_voxel"] = voxel_removed
    report["after_voxel_points"] = int(points.shape[0])
    report["after_voxel_bounds"] = bounds_dict(points)

    points, sor_stats = statistical_outlier_filter(points, args.sor_k, args.sor_std_ratio)
    report.update(sor_stats)
    report["after_sor_points"] = int(points.shape[0])

    points, radius_removed = radius_outlier_filter(points, args.radius, args.min_neighbors)
    report["removed_by_radius"] = radius_removed
    report["output_points"] = int(points.shape[0])
    report["output_bounds"] = bounds_dict(points)

    if points.shape[0] == 0:
        raise RuntimeError("All points were filtered out; relax crop/outlier parameters")

    if not args.dry_run:
        write_binary_xyzi_pcd(dst, points)
        if report_path is not None:
            report_path.parent.mkdir(parents=True, exist_ok=True)
            report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"filter_pcd_for_planning.py: error: {exc}", file=sys.stderr)
        raise SystemExit(1)
