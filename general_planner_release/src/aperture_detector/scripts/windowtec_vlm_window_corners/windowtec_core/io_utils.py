"""Shared file and summary helpers for benchmark runners."""

from __future__ import annotations

import json
import statistics
from pathlib import Path
from typing import Any, Iterable


def read_json(path: str | Path) -> Any:
    with Path(path).open("r", encoding="utf-8") as handle:
        return json.load(handle)


def read_text(path: str | Path) -> str:
    return Path(path).read_text(encoding="utf-8")


def read_jsonl(path: str | Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with Path(path).open("r", encoding="utf-8") as handle:
        for line_no, line in enumerate(handle, start=1):
            stripped = line.strip()
            if not stripped:
                continue
            try:
                row = json.loads(stripped)
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{line_no}: invalid JSONL row: {exc}") from exc
            rows.append(row)
    return rows


def append_jsonl(path: str | Path, rows: Iterable[dict[str, Any]]) -> None:
    out_path = Path(path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("a", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, ensure_ascii=False) + "\n")


def write_json(path: str | Path, data: Any) -> None:
    out_path = Path(path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8") as handle:
        json.dump(data, handle, ensure_ascii=False, indent=2)
        handle.write("\n")


def percentile(values: list[int | float], p: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return float(ordered[0])
    rank = (len(ordered) - 1) * p
    lower = int(rank)
    upper = min(lower + 1, len(ordered) - 1)
    weight = rank - lower
    return float(ordered[lower] * (1 - weight) + ordered[upper] * weight)


def mean(values: list[int | float]) -> float:
    return float(statistics.fmean(values)) if values else 0.0


def rate(rows: list[dict[str, Any]], key: str) -> float:
    if not rows:
        return 0.0
    return mean([float(row.get(key, 0)) for row in rows])


def extract_llamacpp_timings(raw_response: dict[str, Any] | None) -> dict[str, Any]:
    if not raw_response:
        return {}
    keys = [
        "prompt_ms",
        "prompt_n",
        "prompt_per_token_ms",
        "prompt_per_second",
        "predicted_ms",
        "predicted_n",
        "predicted_per_token_ms",
        "predicted_per_second",
        "timings",
    ]
    timings = {key: raw_response[key] for key in keys if key in raw_response}
    if "usage" in raw_response:
        timings["usage"] = raw_response["usage"]
    return timings


def timing_value(row: dict[str, Any], key: str) -> Any:
    timings = row.get("timings", {})
    if key in timings:
        return timings.get(key)
    nested = timings.get("timings", {})
    if isinstance(nested, dict):
        return nested.get(key)
    return None


def timing_values(rows: list[dict[str, Any]], key: str) -> list[float]:
    values: list[float] = []
    for row in rows:
        value = timing_value(row, key)
        if isinstance(value, int | float) and not isinstance(value, bool):
            values.append(float(value))
    return values


def add_timing_summary(summary: dict[str, Any], rows: list[dict[str, Any]]) -> None:
    summary.update(
        {
            "prompt_eval_time_ms_mean": mean(timing_values(rows, "prompt_ms")),
            "prompt_eval_tokens_mean": mean(timing_values(rows, "prompt_n")),
            "prompt_eval_ms_per_token_mean": mean(timing_values(rows, "prompt_per_token_ms")),
            "prompt_eval_tokens_per_second_mean": mean(timing_values(rows, "prompt_per_second")),
            "decode_time_ms_mean": mean(timing_values(rows, "predicted_ms")),
            "decode_tokens_mean": mean(timing_values(rows, "predicted_n")),
            "decode_ms_per_token_mean": mean(timing_values(rows, "predicted_per_token_ms")),
            "decode_tokens_per_second_mean": mean(timing_values(rows, "predicted_per_second")),
            "cache_tokens_mean": mean(timing_values(rows, "cache_n")),
        }
    )


def compact_tools(tools_schema: dict[str, Any]) -> list[dict[str, Any]]:
    tools = tools_schema.get("tools", tools_schema if isinstance(tools_schema, list) else [])
    compact: list[dict[str, Any]] = []
    for tool in tools:
        if not isinstance(tool, dict):
            continue
        args = tool.get("arguments", {})
        required = args.get("required", []) if isinstance(args, dict) else []
        properties = args.get("properties", {}) if isinstance(args, dict) else {}
        arg_names = list(properties.keys()) if isinstance(properties, dict) else []
        compact.append(
            {
                "name": tool.get("name"),
                "required_args": required,
                "args": arg_names,
                "rules": tool.get("rules", [])[:2],
                "benchmark_only": tool.get("benchmark_only", False),
            }
        )
    return compact


def compact_rules(rules: dict[str, Any]) -> dict[str, Any]:
    return {
        "forbidden_patterns": [
            {"id": item.get("id"), "message": item.get("message")}
            for item in rules.get("forbidden_patterns", [])
        ],
        "safety_rules": [
            {"id": item.get("id"), "rule": item.get("rule")}
            for item in rules.get("safety_rules", [])
        ],
        "tool_selection_rules": [
            {
                "id": item.get("id"),
                "prefer": item.get("prefer"),
                "avoid": item.get("avoid"),
                "argument_rule": item.get("argument_rule"),
            }
            for item in rules.get("tool_selection_rules", [])
        ],
    }
