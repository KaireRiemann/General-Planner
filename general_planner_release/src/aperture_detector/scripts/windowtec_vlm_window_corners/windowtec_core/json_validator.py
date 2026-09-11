"""Model-output JSON parsing utilities."""

from __future__ import annotations

import json
import re
from typing import Any


def strip_code_fence(text: str) -> tuple[str, bool]:
    stripped = text.strip()
    fence_match = re.fullmatch(r"```(?:json)?\s*(.*?)\s*```", stripped, flags=re.DOTALL | re.IGNORECASE)
    if fence_match:
        return fence_match.group(1).strip(), True
    return stripped, False


def extract_json_object(text: str) -> str | None:
    start = text.find("{")
    end = text.rfind("}")
    if start == -1 or end == -1 or end <= start:
        return None
    return text[start : end + 1]


def parse_model_json(raw_output: str) -> dict[str, Any]:
    cleaned, had_fence = strip_code_fence(raw_output)
    candidates = [cleaned]
    extracted = extract_json_object(cleaned)
    if extracted and extracted != cleaned:
        candidates.append(extracted)

    last_error: str | None = None
    for candidate in candidates:
        try:
            data = json.loads(candidate)
            if not isinstance(data, dict):
                return {
                    "ok": False,
                    "data": None,
                    "error": "json_root_not_object",
                    "had_code_fence": had_fence,
                    "cleaned": candidate,
                }
            return {
                "ok": True,
                "data": data,
                "error": None,
                "had_code_fence": had_fence,
                "cleaned": candidate,
            }
        except json.JSONDecodeError as exc:
            last_error = str(exc)

    return {
        "ok": False,
        "data": None,
        "error": last_error or "invalid_json",
        "had_code_fence": had_fence,
        "cleaned": cleaned,
    }


if __name__ == "__main__":
    assert parse_model_json('{"plan": []}')["ok"]
    assert parse_model_json('```json\n{"plan": []}\n```')["ok"]
    assert not parse_model_json("not json")["ok"]
    print("json_validator smoke test passed")
