"""OpenAI-compatible chat client used by vLLM and llama.cpp servers."""

from __future__ import annotations

import json
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class ChatResult:
    content: str
    latency_ms: int
    prompt_tokens: int | None = None
    completion_tokens: int | None = None
    total_tokens: int | None = None
    raw_response: dict[str, Any] | None = None


class ModelClientError(RuntimeError):
    """Raised when the model endpoint cannot return a valid chat response."""


class OpenAICompatibleClient:
    def __init__(
        self,
        base_url: str,
        api_key: str,
        model: str | None = None,
        timeout: float = 120.0,
        temperature: float = 0.0,
        top_p: float = 1.0,
        max_tokens: int = 1024,
        max_retries: int = 8,
        disable_thinking: bool = True,
    ) -> None:
        self.base_url = base_url.rstrip("/")
        self.api_key = api_key
        self.timeout = timeout
        self.temperature = temperature
        self.top_p = top_p
        self.max_tokens = max_tokens
        self.max_retries = max_retries
        self.disable_thinking = disable_thinking
        self.model = self.resolve_model(model)

    @classmethod
    def with_resolved_model(
        cls,
        base_url: str,
        api_key: str,
        model: str,
        timeout: float = 120.0,
        temperature: float = 0.0,
        top_p: float = 1.0,
        max_tokens: int = 1024,
        max_retries: int = 8,
        disable_thinking: bool = True,
    ) -> "OpenAICompatibleClient":
        client = cls.__new__(cls)
        client.base_url = base_url.rstrip("/")
        client.api_key = api_key
        client.timeout = timeout
        client.temperature = temperature
        client.top_p = top_p
        client.max_tokens = max_tokens
        client.max_retries = max_retries
        client.disable_thinking = disable_thinking
        client.model = model
        return client

    def _disable_thinking_payload(self) -> dict[str, Any]:
        return {
            "enable_thinking": False,
            "thinking": False,
            "reasoning": False,
            "chat_template_kwargs": {"enable_thinking": False},
        }

    def _request_json(self, path: str, method: str = "GET", payload: dict[str, Any] | None = None) -> dict[str, Any]:
        return self._request_json_at(self.base_url, path, method=method, payload=payload)

    def _request_json_at(
        self,
        base_url: str,
        path: str,
        method: str = "GET",
        payload: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        body = None
        if payload is not None:
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        request = urllib.request.Request(
            f"{base_url}{path}",
            data=body,
            headers={
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json",
            },
            method=method,
        )
        response_body = ""
        for attempt in range(getattr(self, "max_retries", 3) + 1):
            try:
                with urllib.request.urlopen(request, timeout=self.timeout) as response:
                    response_body = response.read().decode("utf-8")
                break
            except urllib.error.HTTPError as exc:
                error_body = exc.read().decode("utf-8", errors="replace")
                if exc.code in {429, 500, 502, 503, 504} and attempt < getattr(self, "max_retries", 3):
                    time.sleep(min(2 ** attempt, 8))
                    continue
                raise ModelClientError(
                    f"model request failed: HTTP {exc.code} {exc.reason}: {error_body[:1000]}"
                ) from exc
            except urllib.error.URLError as exc:
                if attempt < getattr(self, "max_retries", 3):
                    time.sleep(min(2 ** attempt, 8))
                    continue
                raise ModelClientError(f"model request failed: {exc}") from exc
        try:
            data = json.loads(response_body)
        except json.JSONDecodeError as exc:
            raise ModelClientError(f"invalid JSON response from {path}: {response_body[:500]}") from exc
        if not isinstance(data, dict):
            raise ModelClientError(f"invalid response from {path}: expected object")
        return data

    def list_models(self) -> list[str]:
        data = self._request_json("/models")
        models = []
        for item in data.get("data", []):
            if isinstance(item, dict) and item.get("id"):
                models.append(str(item["id"]))
        return models

    def resolve_model(self, model: str | None) -> str:
        if model and model.lower() != "auto":
            return model
        models = self.list_models()
        if not models:
            raise ModelClientError("could not auto-discover model: /models returned no model ids")
        if len(models) > 1:
            print(f"[model_client] /models returned multiple ids; using first: {models[0]}")
        return models[0]

    def chat(
        self,
        messages: list[dict[str, Any]],
        *,
        response_format: dict[str, Any] | None = None,
        extra_payload: dict[str, Any] | None = None,
    ) -> ChatResult:
        payload = {
            "model": self.model,
            "messages": messages,
            "temperature": self.temperature,
            "top_p": self.top_p,
            "max_tokens": self.max_tokens,
        }
        if self.disable_thinking:
            payload.update(self._disable_thinking_payload())
        if response_format is not None:
            payload["response_format"] = response_format
        if extra_payload:
            payload.update(extra_payload)
        if self.disable_thinking:
            payload.update(self._disable_thinking_payload())
        start = time.perf_counter()
        data = self._request_json("/chat/completions", method="POST", payload=payload)
        latency_ms = int((time.perf_counter() - start) * 1000)

        try:
            message = data["choices"][0]["message"]
            content = (message.get("content") or "") if isinstance(message, dict) else ""
        except (KeyError, IndexError, TypeError) as exc:
            raise ModelClientError(f"invalid chat response: {str(data)[:500]}") from exc

        usage = data.get("usage") or {}
        return ChatResult(
            content=content,
            latency_ms=latency_ms,
            prompt_tokens=usage.get("prompt_tokens"),
            completion_tokens=usage.get("completion_tokens"),
            total_tokens=usage.get("total_tokens"),
            raw_response=data,
        )

    def completion(self, prompt: str, *, json_schema: dict[str, Any] | None = None) -> ChatResult:
        base_url = self.base_url[:-3] if self.base_url.endswith("/v1") else self.base_url
        payload: dict[str, Any] = {
            "prompt": prompt,
            "temperature": self.temperature,
            "top_p": self.top_p,
            "n_predict": self.max_tokens,
        }
        if self.disable_thinking:
            payload.update(self._disable_thinking_payload())
        if json_schema is not None:
            payload["json_schema"] = json_schema
        start = time.perf_counter()
        data = self._request_json_at(base_url, "/completion", method="POST", payload=payload)
        latency_ms = int((time.perf_counter() - start) * 1000)

        content = data.get("content")
        if content is None:
            try:
                content = data["choices"][0]["text"]
            except (KeyError, IndexError, TypeError) as exc:
                raise ModelClientError(f"invalid completion response: {str(data)[:500]}") from exc

        usage = data.get("usage") or {}
        return ChatResult(
            content=str(content),
            latency_ms=latency_ms,
            prompt_tokens=usage.get("prompt_tokens"),
            completion_tokens=usage.get("completion_tokens"),
            total_tokens=usage.get("total_tokens"),
            raw_response=data,
        )


def build_messages(system_prompt: str, user_payload: dict[str, Any]) -> list[dict[str, str]]:
    return [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": json.dumps(user_payload, ensure_ascii=False, indent=2)},
    ]
