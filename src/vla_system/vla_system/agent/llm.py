"""OpenAI boundary: speech-to-text and one tool-calling round.

Everything above this file works with plain dicts, so the rest of the system
does not have to know which API shape is in use.
"""

from dataclasses import dataclass, field
import json
import os
from pathlib import Path

from vla_system.agent.prompt import STT_PROMPT, SYSTEM_PROMPT
from vla_system.agent.tools import TOOLS
from vla_system.agent.vision import attach_image


@dataclass(frozen=True)
class ToolCall:
    call_id: str
    name: str
    arguments: dict
    raw_arguments: str
    parse_error: str = ""


@dataclass(frozen=True)
class AgentResponse:
    text: str = ""
    calls: tuple[ToolCall, ...] = field(default_factory=tuple)


class AgentLLM:
    def __init__(
        self,
        model: str = "gpt-5-mini",
        stt_model: str = "gpt-4o-transcribe",
        env_file: str | Path | None = None,
        timeout_s: float = 30.0,
    ):
        from dotenv import load_dotenv

        load_dotenv(Path(env_file).expanduser() if env_file else None)
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            raise RuntimeError("OPENAI_API_KEY 환경변수가 필요합니다.")

        from openai import OpenAI

        self.client = OpenAI(api_key=api_key, timeout=timeout_s)
        self.model = model
        self.stt_model = stt_model

    def transcribe(self, audio_path: str | Path) -> str:
        with Path(audio_path).open("rb") as audio_file:
            response = self.client.audio.transcriptions.create(
                model=self.stt_model,
                file=audio_file,
                language="ko",
                prompt=STT_PROMPT,
            )
        text = response.text.strip()
        if not text:
            raise RuntimeError("STT 결과가 비어 있습니다.")
        return text

    def respond(self, items: list[dict], image: str = "") -> AgentResponse:
        """`image`는 data URL. 이번 호출에만 실리고 기록에는 남지 않는다.

        기록에 넣지 않는 이유는 agent/vision.py 설명에 있다 -- 요약하면, 넣으면
        지난 사진들이 매 호출마다 따라 올라간다.
        """
        response = self.client.responses.create(
            model=self.model,
            instructions=SYSTEM_PROMPT,
            input=attach_image(items, image) if image else items,
            tools=TOOLS,
        )
        return self._parse(response)

    @staticmethod
    def _parse(response) -> AgentResponse:
        calls: list[ToolCall] = []
        for item in getattr(response, "output", []) or []:
            if getattr(item, "type", "") != "function_call":
                continue
            raw = getattr(item, "arguments", "") or "{}"
            try:
                arguments = json.loads(raw)
                error = ""
            except (json.JSONDecodeError, TypeError) as exc:
                # A malformed argument blob is reported back to the model as a
                # tool result rather than raised: the model can correct itself
                # on the next round, whereas an exception would drop the turn.
                arguments, error = {}, f"arguments를 JSON으로 읽을 수 없습니다: {exc}"
            calls.append(
                ToolCall(
                    call_id=getattr(item, "call_id", "") or getattr(item, "id", ""),
                    name=getattr(item, "name", ""),
                    arguments=arguments if isinstance(arguments, dict) else {},
                    raw_arguments=raw,
                    parse_error=error,
                )
            )
        return AgentResponse(
            text=(getattr(response, "output_text", "") or "").strip(),
            calls=tuple(calls),
        )
