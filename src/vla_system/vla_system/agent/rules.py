"""A4의 규칙 기억. 세션 규칙과 장기 규칙을 나눠 담는다.

두 저장소를 나눈 이유
------------------
사용자가 하는 말에는 수명이 다른 두 종류가 섞여 있다. "지금은 사과만 담아줘"는
이번 작업이 끝나면 잊어야 하고, "컵은 깨지니까 절대 담지 마"는 다음에 로봇을
켜도 지켜져야 한다. 하나의 저장소에 넣으면 둘 중 하나는 반드시 틀린다 --
전부 휘발시키면 안전 지시를 잊고, 전부 남기면 일회성 지시가 영원히 따라다닌다.

    세션 규칙   프로세스가 살아 있는 동안만. 파일에 안 남는다.
    장기 규칙   파일에 남고 다음 프로세스가 읽는다.

안전 경계
--------
개인화가 건드릴 수 있는 것은 **무엇을 고를지**(클래스·색 선호, 금지 목록)뿐이다.
집을 수 있는지, 사람 전용인지 같은 판정은 매칭기의 안전 게이트가 따로 보며 이
저장소는 거기에 접근하지 않는다. 금지를 *추가*하는 것은 언제나 허용되고,
위험물 확인 같은 안전 절차를 *해제*하는 규칙은 애초에 저장되지 않는다.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field, asdict
from datetime import datetime, timezone
from pathlib import Path

# 확인 없이 집으면 안 되는 클래스. 개인화로 해제할 수 없다.
HAZARD_CLASSES = frozenset({"scissors", "knife"})


@dataclass
class Rule:
    """학습된 지시 하나."""

    kind: str                       # standing_pick | prohibit
    classes: tuple[str, ...] = ()
    colors: tuple[str, ...] = ()
    reason: str = ""                # 사용자가 댄 근거. 장기 승격의 단서가 된다.
    source: str = ""                # explicit | repetition | safety
    created_at: str = field(
        default_factory=lambda: datetime.now(timezone.utc).isoformat())

    def describe(self) -> str:
        what = "/".join(self.classes) or "무엇이든"
        if self.colors:
            what = "/".join(self.colors) + " " + what
        if self.kind == "prohibit":
            text = f"{what}은(는) 담지 않는다"
        else:
            text = f"{what}이(가) 보이면 계속 담는다"
        return text + (f" ({self.reason})" if self.reason else "")

    def matches_class(self, class_name: str) -> bool:
        return not self.classes or class_name in self.classes

    def matches_color(self, color: str) -> bool:
        return not self.colors or color in self.colors


class RuleStore:
    """세션 규칙 + 장기 규칙.

    장기 규칙만 파일로 오간다. 파일 경로를 주지 않으면 장기 규칙도 메모리에만
    남으므로, 실험에서 에피소드끼리 규칙이 새지 않게 격리하기 쉽다.
    """

    def __init__(self, long_term_path: str | Path | None = None):
        self.long_term_path = Path(long_term_path).expanduser() if long_term_path else None
        self.session: list[Rule] = []
        self.long_term: list[Rule] = []
        # 같은 지시가 몇 번 반복됐는지. 세션 안에서만 센다 -- 사용자의 예시가
        # "현재 프로세스에서 계속 가져다 드릴까요?"였다.
        self.repeat_counts: dict[tuple, int] = {}
        self._load()

    # ------------------------------------------------------------ 영속화

    def _load(self) -> None:
        if not self.long_term_path or not self.long_term_path.exists():
            return
        try:
            raw = json.loads(self.long_term_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return
        for item in raw.get("rules", []):
            item.pop("_", None)
            self.long_term.append(Rule(
                kind=item.get("kind", "prohibit"),
                classes=tuple(item.get("classes", ())),
                colors=tuple(item.get("colors", ())),
                reason=item.get("reason", ""),
                source=item.get("source", ""),
                created_at=item.get("created_at", ""),
            ))

    def _save(self) -> None:
        if not self.long_term_path:
            return
        self.long_term_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {"rules": [asdict(r) for r in self.long_term]}
        self.long_term_path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    # -------------------------------------------------------------- 쓰기

    def add(self, rule: Rule, long_term: bool) -> Rule:
        target = self.long_term if long_term else self.session
        # 같은 내용이 두 번 들어오면 갱신만 한다. 목록이 중복으로 불어나면
        # 나중에 사용자에게 읽어 줄 때 같은 말을 두 번 하게 된다.
        for existing in target:
            if (existing.kind, existing.classes, existing.colors) == \
               (rule.kind, rule.classes, rule.colors):
                return existing
        target.append(rule)
        if long_term:
            self._save()
        return rule

    def forget(self, class_name: str) -> int:
        """해당 클래스에 걸린 규칙을 모두 지운다. 지운 개수를 돌려준다."""
        removed = 0
        for bucket in (self.session, self.long_term):
            before = len(bucket)
            bucket[:] = [r for r in bucket if class_name not in r.classes]
            removed += before - len(bucket)
        self._save()
        return removed

    def bump_repeat(self, key: tuple) -> int:
        self.repeat_counts[key] = self.repeat_counts.get(key, 0) + 1
        return self.repeat_counts[key]

    def end_session(self) -> None:
        """세션 종료. 세션 규칙과 반복 계수는 버리고 장기 규칙만 남긴다."""
        self.session.clear()
        self.repeat_counts.clear()

    # -------------------------------------------------------------- 읽기

    @property
    def all(self) -> list[Rule]:
        return self.session + self.long_term

    def prohibitions(self) -> list[Rule]:
        return [r for r in self.all if r.kind == "prohibit"]

    def standing_picks(self) -> list[Rule]:
        return [r for r in self.all if r.kind == "standing_pick"]

    def is_forbidden(self, class_name: str, color: str) -> Rule | None:
        for rule in self.prohibitions():
            if rule.matches_class(class_name) and rule.matches_color(color):
                return rule
        return None

    def describe_all(self) -> str:
        """사용자가 '지금까지 뭐 기억해?'라고 물었을 때 읽어 줄 문장."""
        if not self.all:
            return "따로 기억하고 있는 규칙은 없습니다."
        parts = []
        for rule in self.long_term:
            parts.append(f"(계속) {rule.describe()}")
        for rule in self.session:
            parts.append(f"(이번만) {rule.describe()}")
        return " / ".join(parts)
