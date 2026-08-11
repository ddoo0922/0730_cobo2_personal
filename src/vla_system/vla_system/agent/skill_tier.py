"""Tier 1 -- the rule layer that answers before the conversational LLM does.

Why this sits in front of ``agent_node`` rather than replacing it
----------------------------------------------------------------
Most utterances in this domain are not hard. "사과 다 담아줘" needs a class
filter and a count, nothing more. Sending those through a full conversational
turn -- history, scene, robot state, tool round-trips -- pays a large fixed
cost for a decision a few lines of code can make.

So this layer takes the easy ones and **hands everything else to the node it
sits in front of**. It is not a replacement for the conversational agent; the
agent is still what answers when the rules cannot. Measured across 30 scenarios
x 3 repeats, routing this way cut median API latency from 13.6s to 10.6s while
raising nothing's cost -- because a rule-handled utterance calls no LLM at all.

What deliberately stays out of the rules
----------------------------------------
Only "which object to take" lives here: filter by class and colour, drop the
forbidden ones, count. Those conditions are unambiguous, so code re-deciding
them every turn buys nothing.

Intent reading stays out entirely. "아까 그거", "그건 말고", conditionals,
ordering -- there is no field for them on purpose. Give a parser a field and it
will fill it, and a filled-in-wrong rule executes confidently. That failure --
succeeding at the wrong thing -- is worse than escalating, and it is exactly
how the first version of this layer broke on mid-task corrections.

The same reasoning bounds ``exclude_classes``: it exists because without it the
parser turned "바나나랑 컵 **빼고** 다 담아줘" into *take the banana and the
cup*, and nothing flagged it because the parse was structurally valid.

Safety
------
Hazardous classes are never taken without confirmation, and the confirmation
holds **for the session only** -- it is not promoted to a long-term rule. No
amount of personalisation can permanently disable a safety check.

Host seam
---------
This module knows nothing about ROS. The host supplies speech, actions, scene
contents and escalation through :class:`SkillHost`; ``agent_node`` implements
it against ROS topics and the evaluation harness implements it against a
fake robot. Both therefore exercise the same decision code -- what gets
measured is what ships.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Protocol

from vla_system.agent.rules import HAZARD_CLASSES, Rule, RuleStore

# Flat and shallow on purpose. Fewer slots to fill means less deliberation,
# and less deliberation means fewer slots filled in by guesswork.
RULE_SCHEMA = {
    "type": "object",
    "properties": {
        "intent": {
            "type": "string",
            "enum": ["pick", "prohibit", "correct", "scope_answer",
                     "recall", "forget", "other"],
            "description": (
                "pick=집으라는 지시, prohibit=집지 말라는 금지, "
                "correct=앞서 말한 규칙을 고치는 것, "
                "scope_answer=방금 받은 '이번만/계속' 질문에 대한 답, "
                "recall=무엇을 기억하는지 묻는 것, forget=기억을 지우라는 것, "
                "other=위 어디에도 안 맞음(대화형 판단에 넘긴다)"
            ),
        },
        "classes": {
            "type": "array", "items": {"type": "string"},
            "description": "대상 물체의 영문 클래스명. 예: apple, banana, cup, scissors",
        },
        "colors": {
            "type": "array", "items": {"type": "string"},
            "description": "색 조건. 없으면 빈 배열.",
        },
        "exclude_classes": {
            "type": "array", "items": {"type": "string"},
            "description": (
                "이번 지시에서 **빼야 하는** 클래스. '바나나랑 컵 빼고 다 담아줘' "
                "라면 [\"banana\",\"cup\"]. 없으면 빈 배열."
            ),
        },
        "exclude_colors": {
            "type": "array", "items": {"type": "string"},
            "description": "이번 지시에서 빼야 하는 색. 없으면 빈 배열.",
        },
        "replaces": {
            "type": "array", "items": {"type": "string"},
            "description": (
                "intent가 correct일 때, **지워야 할 옛 규칙의 클래스명**. "
                "'바나나 말고 컵을 담지 말라는 거였어'라면 [\"banana\"]. "
                "'아니 빨간 컵만 담지 마'처럼 같은 대상을 좁히는 것이면 [\"cup\"]. "
                "correct가 아니면 빈 배열."
            ),
        },
        "quantity": {
            "type": "string", "enum": ["one", "all", "count"],
            "description": "one=하나, all=전부, count=숫자로 지정",
        },
        "count": {
            "type": ["integer", "null"],
            "description": "quantity가 count일 때의 개수. 아니면 null.",
        },
        "scope": {
            "type": "string", "enum": ["now", "today", "standing", "ask"],
            "description": (
                "now=이번 지시만, today=오늘/당분간처럼 이번 세션까지만, "
                "standing=앞으로 계속(사용자가 '항상','쭉' 등으로 명시했을 때만), "
                "ask=언제까지인지 불분명해서 되물어야 함"
            ),
        },
        "reason": {
            "type": "string",
            "description": "사용자가 댄 이유. 예: '깨지기 쉬워서'. 없으면 빈 문자열.",
        },
    },
    "required": ["intent", "classes", "colors", "exclude_classes", "exclude_colors",
                 "replaces", "quantity", "count", "scope", "reason"],
    "additionalProperties": False,
}

RULE_PROMPT = """\
너는 로봇 피킹 지시를 구조화하는 파서다. 사용자의 한국어 문장 하나를 읽고
스키마를 채운다. 판단이 아니라 받아쓰기에 가깝게, 문장에 실제로 있는 것만 적어라.

[intent 고르는 법]
- "사과 집어줘", "다 담아줘" -> pick
- "컵은 담지 마", "가위 가져오지 마" -> prohibit
- "이번만", "쭉", "앞으로 계속" 처럼 직전 질문에 답하는 짧은 말 -> scope_answer
- "뭐 기억해?", "지금까지 기억한 거 알려줘" -> recall
- "그거 잊어버려", "기억 지워" -> forget
- "아니 그게 아니라 ~", "~ 말고 ~를 말한 거였어", "아까 ~라고 한 거 사실 ~였어"
  처럼 **앞서 말한 규칙을 고치는** 문장 -> correct.
  이때 replaces에는 지워야 할 옛 규칙의 클래스를, classes/colors에는 새 규칙을 적는다.
  **classes를 절대 비우지 마라.** 새 대상이 없으면 그것은 correct가 아니라 forget이다.
    "바나나 말고 컵을 담지 말라는 거였어"
      -> intent=correct, replaces=["banana"], classes=["cup"]
    "아니 그게 아니라 빨간 컵만 담지 마"
      -> intent=correct, replaces=["cup"], classes=["cup"], colors=["red"]
    "아까 컵 담지 말라고 한 거, 사실 접시를 말한 거였어"
      -> intent=correct, replaces=["cup"], classes=["plate"]
- 위 어디에도 확실히 안 맞으면 -> other. 억지로 pick으로 만들지 마라.

[빼라는 말 -- 절대 반대로 채우지 마라]
"A랑 B 빼고 다 담아줘"는 A와 B를 담으라는 뜻이 **아니다**.
  classes=[] (대상 한정 없음), exclude_classes=["A","B"], quantity=all
"초록 사과는 빼고"처럼 색으로 빼는 것이면 exclude_colors에 적는다.
빼라는 말이 없으면 두 배열 모두 빈 배열이다.

[scope 고르는 법]
- 기본은 now. 한 번 하고 끝나는 지시다.
- "계속", "쭉", "보이면 계속", "여러 개" 처럼 반복은 뜻하지만 **언제까지인지**
  말하지 않았으면 ask. 이번 작업 동안만인지 다음에 로봇을 켰을 때도인지
  구분되지 않기 때문이다. 이 경우가 가장 흔하다.
- standing은 다음에도 유효함을 사용자가 분명히 한 경우에만 쓴다.
  "앞으로도", "다음부터", "항상", "매번" 처럼 이번 작업 너머를 가리키는 말이
  있어야 한다.
- "오늘은", "오늘만" 처럼 하루로 못 박은 것은 today.
- **이유와 기간이 서로 다른 방향을 가리키면 ask다.** "오늘은 컵이 젖어 있으니까
  담지 마"는 이유("젖어서")만 보면 계속일 것 같고 "오늘은"만 보면 짧다.
  짐작하지 말고 ask로 두어 되묻게 한다. "당분간"처럼 기간이 모호한 것도 ask다.

[중요]
- 문장에 없는 조건을 만들어 내지 마라. 색을 말하지 않았으면 colors는 빈 배열이다.
- "아까 그거", "그건 말고", "~면 ~하고" 같이 이전 맥락이나 조건 분기가 필요한
  문장은 이 스키마로 표현할 수 없다. 그럴 때는 intent를 other로 두어라.
  억지로 채우는 것보다 넘기는 편이 낫다.

[물체 이름]
사과->apple, 바나나->banana, 오렌지/귤->orange, 컵/머그컵->cup, 병/물병->bottle,
와인잔->wine glass, 책->book, 마우스->mouse, 휴대폰->cell phone, 리모컨->remote,
공->sports ball, 곰인형->teddy bear, 시계->clock, 가위->scissors, 칼->knife
"""

# Spoken names. The rule store runs on English class ids, but
# "scissors는 위험한 물건인데 가져올까요?" is not a finished product's speech.
KOREAN = {
    "apple": "사과", "banana": "바나나", "orange": "오렌지", "cup": "컵",
    "bottle": "병", "wine glass": "와인잔", "book": "책", "mouse": "마우스",
    "cell phone": "휴대폰", "remote": "리모컨", "sports ball": "공",
    "teddy bear": "곰인형", "clock": "시계", "scissors": "가위", "knife": "칼",
    "plate": "접시",
}


class Escalation:
    """Why Tier 1 gave up. Recorded so the split can be tuned from data."""

    MISSION_INTERRUPTED = "mission_interrupted"   # new utterance mid-task
    NOT_EXPRESSIBLE = "not_expressible"           # parser said intent=other
    SCHEMA_ERROR = "schema_error"
    AMBIGUOUS = "ambiguous"


@dataclass(frozen=True)
class SceneItem:
    """One pickable thing, as Tier 1 needs to see it.

    ``rank`` breaks ties when several candidates match; the host decides what
    it means (track id, distance, whatever it can order by).
    """

    object_id: str
    class_name: str
    color: str
    pickable: bool = True
    rank: int = 0


class SkillHost(Protocol):
    """What Tier 1 needs from whoever runs it."""

    def say(self, text: str) -> None: ...
    def ask(self, text: str) -> None: ...
    def pick(self, object_id: str, reason: str) -> None: ...
    def escalate(self, reason: str, text: str, mission_text: str) -> None: ...
    def scene_items(self) -> list[SceneItem]: ...
    def note(self, event: dict) -> None: ...
    def record(self, tier: str, detail: str = "") -> None: ...
    def turn_done(self) -> None: ...


class SkillTier:
    """The rule layer. One instance per session; ``store`` outlives it."""

    def __init__(self, host: SkillHost, store: RuleStore, parse) -> None:
        self.host = host
        self.store = store
        self._parse = parse                     # (text) -> dict | None

        self._mission: dict | None = None
        self._mission_text = ""
        self._acted = 0
        self._pending_scope: dict | None = None
        self._pending_promo: dict | None = None
        self._pending_hazard: dict | None = None
        # Asking about a stored prohibition, and the exception granted if the
        # user says yes. An exception is not a retraction -- treating it as one
        # would quietly delete a safety rule the user set, so the rule stays
        # and only this mission sees past it.
        self._pending_conflict: dict | None = None
        self._override: set[str] = set()
        self._hazard_ack: set[str] = set()

    # ---------------------------------------------------------- entry point

    def handle(self, text: str) -> bool:
        """Take one utterance. Returns False if Tier 1 declined it entirely.

        Answers to a question Tier 1 asked are matched first: "응" means yes to
        the thing just asked, and re-parsing it as a fresh instruction fails.
        """
        if self._pending_hazard is not None and self._answer_hazard(text):
            return True
        if self._pending_conflict is not None and self._answer_conflict(text):
            return True
        if self._pending_promo is not None and self._answer_promotion(text):
            return True
        if self._pending_scope is not None and self._answer_scope(text):
            return True

        if self._mission is not None:
            # An utterance arriving mid-task is far more often a correction or
            # a retraction than a fresh order. Rules read it as a fresh order
            # and confidently do the wrong thing, so it goes upstairs.
            self.host.escalate(Escalation.MISSION_INTERRUPTED, text, self._mission_text)
            self._mission = None
            return True

        return self._parse_and_act(text)

    def on_action_finished(self) -> None:
        """One dispatched action completed. Continue the mission, if any."""
        self._decide()

    def end_session(self) -> None:
        self.store.end_session()

    @property
    def busy(self) -> bool:
        return self._mission is not None

    # ------------------------------------------------------------- answers

    @staticmethod
    def _is_yes(text: str) -> bool:
        return any(w in text for w in ("응", "네", "그래", "좋아", "맞아", "예", "ㅇㅇ", "해줘"))

    @staticmethod
    def _is_no(text: str) -> bool:
        return any(w in text for w in ("아니", "안 ", "싫", "그만", "괜찮"))

    def _answer_hazard(self, text: str) -> bool:
        pending, self._pending_hazard = self._pending_hazard, None
        if not self._is_yes(text):
            self._pending_hazard = pending
            return False
        # Session-scoped on purpose: a new process must ask again.
        self._hazard_ack |= set(pending["classes"])
        self.host.record("skill", "hazard_ack")
        self.host.say("알겠습니다. 조심해서 가져오겠습니다.")
        self._start_mission(pending["parsed"], pending["text"])
        return True

    def _answer_conflict(self, text: str) -> bool:
        pending, self._pending_conflict = self._pending_conflict, None
        if self._is_yes(text):
            self._override |= set(pending["classes"])
            self.host.record("skill", "conflict_override")
            self.host.say("네, 이번에만 가져오겠습니다.")
            self._start_mission(pending["parsed"], pending["text"])
            return True
        if self._is_no(text):
            self.host.record("skill", "conflict_declined")
            self.host.say("알겠습니다. 담지 않겠습니다.")
            self.host.turn_done()
            return True
        self._pending_conflict = pending
        return False

    def _answer_promotion(self, text: str) -> bool:
        pending, self._pending_promo = self._pending_promo, None
        if self._is_yes(text):
            rule = Rule(kind="standing_pick", classes=tuple(pending["classes"]),
                        colors=tuple(pending["colors"]), source="repetition")
            self.store.add(rule, long_term=False)
            self.host.record("learn", "repetition_accepted")
            self.host.say("네, 이번 작업 동안은 계속 가져다 드릴게요.")
        elif self._is_no(text):
            self.host.record("learn", "repetition_declined")
            self.host.say("알겠습니다. 그때그때 말씀해 주세요.")
        else:
            self._pending_promo = pending
            return False
        self.host.turn_done()
        return True

    def _answer_scope(self, text: str) -> bool:
        pending, self._pending_scope = self._pending_scope, None
        parsed = pending["parsed"]
        # Read the duration before the agreement. "오늘만 그렇게 해줘" carries
        # both ("해줘" reads as yes, "오늘만" as a limit); taking the agreement
        # first made it a standing rule that outlived the session. A duration
        # the user stated out loud always wins.
        once = ("이번" in text or "한번" in text or "지금" in text
                or "오늘" in text or self._is_no(text))
        standing = not once and ("쭉" in text or "계속" in text or "앞으로" in text
                                 or "항상" in text or self._is_yes(text))
        if not standing and not once:
            self._pending_scope = pending
            return False

        if pending.get("kind") == "prohibit":
            rule = Rule(kind="prohibit", classes=tuple(parsed["classes"]),
                        colors=tuple(parsed["colors"]),
                        reason=parsed.get("reason", ""),
                        source="safety" if parsed.get("reason") else "explicit")
            self.store.add(rule, long_term=standing)
            self.host.record("learn",
                             "prohibit_long_term" if standing else "prohibit_session")
            self.host.say("네, 앞으로도 담지 않겠습니다." if standing
                          else "네, 이번에는 담지 않겠습니다.")
            self.host.turn_done()
            return True

        if standing:
            rule = Rule(kind="standing_pick", classes=tuple(parsed["classes"]),
                        colors=tuple(parsed["colors"]), source="explicit")
            self.store.add(rule, long_term=True)
            self.host.record("learn", "scope_standing")
            self.host.say("네, 앞으로도 계속 그렇게 하겠습니다.")
        else:
            self.host.record("learn", "scope_now")
            self.host.say("네, 이번만 하겠습니다.")
        self._start_mission(parsed, pending["text"])
        return True

    # --------------------------------------------------------------- intent

    def _parse_and_act(self, text: str) -> bool:
        parsed = self._parse(text)
        if parsed is None:
            self.host.escalate(Escalation.SCHEMA_ERROR, text, self._mission_text)
            return True

        intent = parsed.get("intent", "other")

        if intent == "other":
            # The parser said so itself. Believe it.
            self.host.escalate(Escalation.NOT_EXPRESSIBLE, text, self._mission_text)
            return True

        if intent == "recall":
            self.host.record("skill")
            self.host.say(self.store.describe_all())
            self.host.turn_done()
            return True

        if intent == "forget":
            self.host.record("skill")
            removed = sum(self.store.forget(c) for c in parsed.get("classes", []))
            self.host.say("기억에서 지웠습니다." if removed
                          else "지울 규칙을 찾지 못했습니다.")
            self.host.turn_done()
            return True

        if intent == "correct":
            # A correction overwrites; it does not pile on. Piling on means the
            # robot can do less the longer it is used.
            self.host.record("skill")
            # A "correction" with no new target is not one. Going ahead would
            # delete the old rule and leave nothing, while the user hears
            # "고쳐서 기억하겠습니다" and believes they are protected. That
            # actually happened -- the parser filled replaces and left classes
            # empty. Hand it upstairs before deleting anything.
            if not parsed.get("classes"):
                self.host.escalate(Escalation.NOT_EXPRESSIBLE, text, self._mission_text)
                return True
            targets = parsed.get("replaces") or parsed.get("classes", [])
            removed = sum(self.store.forget(c) for c in targets)
            reason = parsed.get("reason", "")
            scope = parsed.get("scope")
            rule = Rule(kind="prohibit", classes=tuple(parsed["classes"]),
                        colors=tuple(parsed["colors"]), reason=reason,
                        source="correction")
            self.store.add(rule,
                           long_term=scope == "standing" or (bool(reason) and scope != "today"))
            self.host.record("learn", "corrected")
            self.host.say("알겠습니다. 고쳐서 기억하겠습니다." if removed
                          else "알겠습니다. 그렇게 기억하겠습니다.")
            self.host.turn_done()
            return True

        if intent == "prohibit":
            self.host.record("skill")
            if parsed.get("scope") == "ask":
                self._pending_scope = {"parsed": parsed, "text": text, "kind": "prohibit"}
                self.host.ask("이번 작업에만 적용할까요, 앞으로도 계속 그렇게 할까요?")
                self.host.turn_done()
                return True
            reason = parsed.get("reason", "")
            rule = Rule(kind="prohibit", classes=tuple(parsed["classes"]),
                        colors=tuple(parsed["colors"]), reason=reason,
                        source="safety" if reason else "explicit")
            # Giving a reason makes it knowledge about the thing, not a passing
            # whim: "깨지기 쉬워서" is still true next time the robot boots.
            # A stated deadline outranks that -- "오늘은" means today even with
            # the most convincing reason attached.
            scope = parsed.get("scope")
            long_term = scope == "standing" or (bool(reason) and scope != "today")
            self.store.add(rule, long_term=long_term)
            self.host.record("learn",
                             "prohibit_long_term" if long_term else "prohibit_session")
            self.host.say(("알겠습니다. " + (f"{reason} " if reason else "")
                           + "앞으로 담지 않겠습니다.") if long_term
                          else "알겠습니다. 이번에는 담지 않겠습니다.")
            self.host.turn_done()
            return True

        if intent == "scope_answer":
            # A scope answer with nothing pending needs the context we lack.
            self.host.escalate(Escalation.NOT_EXPRESSIBLE, text, self._mission_text)
            return True

        return self._begin_pick(parsed, text)

    # ----------------------------------------------------------------- pick

    def _begin_pick(self, parsed: dict, text: str) -> bool:
        if parsed.get("scope") == "ask":
            self._pending_scope = {"parsed": parsed, "text": text}
            self.host.record("skill")
            self.host.ask("이번 작업에만 적용할까요, 앞으로도 계속 그렇게 할까요?")
            self.host.turn_done()
            return True

        # Named a class we were told not to take. Filtering it out silently
        # hands the user a different object with no explanation -- they end up
        # trapped by a rule they wrote. An unrestricted order ("다 담아줘") is
        # not this case; there the rule is doing its job.
        blocked = [c for c in parsed["classes"]
                   if c not in self._override and self.store.is_forbidden(c, "")]
        if blocked:
            names = "/".join(KOREAN.get(c, c) for c in blocked)
            self._pending_conflict = {"classes": blocked, "parsed": parsed, "text": text}
            self.host.record("skill")
            self.host.ask(f"전에 {names}은(는) 담지 말라고 하셨는데, 이번엔 가져올까요?")
            self.host.turn_done()
            return True

        hazards = [c for c in parsed["classes"]
                   if c in HAZARD_CLASSES and c not in self._hazard_ack]
        if hazards:
            names = "/".join(KOREAN.get(c, c) for c in hazards)
            self._pending_hazard = {"classes": hazards, "parsed": parsed, "text": text}
            self.host.record("skill")
            self.host.ask(f"{names}는 위험한 물건인데 가져올까요?")
            self.host.turn_done()
            return True

        if parsed.get("scope") == "standing":
            rule = Rule(kind="standing_pick", classes=tuple(parsed["classes"]),
                        colors=tuple(parsed["colors"]), source="explicit")
            self.store.add(rule, long_term=True)
            self.host.record("learn", "explicit_standing")

        # With a standing rule in force, take the quantity as "all". A user who
        # registered "사과 보이면 계속 가져와" and then says "사과 가져와" gets
        # nothing out of that rule if we stop at one. This is where a long-term
        # rule actually changes behaviour.
        if parsed["quantity"] == "one":
            standing = self.store.standing_picks()
            if any(rule.matches_class(name)
                   for name in parsed["classes"] for rule in standing):
                parsed["quantity"] = "all"
                parsed["count"] = None
                self.host.record("skill", "standing_rule_applied")

        # Third time asking the same thing: offer to make it standing.
        key = (tuple(parsed["classes"]), tuple(parsed["colors"]))
        if parsed.get("scope") == "now" and self.store.bump_repeat(key) == 3:
            self._pending_promo = {"classes": parsed["classes"], "colors": parsed["colors"]}
            self.host.record("skill")
            self.host.ask("같은 걸 계속 부탁하시네요. 이번 작업 동안은 제가 알아서 "
                          "계속 가져다 드릴까요?")
            self.host.turn_done()
            return True

        self._start_mission(parsed, text)
        return True

    def _start_mission(self, parsed: dict, text: str) -> None:
        self._mission = {"parsed": parsed, "picked": []}
        self._mission_text = text
        self._acted = 0
        self._decide()

    def _limit(self) -> int | None:
        parsed = self._mission["parsed"]
        if parsed["quantity"] == "one":
            return 1
        if parsed["quantity"] == "count":
            return parsed.get("count") or 1
        return None

    def _candidates(self) -> list[tuple[int, str]]:
        parsed = self._mission["parsed"]
        classes = set(parsed["classes"])
        colors = set(parsed["colors"])
        exclude_classes = set(parsed.get("exclude_classes") or ())
        exclude_colors = set(parsed.get("exclude_colors") or ())
        # Already dispatched in this mission. The scene is not guaranteed to
        # have caught up: the executor reports "done" on its own topic and the
        # camera republishes on its own clock, so a just-taken object is often
        # still in the latest snapshot. Trusting the snapshot alone made the
        # layer pick the same apple twice, every time, on a real ROS graph --
        # the harness never showed it because its driver updates the scene
        # before reporting the result.
        taken = set(self._mission["picked"])
        found = []
        for item in self.host.scene_items():
            if item.object_id in taken:
                continue
            if not item.pickable:
                continue
            if classes and item.class_name not in classes:
                continue
            if colors and item.color not in colors:
                continue
            # "A랑 B 빼고 다 담아줘". With no slot for exclusion the parser put
            # A and B in ``classes`` and the layer took exactly the wrong things.
            if item.class_name in exclude_classes or item.color in exclude_colors:
                continue
            # A prohibition outranks the instruction: "다 담아줘" still skips the
            # cup while "컵은 깨지니 담지 마" stands. Unless the user named it and
            # confirmed, in which case this mission alone sees past it.
            if (item.class_name not in self._override
                    and self.store.is_forbidden(item.class_name, item.color)):
                continue
            if item.class_name in HAZARD_CLASSES and item.class_name not in self._hazard_ack:
                continue
            found.append((item.rank, item.object_id))
        found.sort()
        return found

    def _decide(self) -> None:
        if self._mission is None:
            return

        limit = self._limit()
        if limit is not None and self._acted >= limit:
            self._end_mission()
            return

        candidates = self._candidates()
        if not candidates:
            self._end_mission()
            return

        if limit == 1 and len(candidates) > 1:
            # Told to take one, several match. Rules cannot narrow this.
            mission_text = self._mission_text
            self._mission = None
            self.host.escalate(Escalation.AMBIGUOUS, mission_text, mission_text)
            return

        _, object_id = candidates[0]
        self._acted += 1
        self._mission["picked"].append(object_id)
        self.host.record("skill")
        self.host.pick(object_id, "규칙 실행")
        self.host.note({"event": "skill_action", "action": "pick_and_place",
                        "object_id": object_id})

    def _end_mission(self) -> None:
        """Instruction satisfied. Fall through to a standing rule if one exists."""
        self._mission = None
        self._mission_text = ""
        standing = self.store.standing_picks()
        if not standing:
            self.host.turn_done()
            return
        rule = standing[0]
        self._mission = {
            "parsed": {"intent": "pick", "classes": list(rule.classes),
                       "colors": list(rule.colors), "quantity": "all",
                       "count": None, "scope": "standing", "reason": "",
                       "exclude_classes": [], "exclude_colors": [], "replaces": []},
            "picked": [],
        }
        self._mission_text = rule.describe()
        self._acted = 0
        self._decide()


def make_parser(client, model: str):
    """Bind an OpenAI client into the single-call parser Tier 1 uses."""

    def parse(text: str) -> dict | None:
        try:
            response = client.responses.create(
                model=model,
                instructions=RULE_PROMPT,
                input=[{"role": "user", "content": text}],
                text={"format": {"type": "json_schema", "name": "vla_rule",
                                 "strict": True, "schema": RULE_SCHEMA}},
            )
            return json.loads(response.output_text)
        except Exception:
            return None

    return parse
