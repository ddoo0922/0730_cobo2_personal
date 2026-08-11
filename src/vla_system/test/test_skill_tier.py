"""Tier 1 rule layer, driven by a fake host and a table-driven parser.

No ROS and no network. The parser is a lookup table so a failure here is a
failure of the decision logic, never of the LLM's mood -- the two are worth
separating because only one of them is ours to fix.
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vla_system.agent.rules import RuleStore                      # noqa: E402
from vla_system.agent.skill_tier import SceneItem, SkillTier      # noqa: E402


class FakeHost:
    def __init__(self, items):
        self.items = list(items)
        self.said, self.asked, self.picked, self.escalated = [], [], [], []
        self.notes, self.records, self.turns = [], [], 0

    def say(self, text): self.said.append(text)
    def ask(self, text): self.asked.append(text)
    def escalate(self, reason, text, mission_text): self.escalated.append(reason)
    def note(self, event): self.notes.append(event)
    def record(self, tier, detail=""): self.records.append((tier, detail))
    def turn_done(self): self.turns += 1
    def scene_items(self): return list(self.items)

    def pick(self, object_id, reason):
        self.picked.append(object_id)
        # A picked object leaves the table, exactly as the real scene reports.
        self.items = [i for i in self.items if i.object_id != object_id]


def parsed(intent="pick", classes=(), colors=(), ex=(), exc=(), rep=(),
           quantity="all", count=None, scope="now", reason=""):
    return {"intent": intent, "classes": list(classes), "colors": list(colors),
            "exclude_classes": list(ex), "exclude_colors": list(exc),
            "replaces": list(rep), "quantity": quantity, "count": count,
            "scope": scope, "reason": reason}


def build(items, table, store=None):
    host = FakeHost(items)
    tier = SkillTier(host, store or RuleStore(),
                     lambda t: table.get(t, parsed(intent="other")))
    return host, tier


TABLE_SCENE = [
    SceneItem("cup_1", "cup", "white", rank=1),
    SceneItem("banana_2", "banana", "yellow", rank=2),
    SceneItem("apple_3", "apple", "red", rank=3),
]


def drive(tier, host, text):
    """One utterance, then let the mission run to completion."""
    tier.handle(text)
    for _ in range(10):
        if not tier.busy:
            break
        tier.on_action_finished()


# --------------------------------------------------------------- exclusion

def test_exclusion_is_not_inverted():
    """The failure this field exists for: "빼고" taken as "take"."""
    host, tier = build(TABLE_SCENE, {
        "바나나랑 컵 빼고 다 담아줘": parsed(ex=["banana", "cup"], quantity="all")})
    drive(tier, host, "바나나랑 컵 빼고 다 담아줘")
    assert host.picked == ["apple_3"]


def test_exclusion_by_colour():
    scene = [SceneItem("g", "apple", "green", rank=1),
             SceneItem("r", "apple", "red", rank=2)]
    host, tier = build(scene, {"초록 사과 빼고 사과 다 담아줘":
                               parsed(classes=["apple"], exc=["green"])})
    drive(tier, host, "초록 사과 빼고 사과 다 담아줘")
    assert host.picked == ["r"]


# ------------------------------------------------- prohibition vs the user

def test_prohibition_filters_broad_orders_silently():
    store = RuleStore()
    host, tier = build(TABLE_SCENE, {
        "컵은 깨지기 쉬우니까 앞으로 담지 마":
            parsed("prohibit", ["cup"], reason="깨지기 쉬워서"),
        "여기 있는 거 다 담아줘": parsed(quantity="all")}, store)
    drive(tier, host, "컵은 깨지기 쉬우니까 앞으로 담지 마")
    drive(tier, host, "여기 있는 거 다 담아줘")
    assert "cup_1" not in host.picked
    assert host.asked == []          # a broad order needs no confirmation


def test_named_prohibition_asks_then_complies():
    store = RuleStore()
    table = {"컵은 깨지기 쉬우니까 앞으로 담지 마":
                 parsed("prohibit", ["cup"], reason="깨지기 쉬워서"),
             "컵 가져와": parsed(classes=["cup"], quantity="one")}
    host, tier = build(TABLE_SCENE, table, store)
    drive(tier, host, "컵은 깨지기 쉬우니까 앞으로 담지 마")
    drive(tier, host, "컵 가져와")
    assert host.asked, "naming a forbidden class must prompt, not silently swap"
    assert host.picked == []
    drive(tier, host, "응 가져와")
    assert host.picked == ["cup_1"]


def test_exception_does_not_erase_the_rule():
    store = RuleStore()
    table = {"컵은 깨지기 쉬우니까 앞으로 담지 마":
                 parsed("prohibit", ["cup"], reason="깨지기 쉬워서"),
             "컵 가져와": parsed(classes=["cup"], quantity="one"),
             "여기 있는 거 다 담아줘": parsed(quantity="all")}
    host, tier = build(TABLE_SCENE, table, store)
    drive(tier, host, "컵은 깨지기 쉬우니까 앞으로 담지 마")
    drive(tier, host, "컵 가져와")
    drive(tier, host, "응 가져와")
    assert store.is_forbidden("cup", "white"), "one exception is not a retraction"


# ------------------------------------------------------------- corrections

def test_correction_overwrites_instead_of_piling_on():
    store = RuleStore()
    scene = [SceneItem("cup_red", "cup", "red", rank=1),
             SceneItem("cup_white", "cup", "white", rank=2),
             SceneItem("apple", "apple", "red", rank=3)]
    table = {"컵은 깨지기 쉬우니까 앞으로 담지 마":
                 parsed("prohibit", ["cup"], reason="깨지기 쉬워서"),
             "아니 빨간 컵만 담지 마":
                 parsed("correct", ["cup"], ["red"], rep=["cup"], reason="깨지기 쉬워서"),
             "여기 있는 거 다 담아줘": parsed(quantity="all")}
    host, tier = build(scene, table, store)
    drive(tier, host, "컵은 깨지기 쉬우니까 앞으로 담지 마")
    drive(tier, host, "아니 빨간 컵만 담지 마")
    drive(tier, host, "여기 있는 거 다 담아줘")
    assert "cup_white" in host.picked, "the narrowed rule must free the white cup"
    assert "cup_red" not in host.picked


def test_correction_without_a_new_target_never_deletes():
    """The worst shape: old rule gone, nothing in its place, user told it worked."""
    store = RuleStore()
    table = {"컵은 담지 마": parsed("prohibit", ["cup"], reason="깨져서"),
             "아니 그게 아니라": parsed("correct", [], rep=["cup"])}
    host, tier = build(TABLE_SCENE, table, store)
    drive(tier, host, "컵은 담지 마")
    drive(tier, host, "아니 그게 아니라")
    assert store.is_forbidden("cup", "white"), "must not delete before escalating"
    assert host.escalated, "an unexpressible correction goes upstairs"


# ------------------------------------------------------------------ expiry

def test_stated_deadline_outranks_a_good_reason():
    store = RuleStore()
    host, tier = build(TABLE_SCENE, {
        "오늘은 컵이 젖어 있으니까 담지 마":
            parsed("prohibit", ["cup"], reason="젖어 있어서", scope="today")}, store)
    drive(tier, host, "오늘은 컵이 젖어 있으니까 담지 마")
    assert store.is_forbidden("cup", "white")
    store.end_session()
    assert not store.is_forbidden("cup", "white"), "'오늘은' must not outlive the session"


def test_yes_inside_a_dated_answer_is_not_a_standing_rule():
    """"오늘만 그렇게 해줘" reads as agreement and as a limit. The limit wins."""
    store = RuleStore()
    table = {"당분간 컵은 담지 마": parsed("prohibit", ["cup"], scope="ask"),
             "오늘만 그렇게 해줘": parsed("scope_answer")}
    host, tier = build(TABLE_SCENE, table, store)
    drive(tier, host, "당분간 컵은 담지 마")
    assert host.asked, "a vague duration must be asked about"
    drive(tier, host, "오늘만 그렇게 해줘")
    assert store.is_forbidden("cup", "white")
    store.end_session()
    assert not store.is_forbidden("cup", "white")


# ------------------------------------------------------------------ safety

def test_hazard_needs_confirmation_and_forgets_it():
    scene = [SceneItem("s1", "scissors", "black", rank=1)]
    store = RuleStore()
    table = {"가위 가져와": parsed(classes=["scissors"], quantity="one")}
    host, tier = build(scene, table, store)
    drive(tier, host, "가위 가져와")
    assert host.asked and host.picked == []
    drive(tier, host, "응 가져와")
    assert host.picked == ["s1"]

    # A new session must ask again -- the acknowledgement is not stored.
    host2, tier2 = build(scene, table, store)
    drive(tier2, host2, "가위 가져와")
    assert host2.asked and host2.picked == []


def test_ambiguous_single_pick_escalates_rather_than_guessing():
    scene = [SceneItem("a1", "apple", "red", rank=1),
             SceneItem("a2", "apple", "red", rank=2)]
    host, tier = build(scene, {"사과 하나 가져와": parsed(classes=["apple"], quantity="one")})
    drive(tier, host, "사과 하나 가져와")
    assert host.picked == []
    assert "ambiguous" in host.escalated


def test_a_stale_scene_does_not_cause_a_second_pick():
    """The executor and the camera run on different clocks. A just-taken object
    is often still in the newest snapshot, and trusting it picked the same
    apple twice on a real ROS graph."""
    scene = [SceneItem("a1", "apple", "red", rank=1),
             SceneItem("a2", "apple", "red", rank=2)]
    host, tier = build(scene, {"사과 다 담아줘": parsed(classes=["apple"], quantity="all")})
    host.pick = lambda oid, reason: host.picked.append(oid)   # scene never updates
    tier.handle("사과 다 담아줘")
    for _ in range(6):
        if not tier.busy:
            break
        tier.on_action_finished()
    assert host.picked == ["a1", "a2"], f"each object once, got {host.picked}"


def test_mid_mission_utterance_goes_upstairs():
    host, tier = build(TABLE_SCENE, {"다 담아줘": parsed(quantity="all")})
    tier.handle("다 담아줘")
    tier.handle("아니 그건 말고")
    assert "mission_interrupted" in host.escalated


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
