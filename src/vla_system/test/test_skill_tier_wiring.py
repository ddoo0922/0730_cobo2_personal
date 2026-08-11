"""AgentNode as a SkillHost: does the rule layer actually reach ROS?

test_skill_tier.py proves the decision logic against a fake host. This proves
the other half -- that the node satisfies the seam the logic expects, and that
the parameter really does turn the whole thing off.

Nothing here calls an API. The parser is replaced with a table and the
publishers are captured, so a failure is a wiring failure.
"""

import sys
from pathlib import Path

import pytest
import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vla_interfaces.msg import RobotState, SceneObject, SceneSnapshot   # noqa: E402
from vla_system.agent.conversation import scene_to_payload              # noqa: E402
from vla_system.agent.rules import RuleStore                            # noqa: E402
from vla_system.agent.skill_tier import SkillTier                       # noqa: E402
from vla_system.nodes.agent_node import AgentNode                       # noqa: E402


def parsed(intent="pick", classes=(), colors=(), ex=(), exc=(), rep=(),
           quantity="all", count=None, scope="now", reason=""):
    return {"intent": intent, "classes": list(classes), "colors": list(colors),
            "exclude_classes": list(ex), "exclude_colors": list(exc),
            "replaces": list(rep), "quantity": quantity, "count": count,
            "scope": scope, "reason": reason}


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def scene(*objects) -> SceneSnapshot:
    message = SceneSnapshot()
    for index, (object_id, class_name, color) in enumerate(objects, 1):
        item = SceneObject()
        item.id = object_id
        item.class_name = class_name
        item.color = color
        item.track_id = index
        item.position_valid = True
        message.objects.append(item)
    return message


class Recorder:
    """Captures what the node would have published."""

    def __init__(self, node):
        self.actions, self.replies, self.llm_turns = [], [], []
        node.publish_action = lambda name, oid, reason, place="": (
            self.actions.append((name, oid, place)) or "id")
        node.publish_reply = lambda kind, text, ids=None: self.replies.append((kind, text))
        node.decide = lambda event: self.llm_turns.append(event)


def build(table, enabled=True):
    node = AgentNode()
    node.set_parameters([
        rclpy.parameter.Parameter("skill_tier_enabled",
                                  rclpy.Parameter.Type.BOOL, enabled),
        rclpy.parameter.Parameter("rule_store_path",
                                  rclpy.Parameter.Type.STRING, ""),
    ])
    node.close()                      # stop the worker; tests drive route() directly
    recorder = Recorder(node)
    if enabled:
        node.skill_tier = SkillTier(node, RuleStore(),
                                    lambda t: table.get(t, parsed(intent="other")))
    return node, recorder


def drive(node, recorder, text):
    node.route({"type": "user_said", "text": text})
    for _ in range(10):
        if node.skill_tier is None or not node.skill_tier.busy:
            break
        # A real robot would report the result; here the object simply leaves.
        done = recorder.actions[-1][1]
        with node.lock:
            node.scene.objects = [o for o in node.scene.objects if o.id != done]
        node.route({"type": "action_finished", "action": "pick_and_place",
                    "result": "succeeded", "detail": ""})


def test_rule_layer_publishes_actions_without_touching_the_llm():
    node, rec = build({"사과 다 담아줘": parsed(classes=["apple"], quantity="all")})
    node.scene = scene(("cup_1", "cup", "white"), ("apple_2", "apple", "red"))
    node.robot_state = RobotState()
    drive(node, rec, "사과 다 담아줘")
    assert [a[1] for a in rec.actions] == ["apple_2"]
    assert rec.llm_turns == [], "a rule-handled utterance must not reach the LLM"
    node.destroy_node()


def test_place_is_left_empty_for_the_bridge_default():
    node, rec = build({"사과 다 담아줘": parsed(classes=["apple"], quantity="all")})
    node.scene = scene(("apple_2", "apple", "red"))
    node.robot_state = RobotState()
    drive(node, rec, "사과 다 담아줘")
    assert rec.actions[0][2] == "", "the rule layer has no destination slot"
    node.destroy_node()


def test_unexpressible_utterance_falls_through_to_the_llm():
    node, rec = build({})           # every lookup misses -> intent=other
    node.scene = scene(("apple_2", "apple", "red"))
    node.robot_state = RobotState()
    node.route({"type": "user_said", "text": "아까 그거 말고 저쪽 거"})
    assert rec.llm_turns, "Tier 1 must hand over what it cannot express"
    assert rec.actions == []
    node.destroy_node()


def test_clarification_reaches_the_reply_topic():
    node, rec = build({"가위 가져와": parsed(classes=["scissors"], quantity="one")})
    node.scene = scene(("s1", "scissors", "black"))
    node.robot_state = RobotState()
    node.route({"type": "user_said", "text": "가위 가져와"})
    kinds = [k for k, _ in rec.replies]
    assert "ask_clarification" in kinds
    assert rec.actions == [], "a hazard must not move before confirmation"
    node.destroy_node()


def test_parameter_off_restores_the_original_path():
    node, rec = build({"사과 다 담아줘": parsed(classes=["apple"])}, enabled=False)
    node.scene = scene(("apple_2", "apple", "red"))
    node.robot_state = RobotState()
    node.route({"type": "user_said", "text": "사과 다 담아줘"})
    assert rec.llm_turns, "with the tier off every utterance goes to the LLM"
    assert rec.actions == []
    assert node.get_skill_tier() is None
    node.destroy_node()


def test_scene_items_carry_what_the_rules_filter_on():
    node, _ = build({})
    node.scene = scene(("cup_1", "cup", "white"), ("apple_2", "apple", "red"))
    items = node.scene_items()
    assert [i.object_id for i in items] == ["cup_1", "apple_2"]
    assert [i.class_name for i in items] == ["cup", "apple"]
    assert [i.color for i in items] == ["white", "red"]
    assert [i.rank for i in items] == [1, 2]
    node.destroy_node()


def test_both_layers_agree_on_what_can_be_picked():
    """두 계층이 여기서 갈라지면 아무것도 안 터지고 Tier 1만 조용히 멈춘다.

    2026-08-11 병합에서 실제로 갈라졌다: 대화 경로는 "전부 집을 수 있다"로
    바뀌었는데(cobot2_ws가 클래스 이름만으로 좌표를 계산한다) 규칙 계층은
    이 ws의 `position_valid`를 계속 보고 있었다. 테이블 보정이 없는 실기
    구성에서 Tier 1은 후보를 하나도 못 찾고 "담을 게 없습니다"라고만 한다.
    """
    node, _ = build({})
    snapshot = scene(("cup_1", "cup", "white"), ("apple_2", "apple", "red"))
    # 이 ws가 3D 위치를 못 잡은 상태 -- cobot2_ws 연동에서는 정상이다.
    for scene_object in snapshot.objects:
        scene_object.position_valid = False
    node.scene = snapshot

    payload = scene_to_payload(snapshot)
    from_llm = [o["pickable"] for o in payload["visible_objects"]]
    from_rules = [i.pickable for i in node.scene_items()]
    assert from_rules == from_llm, (
        f"두 계층의 pickable이 다르다: 규칙={from_rules} 대화={from_llm}")
    node.destroy_node()


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
