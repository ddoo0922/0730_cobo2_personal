"""The complete set of things the model is allowed to do.

This list *is* the control logic. There is no rule engine behind it deciding
which object matches "the red one" or how many apples "the apples" means --
the model reads the scene and calls one of these.

Schemas are in the flat Responses-API shape (``type``/``name`` at the top
level, not nested under ``function``).
"""

# Actions that hand control to the arm. The agent stops its tool loop after
# one of these: the next decision point is the action *completing*, which
# arrives as a robot state event, not as another tool round.
MOTION_TOOLS = ("pick_and_place", "pick_and_hold", "release")

# Actions that end the turn without moving anything.
TERMINAL_TOOLS = ("ask_clarification", "wait")


def _say_argument(description: str) -> dict:
    """Every tool carries the sentence the user hears.

    Making it a required argument rather than hoping for free text alongside
    the call is the difference between the robot explaining itself and the
    robot going silent: a model that answers with a bare function call and no
    message leaves the user watching an arm stop for no stated reason.
    """
    return {
        "type": "object",
        "properties": {"say": {"type": "string", "description": description}},
        "required": ["say"],
        "additionalProperties": False,
    }


def _object_argument(description: str) -> dict:
    return {
        "type": "object",
        "properties": {
            "object_id": {"type": "string", "description": description},
            "say": {
                "type": "string",
                "description": "무엇을 왜 집는지 사용자에게 할 한 문장. 그대로 들린다.",
            },
        },
        "required": ["object_id", "say"],
        "additionalProperties": False,
    }


# vla-bridge-contract.md #5. table/discard 값 자체는 여기서 막지 않는다 -- 실기
# 검증(teach) 여부는 하드웨어 사정이지 스키마 사정이 아니다. 대신
# vla_pick_bridge_node가 allow_unverified_place(기본 false)로 실행을 막는다:
# 모델이 골라도 되고, 브리지가 실제로 보낼지는 따로 판단한다.
PLACE_VALUES = ("basket", "table", "discard")


def _pick_and_place_argument() -> dict:
    return {
        "type": "object",
        "properties": {
            "object_id": {
                "type": "string",
                "description": "scene의 visible_objects에 있는 id를 그대로 쓴다. 예: apple_17",
            },
            "place": {
                "type": "string",
                "enum": list(PLACE_VALUES),
                "description": (
                    "어디에 놓을지. basket=장바구니(사용자가 목적지를 말하지 않았으면 "
                    "이걸 골라라), table=작업테이블 지정 자리, discard=폐기 자리. "
                    "table/discard는 사용자가 명시적으로 그 목적지를 말했을 때만 골라라 "
                    "-- 아직 실기에서 검증되지 않아 브리지가 거부할 수 있고, 그러면 "
                    "그 사실을 그대로 설명해라."
                ),
            },
            "say": {
                "type": "string",
                "description": "무엇을 왜 집는지 사용자에게 할 한 문장. 그대로 들린다.",
            },
        },
        "required": ["object_id", "place", "say"],
        "additionalProperties": False,
    }


TOOLS = [
    {
        "type": "function",
        "name": "pick_and_place",
        "description": (
            "지정한 물체를 집어서 지정한 곳에 놓는다. 사용자가 담으라고/치우라고 명시한 "
            "물체에만 사용한다. 한 번에 하나만 호출할 수 있고, 동작이 끝나면 다시 판단 "
            "기회가 주어진다."
        ),
        "parameters": _pick_and_place_argument(),
        "strict": True,
    },
    {
        "type": "function",
        "name": "pick_and_hold",
        "description": (
            "지정한 물체를 집어서 든 채로 대기한다. 사용자가 직접 건네받으려 하거나 "
            "담을지 말지 아직 정하지 않았을 때 쓴다."
        ),
        "parameters": _object_argument(
            "scene의 visible_objects에 있는 id를 그대로 쓴다. 예: apple_17"
        ),
        "strict": True,
    },
    {
        "type": "function",
        "name": "release",
        "description": (
            "지금 들고 있는 물체를 현재 위치에서 놓는다. robot_state.holding이 "
            "비어 있으면 호출하지 마라."
        ),
        "parameters": _say_argument("무엇을 왜 내려놓는지 사용자에게 할 한 문장."),
        "strict": True,
    },
    {
        "type": "function",
        "name": "cancel_current_action",
        "description": (
            "진행 중인 동작을 즉시 중단한다. 사용자가 방금 지시를 철회했거나 "
            "지금 향하고 있는 대상이 잘못됐다고 판단되면 다른 무엇보다 먼저 호출한다."
        ),
        "parameters": _say_argument("왜 멈추는지 사용자에게 할 한 문장."),
        "strict": True,
    },
    {
        "type": "function",
        "name": "ask_clarification",
        "description": (
            "어떤 물체를 말하는지 애매하면 절대 추측하지 말고 이것을 호출해 되묻는다. "
            "후보 물체의 id를 함께 넘기면 화면에 번호가 붙은 사진으로 제시된다."
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "question": {
                    "type": "string",
                    "description": "사용자에게 물어볼 한 문장.",
                },
                "object_ids": {
                    "type": "array",
                    "items": {"type": "string"},
                    "description": (
                        "헷갈리는 후보들의 id. 사용자가 '1번'이라고 답하면 이 배열의 "
                        "첫 번째를 가리킨다. 후보를 제시할 필요가 없으면 빈 배열."
                    ),
                },
            },
            "required": ["question", "object_ids"],
            "additionalProperties": False,
        },
        "strict": True,
    },
    {
        "type": "function",
        "name": "wait",
        "description": (
            "지금 할 일이 없다. 사용자가 시킨 일을 다 했거나, 아직 아무 지시도 "
            "받지 않았거나, 다음 지시를 기다려야 할 때 호출한다."
        ),
        "parameters": _say_argument(
            "지금 상황과 무엇을 기다리는지 사용자에게 할 한 문장. "
            "예: '사과 담았어요. 더 필요한 거 있으세요?'"
        ),
        "strict": True,
    },
]

TOOL_NAMES = tuple(tool["name"] for tool in TOOLS)
