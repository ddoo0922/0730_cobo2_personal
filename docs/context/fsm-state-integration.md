# FSM 상태 가시성 연동 — VLA 쪽 요청서 (2026-08-11)

> 목적: cobot2_ws `pick_fsm`이 지금 어느 단계인지(VERIFY/LIFT/PLACE/승인대기 등)를
> VLA GUI·에이전트가 실시간으로 볼 수 있게 한다. 현재는 `/vla/pick_result`의 결과
> 경계(accepted/succeeded/failed/rejected)에서만 상태를 받아, 그 사이 진행을 못 본다.
>
> 이 문서는 **VLA 쪽이 무엇을 하고 cobot2_ws에 무엇만 확인하면 되는지**를 정리한 것.
> 계약 정본은 `~/cobot2_ws/md/vla-bridge-contract.md`다(이 문서는 사본이 아니라
> VLA 쪽 구현 노트 + 확인 요청 목록).

## 0. 결론 먼저 — cobot2_ws는 새로 만들 게 없다

`pick_fsm/task_manager.py`가 **이미 `/pick/state`(std_msgs/String)를 매 상태 전이마다
발행**하고 있다(`_to()` → `state_pub.publish(String(data=nxt.name))`, 기본 QoS depth 10).
값은 `pick_fsm/states.py`의 `State` enum 이름 그대로다(`"LIFT"`, `"WAIT_APPROVAL"` 등).

즉 **연동에 필요한 토픽은 이미 존재한다.** VLA 쪽 `vla_pick_bridge_node`가 그걸
구독만 하면 된다 — 이번 변경은 전부 VLA 쪽이다.

## 1. VLA가 구독할 것 (cobot2_ws는 확인만)

| 토픽 | 타입 | QoS | VLA가 하는 일 |
|---|---|---|---|
| `/pick/state` | `std_msgs/String` (`State` enum 이름) | RELIABLE/VOLATILE/depth 10 (task_manager 기본값) | FSM 단계 → `RobotState.status`/`details`/`holding`로 매핑해 `/vla/robot/state` 재발행 |

**cobot2_ws(FSM) 세션에 확인 요청 — 이 4개만:**

1. `/pick/state`의 QoS가 정말 기본값(RELIABLE·VOLATILE·depth 10)인가? VLA는 그 가정으로
   구독한다 — 다르면 조용히 아무 메시지도 안 온다(ROS 2 QoS 불일치 전형).
2. 값은 항상 `State.name` 문자열 그대로인가?(JSON 아님, 접두사 없음) VLA는 대소문자
   그대로 매칭한다.
3. **request_id가 없다** — VLA는 "한 번에 하나만 진행"(task_manager 단일 처리 +
   `vla_pick_bridge`의 `pending_action` 단일)이라는 전제로 **위치 기반 상관**을 한다.
   FSM이 두 pick을 동시에 돌리는 경로가 있나? 없으면 위치 기반으로 충분하고, 있으면
   `/pick/state`에 request_id를 실어줘야 한다.
4. `WAIT_APPROVAL`도 `/pick/state`로 나오는가?(전이표상 `PLAN → WAIT_APPROVAL`이라 나와야
   맞다) VLA는 이걸 잡아 "지금 사람 승인 대기 중"을 사용자에게 보여줄 계획이다 —
   **승인 자체를 자동화하지 않는다**(계약 §4 하드 제약 그대로 유지, `/pick/approve`
   안 부른다).

## 2. (선택) 추가로 붙이면 좋은 cobot2_ws 토픽 — 나중에

지금 당장은 `/pick/state` 하나면 되지만, 아래도 이미 발행 중이라 원하면 나중에 붙인다:

| 토픽 | 타입 | 쓸모 |
|---|---|---|
| `/pick/robot_state_text` / `/pick/robot_state_code` | String / Int8 (`robot_safety_node`) | 물리 팔 안전상태(estop/충돌) — GUI 경고용 |
| `/pick/target_active` | String, TRANSIENT_LOCAL | 지금 FSM이 잡는 대상 class 확인용 |
| `/pick/place_location_active` | String, TRANSIENT_LOCAL | 지금 목적지(basket/table/discard) 확인용 |

이건 이번 범위 밖. 확인 요청 아님 — 기록만.

## 3. VLA 쪽 구현 (cobot2_ws 변경 불필요)

`vla_pick_bridge_node` + `bridge/pick_bridge.py`(순수 매핑 함수):

1. `fsm_state_topic`(기본 `/pick/state`) 파라미터 신설, 구독.
2. `State` 이름 → 매핑:
   - `RobotState.status`: `idle` | `moving` | `holding` | `waiting_approval` | `error`
     (기존 어휘 + `waiting_approval` 하나 추가)
   - `RobotState.details`: 사람이 읽을 단계 라벨(예: "들어올리는 중", "놓는 중", "승인 대기")
   - `holding`: FSM이 `HOLDING_STATES = {VERIFY, LIFT, PLACE, PLACE_RETRY}`에 있으면,
     VLA가 **자기가 보낸 `pending_action`의 object_id/class**로 채운다 — cobot2_ws가
     holding 정보를 안 보내도 되는 이유(계약 §3의 "pick_result에 holding 필드 없음"
     공백을 이렇게 메운다).
3. **에이전트를 깨우지 않는다**: 진행 중 상태는 `current_action`을 채운 채
   `last_result`는 비워 발행 → `agent_node`가 `if not last_result or current_action:
   return`으로 무시한다(라이브 상태는 GUI 표시 전용, 판단 재요청 아님). 결정 시점은
   지금처럼 terminal `/vla/pick_result`만.
4. GUI `robot_state_line`이 진행 중 `details`(단계 라벨)를 함께 보여주도록 확장.

## 4. 열린 결정 (사용자/FSM 확인 대기)

- **A. `RobotState.status`에 `waiting_approval` 추가** — GUI·에이전트 프롬프트가 이 값을
  이해해야 함. 추가할지, 아니면 status는 coarse하게 두고 승인대기를 `details`로만 볼지.
- **B. `/pick/state` QoS/포맷/동시성** — §1의 확인 4개.
- **C. 승인 대기를 에이전트에게도 알릴지** — 알면 "지금 승인 기다리는 중이에요"라고
  사용자에게 말해줄 수 있음(계약 §4가 남긴 열린 질문). 자동 승인은 여전히 안 함.
