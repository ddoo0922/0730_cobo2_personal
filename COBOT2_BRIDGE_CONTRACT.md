<!-- meta
updated: 2026-08-10
status:  live — 값이 바뀌면 이 문서를 덮어쓴다 (append 하지 않는다, 히스토리는 안 남긴다)
owns:    cobot2_ws 가 관리. 정본은 항상 cobot2_ws `voice_processing/vla_command_node.py`
         (`parse_command()`) 코드다 — 이 문서는 그 요약이라 어긋나면 코드가 이긴다.
         "왜 이렇게 됐는지"(히스토리·근거)는 여기 없다 → cobot2_ws
         `md/plans/2026-08-08-vla-integration.md` (그 문서는 계속 불어나도 되는 로그,
         이 문서는 계약만 담는 요약이라 안 불어나야 한다 — 2026-08-10 사용자 지시로 분리)
-->

# cobot2_ws 브리지 계약 — `vla_pick_bridge`가 지금 맞춰야 하는 것

이 repo가 만들 `vla_pick_bridge`가 상대할 것은 cobot2_ws의 `vla_command_node`
하나뿐이다. 그 노드가 받아들이는 것/거부하는 것/부르지 않는 것을 여기 적는다.

## 1. 채널

```
이 ws → /vla/pick_command (std_msgs/String, JSON) → vla_command_node
이 ws ← /vla/pick_result  (std_msgs/String, JSON) ← vla_command_node
```

커스텀 msg 없음. `vla_interfaces`를 cobot2_ws에 가져가지 않는다 — 두 repo가 빌드 버전으로
묶이는 걸 피하기 위한 의도된 설계다.

## 2. `/vla/pick_command` 스키마

```json
{"cmd": "pick", "class": "apple", "place": "basket",
 "request_id": "a17-3", "stamp_ns": 1754640000123456789}
```

| 필드 | 규칙 |
|---|---|
| `cmd` | `pick` \| `pick_and_place`(같은 뜻으로 처리) \| `start` \| `abort` \| `reset`. 그 외 값은 거부 |
| `class` | **필수.** 공백 불가 — 있으면 거부(FSM이 응답을 공백으로 쪼개 첫 단어만 쓰기 때문). 여러 개는 콤마(`apple,orange`). `class_name`으로 보내도 받는다(SceneObject 필드명) |
| `place` | **선택.** `basket` \| `table` \| `discard` 중 하나만 허용, 그 외 값은 거부. 안 보내면 cobot2_ws 파라미터 기본값(`basket`)이 그대로 쓰인다 — §5 참고 |
| `request_id` | 그대로 echo. **결과 판정은 반드시 이걸로 대조** — 핫스팟류 연결 끊김 시 결과를 놓칠 수 있다(QoS VOLATILE) |
| `stamp_ns` | 에코만 됨. TTL은 cobot2_ws가 **수신 시각 기준**으로 계산하므로 두 PC 시계 동기화 불필요 |
| `pixel` + `pixel_wh` | **검증만 되고 선정에 안 쓰인다.** `pixel_policy=warn`(기본)이면 클래스만으로 진행 + 결과에 `ignored:["pixel"]`, `pixel_policy=reject`면 거부. `pixel`만 보내고 `pixel_wh` 안 보내면 무조건 거부 |
| `base_xy` | 무시됨(`ignored`로 회신), 검증도 없음 |
| `approve` 관련 필드 | **없다.** `cmd:"approve"`는 코드 경로 자체가 없어 무조건 거부됨 — §4 참고, 이 필드 자체를 스키마에서 빼는 게 맞다 |

## 3. `/vla/pick_result` 스키마

```json
{"request_id":"a17-3","accepted":true,"result":"succeeded",
 "reason":"...","ignored":[],"stamp_ns":null,"state":"HOME"}
```

`result` ∈ `rejected | accepted | succeeded | failed | superseded`. 성공 판정은 `RELEASE`
**진입이 아니라 그 다음 `HOME` 도달**이다 — `RELEASE → ABORT`도 허용된 전이라 `RELEASE` 만
보고 성공 처리하면 뒤따르는 실패를 놓친다.

## 4. 🔴 승인은 이 브리지가 절대 손대지 않는다 — cobot2_ws가 로컬로 처리한다 (2026-08-10)

**그립 승인 UX를 다시 설계할 필요가 없다.** cobot2_ws 쪽에서 graspgenx 판단 화면을 사람이
직접 보고, **버튼 또는 음성**으로 로컬에서 승인한다(`rqt_panel`의 '승인' 버튼 +
`approve_listener_node`의 음성 명령 — 둘 다 `/pick/approve`를 호출). 이 브리지는:

- `/pick/approve`를 **호출하지 않는다** (`vla_command_node`가 `cmd:"approve"`를 코드
  레벨로 거부하므로 보내도 무의미하다)
- LLM 툴(`agent/tools.py`)에 **승인 관련 툴을 추가하지 않는다** — `ask_clarification`이
  승인 대역을 겸하게 만들 필요 없음. `RobotState.status`에 `waiting_approval`류 값을
  새로 만들지 여부는 여전히 열려 있는 질문(UX상 LLM이 "지금 사람 승인 대기 중"을 알면
  좋다는 점은 남아있음)이지만, **승인 자체를 자동화하는 경로는 만들지 않는다**는 게
  유일한 하드 제약이다.

## 5. `place` 값 대응표 (cobot2_ws 쪽, 참고용)

| 값 | 뜻 | 실기 teach 상태 |
|---|---|---|
| `basket` | 장바구니 | ✅ teach 완료 |
| `table` | 작업테이블 지정 자리 | 🔴 **UNVERIFIED** — home 관절값을 임시로 복사해 둔 자리표시자. 실기에서 안전한 자세로 다시 잡기 전까지 이 값을 실제로 쓰지 말 것 |
| `discard` | 테이블 밖 폐기 | 🔴 **UNVERIFIED** — 위와 동일 |

## 6. `class` 허용 목록 불일치 — 지금 그대로 붙이면 일부 거부된다

cobot2_ws는 `allowed_classes`로 들어온 클래스를 검사해서 밖의 이름을 거부한다(정확한 원인
메시지와 함께). 2026-08-10 기준 실측:

| | 목록 |
|---|---|
| cobot2_ws `config/objects.yaml` (`detect`) | `bottle, cup, spoon, banana, apple, orange, mouse` |
| 이 ws `system.yaml` (`target_classes`, webcam+wrist 공통) | `apple, banana, orange, cup, bottle, wine glass, book` |

**겹침 5개**(`apple, banana, orange, cup, bottle`) — 이것만 지금 바로 통과한다.
**`wine glass`, `book`은 지금 상태로 브리지를 붙이면 즉시 거부된다.** cobot2_ws의
`yolo11n-seg.pt`가 COCO 80종을 아니까(`wine glass`/`book` 둘 다 COCO 클래스) `detect`에
추가하는 건 가능 — 필요하면 cobot2_ws 쪽에 요청할 것 (`config/objects.yaml` 한 줄 추가 +
`yolo_seg_node` 재기동).

## 7. LLM 툴 스키마 — 지금 안 맞는 부분

- `pick_and_place` 스키마에 **`place`/destination 인자가 없다** — §2의 `place` 필드를
  쓰려면 이 스키마에 인자를 추가하고, 브리지가 `RobotAction`의 그 값을 JSON `place`로
  옮겨야 한다.
- `pick_and_hold`/`release` 툴은 **매핑할 데가 없다** — FSM은 항상 `place`까지 간다
  ("들고만 대기"·"현재 위치에 놓기" 개념이 cobot2_ws 쪽에 없음). 이 둘을 프롬프트·스키마
  ·`test_tools_schema.py`에서 빼지 않으면 LLM이 부를 때마다 브리지가 거부만 반복한다.

## 8. cobot2_ws 쪽에 아직 없는 것 (기다리지 않아도 됨)

- `select_by_point()` — `pixel` 좌표로 특정 개체를 찍어 집는 기능. 아직 미구현이라
  `pixel`을 보내도 선정에 안 쓰인다(§2 표). 되묻기("1번")가 실제로 그 개체를 집어야 하는지
  여부는 cobot2_ws 쪽 결정 사항 — 필요 없으면(씬에 같은 클래스 1개만 두는 데모로 범위를
  좁히면) 지금 그대로 접합 가능하다.
