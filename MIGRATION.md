# 규칙 기반 파이프라인 → LLM 판단 구조

2026-08-07 기획서에 따라, 판단 로직을 코드의 상태머신에서 LLM으로 옮겼다.

## 무엇이 바뀌었나

| 항목 | 이전 (7-node ROS 2) | 현재 (4-node + LLM 판단) |
|---|---|---|
| LLM 개입 시점 | 명령 이해 단계 1회 | 사용자 발화 시 + 로봇 액션 완료 시 (결정 시점 기반, 다회) |
| 물건 매칭 | `matcher.py` 규칙 엔진 | LLM이 scene JSON 보고 직접 판단 |
| 다중 인스턴스 큐 | `target_queue` 상태머신 | 대화 히스토리 (LLM의 문맥 기억) |
| 모호성 해결 | `awaiting_clarification` 상태 + 코드 분기 | LLM이 스스로 `ask_clarification` 호출 |
| 명령 표현 | strict JSON schema task-rule (v3) | function calling |
| 노드 수 | 7 | 4 (perception / agent / robot / gui) |
| 정지 처리 | executor FIFO 큐 경유 | GUI 키워드 → `/vla/estop` → `move_stop` (LLM 우회) |
| 모션 중단 | 불가 (블로킹 `movel`) | 가능 (`amovel` + `check_motion` 폴링) |
| 지원 액션 | `pick`만 | `pick_and_place`, `pick_and_hold`, `release`, `go_home` |

## 파일 매핑

| 이전 | 현재 | 비고 |
|---|---|---|
| `nodes/yolo_node.py` + `nodes/scene_node.py` + `nodes/candidate_bridge_node.py` | `nodes/perception_node.py` | 세 단계(탐지→3D→base 변환)를 한 노드로. 중간 토픽 제거 |
| — | `perception/detector.py` | YOLO 로딩/추론과 ROS 이미지 헬퍼 분리 |
| `nodes/task_selector_node.py` | **제거** | LLM 판단으로 대체 |
| `nodes/mission_executor_node.py` | `nodes/robot_node.py` + `robot/moves.py` | FIFO 큐 제거, 단일 액션, 취소 가능 모션 |
| `DoosanPickHardware.execute()` | `robot/moves.py: DoosanArm` | `movel` → `amovel` + 폴링, 취소 지점 삽입 |
| — | `robot/moves.py: DryRunArm` | 하드웨어 없이 대화 흐름 검증용 |
| `core/gripper.py` | `robot/gripper.py` | 이동만 |
| `command/openai_gateway.py` | `agent/llm.py` | strict JSON schema → tool calling |
| `command/prompt.py` | `agent/prompt.py` | 파싱 지침 → 행동 지침 |
| `command/schema.py` | `agent/tools.py` | task-rule 스키마 → TOOLS 스키마 |
| — | `agent/conversation.py` | 대화 히스토리 + situation payload |
| — | `nodes/agent_node.py` | 결정 시점 이벤트 루프 |
| `command/cli.py`, `command/audio.py` | **제거** | GUI가 유일한 입력 경로 |
| `core/matcher.py`, `core/models.py`, `core/validation.py` | **제거** | 규칙 엔진 전체 |
| `core/mission.py` | `robot/moves.py: WorkspaceBounds` | 필요한 부분만 흡수 |
| `perception/semantics.py`, `config/object_attributes.json` | **제거** | 안전 속성 규칙 테이블. 필요하면 scene JSON 필드로 부활 가능 |
| `core/transform.py`, `perception/{tracker,color_features,depth_geometry}.py` | 그대로 유지 | 고정 로직 |

## 메시지 변경

12개 → 5개.

**제거**: `Detection2D`, `Detection2DArray`, `Detection3D`, `Detection3DArray`,
`ObjectAttributes`, `ClassifiedDetection3D`, `ClassifiedDetection3DArray`,
`TaskCandidate`, `TaskDecision`, `TaskDecisionArray`, `RobotTarget`, `MissionStatus`

**신규**:

| 메시지 | 방향 | 역할 |
|---|---|---|
| `SceneObject` / `SceneSnapshot` | perception → agent, robot, gui | LLM이 보는 세계 전부 |
| `RobotAction` | agent → robot | LLM이 고른 tool 호출 하나 |
| `RobotState` | robot → agent, gui | 팔의 실제 상태 (LLM 기억보다 우선) |
| `AgentReply` | agent → gui | 사용자에게 할 말 + 되묻기 후보 |

## 토픽 변경

**제거**: `/vla/active_rule`, `/vla/perception/detections_2d`,
`/vla/perception/scene`, `/vla/task/candidates`, `/vla/task/decisions`,
`/vla/robot/targets`, `/vla/robot/target_point`, `/vla/robot/mission_status`

**신규**: `/vla/user_utterance`, `/vla/estop`, `/vla/scene`,
`/vla/robot/action`, `/vla/robot/stop`, `/vla/robot/state`, `/vla/agent/reply`

**유지**: `/vla/perception/annotated_image`

## 설계 판단 몇 가지

**RobotAction에 좌표를 싣지 않는다.** `object_id`만 보내고, `vla_robot`이 움직이기
직전에 자기 scene 구독에서 최신 좌표를 다시 읽는다. LLM이 본 좌표는 이미 API
왕복 한 번만큼 오래됐다.

**액션을 큐에 쌓지 않는다.** 실행 중 도착한 두 번째 액션은 거부된다. 옛 FIFO 큐는
사용자가 마음을 바꾼 뒤에도 낡은 대상이 팔까지 도달할 수 있는 경로였다.

**`cancel_current_action`은 그 턴을 끝낸다.** 같은 라운드에서 정지와 새 모션을
연달아 발행하면 경쟁 상태가 된다 — 팔이 아직 busy라 거부되거나, 더 나쁘게는
제동 전에 새 명령을 받아들인다.

**파지 완료를 즉시 보고한다.** `pick()`의 `on_grasped` 콜백이 그리퍼가 닫힌 직후
발화한다. 들어올리는 중 정지가 들어와도 "무언가 쥐고 있다"는 사실이 유실되지 않는다.

**대화 히스토리는 턴 경계에서만 자른다.** 임의 인덱스로 자르면 짝 잃은
`function_call_output`이 남고, API가 그 요청을 통째로 거부한다.

**scene은 히스토리에 쌓지 않는다.** 매 호출마다 최신 것만 새로 붙인다. 비용도
비용이지만, 30초 전 좌표를 근거로 추론하는 쪽이 더 위험하다.

**모든 tool이 사용자에게 할 문장을 인자로 갖는다.** 자유 텍스트가 함께 올 것을
기대하지 않는다. 함수 호출만 반환하고 텍스트가 없는 응답은 흔하고, 그러면 팔이
이유 없이 멈춰 선 것처럼 보인다. `say`를 strict required로 두면 그 경우가 없다.

**완료 상태는 `current_action`을 비운 뒤 발행한다.** 아직 이름이 남아 있는
RobotState는 에이전트에게 "실행 중"으로 읽혀 걸러지고, 그 필터가 곧 다음 결정
시점을 만드는 유일한 장치다. 발행 후에 비우면 후속 판단이 통째로 사라진다 —
사과 하나 담고 영원히 대기하게 된다.

## 호환성 주의

- 기존 `/vla/...` 토픽을 직접 구독하던 외부 노드는 전부 새 토픽으로 옮겨야 한다
- task-rule JSON schema v3는 더 이상 존재하지 않는다
- `ros2 run vla_system vla_command` CLI는 제거됐다. 입력은 GUI 하나뿐이다
- wake word는 여전히 없다. 음성은 push-to-talk다

## 보안

정리 이전 원본 `.env`에 실제 OpenAI API 키가 들어 있었다. 이 워크스페이스에는
`.env.example`만 있다. 원본 압축본이 다른 위치로 복사된 적이 있다면 그 키는
폐기하고 새 키로 교체해야 한다.
