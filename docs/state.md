# 세션 상태 — M0609_VLA_system

마지막 갱신: 2026-08-11. 이 파일은 현재 상태로 덮어쓴다 — 로그처럼 쌓지 않는다.
(아래 "2026-08-10" 라벨이 붙은 절들은 그 세션 시점 기록 — 숫자 등은 이후 절이 최신)

## README 동기화 + 의존성 재점검 (2026-08-10, 이번 세션 마지막 작업)

- README를 이 문서·`vla_pick_bridge` 존재·카메라 정정·테스트 개수(265)·실행 명령에
  맞춰 갱신 완료.
- `requirements.txt` 설치 상태 재확인: `.venv` 안 버전 전부 범위 안(numpy 1.26.4,
  opencv-python 4.11, ultralytics 8.4, openvino 2026.3, pymodbus 2.5.3 등), `--user`
  오염 없음(단, `~/.local`에 **이 ws가 만들지 않은** 이전 잔해 발견 — 아래).
  cobot2_ws `colcon build --packages-select voice_processing pick_fsm` 재확인 PASS.
- 🔴 `~/.local/lib/python3.10/site-packages/`에 `anyio`(깨진 잔해)·`sounddevice`
  (중복)·`nvidia`/`cuda`(부분 설치) 발견 — CLAUDE.md §1이 경고하는 패턴과 정확히
  일치. 지금 당장 뭔가를 깨고 있진 않지만(pytest/colcon build 둘 다 정상), 방치할
  이유는 없다. 상세·정리 전 확인 절차는 docs/context/constraints.md 참고 — **삭제는
  안 함, 사용자 확인 필요.**

## 지금 검증된 것 — 로봇/그리퍼 제외 launch·run 점검 (2026-08-10)

`colcon build` 및 `ros2 run`/`ros2 launch`로 실제 기동해서 확인. 상세 원인은
docs/context/constraints.md 참고.

| 노드/실행 경로 | 상태 |
|---|---|
| `colcon build` (`scripts/build.sh` 수정 후) | PASS |
| `perception_node` 단독 | 정상 — webcam 오픈, YOLO-seg CUDA 추론, 15Hz 루프 지속 |
| `vla_system.launch.py` (perception+agent, `enable_robot:=false` 기본값) | 정상 — 두 노드 동시 기동·지속 |
| `agent_node` | 기동까지만 확인(`AgentLLM`은 지연 생성이라 API 키 없이도 뜸). 실제 LLM 왕복 미검증 — 이 ws에 `OPENAI_API_KEY`/`.env` 없음 |
| `wrist_grasp_node` (`enable_wrist_grasp:=true`) | 기동 정상. GraspGenX 저장소 없어 파지계획은 우아하게 거부(README 문서화된 fallback) |
| `vla_gui` | 정상 — Tk 창("VLA Robot Console")이 실제로 뜨는 것을 `xwininfo`로 확인 |
| `table_homography_test` | executable 등록·`dsr_msgs2` import까지만 확인. 로봇 조그가 필요한 대화형 도구라 미실행(범위 밖) |
| `robot_node` / `robot/gripper.py` | 미실행 (요청 범위 제외). launch 기본값이 `enable_robot:=false`인 것만 확인 |
| `./scripts/check.sh` | ROS+install 소싱 후 249 passed. README와 달리 ROS 없이는 실패함 — docs/context/constraints.md 참고 |

## 이번에 고친 것

- `scripts/build.sh` — `colcon build`가 `.venv`를 무시하고 시스템 python3로 노드를
  빌드해버리는 문제 수정 (`.venv` source + `python3 -m colcon build`). 상세는
  docs/context/constraints.md. **사용자가 직접 커밋 예정, 아직 워킹트리에만 있음.**

## 다음에 볼 때 먼저 확인할 것

1. **카메라 구성 재확정** — 고정 카메라가 cobot2_ws FSM과 공유하는 D435i라는 것은
   확인됐지만, 손목 RealSense 구성은 미정. docs/context/constraints.md "카메라 구성" 항목부터 볼 것.
2. `vla_perception.webcam_device`가 실제 로봇 머신에서 맞는 값인지 (카메라 구성이
   바뀌었으므로 재확인 대상).
3. `robot_node`/`gripper.py` 실기 연동은 이번 점검 범위 밖 — 여전히 미검증.

## cobot2_ws 통합 — 이 ws가 필요한 것 / 확인해야 하는 것 (2026-08-10)

원문은 `~/cobot2_ws/md/plans/2026-08-08-vla-integration.md`(§9가 병합된 최종 병합
체크리스트)와 `~/cobot2_ws/md/vla-bridge-contract.md`(계약 요약, 단일 사본) — **여기엔
사본을 두지 않는다(CLAUDE.md §2). 갱신되면 절대경로로 다시 읽을 것.** 아래는 그 문서들을
읽고 이 ws 기준으로 뽑은 액션 체크리스트다.

### ✅ `vla_pick_bridge` 구현 완료 (2026-08-10) — MVP (`place`/`pixel`은 아래 절 참고 — 이후 추가됨)

`nodes/vla_pick_bridge_node.py` + 순수 로직 `bridge/pick_bridge.py`. `enable_pick_bridge:=true`
로 opt-in(`enable_robot`과 상호 배타 — launch 파일 docstring 참고). 검증: `./scripts/check.sh`
(순수 로직 16개 신규 테스트, 265 passed) + 실제 rclpy로 `RobotAction`→`/vla/pick_command`→
가짜 `/vla/pick_result`(accepted→succeeded)→`RobotState`, `pick_and_hold` 로컬 거부, `/vla/estop`→
abort 전달까지 왕복 스모크 확인.

- **입력**: `/vla/robot/action`(`RobotAction`), `/vla/robot/stop`, `/vla/estop`, `/vla/scene`(object_id→class 조회용)
- **출력**: `/vla/pick_command`(JSON) 발행, `/vla/pick_result`(JSON)를 받아 `/vla/robot/state`(`RobotState`)로 역변환
- **핵심 변환**: `object_id`(예: `apple_17`) → `class`(예: `apple`) — 개체 단위 id는 경계를 못 넘고 클래스만 넘는다
- **result 매핑**은 cobot2_ws 문서(§3-3)의 표 그대로: `accepted`→진행중(`moving`, 비종결), `succeeded`→`succeeded`,
  `failed`→`failed`, `rejected`/`superseded`→`rejected`(둘 다 같은 값 — 문서가 그렇게 정함, 추측 아님)
- **아직 안 하는 것 (의도적 범위)**:
  - `holding_object_id`/`class`를 못 채움 — `/vla/pick_result`엔 그 정보가 없음. 필요하면 cobot2_ws `/pick/state`(VERIFY/LIFT/PLACE) 구독을 추가해야 함(안 함)
  - `승인` 관련 툴/경로 없음 — 의도적(아래 표)

### ✅ GUI 기본값을 cobot2_ws 연동으로 전환 (2026-08-11)

`vla_gui.py`에 "cobot2_ws FSM 연동" 체크박스 추가, **기본 켜짐**(`pick_bridge_var =
True`). 켜진 채로 "VLA 시작"을 누르면 `enable_pick_bridge:=true` +
`enable_realsense:=false`(카메라는 cobot2_ws 쪽 launch가 이미 잡고 있다는 전제,
README §4)를 같이 보낸다. 꺼야만 예전 기본 동작(이 ws가 카메라 직접 열고
`enable_robot`/`pick_bridge` 모두 꺼진 단독 모드)으로 돌아간다. `enable_robot`은
여전히 GUI에서 켤 방법이 없음(변경 없음). 파이프라인 시작 로그에 어느 모드인지
채팅으로 표시하도록 추가. 빌드 PASS, `check.sh` 278 passed.

**중요**: 이 변경은 "UI + launch 하나"까지만 자동화한다 — `enable_pick_bridge:=true`
만으로는 cobot2_ws의 `pick_fsm`이 자동으로 돌지 않는다(§3/§4에서 이미 문서화된
`auto_start`/`/pick/start` 제약 그대로 유효, 이 세션에서 새로 검증한 것 아님).
**GUI+pick_bridge 조합의 실기 왕복 테스트는 아직 안 함** — 다음에 테스트할 때
`ROS_DOMAIN_ID`를 cobot2_ws 기본값(93)이 아닌 별도 도메인에서 먼저 해보라는
사용자 요청 있었음(2026-08-11) — 이유는 대화에 명시 안 됨, 아마 실제 cobot2_ws
프로세스와 충돌 없이 GUI 동작만 먼저 확인하려는 의도로 추정.

### ✅ 카메라 토픽 구독 전환 + `pixel`/`pixel_wh` 전송 시작 (2026-08-11)

**카메라**: `vla_perception`이 `cv2.VideoCapture(webcam_device)` 직접 오픈 대신
`image_topic`(기본 `/camera/camera/color/image_raw`) **구독**으로 전환 —
`docs/context/constraints.md` "카메라 구성"에서 두 프로세스의 독립 V4L2 오픈이 충돌
위험이라고 확인한 것의 후속 조치. `system.yaml`/README 갱신, 실제 rclpy로 가짜
`Image`→`/vla/scene` 왕복 확인함(`total_frames=1`, `camera_frame=shared_camera`).
표준 `image_message_to_bgr()`(cv_bridge 없이, 기존 `detector.py` 헬퍼 재사용)로 변환.

**`pixel`/`pixel_wh`**: 카메라가 진짜 공유(같은 물리 D435i)라는 게 확정된 데다 이제
이 ws도 그 카메라를 토픽으로 구독하므로, `vla_pick_bridge`가 보내는 픽셀 좌표가
cobot2_ws의 세그멘테이션과 **재투영 없이 같은 좌표계**를 갖게 됐다. `SceneSnapshot`에
`image_width`/`image_height` 필드 추가(`vla_interfaces`), `vla_pick_bridge_node`가
`SceneObject`의 bbox 중심(`bbox_center()`, `bridge/pick_bridge.py`)을 `pixel`로,
그 프레임 해상도를 `pixel_wh`로 실어 `/vla/pick_command`에 포함. cobot2_ws의
`vla_command_node`를 실제로 띄워 왕복 확인함 — `pixel_policy=warn` 상태로
`ignored:["pixel"]` 응답까지 재현. cobot2_ws가 `select_by_point()`를 구현해도 이 ws
쪽은 추가 변경 불필요.

**cobot2_ws 쪽에 전달함**: `~/cobot2_ws/md/vla-bridge-contract.md` §2/§7/§9에
`select_by_point()` 구체 설계(point-in-mask → centroid-거리 fallback → 임계값 초과
시 거부)를 제안으로 작성해뒀다 — cobot2_ws 세션의 검토/구현 대기.

### ✅ `place` 인자 추가 완료 (2026-08-10)

`RobotAction.place`(`vla_interfaces`) → `agent/tools.py`의 `pick_and_place` 스키마(필수,
enum `basket|table|discard`) → `agent_node.dispatch()`가 `RobotAction.place`에 실음 →
`vla_pick_bridge_node`가 검증 후 `pick_bridge.build_pick_command`로 `/vla/pick_command`
JSON `place` 필드에 실어 보냄. 소스: `~/cobot2_ws/md/vla-bridge-contract.md` §2/§5/§7,
`~/cobot2_ws/src/pick_fsm/pick_fsm/task_manager.py`(`PLACE_LOCATIONS`),
`~/cobot2_ws/src/voice_processing/voice_processing/vla_command_node.py`(`PLACE_VALUES`).

- **LLM 프롬프트 규칙**: 목적지 미언급 시 `basket`, 사용자가 명시했을 때만 `table`/`discard`
  (`agent/prompt.py` #9)
- **🔴 안전 게이트**: `table`/`discard`는 contract §5 기준 아직 실기 미검증(placeholder
  관절값)이라 `vla_pick_bridge_node`의 새 파라미터 `allow_unverified_place`(기본
  `false`)로 막아둠 — LLM이 골라도 브리지가 거부하고 그 사유를 사용자에게 그대로 전달.
  teach 완료되면 파라미터만 뒤집으면 됨(코드 변경 불필요)
- **검증**: `./scripts/check.sh` 순수 로직 271 passed (신규: `place_rejection_reason`
  4종 + 스키마 enum 1종). `colcon build --packages-select vla_interfaces vla_system` 둘
  다 PASS. **실기 미검증** — 실제 cobot2_ws와의 왕복(특히 `table`/`discard` 거부 경로)은
  아직 스모크 테스트 안 함

### 🔴 `auto_start` — cobot2_ws 쪽에서 켜야 하는 것, 이 ws가 대신 켤 수 없음 (2026-08-11)

`vla_pick_bridge`가 `/vla/pick_command`를 쏴도 cobot2_ws의 `pick_fsm`은 `IDLE`에서
`/pick/start`가 불릴 때까지 그대로 멈춰 있다(`pick_fsm/states.py`) — 기본 구성
(cobot2_ws의 `vla_command_node` `auto_start=false`)에서는 **사람이 rqt 버튼이나
`/pick/start`를 직접 눌러야** LLM의 지시가 소비된다. 사용자 확인(2026-08-10 대화) 후
"동작 함수가 곧 시작 트리거"가 되는 그림을 원하면 cobot2_ws 쪽에서

```bash
ros2 launch voice_processing vla_command.launch.py auto_start:=true
```

로 띄워야 한다 — 다른 clone/프로세스라 이 ws의 launch 파일로는 켤 수 없음
(`launch/vla_system.launch.py`의 `vla_pick_bridge` Node 주석에 같은 안내 추가함).
`WAIT_APPROVAL`(실제 grasp 승인)은 별개 파라미터(`require_approval`)라 `auto_start`를
켜도 줄지 않음(contract §0-B/§4) — README.md #3에 같은 내용 반영함.
**미검증**: 이 조합(`enable_pick_bridge:=true` + cobot2_ws `auto_start:=true`)으로
실기 왕복한 적 없음. cobot2_ws 쪽에서 `auto_start:=true`를 실제로 켜는 결정 자체는
사용자 몫 — 이 ws는 문서화만 했다.

### ✅ `vla_pick_bridge` ↔ `vla_command_node` 실기 왕복 스모크 완료 (2026-08-11, `pick_fsm` 없이)

`pick_fsm`도 로봇도 없이(순수 JSON 경계 레벨) 두 프로세스를 실제로 같이 띄워서 확인함:

```bash
# 터미널 1 (cobot2_ws)
export ROS_DOMAIN_ID=93 && source ~/cobot2_ws/install/setup.bash
ros2 run voice_processing vla_command_node --ros-args -p auto_start:=false \
  -p allowed_classes:="apple,banana,orange,cup,bottle,mouse"

# 터미널 2 (이 ws)
export ROS_DOMAIN_ID=93 && source install/setup.bash
ros2 run vla_system vla_pick_bridge_node
```

`ROS_DOMAIN_ID=93`를 **양쪽에 명시적으로** 맞춰야 서로 보인다 — 기본값이 다르다(이 ws
미지정=0, cobot2_ws `.bashrc` 관행 93). 같은 PC라 해도 도메인이 다르면 안 보인다.

**확인된 것**:
- `/vla/scene`(가짜 apple_1) + `/vla/robot/action`(pick_and_place, place=basket)을
  주입 → `vla_pick_bridge`가 `object_id→class` 변환 후 `/vla/pick_command` 발행 →
  `vla_command_node` 로그에 `"내려놓을 위치 지정: basket"` — **`place` 필드가 실제로
  경계를 건너 파싱되는 것 확인** (2026-08-10에 추가한 기능이 이번에 처음 실기로 검증됨)
- **TTL 만료 경로**: 첫 시도는 씬 발행과 `/get_keyword` 호출 사이 시간이 TTL(10s)을
  넘겨 `vla_command_node`가 `"지시 만료"`로 거부 → `/vla/pick_result{result:"rejected"}`
  → `vla_pick_bridge`가 `RobotState.last_result="rejected"`로 정확히 반영. 의도한
  실패가 아니라 실제로 발생한 타이밍 문제였는데, 그 자체가 거부 경로의 유효한 검증이 됨
- **accepted 경로**: `/get_keyword`(Trigger)를 수동 호출해 FSM의 `LISTENING` 풀을
  흉내냄 → `vla_command_node`가 `"FSM 에 전달"` 로그 + `/vla/pick_result
  {result:"accepted", reason:"FSM 이 가져갔다 — 승인 대기는 사람 몫"}` →
  `vla_pick_bridge`가 `RobotState.status="moving"`, `current_action_id` 채움,
  `last_result=""`(비종결) — `result_update()`의 "accepted" 분기 그대로 재현됨

**아직 미검증인 것**: `pick_fsm` 자체(실제 grasp 시퀀스, `WAIT_APPROVAL`, `place_location`
관절 이동), `table`/`discard` 거부 경로(`allow_unverified_place=false` 그대로 둠 —
실기 검증 아님, 코드 레벨 게이트만 있음), `auto_start:=true` 조합.

### 🔴 정정 (2026-08-11): `pick_and_hold`/`release`는 제거하지 않는다

이 절이 예전엔 "제거 권장"이라고 썼는데, `vla_pick_bridge` 관점만 보고 내린 반쪽
결론이었다 — 다시 보니 틀렸다. `agent/tools.py`는 `enable_robot`(단독 모드)과
`enable_pick_bridge`(cobot2_ws 연동) 두 실행 모드가 **공유하는 하나의 스키마**다.
`robot_node.py`(단독 모드)에서 `pick_and_hold`/`release`는 죽은 코드가 아니라 실제
기능이다:

```python
# robot_node.py:52-53
PICK_ACTIONS = ("pick_and_place", "pick_and_hold")
KNOWN_ACTIONS = PICK_ACTIONS + ("release", "go_home")
```

`pick_and_hold`는 집기만 하고 `place()`를 안 불러 `self.holding`을 유지하고,
`release`가 그걸 실제로 내려놓는다(`run_action()` 참고) — 사람이 손으로 건네받는
시나리오에 쓰는 진짜 동작이다. 여기서 지우면 cobot2_ws 연동 모드의 프롬프트만
깔끔해지고 단독 모드의 실기능 하나가 사라진다. **결론: 지금 구조(스키마엔 유지,
`vla_pick_bridge`가 로컬에서 거부)가 맞다 — 손대지 않는다.**

### 넘어오는 설계 결정 — 이 ws가 그대로 따라야 하는 것

| 결정 | 내용 |
|---|---|
| 승인(`/pick/approve`) | 🔴 이 ws에서 절대 호출 안 함. cobot2_ws가 코드 레벨(`BLOCKED_CMDS`)로 무조건 거부 — `require_approval`이 유일한 소프트 안전장치라서 VLA가 대신 눌러주면 안전장치가 0이 된다. GUI에 승인 버튼을 띄워 **사람이** 누르게 하는 안(§6-1 "가")이 권장안 — 아직 GUI에 반영 안 됨 |
| `vla_robot`/`robot/gripper.py` | 🔴 켜는 것 자체가 금지 — DRFL TCP 연결·Modbus 레지스터를 cobot2_ws `pick_fsm`/`OnRobotRGControllerServer`와 장비 레벨로 공유 (기존에 이미 알고 있던 사실, 재확인) |
| 좌표 변환 | `vla_perception`의 table homography 좌표는 이제 안 씀 — 좌표는 cobot2_ws D435i + `T_cam2base`가 만든다. YOLO-seg·추적·개체 handle만 살린다 |
| `vla_interfaces` | 내부 전용으로 강등. 경계는 JSON 하나뿐 — cobot2_ws로 절대 안 넘긴다 |

### 🔴 확인/결정이 안 끝난 것 (cobot2_ws 쪽 문서가 "미확인"으로 남겨둔 것 포함)

1. **되묻기("1번")가 실제 그 개체를 집어야 하는가?** — 경계를 넘는 건 `class`뿐이라, 사과가
   2개면 지금은 `ask_clarification`으로 "1번"을 골라도 FSM이 아무 사과나 집을 수 있다.
   cobot2_ws의 `select_by_point()`가 미구현 상태 — 이게 없으면 되묻기 시나리오가 사실상
   무의미. **답 안 나면**: 씬에 같은 클래스 1개만 두는 데모로 범위를 좁히는 게 임시 대안.
2. **VLA와 `pick_fsm`이 정말 같은 PC/같은 ROS 도메인에서 도는가?** (§0-G, 2026-08-10
   전제 변경 — 두 PC+핫스팟 → 같은 PC로 뒤집힘, 그러나 이 ws 쪽에서 직접 확인한 적 없음).
   같은 PC/도메인이면 `ROS_DOMAIN_ID=93` 맞추기·DDS 프로파일·대역폭 제약이 전부 무의미해짐 —
   확인 전엔 넘겨짚지 말 것.
3. **고정 카메라(D435i, FSM과 공유) 스트림을 이 ws가 로컬 토픽 구독으로 받는지, 전혀 안
   보는지** — 카메라 자체가 공유라는 것은 확정됐음(2026-08-10 사용자 확인,
   docs/context/constraints.md). 스트림을 이 ws가 로컬 구독하는지 여부는 여전히 미확인.
4. **`allowed_classes`/두 YOLO 클래스 목록 일치 — 사용자 결정으로 보류 (2026-08-10, "고민해볼
   예정")**. cobot2_ws `vla_command_node`는 목록 밖 클래스를 거부하므로, 이 ws YOLO
   (`yolo26s-seg`, `target_classes` in `system.yaml`)와 cobot2_ws `grasp_bridge_node`의
   `target_classes`가 이름 단위로 안 맞으면 겹치지 않는 클래스는 전부 거부된다 — 지금
   상태로 브리지를 cobot2_ws에 붙이면 즉시 드러날 문제. 두 YOLO를 통일할지/목록만 맞출지는
   사용자가 아직 결정 안 함 — **먼저 손대지 말 것.**
5. **정지 경로 응답성** — 이 ws `vla_robot`의 20ms 폴링+stop epoch 이중 방어는 브리지
   경로에서 사라진다(`{"cmd":"abort"}`→FSM MoveIt goal 취소로 느려짐). `/safety/stop`을
   브리지가 직접 부를지는 cobot2_ws 쪽 결정 대기.

### 순서 (cobot2_ws 문서 §7 기준, 갱신)

1. ~~`vla_pick_bridge` 작성~~ ✅ 완료(2026-08-10, MVP) — **로봇 없이** JSON 왕복 스모크만
   검증됐고, 실제 cobot2_ws `vla_command_node`와의 왕복은 아직 미검증(그쪽 프로세스가 같이
   떠 있는 상태에서 한 번도 안 돌려봄)
2. 위 3번(카메라 로컬 구독 여부) · 2번(도메인) 확인 — `enable_pick_bridge:=true`를 실제로
   켜기 전에 필요
3. §6-1 승인 정책 확정 — GUI 승인 버튼 아직 없음, 브리지는 이미 승인 경로를 안 엶(설계상 안전)
4. §6-2 되묻기 범위 확정 (`pixel` 넣을지 말지) — 지금 브리지는 `pixel` 자체를 안 보냄(위 참고)
5. ~~`pick_and_hold`·`release` 정리~~ ❌ 취소(2026-08-11) — 단독 모드(`robot_node.py`)의
   실기능이라 지우면 안 됨, 위 "정정" 절 참고
6. (같은 PC 확인되면 생략 가능) 도메인·DDS 프로파일 맞추고 도달성 실측
7. 실기: `require_approval` 켠 채, 같은 클래스 물체 1개인 씬에서 1사이클
