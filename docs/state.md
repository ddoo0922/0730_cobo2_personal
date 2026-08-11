# 세션 상태 — M0609_VLA_system

마지막 갱신: 2026-08-11. 이 파일은 현재 상태로 덮어쓴다 — 로그처럼 쌓지 않는다.

## 🔴 이번 세션(2026-08-11)의 큰 변경 — 실행 스택 삭제 + 상태 가시성 연동

**역할 분담을 확정했다**: cobot2_ws의 `pick_fsm`이 로봇/그리퍼 실행과 정밀 그립
계산을 전담한다. 이 ws(VLA)는 "무엇을(class) 어디로(place) 집을지"만 판단해
`/vla/pick_command`(JSON)로 넘긴다. 그래서:

### 삭제한 것 (cobot2_ws와 중복되던 실행 스택 전체)
- `nodes/robot_node.py`, `robot/moves.py`, `robot/gripper.py`, `robot/__init__.py`
- `nodes/wrist_grasp_node.py`, `grasp/graspgen_client.py`, `grasp/poses.py`, `grasp/__init__.py`
- `perception/wrist_geometry.py`, `perception/wrist_tracking.py`
- 대응 테스트 7개
- `agent/tools.py`의 `pick_and_hold`/`release` 툴 (소비자였던 `robot_node` 삭제로 죽은 코드)
- launch/setup.py/config에서 `enable_robot`/`enable_wrist_grasp`/`motion_enabled`/
  `vla_robot`·`vla_wrist` 파라미터 블록 제거

**남긴 것**: `perception_node`/`table_homography*`는 명령 실행에 안 쓰이는 GUI
디버그 표시용이라 유지. `vla_pick_bridge_node`가 이제 유일한 실행 경로.

### 좌표 판단 위임 (오렌지 pick 버그 수정)
`scene_to_payload`가 `pickable`을 항상 true로, prompt.py 규칙 6 수정, `agent_node`
dispatch에서 `position_valid` 게이트 제거. 이유: 좌표는 cobot2_ws가 계산하니 이 ws의
테이블 보정 유무가 pick을 막으면 안 됨(예전엔 "테이블 보정 없음"으로 오렌지 거부됨).

### 신규: FSM 단계 실시간 가시성 (`/pick/state` 구독)
`vla_pick_bridge_node`가 cobot2_ws의 `/pick/state`(std_msgs/String, FSM enum 이름,
매 전이 발행)를 구독 → `bridge/pick_bridge.py`의 `fsm_state_view()`로
`RobotState.status`/`details`/`holding`에 매핑 → GUI가 "동작 중 · 들어올리는 중",
"사람 승인 대기 ✋" 등 표시. `HOLDING_STATES`(VERIFY/LIFT/PLACE/PLACE_RETRY)일 때
VLA가 자기가 보낸 object_id/class로 `holding`을 채움(계약 §3 공백 메움). 진행 중
상태는 `current_action` 채운 채 발행 → `agent_node`가 무시(GUI 전용, 판단 재트리거
아님). 상세·FSM 세션 확인요청 4개는 [context/fsm-state-integration.md](context/fsm-state-integration.md).

### 검증
- `./scripts/build.sh`(venv shebang 유지) PASS, 테스트 **139 passed**.
- GUI 기동 정상. cross-review(session audit) 통과 — 런타임 죽은 참조 없음,
  holding 초기화 4경로 닫힘, 에이전트 재트리거 방지 보장, 단일스레드 락 불필요.
- 🔴 **실기 미검증**: cobot2_ws `pick_fsm`과의 실제 왕복(특히 `/pick/state` 수신
  여부 — 퍼블리셔 QoS는 다른 clone이라 여기서 확인 불가)은 아직 안 함.

## cobot2_ws 통합 — 여전히 유효한 상태·제약

계약 정본은 `~/cobot2_ws/md/vla-bridge-contract.md`(단일 사본, CLAUDE.md §2 — 여기
사본 안 둠). 아래는 이 ws 기준 액션·확인 목록.

### ✅ 완료 (JSON 경계 레벨 스모크까지)
- `vla_pick_bridge`: `RobotAction`→`/vla/pick_command`, `/vla/pick_result`→`RobotState`.
  핵심 변환 `object_id`(apple_17)→`class`(apple). result 매핑은 계약 §3-3 표 그대로.
- `place` 인자(basket/table/discard): `table`/`discard`는 계약 §5상 placeholder
  관절값이라 `allow_unverified_place`(기본 false)로 브리지가 거부. teach 완료 시
  파라미터만 뒤집음(코드 불변).
- `pixel`/`pixel_wh`: bbox 중심 픽셀 + 프레임 해상도를 `/vla/pick_command`에 실음.
  cobot2_ws가 `select_by_point()` 구현 완료(계약 §8) — `pixel_policy=select`로
  올리면 개체 선정에 쓰임(기본 `warn`, opt-in). 이 ws는 이미 보내고 있어 추가 작업 없음.
- **실기 왕복 스모크**(2026-08-11, `pick_fsm` 없이 `vla_command_node`만): `place`
  필드가 경계 건너 파싱됨 확인, TTL 만료→rejected 경로, accepted 경로(`/get_keyword`
  수동 호출) 모두 재현. `ROS_DOMAIN_ID=93`를 **양쪽 명시**해야 서로 보임(이 ws
  미지정=0, cobot2_ws 관행 93).

### 🔴 cobot2_ws 쪽에서 켜야 하는 것 (이 ws가 대신 못 함)
- `auto_start:=true`: 안 켜면 `pick_fsm`이 `IDLE`에서 `/pick/start` 대기. VLA 지시가
  곧 시작 트리거가 되려면 cobot2_ws에서 `ros2 launch voice_processing
  vla_command.launch.py auto_start:=true`. `WAIT_APPROVAL`(grasp 승인)은 별개
  파라미터라 auto_start로 안 줄어듦(계약 §4).

### 넘어온 설계 결정 — 그대로 따름
- **승인(`/pick/approve`) 절대 호출 안 함** — cobot2_ws가 코드 레벨 거부. 승인은
  cobot2_ws 로컬(rqt 버튼 + 음성). VLA는 `/pick/state=="WAIT_APPROVAL"`을 보고
  "승인 대기 중"을 **표시만** 한다(이번 세션 추가) — 자동화는 안 함.
- **좌표 변환**: `vla_perception`의 table homography 좌표는 실행에 안 씀(GUI 표시용).
  실제 좌표는 cobot2_ws D435i + `T_cam2base`.
- **`vla_interfaces`**: 내부 전용. 경계는 JSON 하나뿐 — cobot2_ws로 안 넘김.

### 🔴 확인/결정이 안 끝난 것
1. **되묻기("1번")가 실제 그 개체를 집는가** — cobot2_ws `select_by_point()`는
   구현됐으나 `pixel_policy=warn`(기본)이라 opt-in. `select`로 안 올리면 사과 2개 중
   "1번"을 골라도 FSM이 아무 사과나 집을 수 있음. 임시 대안: 같은 클래스 1개 씬.
2. **같은 PC/같은 ROS 도메인인가** — 같은 PC로 뒤집혔다고 알려졌으나 이 ws에서 직접
   확인 안 함. `ROS_DOMAIN_ID=93` 양쪽 맞추기 필요(위 스모크 참고).
3. **`allowed_classes` 목록 일치** — 이 ws YOLO(`target_classes` in system.yaml)와
   cobot2_ws의 `grasp_bridge_node` `target_classes`가 이름 단위로 안 맞으면 겹치지
   않는 클래스는 거부됨. 통일할지/목록만 맞출지 사용자 결정 대기 — **먼저 손대지 말 것.**
4. **정지 경로 응답성** — 브리지 경로는 `{"cmd":"abort"}`→FSM MoveIt goal 취소라
   이 ws 예전 20ms 폴링보다 느릴 수 있음. `/safety/stop`을 브리지가 직접 부를지는
   cobot2_ws 결정 대기.
5. **`/pick/state` 실기 수신 확인** — 구독 QoS는 RELIABLE/VOLATILE/depth10로 맞췄으나
   cobot2_ws 퍼블리셔 QoS는 다른 clone이라 확인 불가. 첫 실기 왕복에서 `/pick/state`가
   실제로 GUI에 뜨는지 눈으로 확인할 것(안 뜨면 QoS 불일치 의심).

## 미커밋 / 다음

- **이번 세션 전체가 아직 워킹트리(미커밋)** — 삭제 17 + 상태연동 + 문서. CLAUDE.md §0
  경고(다른 clone이라 커밋 안 하면 소실). 커밋은 사용자 결정 대기.
- `scripts/env.sh`의 `DOOSAN_SETUP` 기본값을 `~/cobot2_ws/install/setup.bash`로 고침
  (기존 `~/cobot_ws`는 이 머신에 없음). GUI 파이프라인은 Doosan 불필요 —
  `table_homography_test`만 필요.
- 🔴 `~/.local/lib/python3.10/site-packages/`에 이 ws가 만들지 않은 잔해
  (`anyio` 깨진 것·`sounddevice` 중복·`nvidia`/`cuda` 부분설치) 있음. 지금 뭘 깨진
  않지만(pytest/colcon 정상) CLAUDE.md §1 패턴. **삭제 안 함, 사용자 확인 필요.**
