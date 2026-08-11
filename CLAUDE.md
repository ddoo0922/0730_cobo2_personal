# CLAUDE.md — M0609_VLA_system

> 공통 규칙(빌드 게이트·셸·금지 규칙·응답 계약)은 `~/.claude/CLAUDE.md`에 있다(계정 전역이라
> 이 디렉토리에서도 적용됨). 여기엔 이 clone/브랜치에서만 참인 것만 적는다.

## 0. 🔴 이 디렉토리의 정체 — "별도 repo"가 아니다 (2026-08-10 확인)

`git remote -v` 결과 `~/M0609_VLA_system`과 `~/cobot2_ws`는 **같은 GitHub remote**
(`gwanhuiGIM/0730_cobo2_personal`)를 가리키는 **서로 다른 독립 clone**이다:

| | 이 디렉토리 | `~/cobot2_ws` |
|---|---|---|
| 브랜치 | `vla_integ` (→ `origin/vla_integ`) | `semi_Final` |
| `.git` | 독립 clone (linked worktree 아님, `git worktree list` 로 확인) | 독립 clone |

과거 커밋(`a2c154b` 등)이 겹치는 이유가 이거다. **다른 세션·다른 clone이라 이 디렉토리에
쓴 변경은 여기서 커밋해야만 살아남는다** — 워킹트리에만 남겨두면 이 브랜치에서 다른 작업
(체크아웃·리셋 등)이 한 번만 일어나도 사라진다(2026-08-10 실제로 `README.md`/`scripts/
env.sh`/`PYTHON_ENV_CONVENTION.md` 세 개가 이렇게 소실됨 — cobot2_ws
`md/plans/2026-08-08-vla-integration.md` §0-F가 사고 기록).

## 1. 파이썬 의존성 — `pip install --user` 금지, venv 필수

같은 계정(`kimkh`)의 `~/cobot2_ws`와 `~/.local`(계정 전역)을 공유한다. `--user`로 깔면
`~/cobot2_ws`의 `colcon build`/`pytest`가 깨진다(2026-08-10 실측 사고 — `opencv-python`이
apt `python3-opencv`를 덮어 rclpy+Qt segfault 조합, `pydantic` v2가
`generate_parameter_library`의 v1 요구와 충돌, `setuptools`+`anyio`가 apt `pytest` 6.x를
깨뜨림).

**규칙**: 이 ws의 파이썬 의존성은 전부 `--system-site-packages` venv 안에서만 설치한다
(`rclpy`/`ament_index_python`이 필요한 진짜 ROS 2 ament_python 패키지라 일반 venv는
`import rclpy`부터 깨진다).

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
# setuptools/packaging 버전이 --system-site-packages 의 apt packaging(21.3)과 안 맞을 수 있다
# (2026-08-10 실측 — vla_interfaces 빌드가 TypeError 로 죽었다):
python3 -m pip install "setuptools<80,>=30.3.0" "packaging>=23"
```

`.venv`는 `scripts/env.sh`가 ROS/Doosan overlay와 함께 자동으로 source한다 — 노드 실행 전엔
항상 `source scripts/env.sh`.

## 2. cobot2_ws 와의 경계 — JSON 하나, 파일 사본은 안 둔다

- **계약(스키마·허용값)**: `~/cobot2_ws/md/vla-bridge-contract.md` — cobot2_ws가 관리하는
  **단일 사본**. 이 디렉토리에 복사해두지 않는다(사본 두면 어긋난다는 게 §0에서 실증됐다).
  이 파일이 갱신되면 절대경로로 다시 읽는다.
- **왜 이렇게 됐는지(설계 히스토리)**: `~/cobot2_ws/md/plans/2026-08-08-vla-integration.md`
  (§9가 이 ws에서 옮겨간 최종 병합 체크리스트 원문).
- **경계는 `/vla/pick_command`↔`/vla/pick_result`(JSON) 하나뿐이다.** `vla_interfaces`를
  cobot2_ws로 넘기지 않는다 — 커스텀 msg를 경계로 쓰면 두 clone이 빌드 버전으로 묶인다.
- **`/pick/approve`를 이 ws에서 부르지 않는다.** cobot2_ws 쪽 `vla_command_node`가
  `cmd:"approve"`를 코드 레벨로 무조건 거부한다(안전장치, 계약 문서 §4). 그립 승인은
  cobot2_ws가 로컬(rqt 버튼 + 음성 `approve_listener_node`)로 처리한다 — 이 ws가 다시
  구현할 필요 없다.

## 3. 지금 남은 일

`vla_pick_bridge`(`RobotAction`→`/vla/pick_command`, `/vla/pick_result`→`RobotState`)와
`pick_and_place`의 `place` 인자(basket/table/discard)는 구현 완료(2026-08-10, 상세는
`docs/state.md` "cobot2_ws 통합"). `table`/`discard`는 cobot2_ws 쪽 teach가 안 끝나
`vla_pick_bridge_node`의 `allow_unverified_place`(기본 false)로 막혀 있다 — teach 끝나면
그 파라미터만 뒤집는다, 코드는 안 건드린다.

🔴 **`robot_node.py`(단독 모드)와 `wrist_grasp_node.py`/`grasp/*`/`robot/*` 전체 삭제
— 2026-08-11 재정정, 2026-08-11 정정을 다시 뒤집음.** 그 정정은 "`pick_and_hold`/
`release`가 단독 모드(`robot_node.py`)에서 실기능이니 스키마에 남긴다"였다. 그런데
`vla_gui.py`가 이미 그날 `enable_robot:=true`를 보내는 경로 자체를 없앴다(GUI
체크박스는 그 전부터 죽어 있었다) — 즉 단독 모드는 코드로만 존재했지 실제로 뜬 적이
없었다. cobot2_ws의 `pick_fsm`이 로봇/그리퍼를 전담하는 아키텍처를 최종으로 확정하면서
`robot_node.py`/`robot/moves.py`/`robot/gripper.py`와, 그 유일한 소비자였던
`wrist_grasp_node.py`/`grasp/graspgen_client.py`/`grasp/poses.py`/
`perception/wrist_geometry.py`/`perception/wrist_tracking.py`를 전부 삭제했다(사용자
승인, 2026-08-11). `agent/tools.py`의 `pick_and_hold`/`release`도 함께 제거 —
소비자(`robot_node.py`)가 없으니 순수 죽은 코드였다. **이제 `vla_pick_bridge_node`가
유일한 실행 주체다.** `perception_node.py`/`table_homography*.py`는 명령 실행에 안
쓰이는(GUI 디버그 표시용) 정보라 남겨뒀다.

남은 것: `vla_pick_bridge`↔cobot2_ws 실기 왕복 스모크(특히 `place` 거부 경로
미검증). 그 외엔 `docs/state.md` "확인/결정이 안 끝난 것" 절 참고 — 대부분 사용자
결정 대기 또는 cobot2_ws 쪽 확인 필요라 이 ws 혼자 진행 못 함.

**🔴 `enable_pick_bridge:=true`만으로는 cobot2_ws FSM이 안 돈다.** `pick_fsm`은
`/pick/start`가 불릴 때까지 `IDLE`에 멈춰 있다 — 기본은 사람이 직접 누르는 것.
cobot2_ws 쪽에서 `vla_command.launch.py auto_start:=true`로 띄워야 VLA의 판단이 곧
시작 트리거가 된다(다른 clone이라 이 ws에서 대신 켤 수 없음). 상세·미검증 여부는
`docs/state.md` "auto_start" 절, README.md #3 참고.

## 4. 하드웨어 — 🔴 카메라 구성 정정 (2026-08-10, 사용자 확인)

M0609 + OnRobot RG2. **고정 카메라는 별도 Logitech C270가 아니라, cobot2_ws의
`pick_fsm`이 쓰는 것과 같은 물리 D435i다(공유).** 아래 두 문단은 예전 설계로,
**틀렸다** — cobot2_ws CLAUDE.md 2절의 "D435i는 그쪽 로봇 전용, 이 ws와 다른 카메라"
서술도 함께 틀렸다는 뜻이니 그쪽을 참고할 때도 이 사실을 우선한다.

**손목(wrist) RealSense 경로는 삭제했다 (2026-08-11).** `vla_wrist`/`wrist_grasp_node`/
GraspGenX/hand-eye 보정 코드는 §3에 적힌 대로 이 ws에서 완전히 제거됐다 — 손목
카메라로 정밀 파지 포즈를 계산하는 건 이제 이 ws의 일이 아니다(cobot2_ws의
`grasp_bridge_node`가 자기 쪽에서 처리). 고정 D435i(`vla_perception`)만 남는다.

두 프로세스(vla_perception, cobot2_ws FSM)가 물리 카메라 하나를 어떻게 나눠 쓰는지
(토픽 공유 vs 각자 독립 오픈)도 미확인 — 독립 오픈이면 V4L2 장치 충돌 가능성이 있다.

<details>
<summary>예전 설계 서술 (틀림, 기록용으로만 남김)</summary>

~~고정 Webcam(Logitech C270, 탐지) + 손목 Intel RealSense D435i(파지 정밀화).
cobot2_ws 쪽 D435i(eye-to-hand, 고정)와는 다른 카메라다 — 혼동 금지~~

</details>
