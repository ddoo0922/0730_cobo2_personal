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

🔴 **`pick_and_hold`/`release` 툴은 제거하지 않는다 — 2026-08-11 정정.** 예전 노트가
"cobot2_ws가 못 받으니 프롬프트에서 지우는 게 맞다"고 했던 건 `vla_pick_bridge`
관점만 본 반쪽 결론이었다. `agent/tools.py`는 `enable_robot`(단독 모드)과
`enable_pick_bridge`(cobot2_ws 연동) 양쪽이 **공유**하는데, `robot_node.py`
(단독 모드)에서 `pick_and_hold`/`release`는 실제로 동작하는 기능이다(`PICK_ACTIONS`,
`self.holding` 상태로 "쥔 채 대기" → 나중에 `release`). 지우면 cobot2_ws 연동 모드의
UX만 좋아지고 단독 모드의 실기능 하나가 없어진다. 지금처럼 스키마엔 남겨두고
`vla_pick_bridge`가 로컬에서 거부하는 현재 구조가 맞다 — 손대지 않는다.

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

**손목(wrist) RealSense D435i 구성은 아직 미정이다.** README의 "고정 웹캠=탐지, 손목
RealSense=파지 정밀화" 투-카메라 설계가 지금도 유효한지 자체가 불확실 — `vla_wrist`,
GraspGenX 경로, hand-eye 보정 관련 작업을 시작하기 전에 반드시 먼저 확인할 것.
상세·후속 확인 목록은 `docs/context/constraints.md` "카메라 구성" 항목.

두 프로세스(vla_perception, cobot2_ws FSM)가 물리 카메라 하나를 어떻게 나눠 쓰는지
(토픽 공유 vs 각자 독립 오픈)도 미확인 — 독립 오픈이면 V4L2 장치 충돌 가능성이 있다.

<details>
<summary>예전 설계 서술 (틀림, 기록용으로만 남김)</summary>

~~고정 Webcam(Logitech C270, 탐지) + 손목 Intel RealSense D435i(파지 정밀화).
cobot2_ws 쪽 D435i(eye-to-hand, 고정)와는 다른 카메라다 — 혼동 금지~~

</details>
