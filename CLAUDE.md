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

## 3. 지금 남은 일 — `vla_pick_bridge` (미착수)

cobot2_ws 쪽(`vla_command_node`)은 완성됐다. 이 ws에 아직 없는 것: `RobotAction`
(`/vla/robot/action`)을 받아 `object_id→class` 변환 후 `/vla/pick_command`로 발행하고,
`/vla/pick_result`를 `RobotState`로 되돌리는 노드 하나. 입출력 표·LLM 툴 스키마에서 고칠
부분(`pick_and_place`에 `place` 인자 추가, `pick_and_hold`/`release` 제거)은
`vla-bridge-contract.md` §7 참고.

## 4. 하드웨어

M0609 + OnRobot RG2 + 고정 Webcam(Logitech C270, 탐지) + 손목 Intel RealSense D435i
(파지 정밀화). cobot2_ws 쪽 D435i(eye-to-hand, 고정)와는 **다른 카메라**다 — 혼동 금지
(cobot2_ws CLAUDE.md 2절이 이미 이 구분을 명시해뒀다: "cobot2_ws가 쓰는 카메라는 고정
D435i 한 대뿐, 손목 카메라 없음"— 그건 그쪽 로봇 얘기고, 이 ws의 손목 RealSense는 이 ws
전용 파이프라인이다).
