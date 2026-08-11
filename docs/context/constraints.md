# 실기/환경 제약 — M0609_VLA_system

실기 검증이나 실제 실행으로 알아낸, 코드/문서만 봐서는 안 드러나는 사실만 적는다.
세션이 바뀌면 이 파일이 유일한 기억이다.

## 카메라 구성 — 🔴 README와 다름 (2026-08-10 사용자 확인)

**고정 카메라는 Logitech C270가 아니라, cobot2_ws의 `pick_fsm`이 쓰는 것과 같은 물리
D435i다 (공유).** 별도의 웹캠이 아니다.

- `CLAUDE.md` §4와 `README.md`("고정 Webcam(Logitech C270)")는 "cobot2_ws 쪽 D435i와는
  다른 카메라다 — 혼동 금지"라고 적어놨는데, 이게 **틀렸다.** 실제로는 같은 D435i를
  vla_perception과 cobot2_ws FSM이 같이 읽는다.
- **손목(wrist) RealSense D435i 구성은 아직 미정이다** (2026-08-10 기준, 사용자 확인).
  README의 "고정 웹캠=탐지, 손목 RealSense=파지 정밀화" 투-카메라 설계 전제가 지금도
  유효한지 자체가 불확실 — 다음에 다룰 때 반드시 먼저 확인할 것. 그때까지 이 구조에
  의존하는 코드(`vla_wrist`, GraspGenX 경로, hand-eye 보정)를 "당연히 그대로 쓴다"고
  가정하지 않는다.
- `vla_perception.webcam_device`(`system.yaml`)가 실제로 어떤 `/dev/video*`를 가리켜야
  하는지는 이 사실이 확정되기 전엔 재확인 대상이다. 지금 값(`/dev/video0`)이 맞는지도
  다시 봐야 한다 — 개발 머신에서의 `/dev/video*` 넘버링은 장치일 뿐 정답이 아니다.
- **cobot2_ws와 물리 카메라를 공유한다는 것 자체가 새로운 제약이다**: 두 프로세스가
  동시에 같은 V4L2 장치를 열면(둘 다 OpenCV 직접 오픈이라면) 충돌한다. 토픽 공유
  (`realsense2_camera` 노드 하나만 띄우고 양쪽이 구독)인지, 각자 독립 오픈인지부터
  확인해야 한다 — 아직 미확인.

## `colcon build`가 venv를 조용히 무시한다 (2026-08-10 실측)

apt로 설치된 `/usr/bin/colcon`은 shebang이 `/usr/bin/python3`다. `.venv`를 활성화한
채로 `colcon build`를 돌려도, colcon 자신은 시스템 파이썬으로 실행되고 있어서
`setup.py`도 시스템 파이썬으로 돌린다. 그 결과 생성되는 `perception_node` 등 콘솔
스크립트의 shebang이 시스템 python3로 박히고, `.venv` 전용 패키지(torch, ultralytics,
openai, pymodbus, sounddevice)가 런타임에 안 보여서 **`colcon build`는 성공(PASS)하고
런타임에만 `ModuleNotFoundError`로 죽는다** — 빌드 로그만 보면 놓친다.

- 재현: `colcon build` → `ros2 run vla_system perception_node` → `ModuleNotFoundError: torch`
- 해결: `python3 -m colcon build`(venv 활성 상태)로 호출 — colcon을 모듈로 실행하면
  현재 파이썬(venv)을 쓴다. shebang이 `.venv/bin/python3`로 정확히 잡히는 것 확인함.
- `scripts/build.sh`에 이미 반영함 (`.venv` source + `python3 -m colcon build`).
  **다른 빌드 스크립트나 수동 빌드 시에도 `colcon build`를 맨손으로 쓰지 말 것.**

## `./scripts/check.sh`가 ROS 없이는 실패한다 (README에 반영 완료, 2026-08-10)

`test_wrist_async_state.py`가 `vla_interfaces`(커스텀 msg, 빌드 산출물)를 import해서
**`install/setup.bash`를 먼저 소싱하지 않으면 collection error로 전체가 실패한다.**
ROS+install 소싱 후엔 265 passed(`vla_pick_bridge` 순수 로직 테스트 16개 추가 포함).
README "테스트" 절에 반영 완료 — 앞으로 "196 passed"/"ROS 불필요" 같은 예전 문구를
README에서 다시 보면 이 항목이 재발한 것이다.

## 이 개발 머신의 `/dev/video*` 매핑 (참고용, 실기 아님)

이 검증을 수행한 개발 머신에서는 `/dev/video0-5`가 RealSense D435I, `/dev/video6-7`이
HD Webcam이었다. 실제 로봇 머신에서는 다를 수 있다 — 그리고 위 "카메라 구성" 항목이
확정되기 전까진 이 매핑 자체가 무의미할 수 있다.

## `~/.local`에 잔해 발견 — pytest/torch 사고 재발 위험 (2026-08-10 실측)

`~/.local/lib/python3.10/site-packages/`에서 발견:

- `anyio` — dist-info 없음, `__version__`도 `None`. 정상 설치가 아니라 깨진 잔해
- `sounddevice-0.5.5.dist-info` + 루스 `.py` 파일 — `.venv`에도 같은 버전이 있음(중복)
- `nvidia/`, `cuda/` — 각각 24K/4K, 정상 CUDA wheel(보통 수백 MB~수 GB)이라기엔 너무
  작다. 부분 설치 잔해로 보임

전부 `INSTALLER: pip`, `REQUESTED` 마커 없음(= 직접 설치가 아니라 뭔가의 의존성으로
끌려온 것) — `torch`/`ultralytics` 등을 **`.venv` 만들기 전에 한 번 `pip install`로
직접 부른 시도**가 있었고 중간에 끊긴 흔적으로 보인다(`.venv` 생성 시각 19:17보다
이 잔해들의 mtime이 18:34~19:04로 더 이르다).

**✅ 사용자 확인(2026-08-10): 이 ws `requirements.txt`를 venv 없이 실수로 설치하다
멈춘 흔적이 맞다.** `sounddevice`(requirements.txt 직접 항목)는 18:34에 완전히
끝났고, `anyio`(openai→httpx의 전이 의존성)·`nvidia`/`cuda`(ultralytics→torch의
CUDA 휠)는 19:04에 설치 도중 끊겼다 — numpy/opencv-python/openvino/pymodbus/torch
본체는 `~/.local`에 전혀 없다. **삭제는 보류.** 지금 당장 뭔가를 깨고 있진 않다 —
`pip check`는 무관한 다른 경고(`onrobot-rg-control`, cobot2_ws 자기 몫 — 상세는 아래
절) 하나뿐이고 apt `pytest`/cobot2_ws `colcon build` 둘 다 정상 동작 확인함
(2026-08-10). ABI 민감한 패키지(numpy/opencv-python/torch 본체)가 없어서 위험도는
낮지만, CLAUDE.md §1이 경고하는 패턴과 일치하므로 나중에 정리할 땐 `pip show --files
<이름>`으로 뭐가 설치한 건지 먼저 확인할 것. README "설치" 절에 확인 안내를 추가해뒀다.

## `onrobot-rg-control`/`pymodbustcp` `pip check` 경고 — 원인 확인·조치 완료 (2026-08-11)

`.venv`(`--system-site-packages`) 안에서 `pip check`를 돌리면 `onrobot-rg-control
2.0.0 requires pymodbustcp, which is not installed`가 뜬다. 위 절에서 "cobot2_ws
자기 몫"이라고만 적어놨던 걸 이번에 실제로 추적함.

**원인**: `~/.bashrc:194-195`가 **모든 인터랙티브 셸에서 무조건**
`source /opt/ros/humble/setup.bash && source ~/cobot2_ws/install/setup.bash`를
실행한다 — 어느 워크스페이스 디렉터리에서 셸을 열든 상관없이. `sys.path`를 실제로
찍어보면 `~/cobot2_ws/install/*/site-packages` 25개 패키지(그중 하나가
`onrobot_rg_control`)가 **`/opt/ros/humble`보다도, 이 ws의 `.venv` site-packages보다도
앞순위**로 들어가 있다. `onrobot-rg-control`은 `pip install`로 이 ws나 `~/.local`에
깔린 게 아니라 cobot2_ws의 `colcon build`가 만든 ament_python 패키지이고, 이게
`pip show`에 "설치된 패키지"로 잡히는 이유가 이거다 — `.bashrc`가 이미 그 경로를
`PYTHONPATH`에 올려놨기 때문.

**이 ws에 실제 영향 없음을 확인함**: `gripper.py`가 cobot2_ws의 `onrobot_rg_control`
패키지를 **import하지 않는다.** 이 ws는 자체 `OnRobotRG` 클래스를 직접 구현했고
(`vla_system/robot/gripper.py:10`), `pymodbus`(`pymodbus.client.sync.ModbusTcpClient`,
`requirements.txt`가 고정한 `pymodbus==2.5.3`)만 쓴다 — 클래스 이름이 `OnRobotRG`로
같아서 앞선 턴에 `grep -l`로 "이 파일들이 그 패키지를 쓴다"고 잘못 단정했던 것을
바로잡는다. `import`도 실제로 성공한다(위 "실제 확인한 것" 참고).

**남는 구조적 위험 (지금은 발현 안 함, 기록용)**: `.bashrc`가 cobot2_ws의 25개
site-packages 경로를 이 ws의 `.venv`보다 앞에 두므로, **cobot2_ws가 이 ws의
`requirements.txt`와 같은 이름의 파이썬 패키지를 나중에 갖게 되면 이 ws의 고정 버전이
조용히 가려진다** — `~/.local` 오염과 같은 계열의 위험을 반대 방향(cobot2_ws → 이
ws)에서 만든다. 2026-08-11 기준 cobot2_ws install space의 패키지명 25개 전부 확인한
결과 `requirements.txt`(numpy/PyYAML/openai/python-dotenv/sounddevice/scipy/
ultralytics/openvino/opencv-python/pymodbus)와 겹치는 이름 없음 — 지금은 안전하지만,
cobot2_ws에 새 파이썬 의존성이 추가될 때마다 재확인 대상이다. `.bashrc:134`의
"2026-07-31 제거: cobot2_ws에서도 cobot1_ws의 dsr_common2가 sys.path에 끼어들어 ws 간
오염"이 이미 한 번 겪은 같은 종류의 사고였다는 점도 참고.

### ✅ 조치 완료: `~/.bashrc`의 무조건 `source ~/cobot2_ws/install/setup.bash` 제거 (2026-08-11)

**사용자 확인**: 지금까지 cobot2_ws 단일 환경만 써서 이 구조가 문제였던 적이 없었지만,
다른 ws(M0609_VLA_system 등)를 병렬로 열어 쓰는 지금은 손대야 하는 것으로 확정.

- `~/.bashrc:194-195`의 무조건 `source ~/cobot2_ws/install/setup.bash`를 제거하고,
  `alias sco="source ~/cobot2_ws/install/setup.bash"`로 opt-in화했다(`cdco`와 대칭 —
  `cdco`=cd, `sco`=source). `/opt/ros/humble/setup.bash`는 그대로 자동 소싱 유지(ROS
  배포판 자체는 워크스페이스 무관하게 필요, 위험 없음).
- **`env -i`로 완전히 클린한 환경을 시뮬레이션해 검증함**: 새 셸에서 `PYTHONPATH`엔
  `/opt/ros/humble`만 남고 cobot2_ws 25개 경로는 더 이상 없음. `sco` alias 정상 동작.
- 🔴 **이미 열려 있던 터미널/셸은 안 바뀐다.** `PYTHONPATH`는 export된 환경변수라
  `.bashrc`를 다시 읽어도(`sob`) 지워지지 않고 그 위에 덧붙기만 한다 — 기존에 이미
  cobot2_ws가 얹힌 채로 상속된 셸(지금 이 세션의 셸 포함)은 재시작(새 터미널/재로그인)
  전까지 예전처럼 cobot2_ws가 PYTHONPATH 맨 앞에 남아 있다.
- **영향받는 것**: `~/.bashrc`의 `br`/`brsi`/`manual`/`auto`/`hom`/`hom0`/`hom90`/
  `emer`/`gripper` 등 cobot2_ws 패키지(`m0609_rg2_bringup`, `dsr_msgs2`,
  `onrobot_rg_control`)에 의존하는 alias들은 **새 터미널에서 `sco`를 먼저 실행해야**
  동작한다 — 더 이상 자동으로 안 얹힌다. cobot2_ws 작업 시작할 때 `cdco && sco`가
  새 습관이 됨.

### 🔴 진짜 원인은 하나 더 있었다 — 이 ws의 `install/`에 cobot2_ws가 박혀 있었음 (2026-08-11)

`.bashrc`를 고친 뒤에도 `source scripts/env.sh`를 거치면 `sys.path`에 cobot2_ws가
**여전히** 나왔다 — 원인은 `.bashrc`가 아니라 **이 ws 자신의 `install/setup.bash`**
였다. colcon이 워크스페이스를 빌드할 때 그 시점 `AMENT_PREFIX_PATH`에 잡혀 있던
"underlay" 목록을 `install/setup.bash`(`prefix_chain.bash.em` 템플릿)에 **그대로
박아 넣는다.** 이 ws가 예전에(cobot2_ws가 무조건 소싱되던 `.bashrc` 아래에서)
`colcon build`된 적이 있어서, 그 산출물엔

```bash
COLCON_CURRENT_PREFIX="/home/kimkh/cobot2_ws/install"
_colcon_prefix_chain_bash_source_script "$COLCON_CURRENT_PREFIX/local_setup.bash"
```

이 줄이 하드코딩돼 있었다 — `.bashrc`를 고쳐도 `source scripts/env.sh` → `source
install/setup.bash`가 이 줄을 통해 cobot2_ws를 다시 끌어왔다. **조치**: `build/`
`install/` `log/`(전부 gitignore 대상, 재생성 가능)를 지우고 `AMENT_PREFIX_PATH`에
cobot2_ws가 전혀 없는 `env -i` 클린 셸에서 `colcon build --symlink-install
--packages-select vla_interfaces vla_system`로 재빌드함. 재생성된
`install/setup.bash`엔 `/opt/ros/humble`만 체인돼 있고 cobot2_ws 줄이 사라진 것
확인. `env -i` 클린 셸 + `scripts/env.sh` 조합으로 `sys.path`에 cobot2_ws 없음 +
`./scripts/check.sh` 271 passed 재확인함(2026-08-11).

**교훈 — 다음에도 같은 일이 재발할 수 있는 지점**: 이 ws를 `colcon build`할 때 셸에
cobot2_ws(또는 다른 ws)가 얹혀 있으면 **그 사실이 `install/setup.bash`에 다시
박제된다.** `.bashrc`를 고쳤다고 끝이 아니라, **빌드하는 셸 자체가 깨끗한지**
(`echo $AMENT_PREFIX_PATH`로 다른 ws 안 보이는지) 매번 확인하는 습관이 필요하다.
