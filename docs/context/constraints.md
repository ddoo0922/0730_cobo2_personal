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

## `./scripts/check.sh`가 ROS 없이는 실패한다 (README와 다름)

README는 "테스트는 순수 로직만 다룬다 — ROS도 카메라도 로봇도 필요 없다"고 적혀
있지만, `test_wrist_async_state.py`가 `vla_interfaces`(커스텀 msg, 빌드 산출물)를
import해서 **`install/setup.bash`를 먼저 소싱하지 않으면 collection error로 전체가
실패한다.** ROS+install 소싱 후엔 249 passed(README에 적힌 196보다 늘었음 — 테스트가
계속 추가된 것으로 보임, 그 자체는 문제 아님). README 테스트 절 업데이트 필요 — 안 함.

## 이 개발 머신의 `/dev/video*` 매핑 (참고용, 실기 아님)

이 검증을 수행한 개발 머신에서는 `/dev/video0-5`가 RealSense D435I, `/dev/video6-7`이
HD Webcam이었다. 실제 로봇 머신에서는 다를 수 있다 — 그리고 위 "카메라 구성" 항목이
확정되기 전까진 이 매핑 자체가 무의미할 수 있다.
