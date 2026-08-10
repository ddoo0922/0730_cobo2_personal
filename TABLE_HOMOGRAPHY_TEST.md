# Webcam table homography click-to-approach test

이 테스트는 기존 VLA/RealSense/GraspGenX와 독립적으로 다음 단계만 검증한다.

1. M0609 TCP를 테이블의 P1 -> P2 -> P3 -> P4에 직접 이동하여 base 좌표를 기록
2. 고정 Webcam 영상에서 같은 물리적 P1 -> P2 -> P3 -> P4를 클릭
3. Webcam pixel `(u,v)` -> M0609 base `(X,Y)` homography 계산
4. 네 TCP 접촉점으로 `Z_table = aX + bY + c` 테이블 평면 계산
5. 테스트 클릭 시 `[X, Y, Z_table + 150 mm]`로 이동

P1..P4는 테이블 둘레를 따라 시계 또는 반시계 방향으로 동일한 순서로 입력한다.

## 1. 빌드

```bash
cd ~/cobot2_ws_vla_refactored
source /opt/ros/humble/setup.bash
source ~/cobot_ws/install/setup.bash
colcon build --symlink-install --packages-select vla_interfaces vla_system
source install/setup.bash
```

## 2. 실제 M0609 bringup

기존 프로젝트에서 사용하던 bringup을 먼저 실행한다.

```bash
ros2 launch m0609_rg2_bringup bringup.launch.py \
  mode:=real host:=192.168.1.100 model:=m0609
```

## 3. 먼저 DRY RUN으로 보정/좌표만 확인

기본 Webcam이 `/dev/video0`인 경우:

```bash
source /opt/ros/humble/setup.bash
source ~/cobot_ws/install/setup.bash
source install/setup.bash

ros2 run vla_system table_homography_test --ros-args \
  --params-file src/vla_system/config/system.yaml
```

Webcam이 `/dev/video1`이면:

```bash
ros2 run vla_system table_homography_test --ros-args \
  --params-file src/vla_system/config/system.yaml \
  -p webcam_device:=/dev/video1
```

### 화면 조작

- `SPACE`: 현재 TCP base pose를 P1, P2, P3, P4 순서로 저장
- Robot 4점 저장 후 `LEFT CLICK`: Webcam에서 같은 P1, P2, P3, P4 순서로 저장
- 4+4점 완료 후 `LEFT CLICK`: 클릭 지점의 base target을 계산
- `U`: 현재 보정 단계의 마지막 점 취소
- `R`: 전체 4점 보정 초기화
- `S`: calibration JSON 저장
- `L`: 저장된 calibration JSON 로드
- `X`: 실제 모션 중 SSTOP 요청
- `Q` 또는 `ESC`: 종료

보정 파일 기본 위치:

```text
~/.ros/vla_table_homography.json
```

## 4. 실제 클릭 이동

DRY RUN에서 XY/Z가 타당한 것을 확인한 뒤에만 `motion_enabled:=true`를 준다.

```bash
ros2 run vla_system table_homography_test --ros-args \
  --params-file src/vla_system/config/system.yaml \
  -p motion_enabled:=true
```

보정이 이미 저장되어 있으면 재입력 없이 바로 TEST 단계로 시작할 수 있다.

```bash
ros2 run vla_system table_homography_test --ros-args \
  --params-file src/vla_system/config/system.yaml \
  -p motion_enabled:=true \
  -p load_calibration_on_start:=true
```

테스트 클릭 시 목표는 항상:

```text
X_target = homography(pixel).X
Y_target = homography(pixel).Y
Z_target = fitted_table_Z(X_target, Y_target) + 150 mm
orientation = 클릭 직전 현재 TCP orientation 유지
```

이다.

## 안전 조건

- 보정한 Webcam 테이블 사각형 밖 클릭은 기본적으로 거부된다.
- 계산된 target이 `system.yaml`의 `workspace_*_mm` 밖이면 거부된다.
- 실제 모션은 기본 `false`이며 실행 시에만 명시적으로 켠다.
- 테스트 중 기존 `vla_robot`이 동시에 M0609을 제어하지 않도록 한다.
