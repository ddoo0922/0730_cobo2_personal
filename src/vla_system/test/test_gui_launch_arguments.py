"""GUI 체크박스 -> `ros2 launch` 인자.

이 파일이 있는 이유는 같은 실수가 두 번 났기 때문이다(2026-08-11).

  1. launch가 `skill_tier_enabled`를 선언은 했지만 agent_node의
     `parameters=[]`로 넘기지 않아 `ros2 launch ... skill_tier_enabled:=true`가
     조용히 무시됐다.
  2. 고친 뒤에도 GUI의 "VLA 시작"이 그 인자를 아예 안 보내서, 체크 여부와
     무관하게 launch 기본값 false가 이겼다.

둘 다 예외를 던지지 않는다. 화면에는 규칙 계층이 켜진 것처럼 보이고, 증상은
며칠 뒤 "말한 규칙이 재시작하면 사라진다"로만 나타난다. 조용히 틀리는 배선은
조용히 틀리지 않게 만들어 두는 수밖에 없다.
"""

import re
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

LAUNCH_FILE = (Path(__file__).resolve().parent.parent
               / "launch" / "vla_system.launch.py")


def gui_arguments(**flags) -> dict[str, str]:
    """GUI가 보내는 `이름:=값`을 딕셔너리로."""
    from vla_system.vla_gui import build_launch_command      # noqa: PLC0415

    command = build_launch_command(**flags)
    return dict(part.split(":=", 1) for part in command if ":=" in part)


ALL_ON = {"pick_bridge": True, "wrist_grasp": True,
          "skill_tier": True, "perception": True}
ALL_OFF = {key: False for key in ALL_ON}


def test_the_rule_layer_checkbox_reaches_the_launch_command():
    assert gui_arguments(**ALL_ON)["skill_tier_enabled"] == "true"
    assert gui_arguments(**ALL_OFF)["skill_tier_enabled"] == "false"


def test_perception_can_be_turned_off_for_a_fake_stage():
    """dryrun_stage.py가 /vla/scene을 대신 낼 때 카메라 노드는 떠 있으면 안 된다."""
    assert gui_arguments(**ALL_OFF)["enable_perception"] == "false"
    assert gui_arguments(**ALL_ON)["enable_perception"] == "true"


def test_no_camera_means_no_camera():
    """"카메라 인식"을 끄면 RealSense도 안 뜬다.

    이 둘이 갈라져 있으면 가짜 무대로 돌릴 때 launch가 장치를 찾다 죽는다 --
    enable_realsense는 원래 pick_bridge에만 묶여 있어서, 단독 모드로 두고
    카메라만 끄면 오히려 카메라를 여는 조합이 나왔다.
    """
    for pick_bridge in (True, False):
        args = gui_arguments(**{**ALL_OFF, "pick_bridge": pick_bridge})
        assert args["enable_realsense"] == "false"


def test_every_argument_the_gui_sends_is_one_the_launch_file_declares():
    """오타나 이름 변경은 `ros2 launch`에서 에러가 아니라 무시로 나타난다."""
    declared = set(re.findall(r'DeclareLaunchArgument\(\s*"([^"]+)"',
                              LAUNCH_FILE.read_text(encoding="utf-8")))
    assert declared, "launch 파일을 못 읽었다"
    unknown = set(gui_arguments(**ALL_ON)) - declared
    assert not unknown, f"launch가 모르는 인자를 보내고 있다: {sorted(unknown)}"


def test_the_launch_file_forwards_the_rule_flag_into_the_agent():
    """선언만 하고 Node(parameters=[])에 안 넘기면 값이 사라진다 -- 실제로 그랬다."""
    source = LAUNCH_FILE.read_text(encoding="utf-8")
    agent = source[source.index('executable="agent_node"'):]
    agent = agent[:agent.index("),\n            Node(")]
    assert '"skill_tier_enabled": skill_tier_enabled' in agent
    assert '"rule_store_path": rule_store_path' in agent


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
