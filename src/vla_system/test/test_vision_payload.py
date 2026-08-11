"""라벨 사진을 모델에게 얹는 부분.

여기서 지키려는 것은 둘이다. **사진이 대화 기록에 들어가지 않을 것**(들어가면
지난 사진이 매 호출마다 따라 올라가 비용이 턴 수에 제곱으로 는다), 그리고
**못 보낼 상황에서 조용히 빠질 것**(카메라가 죽었는데 옛 사진을 보내면 모델은
없어진 물체를 자신 있게 가리킨다).
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vla_system.agent.vision import (                                # noqa: E402
    DATA_URL_PREFIX,
    attach_image,
    encode_frame,
)


def frame(width=640, height=480):
    return np.full((height, width, 3), 200, dtype=np.uint8)


# --------------------------------------------------------------- encoding

def test_a_frame_becomes_a_data_url():
    url = encode_frame(frame())
    assert url.startswith(DATA_URL_PREFIX)
    assert len(url) > len(DATA_URL_PREFIX)


def test_a_wide_frame_is_narrowed_but_a_small_one_is_left_alone():
    """폭을 줄이는 것은 비용보다 라벨 가독성 때문이다 -- 뭉개지면 모델이 id를 지어낸다."""
    import cv2

    big = encode_frame(frame(1920, 1080), max_width=640)
    decoded = cv2.imdecode(
        np.frombuffer(__import__("base64").b64decode(big[len(DATA_URL_PREFIX):]),
                      dtype=np.uint8),
        cv2.IMREAD_COLOR)
    assert decoded.shape[1] == 640
    assert decoded.shape[0] == 360, "가로세로비가 유지돼야 한다"

    small = encode_frame(frame(320, 240), max_width=640)
    decoded = cv2.imdecode(
        np.frombuffer(__import__("base64").b64decode(small[len(DATA_URL_PREFIX):]),
                      dtype=np.uint8),
        cv2.IMREAD_COLOR)
    assert decoded.shape[1] == 320, "작은 프레임을 억지로 키우지 않는다"


def test_nothing_in_means_nothing_out():
    assert encode_frame(None) == ""
    assert encode_frame(np.zeros((0, 0, 3), dtype=np.uint8)) == ""


# ---------------------------------------------------------------- attaching

def test_the_picture_never_enters_the_conversation():
    """이 검정이 비용을 지킨다. attach_image는 사본에만 얹는다."""
    items = [{"role": "user", "content": "첫 턴"},
             {"role": "assistant", "content": "네"},
             {"role": "user", "content": "이거 집어줘"}]
    original = [dict(item) for item in items]

    sent = attach_image(items, "data:image/jpeg;base64,AAAA")

    assert items == original, "기록이 변형됐다 -- 다음 호출부터 사진이 따라다닌다"
    assert sent is not items
    assert isinstance(sent[-1]["content"], list)
    assert sent[-1]["content"][0] == {"type": "input_text", "text": "이거 집어줘"}
    assert sent[-1]["content"][1]["type"] == "input_image"


def test_only_the_newest_turn_carries_a_picture():
    items = [{"role": "user", "content": "첫 턴"},
             {"role": "user", "content": "둘째 턴"}]
    sent = attach_image(items, "data:image/jpeg;base64,AAAA")
    assert sent[0]["content"] == "첫 턴", "지난 턴은 텍스트 그대로여야 한다"


def test_no_picture_is_a_no_op():
    items = [{"role": "user", "content": "사과 집어줘"}]
    assert attach_image(items, "") is items


def test_a_non_user_tail_is_left_alone():
    """도구 결과 뒤에 사진을 끼워 넣으면 요청 자체가 깨진다. 덜 아는 편이 낫다."""
    items = [{"role": "user", "content": "사과 집어줘"},
             {"type": "function_call_output", "call_id": "x", "output": "{}"}]
    assert attach_image(items, "data:image/jpeg;base64,AAAA") is items


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
