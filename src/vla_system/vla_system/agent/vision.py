"""라벨이 그려진 카메라 화면을 모델에게 같이 보여주기 위한 인코딩.

왜 필요한가
----------
`scene`(JSON)만으로는 **"이거 집어줘"를 절대 풀 수 없다.** 그 발화에는 클래스도
색도 위치도 없다 -- 뜻은 전적으로 사람이 무엇을 가리키고 있느냐에 있고, 그건
화면에만 있다. 좌표를 더 촘촘히 실어 보내도 해결되지 않는 종류의 부족함이라
그림 자체를 보여주는 수밖에 없다.

무엇을 보내는가
-------------
`perception_node`가 이미 내보내는 `/vla/perception/annotated_image`다. 여기
그려진 박스 라벨은 `object_id(class, track_id)` -- **모델이 JSON에서 읽는 id와
같은 문자열이다**(`detector.draw_tracks`). 그래서 모델은 "화살표가 가리키는
박스"를 본 뒤 그 라벨을 그대로 도구 인자에 쓸 수 있다. 그림과 JSON을 잇는 것이
이 라벨 하나뿐이라, 라벨 형식을 바꾸면 이 기능이 조용히 망가진다.

무엇을 보내지 않는가
------------------
**기록에 남기지 않는다.** 대화 기록은 60개까지 쌓이는데 거기에 이미지가 섞이면
매 호출마다 예전 사진들이 전부 다시 올라간다 -- 턴이 늘수록 비용이 제곱으로
는다. `attach_image()`가 기록이 아니라 *호출 직전의 사본*에만 그림을 얹는
이유가 이것이다. 기록에는 텍스트만 남는다.
"""

from __future__ import annotations

import base64

DATA_URL_PREFIX = "data:image/jpeg;base64,"


def encode_frame(bgr, max_width: int = 640, quality: int = 70) -> str:
    """BGR 프레임 -> data URL. 실패하면 빈 문자열.

    폭을 줄이는 것은 비용보다 **정확도** 때문이다. 640px에서 박스 라벨은 아직
    읽히지만 그 아래로 내려가면 `apple_1`과 `apple_7`이 뭉개진다 -- 그러면
    모델은 못 읽었다고 말하지 않고 그럴듯한 id를 지어낸다.
    """
    import cv2

    if bgr is None or getattr(bgr, "size", 0) == 0:
        return ""

    height, width = bgr.shape[:2]
    if width > max_width:
        scale = max_width / float(width)
        bgr = cv2.resize(bgr, (max_width, max(1, int(round(height * scale)))),
                         interpolation=cv2.INTER_AREA)

    ok, buffer = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        return ""
    return DATA_URL_PREFIX + base64.b64encode(buffer.tobytes()).decode("ascii")


def attach_image(items: list[dict], data_url: str) -> list[dict]:
    """마지막 user 항목에 그림을 얹은 **사본**을 돌려준다.

    원본 `items`는 건드리지 않는다 -- 그것이 대화 기록이고, 기록에 이미지가
    들어가면 다음 호출부터 계속 따라다닌다(모듈 설명 참고).

    얹을 자리가 없으면(마지막이 user가 아니거나 비어 있으면) 그냥 원본을
    돌려준다. 그림을 못 보내는 것은 이번 판단이 조금 덜 아는 것일 뿐이지만,
    엉뚱한 자리에 끼워 넣는 것은 요청 자체를 깨뜨린다.
    """
    if not data_url or not items:
        return items

    last = items[-1]
    if last.get("role") != "user":
        return items

    content = last.get("content")
    if isinstance(content, str):
        parts: list[dict] = [{"type": "input_text", "text": content}]
    elif isinstance(content, list):
        parts = list(content)
    else:
        return items

    parts.append({"type": "input_image", "image_url": data_url})
    return items[:-1] + [{**last, "content": parts}]
