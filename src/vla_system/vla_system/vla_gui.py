#!/usr/bin/env python3
"""Console for talking to the robot: camera, chat, scene table, stop button.

The GUI holds exactly one piece of decision logic, and it is here because it
must not wait for anything: the stop keyword. A "정지" typed or spoken here is
matched locally and published straight to the robot, bypassing STT-to-LLM
round-trips entirely. Everything else -- what the words meant, which object was
meant, whether to ask back -- is the agent's to decide.

Topics
------
publish:
  /vla/user_utterance   what the user said
  /vla/estop            the hardcoded stop path
subscribe:
  /vla/perception/annotated_image
  /vla/scene
  /vla/agent/reply
  /vla/robot/state
"""

from __future__ import annotations

import base64
import os
import queue
import re
import signal
import subprocess
import threading
import time
import tkinter as tk
import webbrowser
from datetime import datetime
from pathlib import Path
from tkinter import messagebox, ttk

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vla_interfaces.msg import AgentReply, RobotState, SceneSnapshot

from vla_system.process_guard import (
    PIPELINE_PATTERN,
    REALSENSE_PATTERN,
    escalate_termination,
    find_existing_pipeline_pids,
)

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------

UTTERANCE_TOPIC = "/vla/user_utterance"
ESTOP_TOPIC = "/vla/estop"
ANNOTATED_IMAGE_TOPIC = "/vla/perception/annotated_image"
SCENE_TOPIC = "/vla/scene"
REPLY_TOPIC = "/vla/agent/reply"
ROBOT_STATE_TOPIC = "/vla/robot/state"

# cobot2_ws의 grasp_bridge_node가 기본으로 띄우는 viser 웹뷰어 (live_viz_port 기본값,
# graspgenx_perception/grasp_bridge_node.py). ROS 토픽이 아니라 WebSocket/WebGL
# 렌더러라 이 GUI에 임베드할 방법이 없다 -- 브라우저 새 창으로만 연다(2026-08-11).
GRASPGENX_VIZ_URL = "http://localhost:8080"

# Matched before anything else happens to the user's words. Deliberately broad:
# a stop that fires when the user did not quite mean it costs one interrupted
# motion, while a stop that fails to fire costs a collision.
STOP_PATTERN = re.compile(
    r"정지|멈춰|멈춤|그만|중지|스톱|스탑|\bstop\b|\bhalt\b", re.IGNORECASE
)

SAMPLE_RATE = 16_000
MIN_RECORD_SECONDS = 0.35
AUDIO_PATH = Path.home() / ".ros" / "vla_system" / "gui_last_command.wav"
# Every line the pipeline prints, kept on disk. The chat only shows the lines
# matching _INTERESTING_LOG_TOKENS, and ros2 launch's own log directory holds
# nothing when its output is piped -- so without this file a grasp that missed
# leaves no record of what was commanded.
PIPELINE_LOG_PATH = Path.home() / ".ros" / "vla_system" / "pipeline.log"

STT_MODEL = os.getenv("OPENAI_STT_MODEL", "gpt-4o-transcribe")

BASE_LAUNCH_COMMAND = [
    "ros2",
    "launch",
    "vla_system",
    "vla_system.launch.py",
]

_INTERESTING_LOG_TOKENS = (
    "error",
    "warn",
    # Grasp diagnostics. These are INFO, and without them the chat shows a bare
    # "완료했습니다" for a grasp that physically missed -- the match distance and
    # the commanded target are the only way to tell which calibration is off.
    "matched ",
    "grasp plan",
    "executing wrist",
    "observe from",
    "grasp at",
    "graspgenx ready",
    "planner ready",
    "robot motion",
    "stop:",
    "traceback",
    "exception",
    "died",
    "fatal",
    "critical",
    "cannot connect",
    "no module named",
)
_LOG_PREFIX_RE = re.compile(r"^\[([^\]]+)\]")

# UI palette
BG = "#17191c"
PANEL = "#22252a"
PANEL_2 = "#1c1f23"
FG = "#e8eaed"
MUTED = "#9aa0a6"
ACCENT = "#5cc8ff"
GREEN = "#64d98b"
YELLOW = "#f2c94c"
RED = "#ff6b6b"
USER_BG = "#243447"
AI_BG = "#24352d"
SYSTEM_BG = "#2b2d31"
ASK_BG = "#3a3320"


def ros_image_to_rgb(message: Image) -> np.ndarray:
    """Convert common ROS RGB images without cv_bridge."""
    if message.encoding not in ("bgr8", "rgb8"):
        raise ValueError(f"unsupported image encoding: {message.encoding}")
    row = np.frombuffer(message.data, dtype=np.uint8).reshape(
        message.height, message.step
    )
    image = row[:, : message.width * 3].reshape(message.height, message.width, 3)
    if message.encoding == "bgr8":
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    return image.copy()


def rgb_to_tk_photo(rgb: np.ndarray, max_width: int, max_height: int) -> tk.PhotoImage:
    """Resize an RGB array and create a Tk PhotoImage without Pillow."""
    height, width = rgb.shape[:2]
    if width <= 0 or height <= 0:
        raise ValueError("empty image")

    scale = min(max_width / width, max_height / height)
    if scale <= 0:
        scale = 1.0
    scale = min(scale, 1.5)
    target_w = max(1, int(width * scale))
    target_h = max(1, int(height * scale))

    if target_w != width or target_h != height:
        interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
        rgb = cv2.resize(rgb, (target_w, target_h), interpolation=interpolation)

    # Tk 8.6 supports PNG. Encoding to PNG avoids a Pillow dependency.
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    ok, encoded = cv2.imencode(".png", bgr)
    if not ok:
        raise RuntimeError("failed to encode GUI image")
    return tk.PhotoImage(data=base64.b64encode(encoded.tobytes()), format="png")


def crop_object(rgb: np.ndarray, box, padding: int = 18) -> np.ndarray:
    """Cut one detection out of the frame, with a little context around it."""
    height, width = rgb.shape[:2]
    x_min = max(0, int(box[0]) - padding)
    y_min = max(0, int(box[1]) - padding)
    x_max = min(width, int(box[2]) + padding)
    y_max = min(height, int(box[3]) + padding)
    if x_max - x_min < 4 or y_max - y_min < 4:
        raise ValueError("crop is too small")
    return rgb[y_min:y_max, x_min:x_max].copy()


def robot_state_line(state: dict | None) -> str:
    if state is None:
        return "로봇 상태 대기 중"
    status = {
        "idle": "대기",
        "moving": "동작 중",
        "holding": "물체를 든 채 대기",
        "waiting_approval": "사람 승인 대기 ✋",
        "error": "오류",
    }.get(state["status"], state["status"])
    holding = state["holding_class"] or "없음"
    mode = "실제 모션" if state["motion_enabled"] else "DRY-RUN"
    tail = ""
    if state.get("current_action") and state.get("details"):
        # A pick is in flight -- details carries the live FSM step label
        # (vla_pick_bridge fsm_state_callback), which is what the user wants to
        # see instead of a bare "동작 중".
        status = f"{status} · {state['details']}"
    elif state["last_action"]:
        tail = f" | 최근: {state['last_action']} → {state['last_result']}"
    return f"{status} | 들고 있음: {holding} | {mode}{tail}"


# -----------------------------------------------------------------------------
# ROS bridge
# -----------------------------------------------------------------------------


class GuiRosBridge(Node):
    """ROS callbacks only store snapshots; Tk reads them from the GUI thread."""

    def __init__(self, events: queue.Queue):
        super().__init__("vla_gui")
        self.events = events
        self._lock = threading.Lock()

        self._latest_frame: np.ndarray | None = None
        self._keep_frame: np.ndarray | None = None
        self._latest_scene: dict | None = None
        self._latest_state: dict | None = None

        self.frame_count = 0
        self.last_frame_monotonic = 0.0

        stream_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.utterance_publisher = self.create_publisher(
            String, UTTERANCE_TOPIC, command_qos
        )
        self.estop_publisher = self.create_publisher(String, ESTOP_TOPIC, command_qos)

        self.create_subscription(
            Image, ANNOTATED_IMAGE_TOPIC, self._image_callback, stream_qos
        )
        self.create_subscription(
            SceneSnapshot, SCENE_TOPIC, self._scene_callback, stream_qos
        )
        self.create_subscription(
            AgentReply, REPLY_TOPIC, self._reply_callback, command_qos
        )
        self.create_subscription(
            RobotState, ROBOT_STATE_TOPIC, self._state_callback, latched_qos
        )

    # ------------------------------------------------------------- callbacks

    def _image_callback(self, message: Image) -> None:
        try:
            frame = ros_image_to_rgb(message)
        except Exception as exc:
            self.get_logger().warning(f"GUI image decode failed: {exc}")
            return
        with self._lock:
            self._latest_frame = frame
            # A second reference the video loop does not consume, so a
            # clarification crop can still be taken after the frame was drawn.
            self._keep_frame = frame
            self.frame_count += 1
            self.last_frame_monotonic = time.monotonic()

    def _scene_callback(self, message: SceneSnapshot) -> None:
        payload = {
            "calibration_ok": bool(message.calibration_ok),
            "objects": [
                {
                    "id": scene_object.id,
                    "class_name": scene_object.class_name,
                    "confidence": float(scene_object.confidence),
                    "color": scene_object.color or "unknown",
                    "pickable": bool(scene_object.position_valid),
                    "bbox": (
                        float(scene_object.x_min),
                        float(scene_object.y_min),
                        float(scene_object.x_max),
                        float(scene_object.y_max),
                    ),
                    "position": (
                        float(scene_object.position_base.x),
                        float(scene_object.position_base.y),
                        float(scene_object.position_base.z),
                    ),
                }
                for scene_object in message.objects
            ],
        }
        with self._lock:
            self._latest_scene = payload

    def _reply_callback(self, message: AgentReply) -> None:
        self.events.put(
            (
                "reply",
                {
                    "kind": message.kind,
                    "text": message.text,
                    "focus": list(message.focus_object_ids),
                },
            )
        )

    def _state_callback(self, message: RobotState) -> None:
        payload = {
            "status": message.status,
            "holding_id": message.holding_object_id,
            "holding_class": message.holding_class_name,
            "current_action": message.current_action,
            "last_action_id": message.last_action_id,
            "last_action": message.last_action,
            "last_result": message.last_result,
            "details": message.details,
            "motion_enabled": bool(message.motion_enabled),
        }
        with self._lock:
            self._latest_state = payload
        self.events.put(("robot_state", payload))

    # ------------------------------------------------------------------ reads

    def take_frame(self) -> np.ndarray | None:
        with self._lock:
            frame = self._latest_frame
            self._latest_frame = None
            return frame

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "scene": self._latest_scene,
                "state": self._latest_state,
                "keep_frame": self._keep_frame,
                "frame_count": self.frame_count,
                "last_frame_monotonic": self.last_frame_monotonic,
            }

    # --------------------------------------------------------------- publish

    def publish_utterance(self, text: str) -> None:
        self.utterance_publisher.publish(String(data=text))

    def publish_estop(self, reason: str) -> None:
        self.estop_publisher.publish(String(data=reason))


# -----------------------------------------------------------------------------
# GUI
# -----------------------------------------------------------------------------


class VLAApp:
    def __init__(self, root: tk.Tk, bridge: GuiRosBridge, events: queue.Queue):
        self.root = root
        self.bridge = bridge
        self.events = events

        self.transcriber = None
        self.busy = False

        self.audio_stream = None
        self.audio_frames: list[np.ndarray] = []
        self.recording = False
        self.record_started = 0.0

        self.pipeline_process: subprocess.Popen | None = None
        self.photo: tk.PhotoImage | None = None
        self.clarify_photos: list[tk.PhotoImage] = []

        self.last_frame_count = 0
        self.last_fps_time = time.monotonic()
        self.current_fps = 0.0
        self.last_state_key: tuple | None = None

        # 2026-08-11: 기본을 cobot2_ws 연동으로 바꿈 -- 이 GUI가 최종적으로 존재하는
        # 이유가 pick_fsm과 물려 돌리는 것이라, 단독 모드(vla_robot)가 예외가 되어야
        # 한다. 켜져 있으면 enable_pick_bridge:=true + enable_realsense:=false(카메라는
        # cobot2_ws 쪽 launch가 이미 잡고 있다는 전제, README §4)를 같이 보낸다.
        self.pick_bridge_var = tk.BooleanVar(value=True)

        self._configure_window()
        self._configure_style()
        self._build_ui()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.bind("<Escape>", lambda _event: self.emergency_stop("ESC 키"))
        self.root.after(30, self._drain_events)
        self.root.after(33, self._refresh_video)
        self.root.after(200, self._refresh_status)

        self.append_chat(
            "system",
            "준비 완료. VLA를 시작한 뒤 말을 걸어보세요. "
            "'정지'라고 말하거나 ESC를 누르면 LLM을 거치지 않고 로봇이 즉시 멈춥니다.",
        )

    # ------------------------------------------------------------------ UI

    def _configure_window(self) -> None:
        self.root.title("VLA Robot Console")
        self.root.geometry("1460x900")
        self.root.minsize(1160, 720)
        self.root.configure(bg=BG)

    def _configure_style(self) -> None:
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("TFrame", background=BG)
        style.configure("Panel.TFrame", background=PANEL)
        style.configure("TLabel", background=PANEL, foreground=FG)
        style.configure("Muted.TLabel", background=PANEL, foreground=MUTED)
        style.configure(
            "Title.TLabel",
            background=BG,
            foreground=ACCENT,
            font=("TkDefaultFont", 14, "bold"),
        )
        style.configure(
            "Section.TLabel",
            background=PANEL,
            foreground=FG,
            font=("TkDefaultFont", 11, "bold"),
        )
        style.configure("TButton", padding=7)
        style.configure("TCheckbutton", background=PANEL, foreground=FG)
        style.configure(
            "Treeview",
            background=PANEL_2,
            fieldbackground=PANEL_2,
            foreground=FG,
            rowheight=25,
        )
        style.configure("Treeview.Heading", background="#30343a", foreground=FG)

    def _build_ui(self) -> None:
        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill="both", expand=True)
        outer.columnconfigure(0, weight=3)
        outer.columnconfigure(1, weight=2)
        outer.rowconfigure(1, weight=1)

        header = ttk.Frame(outer)
        header.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 8))
        header.columnconfigure(1, weight=1)
        ttk.Label(header, text="VLA Robot Console", style="Title.TLabel").grid(
            row=0, column=0, sticky="w"
        )

        controls = ttk.Frame(header)
        controls.grid(row=0, column=1, sticky="e")

        self.stop_button = tk.Button(
            controls,
            text="■ 정지 (ESC)",
            bg="#8f1d1d",
            fg="#ffffff",
            activebackground=RED,
            activeforeground="#ffffff",
            relief="flat",
            padx=14,
            pady=6,
            font=("TkDefaultFont", 10, "bold"),
            command=lambda: self.emergency_stop("정지 버튼"),
        )
        self.stop_button.grid(row=0, column=0, padx=(0, 12))

        # 2026-08-11 제거: "실제 로봇 모션" 체크박스는 영구 비활성(state="disabled")으로
        # 죽어있던 컨트롤이었다 -- cobot2_ws의 pick_fsm이 로봇을 전담하게 되면서 GUI가
        # enable_robot:=true를 보낼 일이 아예 없어졌고, real_robot_var는 항상 False였다.
        # (기존 팀원 코드, 2026-08-11 사용자 승인 후 제거)

        # 기본 켜짐: cobot2_ws pick_fsm이 카메라를 이미 잡고 있다는 전제로
        # enable_pick_bridge:=true + enable_realsense:=false를 함께 보낸다(README §4).
        # 이 ws 카메라로 단독 실행하려면 체크를 끈다 -- 그러면 enable_realsense:=true로
        # 되돌아간다(예전 기본 동작).
        self.pick_bridge_check = ttk.Checkbutton(
            controls,
            text="cobot2_ws FSM 연동",
            variable=self.pick_bridge_var,
        )
        self.pick_bridge_check.grid(row=0, column=1, padx=(0, 8))

        # cobot2_ws 쪽 grasp_bridge_node가 띄우는 viser 웹뷰어는 ROS 이미지 토픽이
        # 아니라 WebSocket 렌더러라 이 GUI 안에 못 그린다 -- 새 브라우저 창으로만
        # 연다(2026-08-11, 사용자 확인).
        self.viz_button = ttk.Button(
            controls, text="GraspGenX 뷰어", command=self.open_graspgenx_viewer
        )
        self.viz_button.grid(row=0, column=3, padx=(0, 8))

        self.pipeline_button = ttk.Button(
            controls, text="VLA 시작", command=self.toggle_pipeline
        )
        self.pipeline_button.grid(row=0, column=4)

        # ------------------------------------------------- left: perception

        left = ttk.Frame(outer, style="Panel.TFrame", padding=10)
        left.grid(row=1, column=0, sticky="nsew", padx=(0, 8))
        left.columnconfigure(0, weight=1)
        left.rowconfigure(1, weight=4)
        left.rowconfigure(3, weight=2)

        ttk.Label(
            left, text="고정 RealSense / YOLO-seg  (LLM이 보는 화면)", style="Section.TLabel"
        ).grid(row=0, column=0, sticky="w")
        self.video_label = tk.Label(
            left,
            text=f"{ANNOTATED_IMAGE_TOPIC} 대기 중",
            bg="#090a0c",
            fg=MUTED,
            bd=0,
        )
        self.video_label.grid(row=1, column=0, sticky="nsew", pady=(7, 8))

        self.vision_status_var = tk.StringVar(value="장면 데이터 대기 중")
        ttk.Label(left, textvariable=self.vision_status_var, style="Muted.TLabel").grid(
            row=2, column=0, sticky="w", pady=(0, 5)
        )

        # 손목 RealSense 화면(예전 "손목 RealSense + YOLO-seg" 패널)과 GraspGenX
        # 손목 파지 기능(wrist_grasp_node) 모두 제거됨 -- cobot2_ws의 pick_fsm이
        # 유일한 실행 주체가 되면서 robot_node/wrist_grasp_node 스택 전체가
        # 죽은 코드였다(CLAUDE.md #3).

        table_frame = ttk.Frame(left, style="Panel.TFrame")
        table_frame.grid(row=3, column=0, sticky="nsew")
        table_frame.columnconfigure(0, weight=1)
        table_frame.rowconfigure(0, weight=1)

        self.scene_tree = ttk.Treeview(
            table_frame,
            columns=("id", "color", "conf", "pickable", "position"),
            show="headings",
            height=8,
        )
        headings = {
            "id": "LLM이 부르는 이름",
            "color": "color",
            "conf": "conf",
            "pickable": "집기 가능",
            "position": "base 좌표 (m)",
        }
        widths = {"id": 150, "color": 90, "conf": 60, "pickable": 80, "position": 210}
        for name, label in headings.items():
            self.scene_tree.heading(name, text=label)
            self.scene_tree.column(name, width=widths[name], anchor="center")
        self.scene_tree.grid(row=0, column=0, sticky="nsew")

        scrollbar = ttk.Scrollbar(
            table_frame, orient="vertical", command=self.scene_tree.yview
        )
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.scene_tree.configure(yscrollcommand=scrollbar.set)

        # ----------------------------------------------- right: conversation

        right = ttk.Frame(outer, style="Panel.TFrame", padding=10)
        right.grid(row=1, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(1, weight=1)

        ttk.Label(right, text="대화", style="Section.TLabel").grid(
            row=0, column=0, sticky="w"
        )

        self.chat = tk.Text(
            right,
            bg=PANEL_2,
            fg=FG,
            insertbackground=FG,
            relief="flat",
            wrap="word",
            padx=12,
            pady=12,
            state="disabled",
            font=("TkDefaultFont", 11),
        )
        self.chat.grid(row=1, column=0, sticky="nsew", pady=(7, 8))
        self.chat.tag_configure(
            "user_name", foreground=ACCENT, font=("TkDefaultFont", 10, "bold")
        )
        self.chat.tag_configure(
            "assistant_name", foreground=GREEN, font=("TkDefaultFont", 10, "bold")
        )
        self.chat.tag_configure(
            "system_name", foreground=YELLOW, font=("TkDefaultFont", 10, "bold")
        )
        self.chat.tag_configure(
            "user_body", foreground=FG, background=USER_BG, spacing1=3, spacing3=10
        )
        self.chat.tag_configure(
            "assistant_body", foreground=FG, background=AI_BG, spacing1=3, spacing3=10
        )
        self.chat.tag_configure(
            "system_body", foreground=MUTED, background=SYSTEM_BG, spacing1=3, spacing3=10
        )
        self.chat.tag_configure(
            "ask_body", foreground=YELLOW, background=ASK_BG, spacing1=3, spacing3=10
        )

        # Clarification strip: shown only while the agent is waiting for the
        # user to pick between look-alike objects.
        self.clarify_frame = ttk.Frame(right, style="Panel.TFrame")
        self.clarify_frame.grid(row=2, column=0, sticky="ew", pady=(0, 8))
        self.clarify_frame.grid_remove()

        input_box = ttk.Frame(right, style="Panel.TFrame")
        input_box.grid(row=3, column=0, sticky="ew")
        input_box.columnconfigure(0, weight=1)

        self.entry = ttk.Entry(input_box, font=("TkDefaultFont", 11))
        self.entry.grid(row=0, column=0, sticky="ew", padx=(0, 6))
        self.entry.bind("<Return>", lambda _event: self.submit_text())

        self.send_button = ttk.Button(input_box, text="전송", command=self.submit_text)
        self.send_button.grid(row=0, column=1, padx=(0, 6))

        self.voice_button = tk.Button(
            input_box,
            text="🎙 누르고 말하기",
            bg="#343a40",
            fg=FG,
            activebackground="#4a5057",
            activeforeground="#ffffff",
            relief="flat",
            padx=10,
            pady=6,
        )
        self.voice_button.grid(row=0, column=2)
        self.voice_button.bind("<ButtonPress-1>", self._voice_press)
        self.voice_button.bind("<ButtonRelease-1>", self._voice_release)

        self.robot_status_var = tk.StringVar(value="로봇 상태 대기 중")
        ttk.Label(right, textvariable=self.robot_status_var, style="Muted.TLabel").grid(
            row=4, column=0, sticky="w", pady=(8, 0)
        )

        self.bottom_status_var = tk.StringVar(value="ROS 연결됨 | VLA 파이프라인 대기")
        status = tk.Label(
            outer,
            textvariable=self.bottom_status_var,
            bg="#0f1114",
            fg=MUTED,
            anchor="w",
            padx=10,
            pady=5,
        )
        status.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(8, 0))

    # --------------------------------------------------------- conversation

    def append_chat(self, role: str, text: str, tag: str | None = None) -> None:
        role = role if role in {"user", "assistant", "system"} else "system"
        name = {"user": "사용자", "assistant": "AI", "system": "SYSTEM"}[role]
        stamp = datetime.now().strftime("%H:%M:%S")

        self.chat.configure(state="normal")
        self.chat.insert("end", f"{name}  {stamp}\n", f"{role}_name")
        self.chat.insert("end", f" {text.strip()} \n\n", tag or f"{role}_body")
        self.chat.see("end")
        self.chat.configure(state="disabled")

    def submit_text(self) -> None:
        text = self.entry.get().strip()
        if not text:
            return
        self.entry.delete(0, "end")
        self.handle_user_text(text)

    def handle_user_text(self, text: str) -> None:
        """The single funnel for everything the user says, typed or spoken."""
        self.append_chat("user", text)
        if STOP_PATTERN.search(text):
            self.emergency_stop(text)
            return
        self.clear_clarification()
        self.bridge.publish_utterance(text)

    def emergency_stop(self, reason: str) -> None:
        """Straight to the robot. No STT wait, no LLM round-trip, no queue."""
        self.bridge.publish_estop(reason)
        self.append_chat("system", f"정지 명령을 로봇에 즉시 전달했습니다. ({reason})")
        self.clear_clarification()

    def open_graspgenx_viewer(self) -> None:
        """cobot2_ws의 viser 뷰어를 새 브라우저 창으로 연다.

        이 GUI 안에 임베드하지 않는 이유: viser는 WebSocket으로 브라우저에 접속시켜
        클라이언트 쪽(three.js/WebGL)에서 그리는 구조라, sensor_msgs/Image 토픽처럼
        받아서 tk.PhotoImage로 그릴 수 있는 정적 프레임이 아니다. cobot2_ws의
        grasp_bridge_node/graspgen_worker가 이미 떠 있고 live_viz(기본 켜짐)일 때만
        실제로 뭔가 보인다 -- 안 떠 있으면 브라우저가 연결 거부만 보여준다.
        """
        try:
            webbrowser.open(GRASPGENX_VIZ_URL)
            self.append_chat(
                "system",
                f"GraspGenX 뷰어를 새 창으로 열었습니다 ({GRASPGENX_VIZ_URL}). "
                "cobot2_ws의 grasp_bridge_node가 안 떠 있으면 빈 화면/연결 실패만 보입니다.",
            )
        except Exception as exc:
            self.append_chat("system", f"GraspGenX 뷰어를 열지 못했습니다: {exc}")

    # ------------------------------------------------------- clarification

    def clear_clarification(self) -> None:
        for child in self.clarify_frame.winfo_children():
            child.destroy()
        self.clarify_photos.clear()
        self.clarify_frame.grid_remove()

    def show_clarification(self, object_ids: list[str]) -> None:
        """Crop each candidate out of the live frame and number it.

        Numbering is what makes the answer sayable: "왼쪽에서 두 번째 사과"
        is hard to say and harder to parse, "2번" is neither.
        """
        self.clear_clarification()
        if not object_ids:
            return

        snapshot = self.bridge.snapshot()
        frame = snapshot["keep_frame"]
        scene = snapshot["scene"]
        if frame is None or scene is None:
            return

        boxes = {obj["id"]: obj["bbox"] for obj in scene["objects"]}
        shown = 0
        for index, object_id in enumerate(object_ids, start=1):
            box = boxes.get(object_id)
            if box is None:
                continue
            try:
                photo = rgb_to_tk_photo(crop_object(frame, box), 130, 130)
            except Exception:
                continue
            self.clarify_photos.append(photo)

            cell = ttk.Frame(self.clarify_frame, style="Panel.TFrame")
            cell.grid(row=0, column=shown, padx=6, pady=4)
            tk.Label(cell, image=photo, bg=PANEL_2, bd=0).pack()
            tk.Button(
                cell,
                text=f"{index}번",
                bg="#343a40",
                fg=FG,
                activebackground="#4a5057",
                relief="flat",
                padx=8,
                command=lambda answer=f"{index}번": self.handle_user_text(answer),
            ).pack(fill="x", pady=(4, 0))
            shown += 1

        if shown:
            self.clarify_frame.grid()

    # --------------------------------------------------------------- voice

    def _voice_press(self, _event=None):
        if self.busy or self.recording:
            return "break"
        try:
            import sounddevice as sd

            self.audio_frames = []

            def callback(indata, _frames, _time_info, status):
                del status  # non-fatal; the audio callback must stay light
                self.audio_frames.append(indata.copy())

            self.audio_stream = sd.InputStream(
                samplerate=SAMPLE_RATE, channels=1, dtype="int16", callback=callback
            )
            self.audio_stream.start()
            self.record_started = time.monotonic()
            self.recording = True
            self.voice_button.configure(text="● 녹음 중", bg="#9e2a2b")
            self.bottom_status_var.set("음성 녹음 중 — 버튼을 떼면 인식")
        except Exception as exc:
            self.append_chat("system", f"마이크 시작 실패: {exc}")
            self._cleanup_audio_stream()
        return "break"

    def _voice_release(self, _event=None):
        if not self.recording:
            return "break"

        elapsed = time.monotonic() - self.record_started
        self.recording = False
        self._cleanup_audio_stream()
        self.voice_button.configure(text="🎙 누르고 말하기", bg="#343a40")

        if elapsed < MIN_RECORD_SECONDS or not self.audio_frames:
            self.append_chat("system", "음성이 너무 짧아 입력을 취소했습니다.")
            self.bottom_status_var.set("음성 입력 대기")
            return "break"

        audio = np.concatenate(self.audio_frames, axis=0)
        AUDIO_PATH.parent.mkdir(parents=True, exist_ok=True)
        try:
            from scipy.io.wavfile import write

            write(str(AUDIO_PATH), SAMPLE_RATE, audio)
        except Exception as exc:
            self.append_chat("system", f"음성 파일 저장 실패: {exc}")
            return "break"

        self._set_busy(True)
        self.bottom_status_var.set("음성을 텍스트로 변환하는 중...")

        def worker() -> None:
            try:
                self.events.put(("voice_text", self._get_transcriber().transcribe(AUDIO_PATH)))
            except Exception as exc:
                self.events.put(("error", f"음성 인식 실패: {exc}"))
            finally:
                self.events.put(("busy", False))

        threading.Thread(target=worker, name="vla-stt", daemon=True).start()
        return "break"

    def _get_transcriber(self):
        if self.transcriber is None:
            from vla_system.agent.llm import AgentLLM

            self.transcriber = AgentLLM(stt_model=STT_MODEL)
        return self.transcriber

    def _cleanup_audio_stream(self) -> None:
        stream = self.audio_stream
        self.audio_stream = None
        if stream is None:
            return
        for method in ("stop", "close"):
            try:
                getattr(stream, method)()
            except Exception:
                pass

    # ------------------------------------------------------------ pipeline

    def toggle_pipeline(self) -> None:
        if self.pipeline_process is None:
            self.start_pipeline()
        else:
            self.stop_pipeline()

    def _wait_repainting(self, seconds: float) -> None:
        """Sleep without letting the window go grey.

        `update_idletasks` redraws but does not deliver input events, so a click
        during cleanup cannot re-enter `start_pipeline`.
        """
        self.root.update_idletasks()
        time.sleep(seconds)

    def _clear_leftover_pipeline(self, *, include_realsense: bool) -> bool:
        """Kill every leftover pipeline process before starting a new one.

        Two `vla_pick_bridge_node`s racing cobot2_ws's pick_fsm is what this
        prevents: each takes orders from a different agent and they fight
        over /vla/pick_command. Leftovers
        are normal rather than exceptional -- closing or killing the GUI does not
        stop the `ros2 launch` it started, and a node that crashed out of a
        launch can outlive its siblings -- so this runs at every start instead of
        asking the user to go clean up in a terminal.

        ``include_realsense`` must be False whenever this GUI's own launch will
        pass ``enable_realsense:=false`` (cobot2_ws-integration mode, see
        `start_pipeline`) -- otherwise this kills a RealSense process this run
        never intended to touch, e.g. a camera the user started by hand
        (`reals1280` alias) or cobot2_ws's own launch (2026-08-11, real-hardware
        session: the GUI's leftover-cleanup was tearing down a manually-started
        camera and the manually-started pipeline it was supposed to leave
        alone).

        Returns False only when something refused to die, which is worth
        blocking on.
        """

        pattern = f"{PIPELINE_PATTERN}|{REALSENSE_PATTERN}" if include_realsense else PIPELINE_PATTERN
        leftover = find_existing_pipeline_pids(pattern)
        if not leftover:
            return True

        listing = "\n".join(f"  [{pid}] {command}" for pid, command in leftover[:8])
        if len(leftover) > 8:
            listing += f"\n  ... 외 {len(leftover) - 8}개"
        self.append_chat(
            "system",
            f"이전 vla_system 프로세스 {len(leftover)}개를 정리합니다:\n{listing}",
        )

        self.pipeline_button.configure(state="disabled")
        try:
            survivors = escalate_termination(
                [pid for pid, _ in leftover], wait=self._wait_repainting
            )
        finally:
            self.pipeline_button.configure(state="normal")

        if survivors:
            names = {pid: command for pid, command in leftover}
            detail = "\n".join(f"  [{pid}] {names.get(pid, '')}" for pid in survivors)
            self.append_chat("system", f"정리하지 못한 프로세스가 있어 시작을 중단했습니다:\n{detail}")
            messagebox.showerror(
                "프로세스 정리 실패",
                f"다음 프로세스가 SIGKILL에도 종료되지 않았습니다:\n\n{detail}\n\n"
                "이 상태로 시작하면 vla_pick_bridge_node가 둘이 되어 cobot2_ws에 서로 "
                "다른 동작을 동시에 보낼 수 있습니다.\n"
                "터미널에서 직접 확인한 뒤 다시 시도하세요.",
                parent=self.root,
                icon="error",
            )
            return False

        self.append_chat("system", f"이전 프로세스 {len(leftover)}개를 정리했습니다.")
        return True

    def start_pipeline(self) -> None:
        # pick_bridge 켜짐(기본값) = 이 launch가 enable_realsense:=false로 뜬다 =
        # 카메라는 남이 잡고 있다는 전제(cobot2_ws 쪽 launch나 사용자가 직접 켠
        # reals1280 같은 별도 alias) -- 그 카메라 프로세스는 우리 소유가 아니니
        # 정리 대상에서 뺀다. 이 판단이 leftover 정리보다 먼저 있어야 한다.
        pick_bridge = bool(self.pick_bridge_var.get())
        if not self._clear_leftover_pipeline(include_realsense=not pick_bridge):
            return

        # robot_node/wrist_grasp_node는 삭제됐다 -- 로봇 모션·손목 파지 모두
        # cobot2_ws pick_fsm이 전담이라 이 launch에 enable_robot/enable_wrist_grasp
        # 인자 자체가 더 이상 없다(CLAUDE.md #3).
        command = BASE_LAUNCH_COMMAND + [
            f"enable_pick_bridge:={'true' if pick_bridge else 'false'}",
            # pick_bridge 켜짐 = cobot2_ws 쪽 launch가 카메라를 이미 잡고 있다는 전제
            # (README §4) -- 여기서 또 열면 V4L2 충돌 위험. 꺼짐 = 이 ws 단독 실행이니
            # 이 ws가 카메라를 연다(예전 기본값).
            f"enable_realsense:={'false' if pick_bridge else 'true'}",
        ]
        try:
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
        except FileNotFoundError:
            self.append_chat(
                "system",
                "ros2 실행 파일을 찾지 못했습니다. ROS 2 환경을 source한 터미널에서 "
                "GUI를 실행하세요.",
            )
            return
        except Exception as exc:
            self.append_chat("system", f"VLA 파이프라인 시작 실패: {exc}")
            return

        self.pipeline_process = process
        self.pipeline_button.configure(text="VLA 정지")
        self.pick_bridge_check.configure(state="disabled")
        mode_note = (
            " cobot2_ws 연동(pick_bridge) ON -- 이 창의 '전송'/음성 발화가 곧 FSM"
            " 시작 트리거는 아님, cobot2_ws 쪽 auto_start:=true 또는 /pick/start가"
            " 별도로 필요함(README §3)."
            if pick_bridge
            else " 단독 모드(이 ws 카메라 직접 사용, cobot2_ws 미연동)."
        )
        self.append_chat(
            "system",
            f"VLA 파이프라인을 시작했습니다.{mode_note}"
            f"\n전체 로그: {PIPELINE_LOG_PATH}",
        )
        threading.Thread(
            target=self._read_pipeline_output,
            args=(process,),
            name="vla-launch-log",
            daemon=True,
        ).start()

    def _read_pipeline_output(self, process: subprocess.Popen) -> None:
        log_file = None
        try:
            PIPELINE_LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
            log_file = open(PIPELINE_LOG_PATH, "w", encoding="utf-8", buffering=1)
            log_file.write(
                f"# vla_system pipeline log, started {datetime.now().isoformat()}\n"
            )
        except OSError as exc:
            self.events.put(("pipeline_log", f"파이프라인 로그 파일 열기 실패: {exc}"))

        if process.stdout is not None:
            # Per-prefix ("[nodename-N]") set of streams currently mid-traceback,
            # so a crashing node's full stack reaches the chat instead of only
            # launch's one-line "process has died" summary.
            in_traceback: set[str] = set()
            for raw in process.stdout:
                line = raw.rstrip()
                if log_file is not None:
                    log_file.write(raw)
                if not line:
                    continue
                match = _LOG_PREFIX_RE.match(line)
                prefix = match.group(1) if match else None
                remainder = line[match.end() :] if match else line
                lowered = line.lower()

                if prefix is not None and prefix in in_traceback:
                    self.events.put(("pipeline_log", line))
                    if not remainder.startswith((" ", "\t")):
                        # Unindented line after "Traceback ...": the terminal
                        # "ExceptionType: message" line.
                        in_traceback.discard(prefix)
                    continue

                if any(token in lowered for token in _INTERESTING_LOG_TOKENS):
                    self.events.put(("pipeline_log", line))
                    if prefix is not None and "traceback (most recent call last)" in lowered:
                        in_traceback.add(prefix)
        if log_file is not None:
            log_file.close()
        self.events.put(("pipeline_exit", process.poll()))

    def stop_pipeline(self) -> None:
        process = self.pipeline_process
        self.pipeline_process = None
        if process is None:
            return
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait(timeout=8.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
        except ProcessLookupError:
            pass

        self.pipeline_button.configure(text="VLA 시작")
        self.pick_bridge_check.configure(state="normal")
        self.append_chat("system", "GUI가 시작한 VLA 파이프라인을 정지했습니다.")

    # ------------------------------------------------------------- refresh

    def _drain_events(self) -> None:
        while True:
            try:
                kind, payload = self.events.get_nowait()
            except queue.Empty:
                break

            if kind == "reply":
                self._apply_reply(payload)
            elif kind == "robot_state":
                self._apply_robot_state(payload)
            elif kind == "voice_text":
                self.handle_user_text(str(payload))
            elif kind == "error":
                self.append_chat("system", str(payload))
            elif kind == "busy":
                self._set_busy(bool(payload))
            elif kind == "pipeline_log":
                self.append_chat("system", str(payload))
            elif kind == "pipeline_exit":
                self.pipeline_process = None
                self.pipeline_button.configure(text="VLA 시작")
                self.pick_bridge_check.configure(state="normal")
                self.append_chat("system", f"VLA launch 종료: returncode={payload}")

        self.root.after(30, self._drain_events)

    def _apply_reply(self, payload: dict) -> None:
        kind = payload["kind"]
        if kind == "ask_clarification":
            self.append_chat("assistant", payload["text"], tag="ask_body")
            self.show_clarification(payload["focus"])
        elif kind == "error":
            self.append_chat("system", payload["text"])
        else:
            self.append_chat("assistant", payload["text"])

    def _apply_robot_state(self, payload: dict) -> None:
        self.robot_status_var.set(robot_state_line(payload))
        key = (
            payload["last_action_id"],
            payload["last_action"],
            payload["last_result"],
            payload["details"],
        )
        if not payload["last_result"] or key == self.last_state_key:
            return
        self.last_state_key = key
        wording = {
            "succeeded": "완료했습니다",
            "failed": "실패했습니다",
            "cancelled": "중단했습니다",
            "rejected": "받아들이지 않았습니다",
        }.get(payload["last_result"], payload["last_result"])
        detail = f" — {payload['details']}" if payload["details"] else ""
        self.append_chat("system", f"로봇: {payload['last_action']} {wording}{detail}")

    def _refresh_video(self) -> None:
        frame = self.bridge.take_frame()
        if frame is not None:
            try:
                max_w = max(320, self.video_label.winfo_width() - 4)
                max_h = max(240, self.video_label.winfo_height() - 4)
                self.photo = rgb_to_tk_photo(frame, max_w, max_h)
                self.video_label.configure(image=self.photo, text="")
            except Exception as exc:
                self.video_label.configure(image="", text=f"영상 표시 오류: {exc}")
        self.root.after(33, self._refresh_video)

    def _refresh_status(self) -> None:
        snapshot = self.bridge.snapshot()
        scene = snapshot["scene"]

        now = time.monotonic()
        elapsed = now - self.last_fps_time
        if elapsed >= 1.0:
            count = int(snapshot["frame_count"])
            self.current_fps = (count - self.last_frame_count) / elapsed
            self.last_frame_count = count
            self.last_fps_time = now

        frame_age = (
            now - float(snapshot["last_frame_monotonic"])
            if snapshot["last_frame_monotonic"]
            else float("inf")
        )

        self._render_scene_table(scene)
        if scene is None:
            self.vision_status_var.set("장면 데이터 대기 중")
        else:
            pickable = sum(1 for obj in scene["objects"] if obj["pickable"])
            warning = (
                "" if scene["calibration_ok"] else " | ⚠ 테이블 보정 없음 (좌표 사용 불가)"
            )
            self.vision_status_var.set(
                f"물체 {len(scene['objects'])}개 (집기 가능 {pickable}개) | "
                f"GUI {self.current_fps:.1f} FPS{warning}"
            )

        pipeline_state = (
            "GUI launch ON" if self.pipeline_process is not None else "GUI launch OFF"
        )
        camera_state = "camera OK" if frame_age < 1.0 else "camera wait"
        self.bottom_status_var.set(f"{pipeline_state} | {camera_state}")
        self.root.after(200, self._refresh_status)

    def _render_scene_table(self, scene: dict | None) -> None:
        for item in self.scene_tree.get_children():
            self.scene_tree.delete(item)
        if scene is None:
            return
        for obj in sorted(scene["objects"], key=lambda o: o["id"]):
            x, y, z = obj["position"]
            self.scene_tree.insert(
                "",
                "end",
                values=(
                    obj["id"],
                    obj["color"],
                    f"{obj['confidence']:.2f}",
                    "O" if obj["pickable"] else "X",
                    f"({x:.3f}, {y:.3f}, {z:.3f})" if obj["pickable"] else "-",
                ),
            )

    def _set_busy(self, busy: bool) -> None:
        self.busy = busy
        state = "disabled" if busy else "normal"
        self.entry.configure(state=state)
        self.send_button.configure(state=state)
        if not busy:
            self.bottom_status_var.set("명령 입력 대기")

    # --------------------------------------------------------------- close

    def on_close(self) -> None:
        if self.recording:
            self.recording = False
            self._cleanup_audio_stream()
        if self.pipeline_process is not None:
            self.stop_pipeline()
        self.root.destroy()


def main() -> None:
    rclpy.init(args=None)
    events: queue.Queue = queue.Queue()
    bridge = GuiRosBridge(events)
    threading.Thread(
        target=rclpy.spin, args=(bridge,), name="vla-ros-spin", daemon=True
    ).start()

    root = tk.Tk()
    app = VLAApp(root, bridge, events)

    # rclpy.init()이 이미 자체 SIGINT 핸들러를 심어놨다 -- Ctrl+C가 오면 rclpy
    # 컨텍스트만 셧다운되고(spin 스레드가 ExternalShutdownException으로 죽음) Tk
    # mainloop()는 그 사실을 모른다. 그 다음 Python 기본 KeyboardInterrupt가 한 번
    # 더 올라오지만, Tkinter의 콜백 래퍼가 이걸 통째로 삼켜서(예외를 로그만 찍고
    # 계속 진행) mainloop() 밖으로 안 나간다 -- 그래서 Ctrl+C를 여러 번 눌러도 창이
    # 안 닫혔다(2026-08-11 확인). 여기서 SIGINT를 직접 잡아 우리 종료 경로
    # (on_close: 파이프라인 서브프로세스 정리 + destroy)로 보낸다. after(0, ...)로
    # 미루는 이유: 시그널 핸들러 자체는 다음 바이트코드 경계에서만 실행되므로,
    # Tk 위젯 조작은 이미 안전한 시점인 메인 루프 콜백 안에서 하도록 넘긴다.
    def _handle_sigint(_signum, _frame) -> None:
        root.after(0, app.on_close)

    signal.signal(signal.SIGINT, _handle_sigint)

    try:
        root.mainloop()
    finally:
        app._cleanup_audio_stream()
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
