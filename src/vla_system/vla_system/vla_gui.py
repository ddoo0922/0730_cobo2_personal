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

from vla_system.process_guard import escalate_termination, find_existing_pipeline_pids

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------

UTTERANCE_TOPIC = "/vla/user_utterance"
ESTOP_TOPIC = "/vla/estop"
ANNOTATED_IMAGE_TOPIC = "/vla/perception/annotated_image"
# The RealSense rides on the wrist, so this view moves with the arm. It is not
# what the agent reasons about -- it is there to watch the approach by eye.
#
# Prefer the annotated version, which draws what the wrist detector actually
# found: "물체가 보이지 않습니다" is impossible to diagnose from a raw frame,
# because it cannot distinguish an empty view from a detector that is pointed at
# the object and not firing. Falls back to raw if vla_wrist is not running.
WRIST_IMAGE_TOPIC = "/vla/wrist/annotated_image"
WRIST_RAW_IMAGE_TOPIC = "/camera/camera/color/image_raw"
SCENE_TOPIC = "/vla/scene"
REPLY_TOPIC = "/vla/agent/reply"
ROBOT_STATE_TOPIC = "/vla/robot/state"

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
    "enable_realsense:=true",
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
        "error": "오류",
    }.get(state["status"], state["status"])
    holding = state["holding_class"] or "없음"
    mode = "실제 모션" if state["motion_enabled"] else "DRY-RUN"
    tail = ""
    if state["last_action"]:
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
        self._latest_wrist_frame: np.ndarray | None = None
        self._wrist_annotated_monotonic = 0.0
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
            Image, WRIST_IMAGE_TOPIC, self._wrist_image_callback, stream_qos
        )
        self.create_subscription(
            Image, WRIST_RAW_IMAGE_TOPIC, self._wrist_raw_image_callback, stream_qos
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

    def _wrist_image_callback(self, message: Image) -> None:
        try:
            frame = ros_image_to_rgb(message)
        except Exception as exc:
            self.get_logger().warning(f"GUI wrist image decode failed: {exc}")
            return
        with self._lock:
            self._latest_wrist_frame = frame
            self._wrist_annotated_monotonic = time.monotonic()

    def _wrist_raw_image_callback(self, message: Image) -> None:
        """Raw fallback, used only while no annotated frame is arriving."""
        with self._lock:
            recent = time.monotonic() - self._wrist_annotated_monotonic < 2.0
        if recent:
            return
        try:
            frame = ros_image_to_rgb(message)
        except Exception:
            return
        with self._lock:
            self._latest_wrist_frame = frame

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

    def take_wrist_frame(self) -> np.ndarray | None:
        with self._lock:
            frame = self._latest_wrist_frame
            self._latest_wrist_frame = None
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
        self.wrist_photo: tk.PhotoImage | None = None
        self.clarify_photos: list[tk.PhotoImage] = []

        self.last_frame_count = 0
        self.last_fps_time = time.monotonic()
        self.current_fps = 0.0
        self.last_state_key: tuple | None = None

        self.real_robot_var = tk.BooleanVar(value=False)
        self.wrist_grasp_var = tk.BooleanVar(value=False)

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

        self.robot_check = ttk.Checkbutton(
            controls, text="실제 로봇 모션", variable=self.real_robot_var
        )
        self.robot_check.grid(row=0, column=1, padx=(0, 8))

        # Off by default: it loads a second YOLO plus GraspGenX (~1.2 GB VRAM)
        # and is only useful with the RealSense actually mounted on the wrist.
        self.wrist_check = ttk.Checkbutton(
            controls, text="손목 파지 (GraspGenX)", variable=self.wrist_grasp_var
        )
        self.wrist_check.grid(row=0, column=2, padx=(0, 8))

        self.pipeline_button = ttk.Button(
            controls, text="VLA 시작", command=self.toggle_pipeline
        )
        self.pipeline_button.grid(row=0, column=3)

        # ------------------------------------------------- left: perception

        left = ttk.Frame(outer, style="Panel.TFrame", padding=10)
        left.grid(row=1, column=0, sticky="nsew", padx=(0, 8))
        left.columnconfigure(0, weight=1)
        left.rowconfigure(1, weight=4)
        left.rowconfigure(4, weight=3)
        left.rowconfigure(5, weight=2)

        ttk.Label(
            left, text="고정 Webcam / YOLO-seg  (LLM이 보는 화면)", style="Section.TLabel"
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

        ttk.Label(
            left,
            text="손목 RealSense + YOLO-seg  (파지용, 대화 판단에는 쓰이지 않음)",
            style="Section.TLabel",
        ).grid(row=3, column=0, sticky="w", pady=(4, 0))
        self.wrist_label = tk.Label(
            left,
            text=f"{WRIST_IMAGE_TOPIC} 대기 중",
            bg="#090a0c",
            fg=MUTED,
            bd=0,
        )
        self.wrist_label.grid(row=4, column=0, sticky="nsew", pady=(7, 8))

        table_frame = ttk.Frame(left, style="Panel.TFrame")
        table_frame.grid(row=5, column=0, sticky="nsew")
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

    def _clear_leftover_pipeline(self) -> bool:
        """Kill every leftover pipeline process before starting a new one.

        Two `robot_node`s on one arm is what this prevents: each takes orders
        from a different agent and they fight over the same hardware. Leftovers
        are normal rather than exceptional -- closing or killing the GUI does not
        stop the `ros2 launch` it started, and a node that crashed out of a
        launch can outlive its siblings -- so this runs at every start instead of
        asking the user to go clean up in a terminal.

        Returns False only when something refused to die, which is worth
        blocking on.
        """

        leftover = find_existing_pipeline_pids()
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
                "이 상태로 시작하면 robot_node가 둘이 되어 같은 팔에 서로 다른 동작을 "
                "동시에 보낼 수 있습니다.\n"
                "터미널에서 직접 확인한 뒤 다시 시도하세요.",
                parent=self.root,
                icon="error",
            )
            return False

        self.append_chat("system", f"이전 프로세스 {len(leftover)}개를 정리했습니다.")
        return True

    def start_pipeline(self) -> None:
        if not self._clear_leftover_pipeline():
            return

        motion_enabled = bool(self.real_robot_var.get())
        if motion_enabled and not messagebox.askyesno(
            "실제 로봇 모션",
            "robot_node를 motion_enabled=true로 시작합니다.\n"
            "Doosan M0609와 RG2 주변이 안전한 상태인지 확인했습니까?",
            parent=self.root,
        ):
            return

        wrist_grasp = bool(self.wrist_grasp_var.get())
        command = BASE_LAUNCH_COMMAND + [
            f"motion_enabled:={'true' if motion_enabled else 'false'}",
            f"enable_wrist_grasp:={'true' if wrist_grasp else 'false'}",
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
        self.robot_check.configure(state="disabled")
        self.wrist_check.configure(state="disabled")
        self.append_chat(
            "system",
            f"VLA 파이프라인을 시작했습니다. 실행 모드: "
            f"{'실제 로봇' if motion_enabled else 'DRY-RUN'}"
            f"{' | 손목 파지 ON' if wrist_grasp else ''}"
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
        self.robot_check.configure(state="normal")
        self.wrist_check.configure(state="normal")
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
                self.robot_check.configure(state="normal")
                self.wrist_check.configure(state="normal")
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

        wrist = self.bridge.take_wrist_frame()
        if wrist is not None:
            try:
                max_w = max(240, self.wrist_label.winfo_width() - 4)
                max_h = max(180, self.wrist_label.winfo_height() - 4)
                self.wrist_photo = rgb_to_tk_photo(wrist, max_w, max_h)
                self.wrist_label.configure(image=self.wrist_photo, text="")
            except Exception as exc:
                self.wrist_label.configure(image="", text=f"손목 영상 표시 오류: {exc}")
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
    try:
        root.mainloop()
    finally:
        app._cleanup_audio_stream()
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
