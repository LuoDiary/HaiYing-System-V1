# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
from dataclasses import dataclass
from functools import partial
from pathlib import Path
from typing import Any, Callable, Protocol, cast

import cv2
import draccus
from numpy.typing import NDArray
from PIL import Image

try:
    import tkinter as tk
    from PIL import ImageTk
except ImportError as error:
    raise ImportError("lerobot-camera-view requires a Python installation with Tk support.") from error

from lerobot.cameras import ColorMode
from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig
from lerobot.utils.import_utils import require_package
from lerobot.utils.utils import init_logging

logger = logging.getLogger(__name__)
FrameProcessor = Callable[[NDArray[Any]], NDArray[Any]]


class YoloResult(Protocol):
    def plot(self) -> NDArray[Any]: ...


class YoloModel(Protocol):
    def predict(
        self,
        *,
        source: NDArray[Any],
        conf: float,
        imgsz: int,
        device: str | None,
        verbose: bool,
    ) -> list[YoloResult]: ...


@dataclass
class CameraViewConfig:
    camera: OpenCVCameraConfig
    window_name: str = "LeRobot Camera"
    yolo_model_path: Path | None = None
    yolo_confidence: float = 0.25
    yolo_image_size: int = 640
    yolo_device: str | None = None

    def __post_init__(self) -> None:
        if not 0.0 <= self.yolo_confidence <= 1.0:
            raise ValueError("yolo_confidence must be between 0 and 1.")
        if self.yolo_image_size <= 0:
            raise ValueError("yolo_image_size must be greater than 0.")
        if self.yolo_model_path is not None and not self.yolo_model_path.is_file():
            raise FileNotFoundError(f"YOLO model not found: {self.yolo_model_path}")


def _frame_for_display(frame: NDArray[Any], color_mode: ColorMode) -> NDArray[Any]:
    if color_mode == ColorMode.BGR:
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    return frame


def _load_yolo_model(model_path: Path) -> YoloModel:
    require_package("ultralytics-opencv-headless", "yolo", import_name="ultralytics")
    from ultralytics import YOLO

    return cast(YoloModel, YOLO(str(model_path)))


def _run_yolo_inference(
    frame: NDArray[Any],
    color_mode: ColorMode,
    model: YoloModel,
    confidence: float,
    image_size: int,
    device: str | None,
) -> NDArray[Any]:
    bgr_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if color_mode == ColorMode.RGB else frame
    results = model.predict(
        source=bgr_frame,
        conf=confidence,
        imgsz=image_size,
        device=device,
        verbose=False,
    )
    if not results:
        raise RuntimeError("YOLO inference returned no result.")
    annotated_bgr = results[0].plot()
    return cv2.cvtColor(annotated_bgr, cv2.COLOR_BGR2RGB)


def _build_frame_processor(cfg: CameraViewConfig) -> FrameProcessor:
    if cfg.yolo_model_path is None:
        return partial(_frame_for_display, color_mode=cfg.camera.color_mode)

    model = _load_yolo_model(cfg.yolo_model_path)
    return partial(
        _run_yolo_inference,
        color_mode=cfg.camera.color_mode,
        model=model,
        confidence=cfg.yolo_confidence,
        image_size=cfg.yolo_image_size,
        device=cfg.yolo_device,
    )


def preview_camera(camera: OpenCVCamera, window_name: str, frame_processor: FrameProcessor) -> None:
    window = tk.Tk()
    window.title(window_name)
    label = tk.Label(window)
    label.pack()
    running = True

    def request_close() -> None:
        nonlocal running
        running = False

    def request_close_from_event(_event: object) -> None:
        request_close()

    window.protocol("WM_DELETE_WINDOW", request_close)
    window.bind("<KeyPress-q>", request_close_from_event)
    window.bind("<KeyPress-Q>", request_close_from_event)
    window.bind("<Escape>", request_close_from_event)
    window.focus_force()

    try:
        while running:
            frame = camera.read()
            rgb_frame = frame_processor(frame)
            image = Image.fromarray(rgb_frame)
            photo_image = ImageTk.PhotoImage(image=image)
            label.configure(image=photo_image)
            window.update_idletasks()
            window.update()
    finally:
        try:
            window.destroy()
        except tk.TclError:
            pass


@draccus.wrap()
def camera_view(cfg: CameraViewConfig) -> None:
    init_logging()
    frame_processor = _build_frame_processor(cfg)
    camera = OpenCVCamera(cfg.camera)
    try:
        camera.connect()
        logger.info("Camera preview started. Press Q or Esc to exit.")
        preview_camera(camera, cfg.window_name, frame_processor)
    except KeyboardInterrupt:
        logger.info("Camera preview stopped.")
    finally:
        if camera.is_connected:
            camera.disconnect()


def main() -> None:
    camera_view()


if __name__ == "__main__":
    main()
