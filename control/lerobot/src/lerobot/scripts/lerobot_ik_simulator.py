import json
import logging
import threading
import webbrowser
from collections import deque
from dataclasses import asdict, dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import unquote, urlparse
from uuid import uuid4

import draccus

from lerobot.simulators.so101_5dof import (
    DEFAULT_MODEL_FILE,
    DEFAULT_MODEL_ROOT,
    RobotModel,
    load_robot_model,
    make_model_payload,
    plan_cartesian_target,
)
from lerobot.utils.utils import init_logging


logger = logging.getLogger(__name__)
HTML_PATH = Path(__file__).parents[1] / "templates" / "so101_5dof_simulator.html"
THREE_STATIC_ROOT = HTML_PATH.parent / "vendor" / "three"
MAX_REQUEST_BYTES = 64 * 1024
MAX_STORED_PLANS = 20


@dataclass
class IKSimulatorConfig:
    host: str = "127.0.0.1"
    port: int = 8766
    open_browser: bool = True
    model_root: Path = DEFAULT_MODEL_ROOT
    model_file: str = DEFAULT_MODEL_FILE

    def __post_init__(self) -> None:
        if self.host not in {"127.0.0.1", "localhost"}:
            raise ValueError("host must be 127.0.0.1 or localhost for the local simulator")
        if not 1 <= self.port <= 65535:
            raise ValueError("port must be between 1 and 65535")


def _parse_plan_request(payload: dict[str, Any]) -> tuple[list[float], list[float]]:
    target = payload.get("target_position_m")
    initial = payload.get("initial_joint_angles_deg", [0.0] * 5)
    if not isinstance(target, list):
        raise ValueError("target_position_m must be a list")
    if not isinstance(initial, list):
        raise ValueError("initial_joint_angles_deg must be a list")
    return target, initial


class IKSimulatorHandler(BaseHTTPRequestHandler):
    server_version = "LeRobotIKSimulator/1.0"

    def _send_bytes(self, body: bytes, content_type: str, status: HTTPStatus = HTTPStatus.OK) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, payload: dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self._send_bytes(body, "application/json; charset=utf-8", status)

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path in {"/", "/index.html"}:
            self._send_bytes(HTML_PATH.read_bytes(), "text/html; charset=utf-8")
            return
        if path == "/api/model":
            self._send_json(make_model_payload(self.server.robot_model))
            return
        if path == "/api/health":
            self._send_json({"status": "ok", "hardware_connected": False})
            return
        if path.startswith("/static/three/"):
            relative_path = Path(unquote(path.removeprefix("/static/three/")))
            static_path = (THREE_STATIC_ROOT / relative_path).resolve()
            if (
                static_path.is_relative_to(THREE_STATIC_ROOT)
                and static_path.is_file()
                and (static_path.suffix == ".js" or static_path.name == "LICENSE")
            ):
                content_type = (
                    "text/javascript; charset=utf-8"
                    if static_path.suffix == ".js"
                    else "text/plain; charset=utf-8"
                )
                self._send_bytes(static_path.read_bytes(), content_type)
                return
        if path.startswith("/api/plans/"):
            plan_id = unquote(path.removeprefix("/api/plans/"))
            plan = self.server.get_plan(plan_id)
            if plan is None:
                self._send_json({"error": "plan not found"}, HTTPStatus.NOT_FOUND)
            else:
                self._send_json(plan)
            return
        if path.startswith("/model/"):
            relative_path = Path(unquote(path.removeprefix("/model/")))
            mesh_path = (self.server.robot_model.model_root / relative_path).resolve()
            if (
                mesh_path.is_relative_to(self.server.robot_model.model_root)
                and mesh_path.is_file()
                and mesh_path.suffix.lower() == ".stl"
            ):
                self._send_bytes(mesh_path.read_bytes(), "model/stl")
                return
        self._send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        if urlparse(self.path).path != "/api/plan":
            self._send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            return

        try:
            content_length = int(self.headers.get("Content-Length", "0"))
            if not 0 < content_length <= MAX_REQUEST_BYTES:
                raise ValueError("invalid request size")
            payload = json.loads(self.rfile.read(content_length))
            if not isinstance(payload, dict):
                raise ValueError("request body must be a JSON object")
            target, initial = _parse_plan_request(payload)
            result = plan_cartesian_target(self.server.robot_model, target, initial)
            response = asdict(result)
            if result.success:
                response["plan_id"] = self.server.store_plan(response)
            self._send_json(response)
        except (ValueError, TypeError, json.JSONDecodeError) as error:
            self._send_json({"error": str(error)}, HTTPStatus.BAD_REQUEST)

    def log_message(self, format_string: str, *args: object) -> None:
        logger.debug("IK simulator HTTP: " + format_string, *args)


class IKSimulatorServer(ThreadingHTTPServer):
    robot_model: RobotModel

    def initialize_plan_store(self) -> None:
        self._plans: dict[str, dict[str, Any]] = {}
        self._plan_order: deque[str] = deque()
        self._plan_lock = threading.Lock()

    def store_plan(self, payload: dict[str, Any]) -> str:
        plan_id = uuid4().hex
        stored_payload = {**payload, "plan_id": plan_id}
        with self._plan_lock:
            self._plans[plan_id] = stored_payload
            self._plan_order.append(plan_id)
            while len(self._plan_order) > MAX_STORED_PLANS:
                self._plans.pop(self._plan_order.popleft(), None)
        return plan_id

    def get_plan(self, plan_id: str) -> dict[str, Any] | None:
        if not plan_id or "/" in plan_id:
            return None
        with self._plan_lock:
            plan = self._plans.get(plan_id)
            return None if plan is None else dict(plan)


def create_server(host: str, port: int, model_root: Path, model_file: str) -> IKSimulatorServer:
    if not HTML_PATH.is_file():
        raise FileNotFoundError(f"Simulator HTML not found: {HTML_PATH}")
    required_web_assets = (
        THREE_STATIC_ROOT / "three.module.js",
        THREE_STATIC_ROOT / "three.core.js",
        THREE_STATIC_ROOT / "addons" / "controls" / "OrbitControls.js",
        THREE_STATIC_ROOT / "addons" / "loaders" / "STLLoader.js",
    )
    missing_assets = [str(path) for path in required_web_assets if not path.is_file()]
    if missing_assets:
        raise FileNotFoundError(f"Simulator web assets not found: {', '.join(missing_assets)}")
    server = IKSimulatorServer((host, port), IKSimulatorHandler)
    server.robot_model = load_robot_model(model_root, model_file)
    server.initialize_plan_store()
    return server


@draccus.wrap()
def run_simulator(cfg: IKSimulatorConfig) -> None:
    init_logging()
    server = create_server(cfg.host, cfg.port, cfg.model_root, cfg.model_file)
    url = f"http://{cfg.host}:{cfg.port}"
    logger.info("SO-101 5DOF IK simulator: %s", url)
    logger.info("Model: %s", server.robot_model.source_path)
    logger.info("Simulation only: no serial port or robot hardware is accessed.")
    if cfg.open_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        logger.info("IK simulator stopped.")
    finally:
        server.server_close()


def main() -> None:
    run_simulator()


if __name__ == "__main__":
    main()
