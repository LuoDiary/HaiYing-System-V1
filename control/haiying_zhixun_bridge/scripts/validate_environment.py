from __future__ import annotations

import json
import platform
import shutil
import sys
from pathlib import Path


BRIDGE_ROOT = Path(__file__).resolve().parents[1]
if str(BRIDGE_ROOT) not in sys.path:
    sys.path.insert(0, str(BRIDGE_ROOT))

from haiying_zhixun_bridge.config import load_bridge_config
from haiying_zhixun_bridge.ik_client import IkClient


REPOSITORY_ROOT = (
    BRIDGE_ROOT.parent.parent if BRIDGE_ROOT.parent.name == "projects" else BRIDGE_ROOT.parent
)
DEFAULT_CONFIG_PATH = BRIDGE_ROOT / "config" / "arm_bridge.yaml"


def _command_status(name: str) -> str | None:
    resolved = shutil.which(name)
    if resolved is not None:
        return resolved
    suffix = ".exe" if platform.system() == "Windows" else ""
    executable_directory = Path(sys.executable).parent
    command_directory = executable_directory / "Scripts" if platform.system() == "Windows" else executable_directory
    environment_candidate = command_directory / f"{name}{suffix}"
    return str(environment_candidate) if environment_candidate.is_file() else None


def _com_port_names() -> tuple[str, ...]:
    try:
        from serial.tools import list_ports
    except ImportError:
        return ()
    return tuple(port.device for port in list_ports.comports())


def _resolve_package_uri(uri: str) -> dict[str, object]:
    prefix = "package://"
    if not uri.startswith(prefix):
        return {"uri": uri, "resolved": None, "exists": False, "error": "不是 package:// URI"}
    package_and_path = uri[len(prefix) :]
    package_name, separator, relative_path = package_and_path.partition("/")
    if not package_name or not separator or not relative_path:
        return {"uri": uri, "resolved": None, "exists": False, "error": "URI 格式不完整"}
    try:
        from ament_index_python.packages import get_package_share_directory

        resolved = Path(get_package_share_directory(package_name)) / relative_path
    except (ImportError, LookupError) as error:
        source_candidate = REPOSITORY_ROOT / "src" / package_name / relative_path
        if source_candidate.is_file():
            return {
                "uri": uri,
                "resolved": str(source_candidate),
                "exists": True,
                "source_fallback": True,
            }
        return {"uri": uri, "resolved": None, "exists": False, "error": str(error)}
    return {"uri": uri, "resolved": str(resolved), "exists": resolved.is_file()}


def main() -> None:
    config = load_bridge_config(DEFAULT_CONFIG_PATH)
    calibration_path = (
        Path.home()
        / ".cache"
        / "huggingface"
        / "lerobot"
        / "calibration"
        / "robots"
        / config.robot.robot_type
        / f"{config.robot.calibration_id}.json"
    )
    ik_status: dict[str, object]
    try:
        ik_status = IkClient(config.ik).health()
    except (RuntimeError, ValueError) as error:
        ik_status = {"status": "unavailable", "error": str(error)}
    commands = {
        name: _command_status(name)
        for name in ("lerobot-ik-sim", "lerobot-ik-real", "ros2", "colcon", "gazebo", "gz")
    }
    model_status = _resolve_package_uri(config.geometry.urdf_source)
    ros_simulation_available = bool(
        commands["ros2"]
        and commands["colcon"]
        and (commands["gazebo"] or commands["gz"])
        and model_status["exists"]
    )
    report = {
        "project": "海鹰智巡",
        "role": "曹圆圆：机械臂建模、碰撞与仿真—实机桥接",
        "platform": platform.platform(),
        "python": sys.version.split()[0],
        "repository_root": str(REPOSITORY_ROOT),
        "commands": commands,
        "ik_server": ik_status,
        "model": model_status,
        "robot": {
            "port": config.robot.port,
            "port_detected": config.robot.port in _com_port_names(),
            "calibration_file": str(calibration_path),
            "calibration_exists": calibration_path.is_file(),
        },
        "verification_boundary": {
            "python_core": "可在当前环境验证",
            "ros2_gazebo_model": (
                "运行依赖与模型已检测，具体功能仍以本次冒烟测试记录为准"
                if ros_simulation_available
                else "依赖或模型不完整，不得声称已运行验证"
            ),
            "hardware": "本脚本只读，不连接机械臂",
        },
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
