#!/usr/bin/env python3
import math
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
URDF_DIR = PACKAGE_ROOT / "urdf"


def run_xacro(path: Path) -> str:
    result = subprocess.run(
        ["xacro", str(path)],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return result.stdout


def xyz(text: str | None) -> tuple[float, float, float]:
    values = [float(v) for v in (text or "0 0 0").split()]
    return values[0], values[1], values[2]


def matmul(a, b):
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4))
        for i in range(4)
    )


def origin_matrix(element):
    if element is None:
        x = y = z = roll = pitch = yaw = 0.0
    else:
        x, y, z = xyz(element.get("xyz"))
        roll, pitch, yaw = xyz(element.get("rpy"))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )
    return (
        (rotation[0][0], rotation[0][1], rotation[0][2], x),
        (rotation[1][0], rotation[1][1], rotation[1][2], y),
        (rotation[2][0], rotation[2][1], rotation[2][2], z),
        (0.0, 0.0, 0.0, 1.0),
    )


def transform_point(matrix, point):
    x, y, z = point
    vector = (x, y, z, 1.0)
    return tuple(sum(matrix[i][k] * vector[k] for k in range(4)) for i in range(3))


def validate_model(model_name: str, expected_mass: float):
    xml_text = run_xacro(URDF_DIR / model_name)
    root = ET.fromstring(xml_text)
    links = {link.get("name"): link for link in root.findall("link")}
    joints = root.findall("joint")
    child_names = {joint.find("child").get("link") for joint in joints}
    root_links = set(links) - child_names
    if root_links != {"base_footprint"}:
        raise RuntimeError(f"{model_name}: 根 link 异常: {sorted(root_links)}")

    revolute = [joint for joint in joints if joint.get("type") == "revolute"]
    if [joint.get("name") for joint in revolute] != [
        "J1_Rotation",
        "J2_Shoulder_Pitch",
        "J3_Elbow_Pitch",
        "J4_Wrist_Pitch",
        "J5_Wrist_Roll",
    ]:
        raise RuntimeError(f"{model_name}: 5DOF 关节名称或顺序发生变化")

    total_mass = 0.0
    weighted = [0.0, 0.0, 0.0]
    world = {"base_footprint": origin_matrix(None)}
    unresolved = list(joints)
    while unresolved:
        progressed = False
        for joint in unresolved[:]:
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            if parent in world:
                world[child] = matmul(world[parent], origin_matrix(joint.find("origin")))
                unresolved.remove(joint)
                progressed = True
        if not progressed:
            raise RuntimeError(f"{model_name}: 运动树不连通")

    for link_name, link in links.items():
        inertial = link.find("inertial")
        if inertial is not None:
            mass = float(inertial.find("mass").get("value"))
            local_com = xyz(inertial.find("origin").get("xyz"))
            global_com = transform_point(world[link_name], local_com)
            total_mass += mass
            for index in range(3):
                weighted[index] += mass * global_com[index]

        for mesh in link.findall(".//mesh"):
            uri = mesh.get("filename")
            if uri.startswith("package://arm/"):
                mesh_path = PACKAGE_ROOT / uri.removeprefix("package://arm/")
                if not mesh_path.is_file():
                    raise RuntimeError(f"{model_name}: 缺少网格 {mesh_path}")

    if abs(total_mass - expected_mass) > 1e-9:
        raise RuntimeError(
            f"{model_name}: 质量 {total_mass:.9f} kg，不是仓库基线 {expected_mass:.9f} kg"
        )
    center = tuple(value / total_mass for value in weighted)

    with tempfile.NamedTemporaryFile(suffix=".urdf") as expanded:
        expanded.write(xml_text.encode())
        expanded.flush()
        subprocess.run(["check_urdf", expanded.name], check=True, stdout=subprocess.PIPE)

    return len(links), len(joints), total_mass, center


def main():
    results = [
        ("so101_arm.urdf.xacro", 0.571394),
        ("so101_arm_camera.urdf.xacro", 0.606394),
    ]
    for name, expected_mass in results:
        links, joints, mass, center = validate_model(name, expected_mass)
        print(
            f"PASS {name}: links={links}, joints={joints}, "
            f"mass={mass:.6f} kg, zero-pose CoM(base_footprint)="
            f"({center[0]:.4f}, {center[1]:.4f}, {center[2]:.4f}) m"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
