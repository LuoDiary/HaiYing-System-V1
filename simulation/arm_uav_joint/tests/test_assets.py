from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _model_members(xml_text: str) -> tuple[set[str | None], set[str | None]]:
    root = ET.fromstring(xml_text)
    model = root.find("model") if root.tag == "sdf" else root
    assert model is not None
    return (
        {link.get("name") for link in model.findall("link")},
        {joint.get("name") for joint in model.findall("joint")},
    )


def test_every_arm_uav_model_entrypoint_is_a_real_combined_model():
    sdf_path = PACKAGE_ROOT / "models" / "custom_quad_333" / "custom_quad_333.sdf"
    entrypoints = [(sdf_path, sdf_path.read_text(encoding="utf-8"))]
    for xacro_path in (PACKAGE_ROOT / "urdf").glob("*arm*uav*.xacro"):
        expanded = subprocess.run(
            ["xacro", str(xacro_path)],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
        entrypoints.append((xacro_path, expanded))

    for path, xml_text in entrypoints:
        links, joints = _model_members(xml_text)
        assert {"base_link", "base_footprint"} <= links, path
        assert {"rotor_0_joint", "so101_mount_joint"} <= joints, path


def test_custom_quad_contains_so101_payload():
    sdf_path = PACKAGE_ROOT / "models" / "custom_quad_333" / "custom_quad_333.sdf"
    model = ET.parse(sdf_path).getroot().find("model")

    assert model is not None
    assert model.get("name") == "custom_quad_333_so101_v7_belly_final"
    link_names = {link.get("name") for link in model.findall("link")}
    joint_names = {joint.get("name") for joint in model.findall("joint")}
    assert {
        "payload_0p45kg",
        "base_footprint",
        "shoulder",
        "arm_upper",
        "arm_lower",
        "wrist",
        "gripper_base",
    } <= link_names
    assert {
        "payload_0p45kg_joint",
        "so101_mount_joint",
        "J1_Rotation",
        "J2_Shoulder_Pitch",
        "J3_Elbow_Pitch",
        "J4_Wrist_Pitch",
        "J5_Wrist_Roll",
    } <= joint_names
    assert float(model.findtext("link[@name='payload_0p45kg']/inertial/mass")) == 0.45


def test_joint_assets_are_present():
    for relative_path in (
        "models/custom_quad_333/model.config",
        "models/custom_quad_333/custom_quad_333.sdf",
        "models/custom_quad_333/custom_quad_333.sdf.jinja",
        "models/custom_quad_333/meshes/iris.stl",
        "models/custom_quad_333/meshes/iris_prop_ccw.dae",
        "models/custom_quad_333/meshes/iris_prop_cw.dae",
        "models/custom_quad_333/README.md",
        "launch/arm_uav_joint.launch.py",
        "scripts/publish_custom_quad_display.py",
        "scripts/px4_takeoff.py",
    ):
        assert (PACKAGE_ROOT / relative_path).is_file(), relative_path


def test_runtime_files_use_portable_package_paths():
    runtime_files = (
        PACKAGE_ROOT / "launch" / "arm_uav_joint.launch.py",
        PACKAGE_ROOT / "scripts" / "publish_custom_quad_display.py",
        PACKAGE_ROOT / "scripts" / "px4_takeoff.py",
        PACKAGE_ROOT / "models" / "custom_quad_333" / "custom_quad_333.sdf",
    )
    for path in runtime_files:
        content = path.read_text(encoding="utf-8")
        assert "/home/ljj" not in content
        assert "桌面/" not in content
