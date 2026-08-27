from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


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
        "urdf/so101_arm_uav_gazebo.urdf.xacro",
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
        PACKAGE_ROOT / "urdf" / "so101_arm_uav_gazebo.urdf.xacro",
        PACKAGE_ROOT / "models" / "custom_quad_333" / "custom_quad_333.sdf",
    )
    for path in runtime_files:
        content = path.read_text(encoding="utf-8")
        assert "/home/ljj" not in content
        assert "桌面/" not in content


def test_display_publisher_owns_the_new_package():
    script = (
        PACKAGE_ROOT / "scripts" / "publish_custom_quad_display.py"
    ).read_text(encoding="utf-8")
    assert "arm_uav_joint" in script
    assert "custom_quad_333" in script


def test_joint_xacro_reuses_target_arm_description():
    xacro = (
        PACKAGE_ROOT / "urdf" / "so101_arm_uav_gazebo.urdf.xacro"
    ).read_text(encoding="utf-8")
    assert "so-101_description" in xacro
    assert "so101_arm_macro.urdf.xacro" in xacro
