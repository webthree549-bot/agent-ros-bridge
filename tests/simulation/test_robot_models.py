"""
Test robot models in simulation - TDD approach
Week 2 Deliverable: Refined Robot Models
"""

import pytest
import subprocess
import time
import os
import signal
import yaml
from pathlib import Path


class TestTurtleBotModel:
    """Test TurtleBot3 Waffle model in simulation"""

    def test_turtlebot_model_files_exist(self):
        """RED: TurtleBot3 model files should exist"""
        model_path = Path("simulation/models/turtlebot3_waffle")
        assert model_path.exists(), f"Model directory not found: {model_path}"
        assert (model_path / "model.config").exists(), "model.config not found"
        assert (model_path / "model.sdf").exists(), "model.sdf not found"

    def test_turtlebot_model_config_valid(self):
        """RED: model.config should be valid XML"""
        import xml.etree.ElementTree as ET

        config_path = Path("simulation/models/turtlebot3_waffle/model.config")
        assert config_path.exists()

        tree = ET.parse(config_path)
        root = tree.getroot()

        assert root.tag == "model"
        assert root.find("name") is not None
        assert root.find("sdf") is not None

    def test_turtlebot_model_sdf_valid(self):
        """RED: model.sdf should be valid SDF"""
        import xml.etree.ElementTree as ET

        sdf_path = Path("simulation/models/turtlebot3_waffle/model.sdf")
        assert sdf_path.exists()

        tree = ET.parse(sdf_path)
        root = tree.getroot()

        assert root.tag == "sdf"
        model = root.find("model")
        assert model is not None
        assert model.get("name") == "turtlebot3_waffle"

    def test_turtlebot_has_required_links(self):
        """RED: TurtleBot should have all required links"""
        import xml.etree.ElementTree as ET

        sdf_path = Path("simulation/models/turtlebot3_waffle/model.sdf")
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        model = root.find("model")

        link_names = [link.get("name") for link in model.findall("link")]

        required_links = [
            "base_footprint",
            "base_link",
            "wheel_left_link",
            "wheel_right_link",
            "base_scan",
            "camera_link",
        ]

        for link in required_links:
            assert link in link_names, f"Required link '{link}' not found"

    def test_turtlebot_has_required_joints(self):
        """RED: TurtleBot should have all required joints"""
        import xml.etree.ElementTree as ET

        sdf_path = Path("simulation/models/turtlebot3_waffle/model.sdf")
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        model = root.find("model")

        joint_names = [joint.get("name") for joint in model.findall("joint")]

        required_joints = ["wheel_left_joint", "wheel_right_joint"]

        for joint in required_joints:
            assert joint in joint_names, f"Required joint '{joint}' not found"

    def test_turtlebot_has_lidar_sensor(self):
        """RED: TurtleBot should have Lidar sensor"""
        import xml.etree.ElementTree as ET

        sdf_path = Path("simulation/models/turtlebot3_waffle/model.sdf")
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        model = root.find("model")

        # Find base_scan link
        base_scan = None
        for link in model.findall("link"):
            if link.get("name") == "base_scan":
                base_scan = link
                break

        assert base_scan is not None, "base_scan link not found"

        sensors = base_scan.findall("sensor")
        assert len(sensors) > 0, "No sensors found on base_scan"

        lidar = None
        for sensor in sensors:
            if sensor.get("name") == "lidar":
                lidar = sensor
                break

        assert lidar is not None, "Lidar sensor not found"
        assert lidar.get("type") == "ray", "Lidar should be type 'ray'"

    def test_turtlebot_has_camera_sensor(self):
        """RED: TurtleBot should have camera sensor"""
        import xml.etree.ElementTree as ET

        sdf_path = Path("simulation/models/turtlebot3_waffle/model.sdf")
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        model = root.find("model")

        # Find camera_link
        camera_link = None
        for link in model.findall("link"):
            if link.get("name") == "camera_link":
                camera_link = link
                break

        assert camera_link is not None, "camera_link not found"

        sensors = camera_link.findall("sensor")
        assert len(sensors) > 0, "No sensors found on camera_link"

        camera = None
        for sensor in sensors:
            if sensor.get("name") == "camera":
                camera = sensor
                break

        assert camera is not None, "Camera sensor not found"
        assert camera.get("type") == "camera", "Sensor should be type 'camera'"

    def test_turtlebot_has_diff_drive_plugin(self):
        """RED: TurtleBot should have differential drive plugin"""
        import xml.etree.ElementTree as ET

        sdf_path = Path("simulation/models/turtlebot3_waffle/model.sdf")
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        model = root.find("model")

        plugins = model.findall("plugin")
        plugin_names = [(p.get("name"), p.get("filename")) for p in plugins]

        diff_drive_found = any(
            name == "diff_drive" and filename == "libgazebo_ros_diff_drive.so"
            for name, filename in plugin_names
        )

        assert diff_drive_found, "Differential drive plugin not found"

    def test_turtlebot_physics_params_tuned(self):
        """RED: TurtleBot should have tuned physics parameters"""
        import xml.etree.ElementTree as ET

        sdf_path = Path("simulation/models/turtlebot3_waffle/model.sdf")
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        model = root.find("model")

        # Check wheel friction
        for link in model.findall("link"):
            if "wheel" in link.get("name", ""):
                collision = link.find("collision")
                if collision is not None:
                    surface = collision.find("surface")
                    if surface is not None:
                        friction = surface.find("friction")
                        if friction is not None:
                            ode = friction.find("ode")
                            if ode is not None:
                                mu = ode.find("mu")
                                assert mu is not None, "Wheel friction mu not set"
                                assert float(mu.text) > 0, "Wheel friction should be > 0"

    def test_turtlebot_spawns_correctly(self):
        """RED: TurtleBot3 should spawn in Gazebo without errors"""
        # This is an integration test that requires Gazebo
        # For now, we'll check that the model is valid
        import xml.etree.ElementTree as ET

        sdf_path = Path("simulation/models/turtlebot3_waffle/model.sdf")
        assert sdf_path.exists(), "Model SDF file not found"

        # Parse to validate XML
        tree = ET.parse(sdf_path)
        root = tree.getroot()

        # Verify model has required plugins for ROS integration
        model = root.find("model")
        plugins = model.findall("plugin")

        plugin_filenames = [p.get("filename") for p in plugins]

        # Should have at least diff drive and joint state publisher
        assert "libgazebo_ros_diff_drive.so" in plugin_filenames, "Missing diff drive plugin"
        assert (
            "libgazebo_ros_joint_state_publisher.so" in plugin_filenames
        ), "Missing joint state publisher"

    def test_turtlebot_topics_published(self):
        """RED: TurtleBot should publish required topics"""
        # This test verifies the model configuration is correct for topic publishing
        import xml.etree.ElementTree as ET

        sdf_path = Path("simulation/models/turtlebot3_waffle/model.sdf")
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        model = root.find("model")

        # Check diff drive plugin publishes odom and subscribes to cmd_vel
        plugins = model.findall("plugin")

        diff_drive = None
        for plugin in plugins:
            if plugin.get("name") == "diff_drive":
                diff_drive = plugin
                break

        assert diff_drive is not None, "Diff drive plugin not found"

        # Check for cmd_vel topic
        cmd_vel = diff_drive.find("cmd_vel_topic")
        assert cmd_vel is not None, "cmd_vel_topic not configured"
        assert cmd_vel.text == "cmd_vel", f"Expected 'cmd_vel', got '{cmd_vel.text}'"

        # Check for odom topic
        odom = diff_drive.find("odometry_topic")
        assert odom is not None, "odometry_topic not configured"
        assert odom.text == "odom", f"Expected 'odom', got '{odom.text}'"


class TestUR5ArmModel:
    """Test UR5 arm model in simulation"""

    def test_ur5_model_files_exist(self):
        """RED: UR5 model files should exist"""
        model_path = Path("simulation/models/ur5_arm")
        assert model_path.exists(), f"Model directory not found: {model_path}"
        assert (model_path / "model.config").exists(), "model.config not found"
        assert (model_path / "model.sdf").exists(), "model.sdf not found"

    def test_ur5_model_config_valid(self):
        """RED: model.config should be valid XML"""
        import xml.etree.ElementTree as ET

        config_path = Path("simulation/models/ur5_arm/model.config")
        assert config_path.exists()

        tree = ET.parse(config_path)
        root = tree.getroot()

        assert root.tag == "model"
        assert root.find("name") is not None
        assert root.find("sdf") is not None

    def test_ur5_model_sdf_valid(self):
        """RED: model.sdf should be valid SDF"""
        import xml.etree.ElementTree as ET

        sdf_path = Path("simulation/models/ur5_arm/model.sdf")
        assert sdf_path.exists()

        tree = ET.parse(sdf_path)
        root = tree.getroot()

        assert root.tag == "sdf"
        model = root.find("model")
        assert model is not None
        assert model.get("name") == "ur5_arm"

    def test_ur5_has_required_links(self):
        """RED: UR5 should have all required links"""
        import xml.etree.ElementTree as ET

        sdf_path = Path("simulation/models/ur5_arm/model.sdf")
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        model = root.find("model")

        link_names = [link.get("name") for link in model.findall("link")]

        required_links = [
            "base_link",
            "shoulder_link",
            "upper_arm_link",
            "forearm_link",
            "wrist_1_link",
            "wrist_2_link",
            "wrist_3_link",
            "tool0",
        ]

        for link in required_links:
            assert link in link_names, f"Required link '{link}' not found"

    def test_ur5_has_required_joints(self):
        """RED: UR5 should have all required joints"""
        import xml.etree.ElementTree as ET

        sdf_path = Path("simulation/models/ur5_arm/model.sdf")
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        model = root.find("model")

        joint_names = [joint.get("name") for joint in model.findall("joint")]

        required_joints = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        for joint in required_joints:
            assert joint in joint_names, f"Required joint '{joint}' not found"

    def test_ur5_joints_have_limits(self):
        """RED: UR5 joints should have configured limits"""
        import xml.etree.ElementTree as ET

        sdf_path = Path("simulation/models/ur5_arm/model.sdf")
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        model = root.find("model")

        for joint in model.findall("joint"):
            if joint.get("type") == "revolute":
                axis = joint.find("axis")
                assert axis is not None, f"Joint {joint.get('name')} missing axis"

                limit = axis.find("limit")
                assert limit is not None, f"Joint {joint.get('name')} missing limits"

                lower = limit.find("lower")
                upper = limit.find("upper")
                effort = limit.find("effort")
                velocity = limit.find("velocity")

                assert lower is not None, f"Joint {joint.get('name')} missing lower limit"
                assert upper is not None, f"Joint {joint.get('name')} missing upper limit"
                assert effort is not None, f"Joint {joint.get('name')} missing effort limit"
                assert velocity is not None, f"Joint {joint.get('name')} missing velocity limit"

    def test_ur5_has_gripper(self):
        """RED: UR5 should have gripper"""
        import xml.etree.ElementTree as ET

        sdf_path = Path("simulation/models/ur5_arm/model.sdf")
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        model = root.find("model")

        link_names = [link.get("name") for link in model.findall("link")]
        joint_names = [joint.get("name") for joint in model.findall("joint")]

        # Check for gripper links
        assert "gripper_left" in link_names, "gripper_left link not found"
        assert "gripper_right" in link_names, "gripper_right link not found"

        # Check for gripper joints
        assert "gripper_left_joint" in joint_names, "gripper_left_joint not found"
        assert "gripper_right_joint" in joint_names, "gripper_right_joint not found"

    def test_ur5_gripper_is_prismatic(self):
        """RED: Gripper joints should be prismatic"""
        import xml.etree.ElementTree as ET

        sdf_path = Path("simulation/models/ur5_arm/model.sdf")
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        model = root.find("model")

        for joint in model.findall("joint"):
            if "gripper" in joint.get("name", ""):
                assert (
                    joint.get("type") == "prismatic"
                ), f"Gripper joint {joint.get('name')} should be prismatic"

    def test_ur5_has_joint_state_publisher(self):
        """RED: UR5 should have joint state publisher plugin"""
        import xml.etree.ElementTree as ET

        sdf_path = Path("simulation/models/ur5_arm/model.sdf")
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        model = root.find("model")

        plugins = model.findall("plugin")
        plugin_names = [(p.get("name"), p.get("filename")) for p in plugins]

        jsp_found = any(
            name == "joint_state_publisher" and filename == "libgazebo_ros_joint_state_publisher.so"
            for name, filename in plugin_names
        )

        assert jsp_found, "Joint state publisher plugin not found"

    def test_ur5_spawns_correctly(self):
        """RED: UR5 should spawn in Gazebo without errors"""
        import xml.etree.ElementTree as ET

        sdf_path = Path("simulation/models/ur5_arm/model.sdf")
        assert sdf_path.exists(), "Model SDF file not found"

        # Parse to validate XML
        tree = ET.parse(sdf_path)
        root = tree.getroot()

        # Verify model structure
        model = root.find("model")
        links = model.findall("link")
        joints = model.findall("joint")

        # UR5 should have at least 8 links and 6 joints
        assert len(links) >= 8, f"Expected at least 8 links, found {len(links)}"
        assert len(joints) >= 6, f"Expected at least 6 joints, found {len(joints)}"

    def test_ur5_joints_responsive(self):
        """RED: UR5 joints should respond to commands"""
        # This test verifies the model has the necessary plugins for control
        import xml.etree.ElementTree as ET

        sdf_path = Path("simulation/models/ur5_arm/model.sdf")
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        model = root.find("model")

        plugins = model.findall("plugin")
        plugin_filenames = [p.get("filename") for p in plugins]

        # Should have trajectory controller for MoveIt integration
        has_trajectory_controller = "libgazebo_ros_joint_pose_trajectory.so" in plugin_filenames

        assert has_trajectory_controller, "Missing joint trajectory controller plugin"

    def test_ur5_has_force_torque_sensor(self):
        """RED: UR5 should have force/torque sensor at end effector"""
        import xml.etree.ElementTree as ET

        sdf_path = Path("simulation/models/ur5_arm/model.sdf")
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        model = root.find("model")

        # Check for ft_sensor_link
        link_names = [link.get("name") for link in model.findall("link")]
        assert "ft_sensor_link" in link_names, "Force/torque sensor link not found"

        # Find ft_sensor_link and check for sensor
        ft_link = None
        for link in model.findall("link"):
            if link.get("name") == "ft_sensor_link":
                ft_link = link
                break

        assert ft_link is not None, "ft_sensor_link not found"

        # Check for force/torque sensor
        sensors = ft_link.findall("sensor")
        ft_sensor = None
        for sensor in sensors:
            if sensor.get("name") == "ft_sensor":
                ft_sensor = sensor
                break

        assert ft_sensor is not None, "Force/torque sensor not found"
        assert ft_sensor.get("type") == "force_torque", "Sensor should be type 'force_torque'"

        # Check for FT sensor plugin (inside sensor element)
        plugins = ft_sensor.findall("plugin")
        plugin_names = [(p.get("name"), p.get("filename")) for p in plugins]

        ft_plugin_found = any(
            name == "ft_sensor_plugin" and filename == "libgazebo_ros_ft_sensor.so"
            for name, filename in plugin_names
        )

        assert ft_plugin_found, "Force/torque sensor plugin not found"


class TestRobotModelIntegration:
    """Integration tests for robot models"""

    def test_models_directory_structure(self):
        """RED: Models directory should have proper structure"""
        models_dir = Path("simulation/models")
        assert models_dir.exists(), "Models directory not found"

        # Check for required models
        required_models = ["turtlebot3_waffle", "ur5_arm"]

        for model in required_models:
            model_path = models_dir / model
            assert model_path.exists(), f"Model '{model}' not found"
            assert (model_path / "model.config").exists(), f"{model}/model.config not found"
            assert (model_path / "model.sdf").exists(), f"{model}/model.sdf not found"

    def test_model_configs_have_required_fields(self):
        """RED: Model configs should have all required fields"""
        import xml.etree.ElementTree as ET

        models_dir = Path("simulation/models")

        for model_path in models_dir.iterdir():
            if model_path.is_dir():
                config_path = model_path / "model.config"
                if config_path.exists():
                    tree = ET.parse(config_path)
                    root = tree.getroot()

                    assert root.find("name") is not None, f"{model_path.name}: missing name"
                    assert root.find("version") is not None, f"{model_path.name}: missing version"
                    assert root.find("sdf") is not None, f"{model_path.name}: missing sdf"
                    assert (
                        root.find("description") is not None
                    ), f"{model_path.name}: missing description"

    def test_models_have_unique_names(self):
        """RED: All models should have unique names"""
        import xml.etree.ElementTree as ET

        models_dir = Path("simulation/models")
        names = []

        for model_path in models_dir.iterdir():
            if model_path.is_dir():
                config_path = model_path / "model.config"
                if config_path.exists():
                    tree = ET.parse(config_path)
                    root = tree.getroot()
                    name = root.find("name").text
                    assert name not in names, f"Duplicate model name: {name}"
                    names.append(name)
