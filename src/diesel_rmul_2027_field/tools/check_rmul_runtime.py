"""Isolated Gazebo/ROS scene check; captures real renderer output, lidar and contact.

Run after sourcing the updated simulation install. No navigation or SLAM.
Only the process groups created by this check are stopped.
"""

import hashlib
import json
import math
import os
import signal
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from PIL import Image
from rmul_field import PKG, ROOT, A, parts

os.environ["ROS_DOMAIN_ID"] = "97"
os.environ["ROS_LOCALHOST_ONLY"] = "1"
os.environ["GZ_PARTITION"] = "rmul_check_" + uuid.uuid4().hex

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image as ROSImage
from sensor_msgs.msg import LaserScan, PointCloud2


def camera_sdf():
    root = ET.Element("sdf", version="1.9")
    model = ET.SubElement(root, "model", name="rmul_inspection_cameras")
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", name="cameras")
    for name, pose, fov in [
        ("top", "6 4 14 0 1.5707963267948966 1.5707963267948966", "1.05"),
        ("overview", "15 -9 12 0 .65 2.17634", "1.05"),
    ]:
        sensor = ET.SubElement(link, "sensor", name=name, type="camera")
        for tag, value in [
            ("pose", pose),
            ("topic", "/rmul/" + name),
            ("always_on", "true"),
            ("update_rate", "1"),
        ]:
            ET.SubElement(sensor, tag).text = value
        cam = ET.SubElement(sensor, "camera")
        ET.SubElement(cam, "horizontal_fov").text = fov
        im = ET.SubElement(cam, "image")
        for tag, value in [("width", "1500"), ("height", "1000"), ("format", "R8G8B8")]:
            ET.SubElement(im, tag).text = value
        clip = ET.SubElement(cam, "clip")
        ET.SubElement(clip, "near").text = ".05"
        ET.SubElement(clip, "far").text = "50"
    return ET.tostring(root, encoding="unicode")


def rays(xy, angles, height=0.2796202475):
    if (PKG / "maps/rmul_2027/step_geometry.json.gz").is_file():
        from step_field import scan_segments

        segments = scan_segments(PKG, height)
        values = []
        for a in angles:
            dx, dy = math.cos(a), math.sin(a)
            nearest = float("inf")
            for (ax, ay), (bx, by) in segments:
                ex, ey = bx - ax, by - ay
                denom = dx * ey - dy * ex
                if abs(denom) < 1e-12:
                    continue
                qx, qy = ax - xy[0], ay - xy[1]
                distance = (qx * ey - qy * ex) / denom
                u = (qx * dy - qy * dx) / denom
                if distance >= 0.1 and -1e-9 <= u <= 1 + 1e-9:
                    nearest = min(nearest, distance)
            values.append(nearest)
        return np.array(values)
    polygons = [p["polygon"] for p in parts() if p["z0"] <= height <= p["z1"]]
    values = []
    for a in angles:
        dx, dy = math.cos(a), math.sin(a)
        nearest = float("inf")
        for poly in polygons:
            for (ax, ay), (bx, by) in zip(poly, poly[1:] + poly[:1]):
                ex, ey = bx - ax, by - ay
                denom = dx * ey - dy * ex
                if abs(denom) < 1e-12:
                    continue
                qx, qy = ax - xy[0], ay - xy[1]
                distance = (qx * ey - qy * ex) / denom
                u = (qx * dy - qy * dx) / denom
                if distance >= 0.1 and -1e-9 <= u <= 1 + 1e-9:
                    nearest = min(nearest, distance)
        values.append(nearest)
    return np.array(values)


def save_image(msg, path):
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}[msg.encoding]
    data = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    data = data[:, : msg.width * channels].reshape(msg.height, msg.width, channels)[
        :, :, :3
    ]
    if msg.encoding.startswith("bgr"):
        data = data[:, :, ::-1]
    Image.fromarray(data).save(path)
    return {
        "width": msg.width,
        "height": msg.height,
        "encoding": msg.encoding,
        "stddev": float(data.std()),
        "path": str(path),
    }


def main():
    out = ROOT / "output/rmul_runtime" / uuid.uuid4().hex[:8]
    out.mkdir(parents=True)
    share = Path(get_package_share_directory("guodong_sentry_sim"))
    report = {
        "result": "FAIL",
        "installed_share": str(share),
        "scope": "scene rendering, lidar and low-speed wall contact; no navigation/SLAM",
    }
    checked = [
        "launch/sentry_sim.launch.py",
        "worlds/rmul_2027_preview.sdf",
        "models/rmul_2027/model.sdf",
        "models/rmul_2027/materials/textures/official_top.png",
    ]
    if (PKG / "maps/rmul_2027/step_geometry.json.gz").is_file():
        checked += [
            "models/rmul_2027/meshes/step_textured.dae",
            "models/rmul_2027/meshes/step_native.dae",
            "maps/rmul_2027/step_geometry.json.gz",
        ]
    for rel in checked:
        assert (
            hashlib.sha256((share / rel).read_bytes()).digest()
            == hashlib.sha256((PKG / rel).read_bytes()).digest()
        ), rel
    processes = []
    logs = []
    node = None
    pub = None
    try:

        def start(args, name):
            log = (out / (name + ".log")).open("w")
            logs.append(log)
            proc = subprocess.Popen(
                args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
            )
            processes.append(proc)
            return proc

        # Omit world:=... by default to verify the installed default.
        gui = os.environ.get("RMUL_CHECK_GUI") == "1"
        selected = os.environ.get("RMUL_CHECK_WORLD", "rmul_2027")
        if selected not in ("rmul_2027", "rmul_2027_native"):
            raise ValueError(selected)
        world_args = [] if selected == "rmul_2027" else ["world:=" + selected]
        sim = start(
            [
                "ros2",
                "launch",
                "guodong_sentry_sim",
                "sentry_sim.launch.py",
                "headless:=" + ("false" if gui else "true"),
                "rviz:=false",
                "visual_quality:=preview",
            ]
            + world_args,
            "launch",
        )
        report["gui_requested"] = gui
        report["world"] = selected
        rclpy.init()
        node = rclpy.create_node("rmul_scene_probe")
        data = {}
        subscriptions = []
        for typ, topic, key in [
            (LaserScan, "/scan", "scan"),
            (Odometry, "/odom", "odom"),
            (PointCloud2, "/lidar/points", "points"),
            (ROSImage, "/camera/image_raw", "robot_camera"),
            (ROSImage, "/rmul/top", "top"),
            (ROSImage, "/rmul/overview", "overview"),
        ]:
            subscriptions.append(
                node.create_subscription(
                    typ,
                    topic,
                    lambda msg, k=key: data.update({k: msg}),
                    qos_profile_sensor_data,
                )
            )

        def wait_for(predicate, seconds):
            deadline = time.monotonic() + seconds
            while not predicate():
                if time.monotonic() > deadline or sim.poll() is not None:
                    raise TimeoutError(
                        f"Runtime timeout; keys={list(data)}, sim={sim.poll()}"
                    )
                rclpy.spin_once(node, timeout_sec=0.05)

        wait_for(
            lambda: all(k in data for k in ("scan", "odom", "points", "robot_camera")),
            90,
        )
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)
        scan = data["scan"]
        p = data["odom"].pose.pose.position
        report["spawn"] = [p.x, p.y, p.z]
        assert np.linalg.norm(np.array([p.x, p.y]) - A["spawn_xy"]) < 0.03
        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        world_source = ET.parse(share / "worlds" / f"{selected}_preview.sdf")
        spawn_z = next(
            float(inc.findtext("pose").split()[2])
            for inc in world_source.findall(".//include")
            if inc.findtext("name") == "guodong_sentry"
        )
        expected = rays([p.x, p.y], angles, height=0.2796202475 + spawn_z)
        report["world_spawn_z_m"] = spawn_z
        actual = np.array(scan.ranges)
        valid = np.isfinite(actual) & np.isfinite(expected)
        errors = np.abs(actual[valid] - expected[valid])
        report["lidar"] = {
            "rays": len(actual),
            "frame": scan.header.frame_id,
            "compared": int(valid.sum()),
            "median_error_m": float(np.median(errors)),
            "p95_error_m": float(np.quantile(errors, 0.95)),
            "max_error_m": float(np.max(errors)),
            "finite_pattern_agreement": float(
                np.mean(np.isfinite(actual) == np.isfinite(expected))
            ),
        }
        assert len(actual) == 720 and valid.sum() >= 700
        assert report["lidar"]["p95_error_m"] < 0.08
        assert report["lidar"]["finite_pattern_agreement"] > 0.98
        report["pointcloud_bytes"] = len(data["points"].data)
        report["robot_camera"] = save_image(
            data["robot_camera"], out / "robot_camera.png"
        )
        result = subprocess.run(
            [
                "gz",
                "service",
                "-s",
                "/world/sentry_lab/create",
                "--reqtype",
                "gz.msgs.EntityFactory",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "15000",
                "--req",
                "sdf: " + json.dumps(camera_sdf()),
            ],
            capture_output=True,
            text=True,
            timeout=20,
            check=False,
        )
        report["camera_spawn_service"] = result.stdout.strip()
        assert result.returncode == 0 and "true" in result.stdout
        start(
            [
                "ros2",
                "run",
                "ros_gz_bridge",
                "parameter_bridge",
                "/rmul/top@sensor_msgs/msg/Image[gz.msgs.Image",
                "/rmul/overview@sensor_msgs/msg/Image[gz.msgs.Image",
            ],
            "inspection_bridge",
        )
        wait_for(lambda: "top" in data and "overview" in data, 60)
        for name in ("top", "overview"):
            report[name] = save_image(data[name], out / (name + ".png"))
            assert report[name]["stddev"] > 10
        # Contact test against the west wall. Send through the existing watchdog.
        pub = node.create_publisher(Twist, "/cmd_vel", 10)
        cmd = Twist()
        cmd.linear.x = -0.3
        start_wall = time.monotonic()
        start_sim = (
            data["odom"].header.stamp.sec + data["odom"].header.stamp.nanosec * 1e-9
        )
        samples = []
        while True:
            pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.05)
            now = (
                data["odom"].header.stamp.sec + data["odom"].header.stamp.nanosec * 1e-9
            )
            samples.append(data["odom"].pose.pose.position.x)
            if now - start_sim >= 5.0:
                break
            if time.monotonic() - start_wall > 70:
                raise TimeoutError("Contact simulation progress")
        for _ in range(10):
            pub.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.05)
        finalx = data["odom"].pose.pose.position.x
        report["west_wall_contact"] = {
            "minimum_x": min(samples),
            "final_x": finalx,
            "expected_chassis_center_x": 0.5981416147 / 2,
        }
        assert 0.27 < min(samples) < 0.34 and 0.27 < finalx < 0.35, report[
            "west_wall_contact"
        ]
        launch_log = (out / "launch.log").read_text(errors="replace")
        for problem in (
            "Failed to load plugin",
            "Unable to find uri",
            "Unable to load mesh",
            "Unable to load texture",
        ):
            assert problem not in launch_log, problem
        report["render_resource_errors"] = False
        report["result"] = "PASS"
    # 顶层兜底：任何失败都要写进 report 并落盘，供 --json 输出诊断，故这里刻意捕获宽异常。
    except Exception as exc:  # noqa: BLE001
        report["error"] = repr(exc)
    finally:
        if pub is not None and rclpy.ok():
            pub.publish(Twist())
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGTERM)
                    try:
                        proc.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid, signal.SIGKILL)
                        proc.wait()
        for log in logs:
            log.close()
        (out / "results.json").write_text(
            json.dumps(report, indent=2) + "\n", encoding="utf-8"
        )
        print(json.dumps(report, indent=2))
        print(out)
    if report["result"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
