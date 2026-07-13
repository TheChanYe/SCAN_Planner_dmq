#!/usr/bin/env python3

import json
import math
import subprocess
import time

import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry


def main():
    rospy.init_node("send_clicked_navdog_route", anonymous=True)

    print("正在读取机器人当前位置……")
    odom = rospy.wait_for_message(
        "/quad_0/body_pose",
        Odometry,
        timeout=5.0,
    )

    x0 = odom.pose.pose.position.x
    y0 = odom.pose.pose.position.y
    z0 = odom.pose.pose.position.z

    print(
        f"机器人当前位置：x={x0:.3f}, "
        f"y={y0:.3f}, z={z0:.3f}"
    )
    print("请在 RViz 选择 Publish Point，点击障碍物后方。")

    clicked = rospy.wait_for_message(
        "/clicked_point",
        PointStamped,
        timeout=60.0,
    )

    x1 = clicked.point.x
    y1 = clicked.point.y

    dx = x1 - x0
    dy = y1 - y0
    distance = math.hypot(dx, dy)

    if distance < 1.0:
        raise RuntimeError(
            f"目标距离太近：{distance:.3f}m，"
            "请选择至少1米外的位置"
        )

    yaw = math.atan2(dy, dx)
    step = 0.4
    segment_count = max(2, math.ceil(distance / step))

    points = []
    for i in range(segment_count + 1):
        ratio = i / segment_count
        points.append({
            "x": round(x0 + ratio * dx, 3),
            "y": round(y0 + ratio * dy, 3),
            "z": round(z0 if math.isfinite(z0) else 0.3, 3),
            "yaw": round(yaw, 5),
        })

    payload = {
        "ctrl": 1,
        "navigation_data": {
            "max_vx": 0.22,
            "points": points,
        },
    }

    payload_text = json.dumps(
        payload,
        separators=(",", ":"),
    )

    print(
        f"目标位置：x={x1:.3f}, y={y1:.3f}, "
        f"直线距离={distance:.3f}m"
    )
    print(f"路线点数量：{len(points)}")

    subprocess.run([
        "mosquitto_pub",
        "-h", "127.0.0.1",
        "-t", "robot/global_planning/info",
        "-m", '{"ctrl":0}',
    ], check=True)

    time.sleep(0.5)

    subprocess.run([
        "mosquitto_pub",
        "-h", "127.0.0.1",
        "-t", "robot/global_planning/info",
        "-m", payload_text,
    ], check=True)

    print("路线已发送。")


if __name__ == "__main__":
    main()
