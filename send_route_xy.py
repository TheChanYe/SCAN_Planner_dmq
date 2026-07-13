#!/usr/bin/env python3

import argparse
import json
import math
import subprocess
import time

import rospy
from nav_msgs.msg import Odometry


def publish_mqtt(payload):
    subprocess.run(
        [
            "mosquitto_pub",
            "-h", "127.0.0.1",
            "-t", "robot/global_planning/info",
            "-m", json.dumps(payload, separators=(",", ":")),
        ],
        check=True,
    )


def main():
    parser = argparse.ArgumentParser(
        description="从机器狗当前位置发送一条直线导航路线"
    )
    parser.add_argument("goal_x", type=float, help="目标世界坐标 X")
    parser.add_argument("goal_y", type=float, help="目标世界坐标 Y")
    parser.add_argument(
        "--speed",
        type=float,
        default=0.22,
        help="最大线速度，默认 0.22m/s",
    )
    parser.add_argument(
        "--step",
        type=float,
        default=0.4,
        help="路线点间距，默认 0.4m",
    )
    args = parser.parse_args()

    rospy.init_node("send_navdog_route_xy", anonymous=True)

    print("读取机器狗当前位置……")
    odom = rospy.wait_for_message(
        "/quad_0/body_pose",
        Odometry,
        timeout=5.0,
    )

    start_x = odom.pose.pose.position.x
    start_y = odom.pose.pose.position.y
    start_z = odom.pose.pose.position.z

    dx = args.goal_x - start_x
    dy = args.goal_y - start_y
    distance = math.hypot(dx, dy)

    if distance < 0.3:
        raise RuntimeError(
            f"目标距离太近：{distance:.3f}m"
        )

    if args.speed <= 0.0:
        raise RuntimeError("speed 必须大于 0")

    if args.step <= 0.05:
        raise RuntimeError("step 必须大于 0.05")

    yaw = math.atan2(dy, dx)
    segment_count = max(
        2,
        int(math.ceil(distance / args.step)),
    )

    points = []

    for index in range(segment_count + 1):
        ratio = index / segment_count

        points.append({
            "x": round(start_x + dx * ratio, 3),
            "y": round(start_y + dy * ratio, 3),
            "z": round(start_z, 3),
            "yaw": round(yaw, 5),
        })

    print(
        f"起点：({start_x:.3f}, {start_y:.3f})"
    )
    print(
        f"终点：({args.goal_x:.3f}, {args.goal_y:.3f})"
    )
    print(f"距离：{distance:.3f}m")
    print(f"路点：{len(points)} 个")
    print(f"速度：{args.speed:.3f}m/s")

    # 先取消旧任务。
    publish_mqtt({"ctrl": 0})
    time.sleep(0.5)

    payload = {
        "ctrl": 1,
        "navigation_data": {
            "max_vx": args.speed,
            "points": points,
        },
    }

    publish_mqtt(payload)

    print("路线发送成功")


if __name__ == "__main__":
    main()
