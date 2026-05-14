#!/usr/bin/env python3
"""
巡检云台 UDP 联调脚本。

使用说明：
1. 把本脚本放在 inspection_nav/config 目录下运行。
2. 默认会自动读取脚本同级目录下的 point1.yaml。
3. 如果需要切换 YAML、云台 IP、端口、超时时间，可以直接通过命令行参数指定；也可以修改文件上方的默认参数。
4. 该脚本只负责读取 YAML 点位并按巡检程序一致的格式向云台发送 UDP 消息。
5. 当点位的 gimbal_inspect 为 0 时，直接进入下一个目标点；为 1 时会等待云台回复允许继续。

常用示例：
- 直接使用默认参数：python3 gimbal_udp_test.py
- 指定 YAML 和云台地址：python3 gimbal_udp_test.py --config point2.yaml --udp-ip 192.168.1.10 --udp-port 20001
- 指定完整路径：python3 gimbal_udp_test.py --config /home/schwein/inspection_ws/src/inspection_nav/config/point3.yaml

注意：
- YAML 解析依赖 PyYAML；如果系统缺少该环境，请先安装。
- 为了方便统一调试，终端输出文案尽量与巡检程序保持一致。
"""

import argparse
import os
import re
import socket
import sys
import time

try:
    import yaml
except ImportError:
    print("Missing dependency: PyYAML. Install with: pip install pyyaml", file=sys.stderr)
    sys.exit(2)

# =========================
# 关键参数区：按需直接修改
# =========================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG_FILE = os.path.join(SCRIPT_DIR, "point1.yaml")
DEFAULT_ROUTE_ID = 1
GIMBAL_UDP_IP = "127.0.0.1"
GIMBAL_UDP_PORT = 20001
GIMBAL_UDP_TIMEOUT_SEC = 1.0
GIMBAL_UDP_RETRY_INTERVAL_SEC = 1.0

# 允许云台回复后继续的关键字，保持与巡检程序一致
ALLOWED_RESPONSES = {"1", "OK", "CONTINUE", "CONTINUE=1", "TRUE"}


def parse_gimbal_flag(value):
    if value is None:
        return 0
    if isinstance(value, bool):
        return 1 if value else 0
    try:
        return 1 if int(value) != 0 else 0
    except (TypeError, ValueError):
        return 0


def load_goals(path):
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}

    goals = data.get("goals", [])
    if not isinstance(goals, list):
        raise ValueError("YAML field 'goals' is not a sequence.")

    parsed = []
    for index, node in enumerate(goals):
        if not isinstance(node, dict):
            print(f"Skipping malformed goal entry: goals[{index}]", file=sys.stderr)
            continue

        position = node.get("position")
        orientation = node.get("orientation")
        if not isinstance(position, dict) or not isinstance(orientation, dict):
            print(f"Skipping malformed goal entry: goals[{index}]", file=sys.stderr)
            continue

        if any(key not in position for key in ("x", "y", "z")):
            print(f"Skipping malformed goal entry: goals[{index}]", file=sys.stderr)
            continue
        if any(key not in orientation for key in ("x", "y", "z", "w")):
            print(f"Skipping malformed goal entry: goals[{index}]", file=sys.stderr)
            continue

        parsed.append(
            {
                "index": index + 1,
                "name": node.get("name", f"patrol_point_{index + 1}"),
                "gimbal_inspect": parse_gimbal_flag(node.get("gimbal_inspect")),
                "position": {
                    "x": position["x"],
                    "y": position["y"],
                    "z": position["z"],
                },
            }
        )

    return parsed


def guess_route_id(path, provided):
    if provided is not None:
        return provided
    base = os.path.basename(path)
    match = re.search(r"point(\d+)\.ya?ml$", base)
    if match:
        return int(match.group(1))
    return DEFAULT_ROUTE_ID


def build_payload(route_id, index, total):
    return f"INSPECT,{route_id},{index},{total}"


def response_allows_continue(response):
    if response is None:
        return False
    trimmed = response.strip()
    if not trimmed:
        return False
    return trimmed.upper() in ALLOWED_RESPONSES


def ensure_gimbal_socket(sock, ip, port, timeout_sec):
    # UDP connect 只用于固定默认目的地址，方便与巡检程序日志一致。
    sock.settimeout(timeout_sec if timeout_sec > 0 else 1.0)
    sock.connect((ip, port))


def wait_for_gimbal(sock, payload, retry_interval_sec):
    while True:
        try:
            sock.send(payload.encode("utf-8"))
        except OSError as exc:
            print(f"[GIMBAL][UDP] Send failed: {exc}", file=sys.stderr)
            time.sleep(retry_interval_sec if retry_interval_sec > 0 else 1.0)
            continue

        try:
            data = sock.recv(256)
        except socket.timeout:
            print("[GIMBAL][UDP] Waiting for continue command...")
            time.sleep(retry_interval_sec if retry_interval_sec > 0 else 1.0)
            continue
        except OSError as exc:
            print(f"[GIMBAL][UDP] Receive failed: {exc}", file=sys.stderr)
            time.sleep(retry_interval_sec if retry_interval_sec > 0 else 1.0)
            continue

        response = data.decode("utf-8", errors="ignore")
        if response_allows_continue(response):
            return True

        print(f"[GIMBAL][UDP] Response indicates waiting: {response.strip()}")
        time.sleep(retry_interval_sec if retry_interval_sec > 0 else 1.0)


def main():
    parser = argparse.ArgumentParser(
        description="Send gimbal UDP requests by replaying inspection route YAML."
    )
    parser.add_argument(
        "--config",
        default=DEFAULT_CONFIG_FILE,
        help="Path to route YAML (default: %(default)s)",
    )
    parser.add_argument(
        "--route-id",
        type=int,
        default=None,
        help="Route id used in UDP payload (default: inferred from filename)",
    )
    parser.add_argument("--udp-ip", default=GIMBAL_UDP_IP, help="Gimbal UDP IP")
    parser.add_argument("--udp-port", type=int, default=GIMBAL_UDP_PORT, help="Gimbal UDP port")
    parser.add_argument(
        "--timeout",
        type=float,
        default=GIMBAL_UDP_TIMEOUT_SEC,
        help="Socket receive timeout in seconds",
    )
    parser.add_argument(
        "--retry-interval",
        type=float,
        default=GIMBAL_UDP_RETRY_INTERVAL_SEC,
        help="Retry interval in seconds when waiting for continue",
    )

    args = parser.parse_args()

    if args.udp_port <= 0 or args.udp_port > 65535:
        print(f"Invalid UDP port: {args.udp_port}", file=sys.stderr)
        return 2

    if args.timeout <= 0:
        args.timeout = 1.0
    if args.retry_interval <= 0:
        args.retry_interval = 1.0

    route_id = guess_route_id(args.config, args.route_id)

    try:
        goals = load_goals(args.config)
    except (OSError, ValueError) as exc:
        print(f"Failed to load YAML config: {exc}", file=sys.stderr)
        return 2

    if not goals:
        print(f"No goals found for route {route_id} in config file: {args.config}", file=sys.stderr)
        return 2

    print(f"Route {route_id} selected. Loaded {len(goals)} goals. Waiting for move_base action server...")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        ensure_gimbal_socket(sock, args.udp_ip, args.udp_port, args.timeout)
    except OSError as exc:
        print(f"Failed to connect gimbal UDP socket: {exc}", file=sys.stderr)
        return 2

    print(f"move_base connected. Sending route {route_id} goals in sequence.")

    for index, goal in enumerate(goals, start=1):
        position = goal["position"]
        print(
            f"[SEND GOAL {index}/{len(goals)}][Route {route_id}] pos=({position['x']:.3f}, {position['y']:.3f}, {position['z']:.3f})"
        )

        print(f"[GOAL REACHED] Route {route_id}, Goal {index}/{len(goals)}")

        if goal["gimbal_inspect"] != 0:
            payload = build_payload(route_id, index, len(goals))
            print(
                f"[GIMBAL][UDP] Requesting inspect at goal {index}/{len(goals)} -> {args.udp_ip}:{args.udp_port}"
            )
            print("[GIMBAL][UDP] Waiting for continue command...")
            if wait_for_gimbal(sock, payload, args.retry_interval):
                print("[GIMBAL][UDP] Continue navigation.")

    print(f"[NAVIGATION FINISHED] Route {route_id} completed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
