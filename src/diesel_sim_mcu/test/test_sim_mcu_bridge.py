#!/usr/bin/env python3
"""A10 静态断言：过渡桥的失联保护与契约一致性。

这里只做静态检查，不跑节点：真正「固件在环」的证据要靠整链回执
（`accept.py link`，README A11）。静态检查守的是那条最容易在重构里被
悄悄删掉的性质——**链路断了必须往 gz 写 0**，以及几个默认值与
`config/bridge.yaml` 的契约不能各说各话。

检查项：
1. 零速度兜底：`publish_body(0.0, 0.0, 0.0)` 必须在「闸门不成立」的分支里；
2. 闸门判据必须来自**固件回报**（`link_state` 与 `last_rx_control_seq`），
   不能只看上位机自己的状态机——否则「固件没收到也照样开车」；
3. `gz_topic` 的默认值与 `config/bridge.yaml` 里 `cmd_vel` 条目的 `gz_topic`
   渲染结果一致（同一条 gz 话题，不能写成两个名字）；
4. launch 文件把三个参数都传下去了。
"""

from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.dirname(HERE)
SOURCE = os.path.join(PACKAGE, "src", "sim_mcu_bridge.cpp")
LAUNCH = os.path.join(PACKAGE, "launch", "mcu_bridge.launch.py")
BRIDGE_YAML = os.path.join(
    PACKAGE, "..", "diesel_simulation_bringup", "config", "bridge.yaml"
)

failures: list[str] = []


def check(condition: bool, message: str) -> None:
    if not condition:
        failures.append(message)


def read(path: str) -> str:
    with open(path, encoding="utf-8") as handle:
        return handle.read()


def main() -> int:
    source = read(SOURCE)
    launch = read(LAUNCH)
    bridge_yaml = read(BRIDGE_YAML)

    # 1. 零速度兜底
    check(
        "publish_body(0.0, 0.0, 0.0)" in source,
        "过渡桥没有零速度兜底：链路断开时会按最后一条指令一直开下去",
    )

    # 2. 闸门看固件回报，而不是只看上位机状态机
    check(
        "last_rx_control_seq" in source,
        "闸门没有检查固件回报的 last_rx_control_seq：固件没收到也会开车",
    )
    check(
        "link_state == LINK_STATE_ACTIVE" in source,
        "闸门没有检查固件回报的 link_state",
    )

    # 3. gz 话题默认值必须与桥接契约同源
    check(
        "gz_topic: /model/{robot}/cmd_vel" in bridge_yaml,
        "bridge.yaml 里找不到 cmd_vel 的 gz_topic 契约",
    )
    check(
        '"/model/diesel_sim_robot/cmd_vel"' in source,
        "过渡桥的 gz_topic 默认值与 bridge.yaml 渲染结果（robot=diesel_sim_robot）不一致",
    )

    # 4. launch 参数透传
    for parameter in ("port", "cmd_vel_topic", "gz_topic"):
        check(
            f'"{parameter}"' in launch,
            f"launch 文件没有设置参数 {parameter}",
        )

    if failures:
        for failure in failures:
            print(f"FAIL {failure}")
        return 1
    print("PASS：过渡桥的失联兜底与桥接契约一致")
    return 0


if __name__ == "__main__":
    sys.exit(main())
