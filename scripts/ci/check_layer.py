#!/usr/bin/env python3
"""依赖方向门禁：`.diesel-layer` 的层级声明必须自洽且不反向。

依据 `foray_docs/repository_structure.md` §6.5「依赖方向校验」：

    L(n) 不得依赖 L(>n)
    不得依赖装配根仓 diesel_robots
    diesel_interfaces 不得依赖任何自研仓

单仓 CI 只看得到本仓的声明，所以本脚本校验声明本身是否自洽：

  1. 仓根必须有 `.diesel-layer`，且含 `layer:` 与 `depends_on_layers:` 两个键
  2. `layer` 必须是 §1/§2 定义的层级之一
  3. 声明的依赖层级必须都是比自身更低的层（反向依赖在这里就红）
  4. 不得自依赖、不得重复声明
  5. `META` / `T` 这类不在 L0–L5 主干上的层，必须声明空依赖
     （它们要么是元仓、要么是离线工具层，不参与层级序）

跨仓那半条（「不得依赖装配根」）需要看别的仓，属于元仓 `diesel_ws` 集成 CI 的职责，
本脚本不假装自己检查了它。

用法：
    python3 scripts/ci/check_layer.py          # 校验，失败 exit 1
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

LAYER_FILE = Path(".diesel-layer")

# 层级自低到高。数值只用于比较「谁在谁下面」，不参与任何运行时逻辑。
LAYER_ORDER = {
    "X2": 0,  # 契约层（msg/srv/action），零依赖
    "L1": 1,  # 平台：时间/数学/参数/日志/HAL
    "L2": 2,  # 平台：HAL 之上的服务
    "L3": 3,  # 感知与定位
    "L4": 4,  # 规划、控制、决策技能
    "L5": 5,  # 战术层
    "R": 6,  # 装配根（兵种 bringup）
}

# 不在 L0–L5 主干上的层：不参与层级序，但同样要声明依赖为空。
OFF_TREE_LAYERS = {"META", "T"}

KEY_VALUE_RE = re.compile(r"^\s*([A-Za-z_]+)\s*:\s*(.*?)\s*$")


def parse_layer_file() -> tuple[str, list[str]]:
    """解析 `.diesel-layer`，返回 (layer, depends_on_layers)。"""
    if not LAYER_FILE.is_file():
        raise SystemExit(f"::error::仓根缺少 {LAYER_FILE}（规则见 foray_docs §6.5）")

    values: dict[str, str] = {}
    for lineno, raw in enumerate(
        LAYER_FILE.read_text(encoding="utf-8").splitlines(), start=1
    ):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        match = KEY_VALUE_RE.match(line)
        if match is None:
            raise SystemExit(f"::error::{LAYER_FILE}:{lineno}: 无法解析的行「{raw}」")
        key, value = match.groups()
        if key in values:
            raise SystemExit(f"::error::{LAYER_FILE}:{lineno}: 键「{key}」重复声明")
        values[key] = value

    for required in ("layer", "depends_on_layers"):
        if required not in values:
            raise SystemExit(
                f"::error::{LAYER_FILE}: 缺少必备键「{required}:」（规则见 foray_docs §6.5）"
            )

    layer = values["layer"].strip().strip("'\"")
    raw_deps = values["depends_on_layers"].strip()

    # 只支持两种写法：`[]` 或 `[X2, L1]`。刻意不解析嵌套结构 ——
    # 声明文件保持到「一眼能读完」，才可能被持续维护。
    if not (raw_deps.startswith("[") and raw_deps.endswith("]")):
        raise SystemExit(
            f"::error::{LAYER_FILE}: depends_on_layers 必须是 [A, B] 形式，实际为「{raw_deps}」"
        )
    inner = raw_deps[1:-1].strip()
    deps = [item.strip().strip("'\"") for item in inner.split(",")] if inner else []

    return layer, deps


def main() -> int:
    layer, deps = parse_layer_file()
    failures: list[str] = []

    if layer not in LAYER_ORDER and layer not in OFF_TREE_LAYERS:
        failures.append(
            f"未知层级「{layer}」；合法取值：{', '.join([*LAYER_ORDER, *sorted(OFF_TREE_LAYERS)])}"
        )

    if len(set(deps)) != len(deps):
        failures.append(f"depends_on_layers 存在重复项：{deps}")

    if layer in deps:
        failures.append(f"layer「{layer}」不得依赖自身")

    for dep in deps:
        if dep not in LAYER_ORDER and dep not in OFF_TREE_LAYERS:
            failures.append(f"依赖了未知层级「{dep}」")

    if layer in OFF_TREE_LAYERS:
        if deps:
            failures.append(
                f"「{layer}」层不参与层级序，depends_on_layers 必须为空，实际为 {deps}"
            )
    elif layer in LAYER_ORDER:
        for dep in deps:
            if dep not in LAYER_ORDER:
                if dep in OFF_TREE_LAYERS:
                    failures.append(f"「{layer}」不得依赖非层级序的「{dep}」")
                continue
            if LAYER_ORDER[dep] >= LAYER_ORDER[layer]:
                failures.append(
                    f"反向依赖：{layer}（序 {LAYER_ORDER[layer]}）不得依赖 "
                    f"{dep}（序 {LAYER_ORDER[dep]}）"
                )

    if failures:
        for failure in failures:
            print(f"::error::{LAYER_FILE}: {failure}")
        print(
            f"\n依赖方向校验失败：{len(failures)} 处。规则见 foray_docs §6.5。",
            file=sys.stderr,
        )
        return 1

    print(
        f"依赖方向校验通过：layer={layer}，depends_on_layers={deps or '[]'}"
        "（跨仓的「不依赖装配根」由元仓集成 CI 负责）"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
