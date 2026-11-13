# Copyright 2026 ZQU-Foray
"""仿真装配层：把「引擎调用 / 资产选择 / 桥接契约」收敛成可单测的函数。

分层与取舍见 README §3.1 与 decision.md D29。三个子模块各自负责一件事：
`engine_gz`（gz 启动参数）、`bridge`（桥接契约渲染）、`profile`（场地与机器人规格）。
"""

from .errors import AssemblyError

__all__ = ["AssemblyError"]
