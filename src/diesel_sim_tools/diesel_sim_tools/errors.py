# Copyright 2026 ZQU-Foray
"""装配层的统一异常。

装配错误必须**启动即失败**：gz 侧话题写错时桥接照建、日志不报错、数据全断
（decision.md D20），能在 profile / 契约被读的那一刻崩掉，比运行十分钟后
从「车不动」倒查便宜得多。因此这里只有一个异常类型，所有校验都抛它。
"""


class AssemblyError(RuntimeError):
    """装配输入（profile / 桥接契约）不合法，或渲染后仍有未替换的占位符。"""
