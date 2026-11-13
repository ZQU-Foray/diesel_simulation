#!/usr/bin/env python3
"""virtual_cdc 的自测：两向字节必须原样通过，退出后不留符号链接。

为什么要有这个用例：虚拟 CDC 是 A10 的地基，一旦它偷偷改写字节（回显、CR/LF 转换、
半包截断），上位机与固件的握手会以一个极难定位的姿态失败——CRC 对不上、帧头重扫、
最终表现为「链路起不来」。所以这里直接对字节断言，而不是只看进程是否活着。

跑法：python3 test/test_virtual_cdc.py
"""

from __future__ import annotations

import os
import select
import sys
import tempfile
import threading
import time
import traceback

sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"
    ),
)

from virtual_cdc import VirtualCdc


def _open(path: str) -> int:
    return os.open(path, os.O_RDWR | os.O_NOCTTY)


def _read_exactly(fd: int, size: int, timeout: float = 2.0) -> bytes:
    deadline = time.monotonic() + timeout
    chunks = b""
    while len(chunks) < size and time.monotonic() < deadline:
        readable, _, _ = select.select([fd], [], [], 0.1)
        if not readable:
            continue
        chunks += os.read(fd, size - len(chunks))
    return chunks


def _relay_until(cdc: VirtualCdc, stop: threading.Event) -> None:
    while not stop.is_set():
        cdc.relay_once()


def test_round_trip_both_directions() -> None:
    with tempfile.TemporaryDirectory() as workdir:
        host_link = os.path.join(workdir, "host")
        mcu_link = os.path.join(workdir, "mcu")
        cdc = VirtualCdc(host_link, mcu_link)
        assert os.path.islink(host_link), "host 侧应发布成符号链接"
        assert os.path.islink(mcu_link), "mcu 侧应发布成符号链接"

        stop = threading.Event()
        relay = threading.Thread(target=_relay_until, args=(cdc, stop), daemon=True)
        relay.start()
        try:
            host_fd = _open(host_link)
            mcu_fd = _open(mcu_link)
            try:
                payload_down = bytes(range(256)) * 4
                os.write(host_fd, payload_down)
                assert (
                    _read_exactly(mcu_fd, len(payload_down)) == payload_down
                ), "host→mcu 方向的字节被改写或截断"

                payload_up = b"\xaa\x55robolink-frames\x00\xff" * 32
                os.write(mcu_fd, payload_up)
                assert (
                    _read_exactly(host_fd, len(payload_up)) == payload_up
                ), "mcu→host 方向的字节被改写或截断"
            finally:
                os.close(host_fd)
                os.close(mcu_fd)
        finally:
            stop.set()
            relay.join(timeout=2.0)
            assert not relay.is_alive(), "中继线程未退出"
            cdc.close()

        assert not os.path.lexists(host_link), "退出后不应残留 host 链接"
        assert not os.path.lexists(mcu_link), "退出后不应残留 mcu 链接"


def test_rejects_same_path() -> None:
    from virtual_cdc import parse_args

    try:
        parse_args(["--host-link", "/tmp/same", "--mcu-link", "/tmp/same"])
    except SystemExit as exc:
        assert exc.code == 2, f"同一路径应被 argparse 拒绝，实际退出码 {exc.code}"
        return
    raise AssertionError("--host-link 与 --mcu-link 相同时应报错退出")


def main() -> int:
    failures = 0
    for name, case in (
        ("round_trip_both_directions", test_round_trip_both_directions),
        ("rejects_same_path", test_rejects_same_path),
    ):
        try:
            case()
            print(f"PASS {name}")
        except Exception:  # noqa: BLE001 - 自测脚本把任何异常都当失败并打印现场
            failures += 1
            print(f"FAIL {name}")
            traceback.print_exc()
    if failures:
        print(f"virtual_cdc 自测失败：{failures} 项")
        return 1
    print("virtual_cdc 自测通过：双向字节原样、无残留链接")
    return 0


if __name__ == "__main__":
    sys.exit(main())
