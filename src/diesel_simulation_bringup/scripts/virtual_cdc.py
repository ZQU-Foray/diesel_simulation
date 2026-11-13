#!/usr/bin/env python3
"""一对 PTY 顶替 USB CDC，让「上位机 ↔ 下位机」在仿真里走真字节流（A10）。

为什么需要：仿真默认把 `/cmd_vel` 直接桥进 gz，协议层被完全绕过，验不到「发一帧、收一帧」
这段最容易出错的接口。真车侧的栈是 RoboLink v0.2 over USB CDC（PC 侧
`diesel_platform::link`，固件侧 `hal::UsbCdc`），两边的读写接口都是「一段字节流的
read/write」，所以仿真里只要给两端各一个字符设备、把字节原样互相搬过去，就能跑真代码。
详见 decision.md D30。

实现：`pty.openpty()` 建两对，各自 slave 设成 raw（关回显、不转换换行），父进程保留
master 与 slave 的 fd 让设备节点一直存在，再用 `select` 在两个 master 之间双向中继。
不依赖 `socat`——它在文档里只是旁证手段。

用法：
    ros2 run diesel_simulation_bringup virtual_cdc.py \\
        --host-link /tmp/diesel_host --mcu-link /tmp/diesel_mcu

参数：
    --host-link   上位机侧设备路径（符号链接，指向内部 PTY 从端）
    --mcu-link    下位机侧设备路径（同上）
    --echo-log    可选：把中继过的字节按方向 + 十六进制前缀追加写到该文件，便于取证
    --chunk       单次读写字节上限，默认 4096
"""

from __future__ import annotations

import argparse
import contextlib
import errno
import os
import pty
import select
import signal
import sys
import termios
import tty


class VirtualCdc:
    """一对 PTY + 两个稳定路径符号链接，外加双向中继所需的 fd。"""

    def __init__(self, host_link: str, mcu_link: str) -> None:
        self.host_link = host_link
        self.mcu_link = mcu_link
        # 每端一对 (master, slave)：对端进程打开 slave 路径，我们在 master 之间搬字节。
        self._host_master, self._host_slave = pty.openpty()
        self._mcu_master, self._mcu_slave = pty.openpty()
        for slave in (self._host_slave, self._mcu_slave):
            # raw：不回显、不做 CR/LF 转换、不做流控——USB CDC 上也是一串裸字节。
            tty.setraw(slave)
            attrs = termios.tcgetattr(slave)
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(slave, termios.TCSANOW, attrs)
        for master in (self._host_master, self._mcu_master):
            os.set_blocking(master, False)
        self._links: list[str] = []
        self.host_path = self._publish(self._host_slave, host_link)
        self.mcu_path = self._publish(self._mcu_slave, mcu_link)

    @staticmethod
    def _publish(slave_fd: int, link: str) -> str:
        """把内部 PTY 从端发布成稳定路径；已存在的同名链接先摘掉。"""
        target = os.ttyname(slave_fd)
        if os.path.islink(link) or os.path.exists(link):
            os.unlink(link)
        os.symlink(target, link)
        return link

    @property
    def masters(self) -> tuple[int, int]:
        return self._host_master, self._mcu_master

    def close(self) -> None:
        for link in (self.host_link, self.mcu_link):
            try:
                if os.path.islink(link):
                    os.unlink(link)
            except OSError:
                pass
        for fd in (
            self._host_master,
            self._host_slave,
            self._mcu_master,
            self._mcu_slave,
        ):
            try:
                os.close(fd)
            except OSError:
                pass

    def relay_once(self, chunk: int = 4096) -> tuple[int, int]:
        """等一次可读事件并搬运字节，返回 (host→mcu, mcu→host) 的字节数。"""
        readable, _, _ = select.select(list(self.masters), [], [], 0.2)
        moved = [0, 0]
        for index, fd in enumerate(self.masters):
            if fd not in readable:
                continue
            data = self._read(fd, chunk)
            if not data:
                continue
            other = self.masters[1 - index]
            self._write(other, data)
            moved[index] = len(data)
        return moved[0], moved[1]

    @staticmethod
    def _read(fd: int, chunk: int) -> bytes:
        try:
            return os.read(fd, chunk)
        except OSError as exc:
            # EIO/EAGAIN：对端还没打开或暂时没数据，不是错误。
            if exc.errno in (errno.EIO, errno.EAGAIN, errno.EWOULDBLOCK):
                return b""
            raise

    @staticmethod
    def _write(fd: int, data: bytes) -> None:
        while data:
            try:
                written = os.write(fd, data)
            except OSError as exc:
                if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    select.select([], [fd], [], 0.05)
                    continue
                raise
            data = data[written:]


def _hexdump(data: bytes, limit: int = 32) -> str:
    return data[:limit].hex()


def run(args: argparse.Namespace) -> int:
    cdc = VirtualCdc(args.host_link, args.mcu_link)
    stop = {"flag": False}

    def _stop(_signum: int, _frame: object | None) -> None:
        stop["flag"] = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    print(
        f"虚拟 CDC 就绪：host={cdc.host_path} -> {os.readlink(cdc.host_path)} "
        f"mcu={cdc.mcu_path} -> {os.readlink(cdc.mcu_path)}",
        flush=True,
    )
    totals = [0, 0]
    with contextlib.ExitStack() as stack:
        echo_log = (
            stack.enter_context(open(args.echo_log, "ab", buffering=0))
            if args.echo_log
            else None
        )
        try:
            while not stop["flag"]:
                moved = cdc.relay_once(args.chunk)
                if not any(moved):
                    continue
                for index, count in enumerate(moved):
                    if not count:
                        continue
                    totals[index] += count
                    if echo_log:
                        direction = "host->mcu" if index == 0 else "mcu->host"
                        echo_log.write(f"{direction} {count}B\n".encode())
        finally:
            print(
                f"虚拟 CDC 退出：host->mcu={totals[0]}B mcu->host={totals[1]}B",
                flush=True,
            )
            cdc.close()
    return 0


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="一对 PTY 顶替 USB CDC：两端各一个字符设备，字节原样双向中继。",
    )
    parser.add_argument(
        "--host-link", default="/tmp/diesel_host", help="上位机侧设备路径"
    )
    parser.add_argument(
        "--mcu-link", default="/tmp/diesel_mcu", help="下位机侧设备路径"
    )
    parser.add_argument("--echo-log", default=None, help="可选：中继字节的取证日志")
    parser.add_argument(
        "--chunk", type=int, default=4096, help="单次读写上限（默认 4096）"
    )
    args = parser.parse_args(argv)
    if args.chunk <= 0:
        parser.error("--chunk 必须为正数")
    if os.path.abspath(args.host_link) == os.path.abspath(args.mcu_link):
        parser.error("--host-link 与 --mcu-link 不能是同一个路径")
    return args


def main(argv: list[str] | None = None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
