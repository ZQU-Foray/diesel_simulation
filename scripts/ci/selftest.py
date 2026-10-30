#!/usr/bin/env python3
"""门禁自检：往仓库副本里注入已知缺陷，断言对应门禁确实会红。

为什么要「自检门禁」：三门禁（file-hygiene / doc-sync / layer-check）绿了，只说明
当前仓库没被它们拦下，不说明它们还能拦人。一个被改坏的 `check_style.py`（比如
正则写成永不命中）会安静地一直通过——比没有门禁更危险。本脚本给每条门禁喂一个
**必然违规**的样本，红不了就报错。

只在临时目录里动手，不碰当前工作区：

  1. 把仓库复制到临时目录（排除 .git / 构建产物），`git init` + 全量提交，
     因为三门禁都以 `git ls-files` 为准，副本必须自己是个干净的 Git 仓。
  2. 干净副本：三门禁必须全绿（基线）。
  3. 逐条注入缺陷 → 断言对应门禁非零退出：
       - 行尾 CRLF             → file-hygiene
       - tree.md 少一行        → doc-sync
       - tree.md 多一个文件    → doc-sync（未登记）
       - 删掉已登记的文件      → doc-sync（虚构条目）
       - .foray-layer 反向依赖 → layer-check

用法：
    python3 scripts/ci/selftest.py          # 全部通过 exit 0，否则 exit 1
"""

from __future__ import annotations

import shutil
import subprocess
import sys
import tempfile
from collections.abc import Callable
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

# 复制副本时跳过的目录：版本库、构建产物、本轮任务的本地状态。
IGNORE_DIRS = shutil.ignore_patterns(
    ".git",
    ".dsh",
    "build",
    "install",
    "log",
    "__pycache__",
    "*.egg-info",
)

GATES = {
    "file-hygiene": "scripts/ci/check_style.py",
    "doc-sync": "scripts/ci/check_tree_doc.py",
    "layer-check": "scripts/ci/check_layer.py",
}

# 用作注射目标的普通文本文件与 tree.md 里一定存在的条目
TEXT_TARGET = "README.md"
# tree.md 的目录树行只写文件名，所以用文件名匹配（唯一即够）。
TREE_TARGET_MISSING = "test_manifest.py"
UNTRACKED_EXTRA = "src/foray_simulation_bringup/scripts/zz_selftest_probe.py"


def run_git(repo: Path, *args: str) -> None:
    """在副本里跑一条 git 命令，失败直接抛出。"""
    subprocess.run(
        ["git", "-c", "user.email=ci@example.com", "-c", "user.name=ci", *args],
        cwd=repo,
        check=True,
        capture_output=True,
        text=True,
    )


def run_gate(repo: Path, gate: str) -> int:
    """返回门禁退出码。"""
    result = subprocess.run(
        [sys.executable, GATES[gate]],
        cwd=repo,
        capture_output=True,
        text=True,
        check=False,
    )
    return result.returncode


def make_repo_copy(workdir: Path) -> Path:
    """复制仓库并初始化为干净 Git 仓，返回副本路径。"""
    copy = workdir / "repo"
    shutil.copytree(REPO_ROOT, copy, ignore=IGNORE_DIRS)
    run_git(copy, "init", "-q")
    run_git(copy, "add", "-A")
    run_git(copy, "commit", "-q", "-m", "selftest baseline")
    return copy


def inject_crlf(repo: Path) -> None:
    target = repo / TEXT_TARGET
    data = target.read_bytes().replace(b"\n", b"\r\n")
    target.write_bytes(data)
    run_git(repo, "commit", "-aqm", "inject: crlf")


def inject_tree_missing_line(repo: Path) -> None:
    tree = repo / "tree.md"
    lines = tree.read_text(encoding="utf-8").splitlines(keepends=True)
    kept = [line for line in lines if TREE_TARGET_MISSING not in line]
    if len(kept) == len(lines):
        raise SystemExit(
            f"::error::自检前置失败：tree.md 里找不到 {TREE_TARGET_MISSING}"
        )
    tree.write_text("".join(kept), encoding="utf-8")
    run_git(repo, "commit", "-aqm", "inject: tree missing line")


def inject_untracked_file(repo: Path) -> None:
    extra = repo / UNTRACKED_EXTRA
    extra.write_text('"""自检探针：不应存在于真实仓库。"""\n', encoding="utf-8")
    run_git(repo, "add", "-A")
    run_git(repo, "commit", "-qm", "inject: extra file")


def inject_deleted_file(repo: Path) -> None:
    target = repo / "src/foray_simulation_bringup/test" / TREE_TARGET_MISSING
    target.unlink()
    run_git(repo, "commit", "-aqm", "inject: deleted file")


def inject_layer_reverse(repo: Path) -> None:
    layer_file = repo / ".foray-layer"
    layer_file.write_text(
        "# 自检注入：L1 反向依赖 L5（L(n) 不得依赖 L(>n)）\n"
        "layer: L1\n"
        "depends_on_layers: [L5]\n",
        encoding="utf-8",
    )
    run_git(repo, "commit", "-aqm", "inject: layer reverse")


CASES: list[tuple[str, str, Callable[[Path], None]]] = [
    ("crlf", "file-hygiene", inject_crlf),
    ("tree-missing-line", "doc-sync", inject_tree_missing_line),
    ("untracked-file", "doc-sync", inject_untracked_file),
    ("deleted-file", "doc-sync", inject_deleted_file),
    ("layer-reverse", "layer-check", inject_layer_reverse),
]


def main() -> int:
    """跑基线 + 每条注入用例，返回进程退出码。"""
    failures = 0
    with tempfile.TemporaryDirectory(prefix="foray-gate-selftest-") as tmp:
        workdir = Path(tmp)
        for name, gate, inject in CASES:
            copy = make_repo_copy(workdir)
            inject(copy)
            code = run_gate(copy, gate)
            if code == 0:
                print(f"::error::自检失败：注入「{name}」后 {gate} 仍然通过")
                failures += 1
            else:
                print(f"OK 注入「{name}」→ {gate} 拦下（exit {code}）")
            shutil.rmtree(copy)

        clean = make_repo_copy(workdir)
        for gate in GATES:
            code = run_gate(clean, gate)
            if code != 0:
                print(f"::error::自检失败：干净副本上 {gate} 未通过（exit {code}）")
                failures += 1
        shutil.rmtree(clean)

    if failures:
        print(f"门禁自检失败：{failures} 处")
        return 1
    print(f"门禁自检通过：{len(CASES)} 条注入全部被拦下，干净副本 {len(GATES)} 门全绿")
    return 0


if __name__ == "__main__":
    sys.exit(main())
