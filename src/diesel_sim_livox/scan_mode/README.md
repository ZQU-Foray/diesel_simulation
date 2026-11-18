# scan_mode — Livox 官方非重复扫描角度表

`mid360.csv.gz` 是 Livox 官方仿真插件
[`Livox-SDK/livox_laser_simulation`](https://github.com/Livox-SDK/livox_laser_simulation)
`scan_mode/mid360.csv` 的 gzip 压缩副本（原始 17 MB → 6.5 MB，内容逐字节一致）。

- 表头：`Time/s,Azimuth/deg,Zenith/deg`；800000 个数据点。
- `elevation = 90 - zenith`（zenith 90° = 水平），实际范围约 −7.21°…+52.16°，
  与 Mid-360 标称 −7°…+52° 垂直视场吻合。
- 每 4 点一组 = 4 个发射器（`line = i % 4`，对应 point_lio `scan_line: 4`）。
- 方位角快速回绕、俯仰角慢漂，两者周期非整数比 ⇒ **非重复扫描**。
  800000 点 @200k/s = 4 s = 40 帧（每帧 20000 点）为一个完整表周期。

插件 `LivoxMid360System` 用 zlib 解压到内存后流式解析（`PatternTable::LoadCsv(istream)`），
不落临时文件。选 gzip 而非裸 CSV：仓库无 git-lfs，裸 17 MB 会让 .git 体积扩约 3×。

> 注意：官方仓库只给了 **Mid-360** 的角度表，没有 Mid-360S 的。两者是否同一 pattern
> 官方未说明；本仿真按 Mid-360 表实现，若 Mid-360S 实测 pattern 不同需另行替换。
