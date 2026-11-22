# 构建输出目录（2026-10-25 起统一为一个名字）

## 约定

Keil 工程 `MDK-ARM/BldcClamp.uvprojx` 的输出固定为：

```xml
<OutputDirectory>..\_archive\builds\</OutputDirectory>
<OutputName>BldcClamp</OutputName>
```

⇒ 每次编译**只产出一套** `BldcClamp.*`，不再出现
`f20k / f15k / A2 / A2b / A2c / A2d / B1_trimslew / BASELINE_A_*` 这种一版一个文件名的漂移。

| 产物 | 说明 |
|---|---|
| `BldcClamp.hex` | **烧录用**（Keil Flash → Download 也会直接烧这个） |
| `BldcClamp.axf` | 调试用（J-Link / STM32CubeProgrammer 都认） |
| `BldcClamp.map` | 内存/符号表 |
| `BldcClamp.htm` | 静态调用图 |
| `BldcClamp.build_log.htm` | 编译日志（0 Error / 0 Warning 就看这个） |
| `*.o` `*.crf` `*.d` `*.lnp` `*.sct` `*.dep` | 中间文件，可随时删除 |

> ⚠ **判断烧的是哪一版，看 `[FW] build ...` 那行 tag，不要看文件名。**
> 文件名永远叫 `BldcClamp`，内容随源码变。

## 最近版本记录（2026-10-25）

**当前基线 = `A2g`**（源码 = 工作区，git tag `A2g`，`BldcClamp.hex` 就是它）。

| tag | 内容 | 结果 |
|---|---|---|
| `A2d` | 「精简代码」后第一版（= `A2c` 删掉 E 键）。力矩档 **2.50 A** / 摩擦前馈 **0.20 A** / RTT 4096 | A/B 对比输，已汰汰（`max\|tv\|` 60.6°、驱动限流打满、报 `[POS] BLOCKED?`） |
| `A2e` | ❌ 低速段按阈值切静摩擦前馈 → ±1.45 A bang-bang，起步/停止抖 | 已回退 |
| `A2f` | = `A2d` + 力矩档 **3.00 A** + 全关断双零点 + RTT 8192/整行健全性检查 | 中间版 |
| **`A2g`** | = `A2f` + 摩擦前馈 **0.35 A**。`max\|tv\|` 57.8° → **40.8°** | ✅ **当前基线** |
| `A2h` | ❌ 参考调速器（限幅+虚拟时钟）→ 「走半圈慢、又加速」循环 | 已回退 |

- 旧版本靠 **git tag** 追溯：`git checkout <tag> -- Core/ Middlewares/`，重编即可。
  **不要**回退 `MDK-ARM/BldcClamp.uvprojx`（输出名/目录的固定设置在那里）。
- 旧版本的 `.hex` 备份已于 2026-10-25 按用户要求删除，只保留当前构建产物。
- 「最后一圈一格一格跳」在 `A2d`/`A2g` 上都有 ⇒ **机构极限**（6 步霍尔 15.47° 量化 + 齿槽），
  低速段（< ~600 °/s）必然按格跳，软件不再追。

## 历史快照

`_archive/snapshots/`（`cal30_15kHz` / `cal30_20kHz` / `cal30_20k_routeB`）
与旧的 `f15k/f16k/f20k*.hex` 已于 2026-10-25 删除 —— 频率对比实验已结束，定版 20 kHz。
这些源码快照在 git 里有记录，需要时用 `git log -- _archive/snapshots` 找回。
