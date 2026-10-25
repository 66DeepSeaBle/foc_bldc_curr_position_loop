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

## 历史快照

`_archive/snapshots/`（`cal30_15kHz` / `cal30_20kHz` / `cal30_20k_routeB`）
与旧的 `f15k/f16k/f20k*.hex` 已于 2026-10-25 删除 —— 频率对比实验已结束，定版 20 kHz。
这些源码快照在 git 里有记录，需要时用 `git log -- _archive/snapshots` 找回。
