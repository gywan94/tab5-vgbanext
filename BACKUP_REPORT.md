# tab5-vgbanext 备份报告

**备份日期:** 2026-06-16
**状态:** 真机多 ROM 验证通过的可用成品（COM4，M5Stack Tab5 / ESP32-P4）

这是在尝试 THREADED_RENDERER（冲 60fps）之前的稳定快照。如果后续改动把东西搞挂，
回退到这个备份即可。

---

## 这一版包含什么

VBA-Next（libretro/vba-next）GBA 模拟器移植到 Tab5，复用 tab5-gba 框架。

**关键实现：**
- `components/vbanext/` — 封装 vba-next 核心 + libretro.c（编译开关：`USE_TWEAKS=1`
  `FRONTEND_SUPPORTS_RGB565=1 LOAD_FROM_MEMORY=1 INLINE=inline HAVE_NEON=0`，线程渲染器关）
- `components/vgba_run/` — 5 回调 libretro 前端 + **双核流水线**（core0 仿真 /
  core1 blit+音频，NB=2 ping-pong + free_q/disp_q）
- ROM 经 `romdata` flash 分区（`esp_partition_mmap`，路径 `flash:romdata`）直接运行
- main.c 自动运行 flash:romdata（无 SD / 无触摸，便于测帧率）

**vendored vba-next 的必要补丁（重新 clone 会丢，务必保留）：**
1. `libretro_save_buf`（136KB）→ PSRAM（`EXT_RAM_BSS_ATTR`），否则链接时片内 DRAM 溢出
2. 32MB 平铺 ROM → 按实际大小（power-of-two）分配（`g_rom_alloc/g_rom_mask/g_swi_base`），
   全部 ROM 掩码 wrap，HLE-BIOS SWI trampoline + myROM 两个高位字面量重定位
3. `memalign_alloc`（memalign.c）→ `heap_caps_malloc(SPIRAM)`，否则多 MB ROM 分配落到
   片内 RAM 失败
4. `utilLoad` + filestream include 用 `#ifndef LOAD_FROM_MEMORY` 屏蔽

---

## 真机测试结果

### 移植正确性 — 多 ROM 交叉测试（全部正常运行，无崩溃）

| ROM | 文件大小 | 实测 ROM 大小 | rom buffer | trampoline 位置 | FPS | core0 |
|-----|---------|--------------|-----------|----------------|-----|-------|
| koudai（宝可梦） | 8.00 MB | 7.49 MB（裁掉 0xFF padding） | 8 MB | 0x7E2000 | **46–47** | 20ms |
| sizhijian | 7.47 MB | 7.47 MB | 8 MB | 0x7E2000 | **41** | 23ms |
| Yoshi Sample (En) | 2.27 MB | 2.27 MB | 4 MB | 0x3E2000 | **28** | 34ms |

> 三种 ROM 大小/类型差异巨大，分别走 4MB / 8MB 两种 buffer、trampoline 重定位到不同高位
> 地址，全部正确运行 → ROM 缩容 + BIOS trampoline 重定位逻辑跨尺寸跨游戏验证通过。

### 优化进程（Yoshi = 100% ARM 最坏情况）

| 步骤 | FPS | core0 | 改动 |
|------|-----|-------|------|
| 基线 | 19 | 43ms | 单核 / USE_TWEAKS=0 |
| + `USE_TWEAKS=1` | 24 | 33ms | 空转循环加速，−10ms |
| + 双核流水线 | **28** | 34ms | blit+audio 移到 core1，WAIT=0 |

**累计 19 → 28 fps（+47%），全程无崩溃。**

### 性能分布
- 宝可梦类（THUMB 多）：~46fps，基本可玩
- 普通游戏：~41fps
- ARM 极限负载（Yoshi）：28fps（地板）
- 距 60fps 的差距（典型 20ms → 16.7ms）需 THREADED_RENDERER 或 dynarec

---

## 启动器/烧录速查

```powershell
. C:\Espressif\frameworks\esp-idf-v5.5.2\export.ps1
cd tab5-vgbanext\apps\vgba_tab5
idf.py -p COM4 -b 460800 build flash          # 烧固件
# 换测试 ROM（必须先 erase，否则尾部残留数据导致大小误判）：
esptool.py --chip esp32p4 -p COM4 -b 460800 erase_region 0x410000 0x800000
esptool.py --chip esp32p4 -p COM4 -b 460800 write_flash  0x410000 <rom.gba>
```

串口抓日志（不用 idf monitor）：开 COM4@115200，DtrEnable=$false，RtsEnable 脉冲
true→false 复位，循环 ReadExisting。

---

## 已知限制 / 下一步
- 音频在 <60fps 时会有欠载（产出慢于 48kHz 实时消费）——到 60fps 才平滑。
- 下一步尝试 **THREADED_RENDERER**：把 PPU 逐行渲染甩到 core1（现在 core1 只用 ~6ms
  很闲）。风险：vba-next worker 忙等自旋 + pthread 无核亲和 → 需钉 core1、防 core1
  idle 看门狗、与显示任务共核。预期 ~28→~38fps（Yoshi）。
- 终极天花板：ARM/THUMB→RISC-V dynarec（数周，复用 tab5-gba 经验）。
