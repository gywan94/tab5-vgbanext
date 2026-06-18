# tab5-vgbanext 优化计划

M5Stack Tab5 (ESP32-P4, 双核 RISC-V @360MHz, 32MB PSRAM) 上 VBA-Next GBA 模拟器的优化路线。

## 进度（真机实测，Yoshi Sample = 100% ARM 最坏情况）

| 步骤 | FPS | core0(仿真) | 备注 |
|------|-----|-------------|------|
| 基线 | 19 | 43ms | 单核 / USE_TWEAKS=0 / -O3 / 全 PSRAM |
| + `USE_TWEAKS=1` | 24 | 33ms | 空转循环加速,-10ms |
| + 双核显示/音频流水线 | **28** | 34ms | blit+audio 移到 core1,WAIT=0(完全隐藏) |
| + **THREADED_RENDERER → core1** | — | **−3ms** | ✅ **已做,真机 koudai 46→54fps / CPU 20→17ms,稳定不崩** |

**累计 19 → 28 fps (+47%);叠加 THREADED_RENDERER 后 koudai 46→54(+17%)。**

## ✅ THREADED_RENDERER 已落地(2026-06-17,推翻之前"跳过"的结论)

JIT 卡墙后转回双核剥离。VBA-Next 自带线程渲染器(`thread.c` + renderer_context 的 io/palette/oam
影子,~10KB static,**不拷 VRAM**),之前关着。开法:CMake `THREADED_RENDERER=1`(核心真正 #if 的宏,
**不是** `USE_THREADED_RENDERER`)+ `DEFAULT_THREADED_RENDERER_ENABLED=1`,把 `src/thread.c` 加进 vbanext
SRCS。thread.c 加了 `#elif defined(ESP_PLATFORM)` 分支:`xTaskCreatePinnedToCore(…, 优先级4, core1)`。
worker 在 renderer_state 上**忙等自旋**,所以:钉 core1、**优先级 4 < 显示任务的 5**(显示任务抢占自旋去贴图)
+ 关 core1 idle WDT(`CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1=n`)。COUNT=1。
**真机(guru=0 wdt=0 稳定):koudai 46→54fps(CPU 20→17ms);Yoshi(ARM 最坏)28→35-36fps(CPU 34→27ms,−7ms!)。**
Yoshi 渲染占比更大 → 省更多。净省 = 渲染移到 core1 − core0 仍付的每行 io/palette/oam 快照。这是新 shipped 配置(JIT 仍 disarmed)。
**注意**:只影子了 io/palette/oam,**没影子 VRAM** → 极少数靠帧内改 VRAM 内容做光栅特效的游戏可能渲染略错
(多数光栅特效是滚动/调色板,已覆盖)→ 让用户肉眼检查有无花屏。之后 core0 = CPU+声音+快照,双核接近到顶。

---

## 关键测量:渲染只占 ~5ms(2026-06-16,真机)

用 `USE_FRAME_SKIP=1` + `SetFrameskip(0x0F)`(每 16 帧只渲染 1 帧)量 core0 的非渲染成本:

| ROM | 全渲染 CPU | 1/16 渲染 CPU | → 渲染成本 | → 解释器成本 |
|-----|-----------|---------------|-----------|-------------|
| koudai(宝可梦) | 20ms (46fps) | **15ms (59–60fps)** | ~5ms | ~15ms |

**结论彻底改变了优化排序:渲染(PPU)只占 ~5ms,解释器才是大头(koudai 15ms / Yoshi ~28ms)。**

含义:
- **THREADED_RENDERER 把渲染甩给 core1,最多只省 ~5ms** → koudai 46→~60(它正好卡在 16.7ms 边
  上,值);Yoshi 34→~29ms→~35fps(仍受解释器限)。**为 5ms 上忙等自旋 + 双 core1 任务 + 看
  门狗 的高风险集成,性价比低。**
- **帧跳(frameskip)** 用同样的 ~5ms 省下来(跳帧不渲染),koudai 实测直接 59–60fps,**零风险、
  已做成 SELECT+R 切换的功能**(0/1/2)。代价是画面变跳。
- **要让重负载游戏(Yoshi 类)上 60,唯一的路是 dynarec。**

**修订建议:跳过 THREADED_RENDERER**(5ms 收益不抵风险,frameskip 已平替)。下一步要么接受
frameskip 作为轻/中负载游戏的近 60fps 方案,要么直接投入 **dynarec** 破解释器天花板。

**结论：解释器是真瓶颈(占 ~75–85% 帧时间),渲染只占 ~5ms。** 60fps 需要把 43ms 砍到 ≤16.7ms（≈2.6×）。
显示+音频共 6ms，双核搬运的天花板只有 +3fps。

参考：同一个 Yoshi ROM 在 mGBA 解释器上约 14fps（见 tab5-gba），VBA-Next 解释器已 19fps。

---

## 改进尝试记录（loop 逐项,三 ROM）

基线(USE_TWEAKS + 双核 + frameskip@0,全 PSRAM,-O3):
**koudai 46 / sizhijian 41 / Yoshi 28 fps。**

| # | 改动 | koudai | sizhijian | Yoshi | 结论 |
|---|------|--------|-----------|-------|------|
| 基线 | — | 46 (20ms) | 41 | 28 | — |
| E1 | vbanext `-O3`→`-O2` | 46 (21ms) | — | — | ❌ 略差,回退 -O3(巨型 CPULoop 下 -O3 仍更优) |
| E2 | IWRAM/BIOS/IO→片内 SRAM | 46 (21ms) | — | — | ❌ 0 提升(L2 已盖住 PSRAM 延迟,同 mGBA),回退;保留片内 SRAM 给未来 dynarec |

> E1/E2 都在 koudai(代表性中负载)上测;两者都无提升 → **廉价的编译器/内存放置杠杆已榨干**。
> 解释器是纯吞吐瓶颈(不受内存延迟/编译器影响)。剩余只有:THREADED_RENDERER(仅 5ms,
> 已被 frameskip 平替,不值高风险集成)和 dynarec(数周,真正的天花板)——均超出本轮范围。
> **→ 廉价优化空间耗尽,转入 /code-review。**

### /code-review 结果(high effort,已应用)
审了 vba-next 核心补丁 diff + vgba_run 胶水。两条 finding:
1. **(已修)** `CPULoadRomData` 的 next_pow2 循环在 32MB cap 之前跑 → >1GB 的 size 会让 n 溢出成 0
   死循环;另外 cap 后 memcpy 仍按原 size 拷 → >32MB 文件会溢出 rom 缓冲。修法:先把 need clamp 到
   0x2000000(同时挡住循环溢出),memcpy 前把 size clamp 到 g_rom_alloc。
2. **(已修)** vgba_run 里 `extern retro_set_audio_sample` 死声明,删除。
其余全部 verify 通过(双核交接无竞态、trampoline 重定位顺序、掩码 wrap、EXT_RAM_BSS_ATTR、
队列失败清理、free() 对 heap_caps 有效)。修完 koudai 仍 46fps,无回归。

---

## Phase 0 — 先量，别猜（半天，必做）

优化前必须知道 43ms 花在哪。没有 profile 就调优会像 mGBA dynarec 早期那样白费功夫。

- [ ] **加帧内 profiler**：把 CPU 时间拆成 ① ARM/THUMB 指令执行 ② PPU 逐行渲染 ③ 内存访问。
      在 `gba.c` 的 `CPULoop` / 渲染入口插 `esp_timer` 计数，每 60 帧打印占比（仿 tab5-gba 的 `g_gba_prof`）。
- [ ] **两类 ROM 对照**：Yoshi（ARM 密集）vs 宝可梦（THUMB 多 + 空转循环多）。两者 profile 差异决定后续杠杆排序。

> 关键未知数：渲染（PPU）在 43ms 里占多少。若 ~10–14ms，则 Phase 1 的线程渲染能直接 +8~10fps；若 <5ms，则重心全在指令执行，直接上 dynarec。

---

## Phase 1 — 低成本高回报（1–2 天，配置/集成层面）

按预期回报排序：

1. **`USE_TWEAKS=1` —— VBA-Next 内建空转循环加速**（改 1 行 CMake）
   - 机制：检测游戏“等 vblank/IRQ”的忙等循环并快进，跳过无效仿真。
   - 预期：宝可梦这类空转多的游戏 **可能大涨**；Yoshi 这种纯计算 ARM 收益小。
   - 风险：低（上游默认在真机用得很多），个别游戏可能时序敏感 → 做成可关。

2. **`THREADED_RENDERER` —— PPU 逐行渲染甩给 core1**（中等集成）
   - 机制：VBA-Next 自带多线程逐行渲染器（1–4 worker）。worker 钉在 core1，core0 跑 CPU 的同时 core1 渲染上一行。等价于 mGBA 端做过的 proxy-render 拆核。
   - 预期：把渲染那部分（按 Phase 0 实测，估 ~10ms）从 43ms 里挪走 → **core0 ~33ms → ~28fps**。这是单项最可能的大头。
   - 工作量：开 `USE_THREADED_RENDERER=1` + 编 `thread.c`，把 sthread 映射到 FreeRTOS 任务并 `xTaskCreatePinnedToCore(..., core1)`。
   - 风险：中（VRAM/调色板的双缓冲一致性，上游已设计好 renderer_context 影子缓冲）。

3. **双核流水线（显示+音频搬到 core1）**（小，已有蓝本）
   - 机制：照搬 tab5-gba 的 ping-pong：core0 `retro_run()`，core1 PPA 缩放+DSI+I2S。pix 是整帧，比 mGBA 的 proxy 还简单。
   - 预期：藏掉 VID+AUD 6ms → +2~3fps。和 #2 叠加（#2 管渲染、#3 管出图/出声）。
   - 风险：低。

4. **可选帧跳（跳渲染，不跳仿真）**（小）
   - 机制：VBA-Next 的 `USE_FRAME_SKIP`/`SetFrameskip`——被跳的帧不做 PPU 渲染，直接省掉渲染那段 CPU。SELECT+R 循环 0/1/2，仿 tab5-gba。
   - 预期：跳 1 帧 ≈ 省 ~5ms/帧均摊 → 画面减半但流畅度感知提升；给玩家自己权衡。
   - 风险：低（画面变跳）。

5. **编译器 A/B**（半天）
   - `-O2` vs `-O3`（大解释器有时 -O2 的 icache 局部性更好）、`-funroll-loops`、`-fno-jump-tables` 等对那个巨型 switch 的影响，逐项实测。
   - 风险：低，纯实验。

---

## Phase 2 — 代码/数据放置（2–3 天，低置信度，先验证再做）

> tab5-gba 的结论：IWRAM 搬进片内 SRAM **0 提升**——L2(256KB) 已经把 PSRAM 延迟盖住了。所以这一档默认低优先，靠 Phase 0 profile 决定是否值得。

- [ ] 用 `IRAM_ATTR` 把 Phase 0 找出的最热函数（CPULoop 内核、ARM/THUMB 译码分发）钉进片内指令 RAM，绕开 flash-cache 抖动。
- [ ] 热数据（IWRAM 32K / ioMem / palette）试放片内 SRAM（改 memalign 分类，而非一刀切 PSRAM）。
- [ ] L2 已是 256KB（512KB 开不了机），到顶。CPU 已 360MHz（400 会掉到 90MHz），到顶。

---

## Phase 3 — Dynarec（数周，唯一能破天花板的）

43ms 的本质是 ARM7TDMI 解释器。和 mGBA 当年一样，真正翻倍要靠 **ARM/THUMB → RISC-V JIT**。

- 可复用 tab5-gba-dynarec 的全部经验：fit-gated 块构建、64KB exec arena、避免 never-run 块耗尽 arena、块链接是 FPS 的真正杠杆、不要给块缓存加字段（内部 SRAM 爆 → 音频 OOM）。
- VBA-Next 的内存模型和 mGBA 不同（扁平 rom + map[]），dynarec 要重写，但热路径（THUMB ALU/load/store/branch）套路一致。
- 预期：参照 mGBA，重场景可从 ~20fps → ~40fps 级别。
- 风险：高、周期长。建议 Phase 1/2 把解释器路线榨干、确认仍不够后再上。

---

## 建议执行顺序

1. **Phase 0 profiler**（必做，半天）——决定后面一切。
2. **USE_TWEAKS=1 + 双核显示/音频流水线**（1 天）——先吃掉最容易的，拿到新基线。
3. **THREADED_RENDERER 拆渲染到 core1**（按 profile，若渲染占比大则这是最大单项）。
4. 帧跳 + 编译器 A/B 收尾 Phase 1。
5. 再评估是否进 Phase 2 / Phase 3。

## 关键数字（决策用）
- 帧预算 60fps = 16.7ms；30fps = 33.3ms。
- 当前 core0 = 43ms(CPU) + 6ms(VID+AUD，可搬走) = 49ms → 19fps。
- 仅搬走 VID+AUD：43ms → ~23fps。
- 搬走渲染(估 10ms)到 core1 + 搬走出图：~33ms → ~28–30fps。
- 要 60fps：必须 dynarec。
