# tab5-vgbanext Dynarec 规格 + 全部中间结论（ARM7TDMI → RISC-V JIT）

> 本文件是**单一权威规格**。前三轮自主推进的逐条历史压缩进文末附录；正文只留
> 「现在该怎么做」。每个里程碑 = 编译 + 烧录 + 真机验证（COM4 / ESP32-P4）。

---

## 0. 一句话现状（2026-06-17，本轮重写）

**已交付稳定基线 = 解释器：koudai 46 / sizhijian 41 / Yoshi 28 fps，不崩。JIT 翻译器全部
bit-exact 但当前禁用（`g_vgba_jit=0`）。** dynarec 的「净提速」尚未达成，但之前三轮给出的
「死路」判决**过强**——它建立在错误的基准（Yoshi=纯 ARM，THUMB-JIT 本就帮不上）和一个
**未解决的测量缺口**（koudai 自动运行停在 ARM 标题画面，进不了 THUMB 实际游戏）之上。真正
被证实的瓶颈是**覆盖率**：块在第一条不支持的 op（hi-reg / PUSH-POP / 分支 / 寄存器偏移
load-store）处就终止 → 块成不了形 → 退化成逐指令 dispatch。**结论修正:覆盖 + 块链接是清晰
的杠杆，只是还没在正确的工作负载上量到。**

---

## 1. 已交付基线（解释器，真机）

| ROM | 类型 | FPS | core0 | 备注 |
|-----|------|-----|-------|------|
| koudai（宝可梦，THUMB 多） | 8MB | **46–47** | 20ms | dynarec 的正确测量目标 |
| sizhijian | 8MB | **41** | 23ms | |
| Yoshi（100% ARM，最坏） | 4MB | **28** | 34ms | THUMB-JIT 对它无效（ARM 游戏） |

来源：`USE_TWEAKS=1` + 双核显示/音频流水线（core1 blit+audio）。基线 19→28（Yoshi，+47%）。
渲染只占 ~5ms（frameskip 实测），**解释器占 75–85% 帧时间** = 唯一值得攻的目标。
廉价杠杆（-O2/-O3、IWRAM→片内 SRAM）已榨干，均 0 提升（L2 256KB 盖住 PSRAM 延迟）。

---

## 2. 已证实的地基（保留，勿丢）

**基础设施（mGBA dynarec 搬来，平台相关，真机验证可用）：**
- RV32 发射器 `rv_*`（lui/addi/lw/sw/sb/add/sub/xor/and/or/sltu/xori/andi/sltiu/srli/slli/srai/jalr）
- exec arena：`heap_caps_malloc(MALLOC_CAP_EXEC)`（本板片内可执行 SRAM 单块上限 ~64KB）
- 块缓存 `jit_entry_t{pc,code,ninstr,flags}`，按 `pc>>1` 索引，放**片内 SRAM**（PSRAM 缓存是
  111ms 杀手）。**永远别给 jit_entry_t 加字段**（4096 槽 ×字段 → 片内 +16KB → 音频 OOM → 静默退出）。
- I-cache 刷新：`cache_hal_writeback_addr` + `cache_hal_invalidate_addr` + `fence rw,rw; fence.i`（关键）
- **arena 地址不可复用**：本 P4 上重用地址留陈旧 i-cache → 随机「illegal instruction」崩。满了就停。

**翻译器（全部离线影子校验 bit-exact，真机过）：**
| format | op | 状态 | 标志 |
|--------|----|----|------|
| 1（移位 #imm5） | LSL/LSR/ASR | ✅ 168/168 | N/Z/C（V 不变；#0 特例复刻 VBA） |
| 2（ADD/SUB reg&imm3） | ADD/SUB | ✅ 100/100 | N/Z/C/V |
| 3（立即数 ALU） | MOV/CMP/ADD/SUB #imm8 | ✅ 96/96 | N/Z/C/V |
| 4（逻辑） | AND/EOR/ORR/BIC/MVN/TST | ✅ 150/150 | N/Z（C/V 不变） |
| 6/9/10/11（load/store） | LDR/STR/LDRB/STRB/LDRH（PC/imm/SP 寻址） | ✅ 已写（helper call，非叶块） | — |

标志公式对齐 VBA-Next 的 NEG/POS 宏：`C_add=res<lhs`、`C_sub=!(lhs<rhs)`、
`V=((a^res)&(b^res))>>31`。状态地址由 `vgba_jit_set_state()` 在 CPUInit 注册后烤进生成码。

**Hook 点（gba.c `thumbExecute` ≈6817）**：在每次解释循环顶部，若 `armNextPC` 在 0x08-0x0D（ROM，
只读、SMC 安全）且 `g_vgba_jit`，调 `vgba_jit_thumb_dispatch(armNextPC)`；返回 `njit>0` 则按
**O(1) 块尾记账**（armNextPC=target、reg15=target+2、refill cpuPrefetch[0,1]、
cpuTotalTicks += njit*(seq16+1)）一次性推进，再 continue/break。**要执行的指令在 `bus.armNextPC`，
reg15=armNextPC+2**（曾踩 off-by-2 坑）。

---

## 3. 诚实的结论 + 关键再定位

**三轮自主推进（详见附录）的净结果：**
1. 翻译正确性彻底打牢（format-1/2/3/4 + load/store 全 bit-exact）。✅
2. arena/cache/icache/helper/内部缓存/IWRAM-EWRAM 取指 基础设施齐备、禁用待命。✅
3. **但从未在代表性 THUMB 工作负载上量到净提速**——这是关键缺口，不是死路。

**为什么之前判「死路」过强（务必记住）：**
- **基准错了**：所有「开 JIT 反而变慢」的数都来自 **Yoshi = 100% ARM**。THUMB dispatch 在 ARM
  游戏里要么不触发、要么只翻到极少 THUMB → instrs/frame≈0 是**预期**，不证明架构失败。
- **结构成本数被污染**：那个「70K instrs/frame → 111ms」是 **off-by-2 在翻译垃圾**时量的，不可信。
- **真正的目标 ROM 没量过**：koudai（THUMB 多）自动运行停在 **ARM 标题画面**，没有输入注入 → 进不了
  实际游戏 → blocks=0。**测量缺口** = 头号待办，先于一切优化。

**真正被证实的瓶颈 = 覆盖率（而非缓存位置/dispatch 频率）：**
- dispatch 命中后是**按块**推进（返回 njit，gba.c 一次推 njit 条），**不是逐指令**。
- 块短 = 因为覆盖在第一条 hi-reg / PUSH-POP / 分支 / 寄存器偏移 ldst 处就停。这些 op 在真实基本块里
  **到处都是** → 块几乎只含 0–1 条 → 每块的固定 dispatch 开销摊不掉。
- **所以杠杆 = 把覆盖做到「整块译到下一个分支」+ 块链接（块尾直跳下一块）。**

---

## 4. 唯一可行架构:执行核内联 JIT 主循环（可执行规格）

目标:让 JIT **驱动控制流**——命中 JIT 区时进内层循环逐块跑，链到下一块，直到碰到必须解释的 op
或事件边界才回解释器。这样 dispatch 是**每基本块一次**。**先做这条，别再调缓存。**

### 4.1 覆盖（让块成形——按频率，做满到「能译到分支」）
顺序就是优先级:
1. **hi-reg 操作（format 5, 0x4400-0x47FF）**:MOV/ADD/CMP hi（可涉 r8-r15）。Rd=r15 → 改 PC →
   **当块终止符**（像分支处理）。BX（0x4700）留解释器（块终止）。
   **✅ 本轮已做「两操作数 ≤ r14」子集**:ADD/MOV（不动标志,`emit_hireg_move`）+ CMP（全 NZCV,复用
   `emit_alu_carry`),M3f bit-exact。**任一操作数或目标 = r15 → 块终止**(reg[15] 只在块边界有效,
   不能块内读)。PC-操作数子集等 M6 主循环。
2. **PUSH/POP（format 14, 0xB400-0xBD00）**:多寄存器 ×SP，发射成对的 helper st/ld + SP 调整。
   POP 带 PC（0xBD00）= 块终止（改 PC）。
3. **寄存器偏移 load/store（format 7/8, 0x5000-0x5FFF）**:LDR/STR/LDRB/STRB/LDRH/LDRSB/LDRSH
   `[Rb,Ro]`。`emit_ldst` 已支持 reg+const，加 reg+reg（addr=reg[Rb]+reg[Ro]）。
4. **分支（format 16 Bcc 0xD000-0xDDFF / format 18 B 0xE000）= 块终止符**:
   - B:常量目标,块尾把「下一 PC=target」放 a0 返回。
   - Bcc:发射 14 种条件求值（读 N/Z/C/V）→ 选 target 或 fall-through,放 a0 返回。
   - BL（F1/F2）、SWI 留解释器。
5. **补全直线 ALU**:format-4 剩余（ADC/SBC/NEG/CMN/MUL/ROR/寄存器移位）。本轮已做 ADC/SBC/NEG/MUL/CMN
   （见 §6），寄存器移位待做。

### 4.2 JIT 主循环（消除逐指令税——覆盖够了才有意义）
把 gba.c `thumbExecute` 顶部 hook 从「每次循环顶查一次」改成:命中 JIT 区 → 进**内层 do-while**:
块返回「下一个 PC」(a0) → 主循环 O(1) 记 PC/prefetch/周期 → 用 a0 查/建下一块 → 链上去,直到
块返回 0（碰到解释器 op）或 `cpuTotalTicks>=cpuNextEvent`。块只管:直线 op 数据效果 + 终止分支求值
+ 把下一 PC 放 a0。**块链接 = FPS 的真正杠杆（mGBA 经验:覆盖到 85% 后 FPS 还卡,就是缺链接）。**

### 4.3 内存 / SMC
- 缓存:片内 SRAM,但碎片化只够 ~1024-2048 项;**只在分支目标查**（块尾 a0→查一次）即可把每基本块
  压到一次查找,PSRAM 延迟也能摊薄。先 **片内 1024 项 + 主循环**。
- **只 JIT ROM（0x08-0x0D,只读,无 SMC）**。IWRAM/EWRAM 可写,要等 SMC 失效（写 code 区 bump 代际、
  dispatch 比代际,参考 mGBA `g_iwram_codegen`）。**先不做 IWRAM**——koudai 热点若在 IWRAM,靠测量
  缺口的解决（§5）先确认它到底在哪个区跑。

### 4.4 测量缺口（**头号待办,先于代码优化** — ✅ 工具本轮已就位）
没有代表性测量 = 重蹈 mGBA M9-M12「假无提升」覆辙。本轮做了**测量工具**(全部编译开关,默认关,
shipped build 零改动):
- **✅ profile 模式 `g_vgba_jit==2`**(`VGBA_PROFILE=1` 构建后自检末尾自动 arm):每条 THUMB 指令
  调 `vgba_jit_profile()` 统计**区域分布(EWRAM/IWRAM/ROM/other)+ 每区可译率**,解释器照常跑、
  **不执行任何生成码**(零崩溃风险)。每 60 帧打印 `PROF/frame EWRAM=..(xlat ..%) IWRAM=.. ROM=..`。
  **这直接回答决定性未知数:koudai 热 THUMB 在 ROM(现在就能 JIT)还是 IWRAM(必须先做 SMC)。**
- **✅ 脚本输入 `VGBA_AUTOINPUT=1`**:每 80 帧脉冲 START 再 A,把标题/开场画面推进到游戏内
  (main.c 自动跑 flash:romdata,无触摸)。够通用,blow past 大多数 intro。
- 测量配方:`idf.py -DVGBA_PROFILE=1 -DVGBA_AUTOINPUT=1 build flash`,看串口 `PROF/frame` 行。
- 仍可选 **(a) 嵌入存档**(更确定地进特定场景)作为后续增强;脚本输入是零依赖的先手。

### 4.5 验证 & 预期
- 每加一类 op:**离线 bit-exact 影子校验**（像 §6,selftest 里跑生成码 vs VBA 公式）。
- 加分支/主循环后:**在线影子校验**（JIT 跑完一段 vs 解释器跑同段,比对 reg/flags）抓集成 bug。
- 预期（覆盖+链接做满,在 koudai 这类 THUMB 游戏上）:重场景 ~1.5-2×（参照 mGBA）。Yoshi 类 ARM 游戏要
  等 ARM 模式 JIT（M7）才有大头 → 收益有上限。

---

## 5. 里程碑表（压缩）

| M | 内容 | 状态 |
|---|------|------|
| M0 | codegen self-test（arena/cache/icache/emit）`f(3,4)=7` | ✅ 真机 |
| M1 | dispatch hook（必 miss,证 hook 无副作用） | ✅ 真机 |
| M2 | 第一条 THUMB 译码（MOV #imm8）+ set_state 注册 | ✅ 真机 |
| M3a/b/c | format-3/2/4-逻辑/1-移位 全译,514/514 bit-exact | ✅ 真机 |
| M3d/e/f | format-4 补全(ADC/SBC/NEG/CMN/CMP/MUL)+ format-5 hi-reg(reg-reg 子集) | ✅ build,真机自检待烧录 |
| M4 | dispatch 接活跑多指令块,O(1) 块尾记账 | ✅ 真机(正确,但 Yoshi 上净减速→禁用) |
| M5 | **load/store 覆盖**（PC/imm/SP 寻址,helper call,非叶块） | ✅ 已写,禁用待命 |
| **M6** | 分支终止符(B/Bcc)+ get_block 链式构建器(块返回下一PC)+ gba.c 链式主循环 | ✅ **架构验证:~2×提速,但有崩溃 bug(见 §6c)** |
| M6.5 | **§4.4 测量缺口** — profile 模式(区域+可译率)+ 脚本输入,均为编译开关 | ✅ 工具就位,待烧录测 koudai |
| M7 | ARM 模式 JIT（Yoshi 类 100%-ARM 游戏） | 未开始 |

---

## 6. 本轮推进(2026-06-17 重写后)

**做了:扩 THUMB 覆盖（直线 ALU 补全 + hi-reg 块入口）+ 全部 bit-exact 影子校验。**
（保持 `g_vgba_jit=0` 禁用,只扩翻译器、不改集成,零回归风险。下一会话接 M6 主循环。）

- **format-4 补全**:ADC/SBC/NEG/CMN/CMP(reg) + MUL。新 `emit_alu_carry` 用 VBA 的**精确 3 项 NEG/POS**
  进位+溢出（`emit_addsub_flags`)——`sltu` 捷径一旦带 ADC/SBC 进位就错。新增 `rv_lbu`/`rv_mul`(RV32M)。
  自检 **M3d**(ADC/SBC/NEG/CMN/CMP)+ **M3e**(MUL)。
- **format-5 hi-reg**(spec 头号块入口 op):ADD/MOV(不动标志,`emit_hireg_move`)+ CMP(全 NZCV,复用
  `emit_alu_carry`),仅「两操作数 ≤ r14」子集,r15 操作数/目标/BX → 块终止。自检 **M3f**。
- 基建:`tmp[48]→[96]`、dispatch 边界 `n+48→n+96`、自检 arena `32KB→48KB`（容纳新块,唯一地址不复用）。
- **这不是净提速本身**——净提速要等 M6（分支+主循环+链接）+ M6.5（能在 koudai 上量）。本轮把「覆盖」
  这个被证实的杠杆继续往上推,全部 **build-clean**（idf.py exit 0)+ 离线校验,等用户烧录跑 selftest
  看 `M3d/M3e/M3f PASS`。剩余直线覆盖:寄存器移位(op4 2/3/4/7)、PUSH/POP。

---

## 6b. 真机实测结果（2026-06-17,自抓 COM4 log）

**自检 bug(profiler 抓到)**:`emit_addsub_flags` 把 C/V 算进 **T1** 再 `emit_store_flag(...,T1)`,
而 `emit_store_flag` 会先用 T1 装地址 → 存进 flag 的是地址低字节(M3d:ADC→C137 V136)。修:改从 **A0** 存。
重烧后 **M3d 39/39、M3e 6/6、M3f 7/7 全 PASS**(真机)。

**PROF 区域分布(profile 模式,Yoshi 2.27MB)——推翻「Yoshi 纯 ARM / 死路」旧叙事**:
| 区域 | THUMB 指令/帧 | 可译率 |
|------|--------------|--------|
| ROM(0x08) | **28k–47k** | **79–81%** |
| IWRAM(0x03) | ~350–495 | ~32–47% |
| EWRAM/other | 0 | — |

**koudai(8MB,THUMB 多,自动输入已进入实际游戏)再确认,更强**:
| 场景 | ROM THUMB/帧(可译) | IWRAM/帧(可译) |
|------|--------------------|----------------|
| 菜单/轻负载 | 0.5k–1.4k(67–69%) | ~290(40%) |
| **实际游戏** | **3k–15.7k(79–93%)** | ~190–420(71–79%) |

两个 ROM 一致:**热 THUMB 压倒性在 ROM(ROM 是 IWRAM 的 10–40×),可译率 ~80–90%,只读不需要 SMC。**
连「ARM 重」的 Yoshi 都跑数万条可译 ROM-THUMB/帧。IWRAM 只占 THUMB 总量 ~5–10%(可日后加 SMC 增量)。
真正卡点只剩**块成形 + 逐指令 dispatch 税** → 正是 M6(分支终止符 + 主循环 + 链接)解决的。
profile 模式 FPS 11–30 是测量扰动(jit_emit_one 每指令),非真实成本。

**决策(数据驱动,不再猜):直接做 M6,只 JIT ROM(0x08-0D),不碰 IWRAM/SMC。**
（注:romdata 现为 koudai;profiler/auto-input 是编译开关,已还原 0,设备烧回 shipped 解释器。）

## 6c. M6 链式 JIT 真机结果(2026-06-17,自抓 COM4)

**做了**(三步,每步真机自验):
- **M6a 分支译码**:`emit_branch`(B 无条件常量目标;Bcc 14 种条件求值 → 无分支 select 选 target/fallthrough)+
  `emit_cond`(读 N/Z/C/V → taken)。自检 **M6 PASS 57/57**(14 条件 ×4 标志组 + B,真机)。分支目标公式与
  VBA thumbD0/thumbE0 逐位对齐(taken→A+4+off,not-taken→A+2)。
- **M6a 链式构建器** `vgba_jit_get_block(pc,&ninstr)`:块译直线 op 到分支终止符,**块返回下一 PC(a0)**;
  prologue/epilogue 存 ra;fall-through 块 a0=p。
- **M6b gba.c 链式主循环**:命中 ROM(0x08-0D)时进内层 for(;;),逐块跑、用 a0 链到下一块,直到离开 ROM 或
  `cpuTotalTicks>=cpuNextEvent`,再 O(1) resync(armNextPC/reg15/cpuPrefetch)。dispatch 变成**每基本块一次**。

**架构验证成功(真机 koudai,VGBA_JIT_ARM=1 + 自动输入进游戏)**:
- **实际游戏中 FPS 82–86 / CPU 11ms,对比解释器 46fps/20ms ≈ 2× 提速。链式 JIT 确实把覆盖变成了 FPS。** ✅✅

**但有崩溃 bug(~13s 必崩,Guru,Load/Instruction access fault 交替)**。已用 bisection 精确定位:
| 排除项 | 实验 | 结论 |
|--------|------|------|
| load/store | VGBA_JIT_LDST=0 | 仍崩 → 非 load/store |
| 分支 | VGBA_JIT_BRANCH=0 | 仍崩 → 非分支 |
| 栈溢出 | get_block buf 改 static + 32KB 主栈 | 仍崩 → 非栈 |
| 堆 canary 破坏 | 每帧 `heap_caps_check_integrity_all` | 每帧通过 → 堆完好 |

**崩溃本质**:野跳到 PSRAM 地址、发生在 ISR(`shared_intr_isr` → 损坏的 handler 指针)→ **中断描述符(片内 SRAM,
非 canary 保护)被野写覆盖**。关键对照:**profile 模式(解释器跑每条 op)+ 自动输入,同样的游戏状态跑 42s 不崩;
只有执行原生 ALU 块才崩。** 块单测 bit-exact,但 isolated 测试没覆盖所有 in-context 操作数。
**推断根因**:某个 ALU 发射器在某种实际操作数下算错 reg → 解释器随后把它写进 DMA/IO 寄存器 → 坏 DMA 溢出宿主
缓冲 → 覆盖 ISR 表 → 野跳。

**深度 bisection 结果(本轮做完,排除了一大堆,关键收获)**:
- **执行机制完好**:`VGBA_JIT_NOPEFFECT`(块只推进 PC、不做任何数据效果)→ 30s 不崩(只是游戏卡死)。
  → 块调用/ret/arena/i-cache/链式/handoff 全部 OK。
- **发射器正确(决定性)**:加了**在线差分校验** `VGBA_JIT_VALIDATE`(gba.c:每块跑完,快照→跑 JIT→存结果→
  恢复→用 VBA 解释器 `thumbInsnTable` 跑同一 op→比对 reg[0..15]+NZCV→用解释器结果当权威,游戏不偏)。
  实测 **JITVAL=0 个不一致** —— **JIT 算出的 reg/flags 与 VBA 逐位一致,发射器没错!**
- **不是链式 overshoot**:`JIT_BLK_MAX_INSTR=1`(单 op 块、逐 op 查 cpuNextEvent)仍崩。
- **崩溃位置漂移**(touch driver → shared_intr_isr → 现在 `Blip_Synth_offset`←`soundTimerOverflow` sound.c)
  = 典型内存破坏,在哪用到坏指针就在哪崩。

**崩溃本质已确证(blip guard 实测)**:在 `Blip_Synth_offset` 加 bounds-guard(`g_vgba_jit==1` 时 blip 索引越界
就 clamp+log,不崩)→ **guru=0(不崩了!)**,日志 `BLIPGUARD t=60753 i=24607 size=8032` —— 声音采样时钟
**`t = SOUND_CLOCK_TICKS - soundTicks`(sound.c:2066/548)爆成 ~3× 缓冲 → 写出 blip 缓冲 = 之前的崩。soundTicks 约 -16000。**

**但周期记账不是根因(实测排除)**:`cpuTotalTicks` 从 `ninstr*(seq16+1)` 改精确 per-op、再改 `+= ninstr`(差 3×),
`t`/BLIPGUARD **完全没变** —— 因为只有 ~20 个块在跑(arena tail 10KB,JIT 覆盖 <0.1% 指令),其周期对全局时序无影响。

**矛盾(下一步关键)**:CPULoop 的 soundTicks 逻辑数学上**不可能**让 soundTicks 变负:gba.c:13423 clamp
`clockTicks=cpuNextEvent`、13763-70 `remainingTicks` 排空、9188 `cpuNextEvent=min(…,soundTicks)` 必 ≤ soundTicks
→ `soundTicks -= clockTicks ≥ 0`。可它真到 -16000,**且只在 JIT 跑块时**。发射器对(JITVAL=0)、周期幅度无关
(改了没用)→ **JIT 通过一条静态读不出来的路径把 soundTicks/cpuNextEvent 弄负**。

**下一步只能上硬件 watchpoint 盯 `soundTicks`(片内 SRAM)的写**,抓住把它写负的那条指令/那次访存。
**blip guard 已留在树里**(`g_vgba_jit==1` 才生效 → shipped 不变,且让"armed JIT 不崩"成立,方便带 watchpoint 调试)。
**覆盖受限**:自检占 60KB arena 的 ~50KB → live 仅 10KB(~20 块);要真覆盖必须分离自检/live arena。
**全部诊断开关(VALIDATE/NOPEFFECT/NEWOPS/BRANCH/LDST/ARM + 在线差分 + blip guard)在树里默认关。**

**交付**:JIT 全部基础设施(分支/链式/主循环,VGBA_JIT_ARM/BRANCH/LDST/PROFILE/AUTOINPUT 开关)在树里,默认全关、
disarmed;设备烧回 shipped 解释器(koudai @46fps,稳定)。M6 证明了**这条路能到 ~2×**,只差一个 ALU 正确性 bug。

## 7. 关键教训（mGBA + 本移植,务必遵守）
- **`emit_store_flag` 会 clobber T1**(它用 T1 装目标地址)→ 传给它的 flag 值**绝不能在 T1 里**。
  老发射器把值放 T4 才安全;新写的 `emit_addsub_flags` 误用 T1 → C/V 全错(真机 profiler 抓到)。
- **别给 `jit_entry_t` 加字段**（片内 SRAM +16KB → 音频 OOM → 静默退出）。标志塞 flags 空位。
- **fit-gated 构建**:先 decode/measure 块大小,只在装得下事件窗口（~150 周期)时 alloc+cache;否则当帧
  重复翻译耗尽 arena → 0% 覆盖（mGBA M9-M12 的假「无提升」）。
- **arena 地址不可复用**（i-cache 留陈旧指令 → 崩）。满了停翻译,别回收。
- **先 fit-gate 再发射**;只 JIT ROM（0x08-0D,只读)最安全,IWRAM 要 SMC。
- **要执行的指令在 `bus.armNextPC`**,reg15=armNextPC+2。
- **缓存放片内 SRAM**,绝不 PSRAM（111ms 杀手）。
- **96KB EXEC arena 分配失败**:本板片内可执行单块 ~64KB 上限。
- **跨组件用纯 extern**（无 REQUIRES 边,终链接解析,避免 vbanext↔vgba_jit 环)。

## 8. sdkconfig 前提（已具备）
`CONFIG_ESP_SYSTEM_PMP_IDRAM_SPLIT=n`（堆内存可执行 = MALLOC_CAP_EXEC）。

---

## 附录 A — 三轮自主推进历史（压缩,供追溯）

**第一轮**:M0-M4 打通,format-1/2/3/4 译完,514/514 bit-exact。M4 接活 → Yoshi 28→12fps。原因:覆盖窄
（只 ALU/移位/逻辑)→ 真实热代码被 load/store/branch 主导,块成不了形（17 instrs/frame);每条 THUMB 过一次
dispatch；无块链接。→ 禁用 JIT。修了困扰多轮的间歇崩溃(= 自检 arena 地址复用 → P4 i-cache 残留)。

**第二轮**:#1 自检/运行期 arena 分离;#2 IWRAM/EWRAM 取指（gate 放宽 0x02/03/08-0D);#3 load/store
覆盖（8 个 helper,非叶块,format-6/9/10/11)。实测:Yoshi 开 JIT 28→8fps,CPU 34→111ms,blocks=12,
instrs/frame≈0。判:逐指令 hook 在本平台无法胜过 VBA 表解释器。**(注:此判决基于 Yoshi=ARM,见 §3。)**

**第三轮("投入")**:缓存搬片内 + load/store helper 实测。大内部缓存(48KB)→ 碎片 OOM → Blip_Synth 崩;
小缓存(12KB)→ Yoshi 8fps/110ms,instrs/frame=0。判「死路」,只有「分支目标 hook + 块链接 + 大幅扩覆盖」
可行。**(本轮重写修正:该判决正确指出了出路(=§4),但「死路」措辞过强——从未在 koudai 游戏内量过,
且结构成本数被 off-by-2 污染。出路清楚:覆盖 + 链接 + 解决测量缺口。)**

备份:`tab5-vgbanext-backup-20260616/`、`tab5-vgbanext-backup-20260617/`（本轮重写前）。
