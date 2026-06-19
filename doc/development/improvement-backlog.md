# AM32 改进清单

> 生成日期: 2026-06-19
> 基线: `feat/rpm-closed-loop-control` (基于 `upstream/main` @ a864fdf)
> 审计范围: 二次开发引入问题 + 官方仓库通用改进建议

---

## 目录

- [A 部分：二次开发引入问题](#a-部分二次开发引入问题)
  - [A-P0 阻塞级](#a-p0-阻塞级)
  - [A-P1 高优先级](#a-p1-高优先级)
  - [A-P2 中优先级](#a-p2-中优先级)
  - [A-P3 低优先级](#a-p3-低优先级)
- [B 部分：官方仓库改进建议](#b-部分官方仓库改进建议)
  - [B-P0 严重](#b-p0-严重)
  - [B-P1 主要](#b-p1-主要)
  - [B-P2 次要](#b-p2-次要)
- [C 部分：架构演进建议](#c-部分架构演进建议)

---

## A 部分：二次开发引入问题

本地分支 `feat/rpm-closed-loop-control` 中引入或暴露的问题。修改面涉及
`Src/main.c`, `Src/speed_controller.c/h`, `Src/DroneCAN/DroneCAN.c`,
`Inc/common.h`, `Inc/eeprom.h`, `Inc/version.h` 等文件。

### A-P0 阻塞级

#### A-P0.1 `setInput()` 函数过长（~344 行）

| 项目 | 值 |
|------|-----|
| **文件** | `Src/main.c:1052-1396` |
| **引入时机** | 本地 RPM 模式分支新增了 `rpm_command_active` 抢占逻辑、`prop_brake_active` 交互，函数进一步膨胀 |
| **问题** | 344 行单一函数承载：输入仲裁、方向切换（单向/双向/RC Car）、正弦/梯形过渡、比例刹车、限流/限温/失步保护。违反 SRP。 |
| **具体表现** | `setInput()` 内部有 `#ifndef BRUSHED_MODE` 包裹的 ~200 行块；`if (dshot)` / `if (!dshot)` 几乎重复的方向判定；`if (eepromBuffer.rc_car_reverse)` 和 `else` 分支逻辑镜像对称 |
| **建议** | 拆分为 4-5 个单一职责函数：`processDirection()`, `processSineTransition()`, `processBrakeLogic()`, `applyThrottleLimits()`, `processRpmOverride()` |
| **收益** | 可读性 ++，测试可行，减少引入新 bug 的概率 |

#### A-P0.2 RPM 油门路径与普通模式油门逻辑重复

| 项目 | 值 |
|------|-----|
| **文件** | `Src/main.c:1021-1050` (`rpm_update_throttle`) vs `Src/main.c:1052-1396` (`setInput` 下半段) |
| **引入时机** | 本地新增 `rpm_update_throttle()` |
| **问题** | `rpm_update_throttle()` 中 `map(raw, 47, 2047, minimum_duty_cycle, 2000)` + 限流 + 失步保护 + 限温逻辑，与 `setInput()` 中普通模式下半段几乎相同（`map(input, 47, 2047, ...)` + `use_current_limit` + `stall_protection_adjust`） |
| **建议** | 提取公共油门限幅函数 `uint16_t apply_throttle_limits(uint16_t raw_dc, uint16_t min_dc)`，两处统一调用 |
| **收益** | 消除重复逻辑，避免一处修另一处忘的风险 |

---

### A-P1 高优先级

#### A-P1.1 `FIXED_SPEED_MODE` 与 DroneCAN RPM 模式代码路径重复

| 项目 | 值 |
|------|-----|
| **文件** | `Src/main.c:1509-1516` 和 `Src/main.c:1522-1526` |
| **问题** | 两条路径都在 1kHz 块中调用 `SC_Update()` + 将输出映射到 `duty_cycle_setpoint`，但入口条件不同（`#ifdef FIXED_SPEED_MODE` vs `if (rpm_command_active)`），导致维护时需要同时更新两处 |
| **建议** | 统一入口：`if (rpm_command_active || defined(FIXED_SPEED_MODE)) { SC_Update(); apply_rpm_output(); }` |
| **收益** | 消除编译条件分支的分歧，未来只需维护一个 RPM 控制链路 |

#### A-P1.2 DroneCAN 输入仲裁复杂度高

| 项目 | 值 |
|------|-----|
| **文件** | `Src/DroneCAN/DroneCAN.c` (`handle_RPMCommand`, `handle_RawCommand`, `DroneCAN_update`) |
| **问题** | Raw/RPM 互斥锁 + 250ms 超时 + 1s pending 切换判定，状态机状态数 5+，分支覆盖未通过自动化验证 |
| **具体场景** | 超时清除 `input_mode` 但保留 `armed` → 另一通道恢复后判定期内可能发出未定义行为；`pending_mode` 与 `input_mode` 的竞争条件 |
| **建议** | 1) 为仲裁状态机添加显式的 `enum canArbState` 枚举；2) 每个仲裁转换点添加断言（`assert(armed == 1)` 等）；3) 考虑单通道模式（FC 端决定哪个激活）简化仲裁 |
| **收益** | 降低竞态风险，提高协议鲁棒性 |

#### A-P1.3 `SC_Update()` 系数刷新频率可优化

| 项目 | 值 |
|------|-----|
| **文件** | `Src/speed_controller.c:124-130` |
| **问题** | Vbat/k_drag/motor_kv 比较每次 `SC_Update` 调用都执行（1kHz），但三者中除 Vbat 外基本不变 |
| **建议** | 将系数刷新改为事件驱动：Vbat 仅在 ADC 更新时标记 dirty；k_drag/kv 仅在 ParamSet 时标记 |
| **收益** | 减少 1kHz 循环中的条件分支开销约每个 cycle 3 次 16-bit 比较 + 2 次 64-bit 乘除 |

#### A-P1.4 `rpm_target_pending` 缺超时上限

| 项目 | 值 |
|------|-----|
| **文件** | `Src/main.c` (`tenKhzRoutine` 中的 RPM 换向处理) |
| **问题** | 方向切换条件 `(commutation_interval > reverse_speed_threshold && duty_cycle < 200)` 可能长期不满足（如电机带载减速慢），导致 `rpm_target_pending` 永久重试 |
| **建议** | 添加重试次数计数器，超过 N 次（如 100 次 = 100ms）后强制切换或停车 |
| **收益** | 防止方向切换死锁 |

---

### A-P2 中优先级

#### A-P2.1 `common.h` 全局变量耦合加剧

| 项目 | 值 |
|------|-----|
| **文件** | `Inc/common.h` |
| **问题** | 本地分支新增了 `rpm_command_active`, `rpm_target_value`, `rpm_target_pending`, `MAXIMUM_RPM_SPEED_CONTROL` 等 extern 全局声明，总数已达 ~45 个 |
| **建议** | 对 RPM 相关状态，建立 `static struct rpm_state { ... }` 并封装访问器；同时不影响编译 |
| **收益** | 为后续架构改进铺垫，提高可测试性 |

#### A-P2.2 `speed_controller.c` 对 `motor_kv` 的外部依赖

| 项目 | 值 |
|------|-----|
| **文件** | `Src/speed_controller.c:5` — `extern uint16_t motor_kv;` |
| **问题** | SpeedController 模块隐含依赖全局 `motor_kv`，使其难以独立测试 |
| **建议** | 将 KV 作为 `SC_SetKv(sc, kv)` 或 `SC_Init` 参数传入 |
| **收益** | 消除隐式依赖，模块化程度 + |

#### A-P2.3 `SC_Init` 默认参数与 EEPROM 加载顺序耦合

| 项目 | 值 |
|------|-----|
| **文件** | `Src/main.c:1757-1766` |
| **问题** | `main()` 中先 `SC_Init(&sc)` 再用 `SC_RestoreFromEeprom(&sc)` 覆盖；若 `loadEEpromSettings()` 或 `eeprom_version` 条件不匹配，部分默认值可能错误暴露 |
| **建议** | 将 `SC_Init` 的默认参数宏（`SC_DEFAULT_*`）与 `loadEEpromSettings` 中的后备值统一来源，或使用 designated initializer 编译期常量 |
| **收益** | 消除默认值分歧点 |

#### A-P2.4 `doc/` 目录下两份 RPM 文档内容重叠

| 项目 | 值 |
|------|-----|
| **文件** | `doc/development/rpm-closed-loop-control.md` (228 行)、`doc/development/rpm-closed-loop-spec.md` (393 行) |
| **问题** | 两份文档均描述 RPM 闭环特性，前者侧重实现说明，后者侧重设计规格和审计记录，部分内容重复（PID 参数表、仲裁逻辑） |
| **建议** | 合并为一份文档，按 `# 设计`/`# 实现`/`# 审计` 分节 |
| **收益** | 减少信息分散 |

---

### A-P3 低优先级

#### A-P3.1 `speed_controller.h` 中 Shift 溢出保护

| 项目 | 值 |
|------|-----|
| **文件** | `Inc/speed_controller.h:27-28` |
| **问题** | `SC_K1_SHIFT = 10`, `SC_K2_SHIFT = 28` 无编译期检查是否超过 int32 位移范围（>= 32 时 UB） |
| **建议** | 添加 `_Static_assert(SC_K1_SHIFT < 32 && SC_K2_SHIFT < 32, "shift too large");` |
| **收益** | 防止未来修改时引入未定义行为 |

#### A-P3.2 `Inc/version.h` 与 EEPROM_VERSION 同步约束

| 项目 | 值 |
|------|-----|
| **文件** | `Inc/version.h` (VERSION_MAJOR/MINOR), `Inc/version.h` (EEPROM_VERSION) |
| **问题** | `EEPROM_VERSION` 独立递增无注释说明触发条件（什么情况下需要 bump） |
| **建议** | 在 `#define EEPROM_VERSION` 上添加注释说明递增规则：`// bump when struct layout changes; do NOT bump for minor param additions in reserved space` |
| **收益** | 降低版本管理失误概率 |

---

## B 部分：官方仓库改进建议

以下问题存在于本地分支**未修改**的上游文件中，属于官方仓库通用改进方向。

### B-P0 严重

#### B-P0.1 `save_flash_nolib()` 无超时保护（全部 MCU）

| 项目 | 值 |
|------|-----|
| **文件** | `Mcu/*/Src/eeprom.c`（所有 MCU 副本） |
| **行号** | `save_flash_nolib()` 中 3 处 `while ((FLASH->SR & FLASH_SR_BSY) != 0)` 循环 |
| **风险** | 若 Flash 控制器因硬件故障、时钟丢失或电气噪声挂起，CPU 永久阻塞在忙等待中。看门狗超时触发 NVIC_SystemReset 后进入 bootloader，但 bootloader 中同样可能重入 Flash 操作。 |
| **重现条件** | Flash 编程期间的电源抖动、EMI 干扰、时钟瞬态失效 |
| **建议** | ```c
// 添加超时计数器
volatile uint32_t timeout = 1000000;
while ((FLASH->SR & FLASH_SR_BSY) != 0) {
    if (--timeout == 0) {
        // 标记错误，恢复寄存器，返回错误码
        FLASH->CR &= ~(FLASH_CR_PG | FLASH_CR_PER);
        return -1;
    }
}
``` |
| **收益** | 防止永久挂死，使看门狗有机会干净复位 |

#### B-P0.2 `map()` 递归实现导致栈溢出风险

| 项目 | 值 |
|------|-----|
| **文件** | `Src/functions.c:22-34` |
| **当前实现** | 用二分法递归实现线性映射，递归深度 O(log₂(range)) |
| **风险** | 当 range = 2³² 时递归深度可达 32 层。Cortex-M0 栈帧每层 ~32 字节 → 约 1KB 栈空间。在 ISR 中调用 `map()` 时（如 `setInput` 中多处调用），主栈 + ISR 栈总消耗可能触及栈底。 |
| **建议** | 恢复被注释掉的线性实现；或保留二分法但改为迭代（while 循环） |
| **收益** | 消除栈溢出导致无征兆重启的硬故障 |

#### B-P0.3 `delayMillis()` 中断嵌套导致时序崩溃

| 项目 | 值 |
|------|-----|
| **文件** | `Src/functions.c:40-47` |
| **当前实现** | `while (millis-- > 0) delayMicros(1000)`，自曝问题："possibility of nested delay due to tenKhzRoutine" |
| **风险** | 若在 20kHz ISR 中调用此函数（或含此函数的调用链），中断嵌套会阻塞所有更低优先级中断 → DShot 丢帧、比较器中断丢失 → 失步 |
| **建议** | 全局替换为基于 systick 或 TIM 计数器的非阻塞延迟：`uint32_t t0 = get_tick_ms(); while (get_tick_ms() - t0 < ms);` 并确保 20kHz 中绝不用阻塞延迟 |
| **收益** | 时序可预测性恢复 |

---

### B-P1 主要

#### B-P1.1 EEPROM 无磨损均衡

| 项目 | 值 |
|------|-----|
| **文件** | `Mcu/*/Src/eeprom.c` |
| **问题** | 每次 `saveEEpromSettings()` 整页擦写 200 字节。STM32F0 Flash 寿命 ~10k 次。若 DroneCAN 参数逐项保存，数月内可能耗尽。 |
| **建议** | 采用单页环形缓冲区方案：每页末 2 字节作序列号，写入时追加到最新条目后，擦除时切换页 |
| **收益** | 寿命可延长 100-1000 倍 |

#### B-P1.2 NXP `delayMicros()` 无超时

| 项目 | 值 |
|------|-----|
| **文件** | `Src/functions.c` (NXP 路径) |
| **代码** | `while (!(UTICK0->STAT & UTICK_STAT_INTR_MASK)) {}` |
| **风险** | 若 UTICK 中断丢失，永久阻塞 |
| **建议** | 添加与 B-P0.1 相同的超时计数器保护 |
| **收益** | 跨 MCU 一致性 + 防止 NXP 平台挂死 |

#### B-P1.3 `__disable_irq()` 临界区粒度过粗

| 项目 | 值 |
|------|-----|
| **文件** | `Src/main.c:851-877` (`commutate()`) |
| **问题** | `__disable_irq()` / `__enable_irq()` 包裹整个 `comStep()`，若 comStep 含慢速 GPIO 操作或函数调用，DShot 输入中断可能被延迟数十微秒 |
| **建议** | 1) 优先使用 `__get_PRIMASK()` / `__set_PRIMASK()` 保存/恢复；2) 或仅屏蔽 DShot DMA 中断而非全局关中断；3) 缩短临界区到仅保护寄存器写操作 |
| **收益** | 降低 DShot 遥测抖动，提高高速通信可靠性 |

#### B-P1.4 魔法数字泛滥

| 数字 | 出现次数（约） | 含义 | 涉及文件 |
|------|---------------|------|---------|
| 47, 48 | 30+ | 最小油门值 | `main.c`, `signal.c` |
| 1047, 1048 | 10+ | DShot 中性点 ±47 | `main.c` |
| 1000, 1500, 2000 | 15+ | PWM 脉宽阈值 | `main.c`, `signal.c` |
| 2047 | 15+ | 最大油门值 | `main.c`, `speed_controller.c` |
| 137 | 3 | 正弦模式切换点 | `main.c` |
| 8192, 12495/10 | 2 | FF 模型缩放系数 | `speed_controller.c` |
| 370 | 1 | 电池节数检测 | `main.c` |
| CURRENT_OFFSET (498) | 各 MCU | 电流采样偏置 | 各硬件组 target |

**建议**: 在 `targets.h` 或各模块头文件中定义语义宏：
```c
#define DSHOT_MIN_THROTTLE   47
#define DSHOT_NEUTRAL       1047
#define SERVO_NEUTRAL_US   1500
#define FF_K1_NUMERATOR    8192
#define CELL_VOLT_DIVISOR   370
```
**收益**: 修改配置时无需海量搜索替换，新人入职障碍降低。

#### B-P1.5 中断优先级动态调整的 MCU 覆盖不全

| 项目 | 值 |
|------|-----|
| **文件** | `Src/main.c:2150-2170` |
| **当前** | 仅 `#if !defined(MCU_G031) && !defined(NEED_INPUT_READY)` 激活 |
| **问题** | G031 和所有 NXP MCU 跳过此优先级动态调度；若这些 MCU 运行 dshot_telemetry 且高速运转，DShot 可能抢占换向中断 |
| **建议** | 确认 G031/NXP 的 NVIC 优先级配置是否与 STM32 兼容，若兼容则移除 `#ifdef` 守卫 |
| **收益** | 跨平台实时性保证一致 |

---

### B-P2 次要

#### B-P2.1 各 MCU `eeprom.c` 代码重复

| 项目 | 值 |
|------|-----|
| **涉及** | 11 个 MCU 文件夹下的 `eeprom.c` |
| **问题** | Flash 锁/擦/写/读逻辑在每个 MCU 中手写，差异仅寄存器名（如 `FLASH->KEYR` vs `TIMER_CCHP`） |
| **建议** | 提取公共 Flash HAL 层（参见 `Mcu/common/`），各 MCU 仅提供宏定义的寄存器映射 |
| **收益** | 减少 ~700 行重复代码，统一 Bug 修复 |

#### B-P2.2 注释掉的死代码

| 内容 | 位置 | 行数 |
|------|------|------|
| `pwmSin[]` SV PWM 表 | `Src/main.c` | ~130 行 |
| `interruptRoutine()` 中 stuck 检测 | `Src/main.c` | ~15 行 |
| 旧 `map()` 实现 | `Src/functions.c` | ~10 行 |
| `// todo` 未实现标记 | 多处 | ~10 处 |

**建议**: 一次性 `git clean && git commit` 清理。

#### B-P2.3 `-flto` 编译优化未启用

| 项目 | 值 |
|------|-----|
| **文件** | `Makefile:14` `CFLAGS_BASE` |
| **当前** | `-O3 -ffunction-sections -fomit-frame-pointer -fsingle-precision-constant -ffast-math` |
| **建议** | 添加 `-flto` 到 `CFLAGS_BASE`，同时添加 `-flto` 到 `LDFLAGS_COMMON`。需验证各 MCU 工具链是否支持 LTO。 |
| **预期收益** | 代码尺寸减小 5-15%，跨模块内联机会增加，适合 Flash 紧张的 MCU（如 F031 32KB） |

#### B-P2.4 `DRONECAN_SUPPORT` `#ifdef` 守卫的测试覆盖率盲区

| 项目 | 值 |
|------|-----|
| **问题** | DroneCAN 功能仅在某几个目标板上启用，其他目标板编译时 `#if DRONECAN_SUPPORT` 为 false，导致代码可达性差异大 |
| **建议** | CI 中添加一个启用 DroneCAN 的目标板编译验证 |

#### B-P2.5 `format.sh` 代码格式化未集成到 CI

| 项目 | 值 |
|------|-----|
| **问题** | 仓库根目录存在 `format.sh` 但 CI (`CICD_build.yml`) 中未调用，导致各贡献者提交的代码风格不一致 |
| **建议** | CI 中添加 `format.sh --check` 步骤 |

---

## C 部分：架构演进建议

长期（需要官方仓库接受或 fork 内独立演进）的架构方向。

### C.1 引入 `struct motor_state` 聚合关键状态

```
当前: 45+ 松散全局变量 (average_interval, battery_voltage, duty_cycle, ...)
建议: struct MotorState { uint16_t avg_interval; uint16_t vbat; ... };
意图: 便于单元测试、传递、状态机管理
```

### C.2 中断下半部模式（Bottom Half）

将 20kHz ISR 中的非紧急工作（遥测合成、LED 更新、电流积分）推迟到主循环，缩短 ISR 时间。

### C.3 统一错误报告框架

```c
enum fault_code {
    FAULT_NONE = 0,
    FAULT_FLASH_TIMEOUT,
    FAULT_OVERCURRENT,
    FAULT_OVERTEMP,
    FAULT_LOW_VOLTAGE,
    FAULT_DESYNC,
    FAULT_STALL,
};
```
提供 `get_last_fault()` 查询接口，DroneCAN 可读取。

### C.4 Flash 驱动抽象层

消除 11 份 `eeprom.c` 代码重复，统一到 `Mcu/common/flash_driver.h` + 各 MCU 的寄存器映射头文件。

### C.5 DroneCAN 参数变更回调机制

```c
typedef void (*param_change_cb_t)(const void *ptr);
void register_param_callback(const char *name, param_change_cb_t cb);
```
参数变更时自动触发对应模块回调（如 `motor_kv` → `update_coeffs`），消除手动同步点。

---

## 优先级速查

| 编号 | 等级 | 标题 | 归属 | 改造成本 | 收益 |
|------|------|------|------|---------|------|
| A-P0.1 | 🔴 Block | `setInput()` ~344 行函数拆分 | **本地** | 中 | 可维护性 ++ |
| A-P0.2 | 🔴 Block | RPM/普通模式油门限幅重复 | **本地** | 低 | 消除重复逻辑 |
| A-P1.1 | 🟠 High | FIXED_SPEED_MODE 与 RPM 路径重复 | **本地** | 低 | 统一控制链路 |
| A-P1.2 | 🟠 High | DroneCAN 仲裁复杂度 | **本地** | 中 | 协议鲁棒性 + |
| A-P1.3 | 🟠 High | SC 系数刷新频率可优化 | **本地** | 低 | 减少 1kHz 开销 |
| A-P1.4 | 🟠 High | rpm_target_pending 缺超时 | **本地** | 低 | 防死锁 |
| A-P2.1 | 🟡 Med | common.h 全局变量耦合 | **本地** | 中 | 可测试性 + |
| A-P2.2 | 🟡 Med | motor_kv 外部依赖 | **本地** | 低 | 模块化 + |
| A-P2.3 | 🟡 Med | SC_Init/EEPROM 默认值耦合 | **本地** | 低 | 减少配置分歧 |
| A-P2.4 | 🟡 Med | 两份 RPM 文档重叠 | **本地** | 低 | 信息集中 |
| A-P3.1 | ⚪ Low | Shift 溢出保护 | **本地** | 低 | 防 UB |
| A-P3.2 | ⚪ Low | EEPROM_VERSION 注释 | **本地** | 低 | 版本管理 |
| B-P0.1 | 🔴 Critical | Flash 无超时保护 | **官方** | 低 | 防永久挂死 |
| B-P0.2 | 🔴 Critical | map() 递归栈溢出 | **官方** | 低 | 消除硬故障 |
| B-P0.3 | 🔴 Critical | delayMillis 中断嵌套 | **官方** | 中 | 时序可预测 |
| B-P1.1 | 🟠 Major | EEPROM 磨损均衡 | **官方** | 高 | 延长硬件寿命 |
| B-P1.2 | 🟠 Major | NXP delayMicros 无超时 | **官方** | 低 | 防 NXP 挂死 |
| B-P1.3 | 🟠 Major | disable_irq 临界区粗 | **官方** | 中 | 降低 DShot 抖动 |
| B-P1.4 | 🟠 Major | 魔法数字泛滥 | **官方** | 中 | 可维护性 + |
| B-P1.5 | 🟠 Major | 中断优先级 MCU 覆盖不全 | **官方** | 低 | 实时性一致 |
| B-P2.1 | ⚪ Minor | eeprom.c 代码重复 | **官方** | 高 | 消除 ~700 行重复 |
| B-P2.2 | ⚪ Minor | 注释死代码 | **官方** | 低 | 可读性 |
| B-P2.3 | ⚪ Minor | -flto 未启用 | **官方** | 低 | 代码尺寸 -5~15% |
| B-P2.4 | ⚪ Minor | DroneCAN 测试覆盖盲区 | **官方** | 低 | 质量保证 |
| B-P2.5 | ⚪ Minor | format.sh 未集成 CI | **官方** | 低 | 代码风格一致 |

---

## 快速上手建议

**本地分支上可立即开始的改进**（不依赖上游）：

1. **A-P0.2** (油门限幅提取) → 修改仅限 `Src/main.c`，低风险，消除最明显的重复
2. **A-P1.4** (rpm_target_pending 超时) → 添加一个计数器字段，改造成本极低
3. **A-P1.3** (SC 系数刷新优化) → 替换比较为 dirty flag
4. **A-P3.1** (shift 溢出保护) → 一行 `_Static_assert`

**依赖上游合入后清理的**：

1. B-P0.1~B-P0.3 属于对上游全覆盖的 Bug 修复，本地 fork 可直接修
2. B-P1.4（魔法数字）建议统一做一次全仓库宏化
