# AM32 电机控制规格说明书

**分支**: `feat/rpm-closed-loop-control-v2`
**版本**: 2.20 (spec v4, cleanroom rebuild)
**日期**: 2026-06-23

> 本文档是 AM32 电机控制子系统的设计真相源,按三层架构组织。
> **Part II(上层/下层界面约束)已固化**,中间层重构不可逾越。
> **Part III(中间层)为当前实现,⚠️ 待重构** —— `setInput()` 等遗留路径
> 后续大改或弃用,届时同步更新本文档。
>
> 替代旧文档 `spec_rpm_closed_loop_control.md`(已弃用)。

---

# Part I — 架构总览

## 1. 概述与目标

为 AM32 ESC 固件添加基于 DroneCAN 的电机控制能力,核心是 RPM 闭环速度跟踪
(高精度、高响应、宽速域),支持双向(正/反转)与制动换向,面向水下机器人
等需要推力精确控制的应用。

### 1.1 核心特性

- **前馈 + I-P 复合控制器**: 物理模型前馈 `k1*RPM + k2*RPM²` + 积分-比例反馈
- **自适应增益**: 根据观测 e_com 时间动态调整 PID 增益,适应宽速域
- **状态观测器**: α=1/4 低通滤波 + 线性外推预测
- **双向 + 制动换向**: `IDLE/BRAKE/RUNNING` 状态机,高速方向冲突先制动
- **DroneCAN 集成**: `RawCommand`(1030)与 `RPMCommand`(1031),带符号输入
- **EEPROM 持久化**: PID 参数可保存 (EEPROM v4)
- **E230 排除**: 32KB 闪存约束下 RPM 闭环经 `#ifndef MCU_GDE23` 整体排除

### 1.2 编译守卫与分层

| 代码域 | 守卫 | 层 | 说明 |
|--------|------|----|------|
| DroneCAN 输入界面 (`struct DroneCANInput`, ISR) | `#if DRONECAN_SUPPORT` | 上层 | 仅 DroneCAN 目标 |
| 底层控制原语 (`motor_control.c`) | 无守卫 | 下层 | 所有 MCU 共享 |
| RPM 闭环 (`speed_controller.c`, `rpm_control_loop`) | `#ifndef MCU_GDE23` | 中层 | E230 排除 |
| DroneCAN 中层路由 (`processDroneCANInput`) | `#if DRONECAN_SUPPORT` | 中层 | 仅 DroneCAN 目标 |

> 底层 `motor_control.c` 无守卫,所有 MCU 家族可用。RPM 闭环受 E230 排除
> (32KB 放不下)。所有 E230 目标均未定义 `DRONECAN_SUPPORT`,两组守卫不冲突。

## 2. 三层架构总览

```
┌─────────────────────────────────────────────────────────────────┐
│ UPPER  输入层                                                    │
│   DroneCANInput (ISR赋值) / signed target (rpm, duty_setpoint)  │
│   界面: dronecan_input.{raw_throttle, rpm_target, mode, ready}  │
│   符号语义: rpm符号=目标方向, (与此不同：duty_setpoint符号=驱动/制动模式)   │
├─────────────────────────────────────────────────────────────────┤
│ MIDDLE 协调层 (1kHz, ⚠️待重构)                                   │
│   状态机: rpm_control_loop (IDLE/BRAKE/RUNNING)                  │
│   路由: processDroneCANInput → raw/rpm 路径                      │
│   职责: 方向协调、速度环、限幅映射、启动重启、换向逻辑           │
│   遗留: setInput() 3D扩采路径 (后续弃用)                         │
├─────────────────────────────────────────────────────────────────┤
│ LOWER  执行层                                                    │
│   motor_control.c 原语 (motor_apply_duty/motor_commutate/...)    │
│   → 硬件 (PWM/相位/比较器)                                       │
│   forward全局: 开环换向序方向 (step++/step--)                    │
└─────────────────────────────────────────────────────────────────┘
```

### 2.1 符号语义(关键)

| 层 | 变量 | 符号含义 |
|----|------|----------|
| 上层 (RPM) | `rpm` | 符号 = **目标方向** (正=正转,负=反转) |
| 上层 (RAW) | `duty_cycle_setpoint` | 正幅值;方向由 `forward` 全局携带 |
| 下层 (共享) | `duty_cycle` | 符号 = **驱动/制动/滑行模式** |

中层的 `rpm_control_loop()` 负责把"目标方向符号"翻译为"驱动/制动模式符号":
方向匹配→正(驱动),方向冲突且高速→负(制动等待),低速后翻转 `forward`→正(驱动)。

### 2.2 数据流

```
DroneCAN帧 → ISR: 解码+赋值 dronecan_input + ready=true + last_command_us
                          ↓
1kHz循环: processDroneCANInput 消费 ready
          ├─ RAW → processDroneCANRaw (当前走setInput,待重构为符号直消费)
          └─ RPM → processDroneCANRPM → setRPM → rpm_active=1
                          ↓
1kHz循环: rpm_control_loop 状态机
          → motor_set_forward / motor_start / motor_apply_duty
                          ↓
10kHz循环: tenKhzRoutine
          duty_cycle=duty_cycle_setpoint → ramp → 单点钳位
          → motor_apply_duty(duty_cycle) → PWM硬件
```

---

# Part II — 界面约束(已固化)

> 本部分定义上层输入界面与下层控制原语,是中间层重构不可逾越的约束边界。

## 3. 上层:DroneCAN 输入界面

### 3.1 界面定义 (`Inc/common.h`)

```c
#ifndef RPM_SHIFT
#define RPM_SHIFT 0
#endif

#define DRONECAN_MODE_NONE  0
#define DRONECAN_MODE_RAW   1
#define DRONECAN_MODE_RPM   2

struct DroneCANInput {
    volatile int16_t raw_throttle;   // RawCommand: -8192~+8191 (int14 符号扩展)
    volatile int16_t rpm_target;     // RPMCommand: ±32767 (int18 >> RPM_SHIFT 压缩)
    volatile uint8_t mode;           // NONE / RAW / RPM
    volatile bool ready;             // 有新数据待处理
};
extern struct DroneCANInput dronecan_input;
extern bool dronecan_armed;
```

### 3.2 DSDL 类型与符号扩展(无损性)

DroneCAN 带符号整数经 `canardDecodeScalar(..., Nbits, true, ...)` 解码,
`true` = 符号扩展。C 结构体字段类型保证无损承载:

| 消息 | DSDL | C 字段 | 值域 | 存入 | 无损 |
|------|------|--------|------|------|------|
| RawCommand (1030) | `int14[<=20]` | `int16_t` | -8192~+8191 | `raw_throttle` 直接 | ✅ 14bit→16bit |
| RPMCommand (1031) | `int18[<=20]` | `int32_t` | -131072~+131071 | `rpm_target = (int16_t)(raw>>RPM_SHIFT)` | 见下 |

**RPM_SHIFT 压缩** (`int32_t` 算术右移,符号扩展):

| RPM_SHIFT | 压缩范围 | 分辨率 | int16 范围 | 覆盖全 18-bit |
|-----------|---------|--------|-----------|--------------|
| 0 (默认) | ±32767 | 1 RPM | ±32767 | ❌ 超 int16 截断 |
| 1 | ±65535 | 2 RPM | ±32767 | ❌ |
| 2 (推荐全覆盖) | ±32767 | 4 RPM | ±32767 | ✅ |

> 实际部署 RPM 远小于 ±32767,`RPM_SHIFT=0` 不影响。需覆盖全 18-bit 用 2。
> 配置: `make <target> CFLAGS="-DRPM_SHIFT=2"`

### 3.3 ISR 赋值实现 (`Src/DroneCAN/DroneCAN.c`)

```c
static void handle_RawCommand(CanardInstance *ins, CanardRxTransfer *transfer) {
    struct uavcan_equipment_esc_RawCommand cmd;
    if (uavcan_equipment_esc_RawCommand_decode(transfer, &cmd)) return;
    if (cmd.cmd.len <= eepromBuffer.can.esc_index) return;
    dronecan_input.raw_throttle = cmd.cmd.data[esc_index];  // int14→int16
    dronecan_input.mode = DRONECAN_MODE_RAW;
    dronecan_input.ready = true;
    canstats.last_command_us = micros64();
}

static void handle_RPMCommand(CanardInstance *ins, CanardRxTransfer *transfer) {
    struct uavcan_equipment_esc_RPMCommand cmd;
    if (uavcan_equipment_esc_RPMCommand_decode(transfer, &cmd)) return;
    if (cmd.rpm.len <= eepromBuffer.can.esc_index) return;
    int32_t raw_rpm = cmd.rpm.data[esc_index];               // int18→int32
    dronecan_input.rpm_target = (int16_t)(raw_rpm >> RPM_SHIFT);
    dronecan_input.mode = DRONECAN_MODE_RPM;
    dronecan_input.ready = true;
    canstats.last_command_us = micros64();
}
```

**ISR 契约**: 仅解码 + 验证 esc_index + 赋值 + 置 ready + 更新时间戳。
**无控制逻辑、无缩放、无方向判定。** 中层 1kHz 循环消费 `ready`。

> 看门狗喂狗: 父回调 `onTransferReceived()` 对任何 DroneCAN 帧执行
> `signaltimeout = 0`,故 `handle_*Command` 不需单独喂狗。

### 3.4 带符号输入 vs 传统 3D 扩采

| 属性 | 传统 DShot/PWM (3D 扩采) | DroneCAN 带符号输入 |
|------|--------------------------|---------------------|
| 原始数据 | 单值 0-2047 (区域编码) | 带符号 int14/int18 |
| 方向编码 | 区域: 48-1047 反 / 1048-2047 正 | 符号位: +正 / -反 |
| 零点 | 1000 (servo) 或 47/1047 (dshot) | 0 |
| 方向判定 | `setInput()` 内部区域比较 + return_to_center | 中层直接看符号 |
| 扩采需求 | 需 `setInput()` 300 行 3D 扩采 | **无需扩采**,符号即方向 |
| 换向协调 | `setInput()` 内 prop_brake + reverse_speed_threshold | 中层状态机 (§6) |

> DroneCAN 路径天然带符号,无需传统 3D 扩采。当前 `processDroneCANRaw`
> 仍把带符号值映射回区域编码再调 `setInput()` —— 过渡实现,中层重构后
> 直接消费符号,弃用 `setInput()`。

## 4. 下层:底层控制原语

### 4.1 接口表 (`Inc/motor_control.h`)

底层统一封闭在 `Src/motor_control.c`,提供以下原语。中层通过组合这些原语
实现速度控制/换向/启动重启。

#### 换向序方向(开环状态)

| 原语 | 签名 | 安全契约 | 说明 |
|------|------|----------|------|
| `motor_set_forward` | `void motor_set_forward(char dir)` | dir 钳位 {0,1} | 1=正序 step++,0=逆序 step-- |
| `motor_get_forward` | `char motor_get_forward(void)` | — | 读当前 forward |

#### 换向步进

| 原语 | 签名 | 安全契约 | 说明 |
|------|------|----------|------|
| `motor_commutate` | `void motor_commutate(void)` | 临界区保护 comStep | 按 forward 推进一步 |
| `motor_comstep` | `void motor_comstep(uint8_t step)` | step 钳位 [1,6] + 临界区 | 直接设步位置,启动对齐用 |

#### 相位输出模式

| 原语 | 签名 | 安全契约 | 说明 |
|------|------|----------|------|
| `motor_all_off` | `void motor_all_off(void)` | — | 三相浮空(滑行) |
| `motor_full_brake` | `void motor_full_brake(void)` | 置 running=0 | 三相低边短接(全制动) |
| `motor_prop_brake` | `void motor_prop_brake(int16_t duty)` | duty 钳位 [-2000,0] | 低边 PWM 比例制动 |
| `motor_active_brake` | `void motor_active_brake(void)` | — | comStep(2) 主动制动 |

#### PWM 应用(signed duty → 硬件,三态合一)

| 原语 | 签名 | 安全契约 | 说明 |
|------|------|----------|------|
| `motor_apply_duty` | `void motor_apply_duty(int16_t duty)` | duty 钳位 ±2000 | +驱动/-制动/0滑行,方向由 forward 决定 |

```c
void motor_apply_duty(int16_t duty) {
    if (duty > 2000)  duty = 2000;
    if (duty < -2000) duty = -2000;
    if (duty > 0)        // DRIVE: 高侧 PWM
        adjusted_duty_cycle = ((duty * tim1_arr) / 2000) + 1;
    else if (duty < 0)   // BRAKE: 低侧 PWM
        adjusted_duty_cycle = tim1_arr - (((uint16_t)(-duty) * tim1_arr) / 2000);
    else {               // COAST (或 brake_on_stop==2 主动制动)
        if ((eepromBuffer.brake_on_stop == 2) && armed)
            motor_active_brake(), adjusted_duty_cycle = DEAD_TIME + ...;
        else
            adjusted_duty_cycle = 0;
    }
}
```

#### BEMF 换向感知

| 原语 | 签名 | 说明 |
|------|------|------|
| `motor_enable_comparator` | `void motor_enable_comparator(void)` | 使能零交叉中断(无感换向闭环) |
| `motor_disable_comparator` | `void motor_disable_comparator(void)` | 屏蔽零交叉中断(开环/启动) |

#### 点火(完整启动序列,原子)

| 原语 | 签名 | 安全契约 | 说明 |
|------|------|----------|------|
| `motor_start` | `void motor_start(int16_t duty)` | 原子序列 | allOff+commutate+interval+running+enableComp+duty |

```c
void motor_start(int16_t duty) {
    allOff();                    // 相位清零
    commutate();                 // 首步换向(按当前 forward)
    commutation_interval = 10000;
    SET_INTERVAL_TIMER_COUNT(5000);
    running = 1;
    enableCompInterrupts();      // 开 BEMF 感知
    last_duty_cycle = min_startup_duty;
    duty_cycle_setpoint = duty;
}
```

#### 状态与兼容

| 原语 | 签名 | 说明 |
|------|------|------|
| `motor_is_running` | `bool motor_is_running(void)` | running != 0 |
| `motor_set_duty` | `void motor_set_duty(int16_t duty)` | 仅设 duty_cycle_setpoint(兼容,可能弃用) |
| `motor_coast` | `void motor_coast(void)` | all_off + running=0 + duty=0(兼容) |

### 4.2 forward 契约

`forward` 是开环方向状态(全局 `char forward`)。底层按它决定 `commutate()`
中 step++/step--,**不验证实际转速方向**。中层负责在合适时机(启动前、
低速换向时)调用 `motor_set_forward` 使其与期望物理方向一致。

### 4.3 安全边界

| 边界 | 底层负责 | 中层负责 |
|------|----------|-----------|
| duty 物理限幅 ±2000 | ✅ `motor_apply_duty`/`motor_prop_brake` | — |
| comStep 临界区 | ✅ `motor_commutate`/`motor_comstep` | — |
| step ∈ [1,6] | ✅ `motor_comstep` | — |
| forward ∈ {0,1} | ✅ `motor_set_forward` | 设置正确方向值 |
| motor_start 原子性 | ✅ | 调用时序 |
| armed 检查 | — | ✅ |
| 速度环/FF/PID | — | ✅ |
| 换向协议(制动等待反转) | — | ✅ |
| 熄火重启 | — | ✅ |
| ramp/启动钳位/电流/堵转 | — | ✅ |
| forward 与实际方向一致 | — | ✅ (开环,中层保证) |

---

# Part III — 中间层(当前实现,⚠️ 待重构)

> 本部分描述当前中间层实现。后续将大幅修改(`setInput()` 等弃用,
> `processDroneCANRaw` 改为符号直消费)。底层原语(§4)和上层界面(§3)
> 约束不变。

## 5. 速度控制器

### 5.1 前馈计算

基于电机物理模型(反电动势 + 风阻/负载阻力):

```
thr = (k1 × RPM) >> SC_K1_SHIFT + (k2 × RPM²) >> SC_K2_SHIFT
```

- `k1 = 8192 × 10 << SC_K1_SHIFT / (KV × Vbat)` — 反电动势系数
- `k2 = k_drag × 10 << SC_K2_SHIFT / (10⁶ × Vbat)` — 阻力系数
- `SC_K1_SHIFT = 10`, `SC_K2_SHIFT = 28` — 定点缩放
- 系数在 Vbat 变化或 `coeff_dirty` 时重算

前馈输出归一化到 DShot 范围,以 `thr_max`(对应 `MAXIMUM_RPM_SPEED_CONTROL`)为满量程:
```
ff = 48×10000 + thr × (2047-48) × 10000 / thr_max
```

> **水下应用**: 螺旋桨水阻远大于空气,`k_drag`(`speed_ff_kdrag`)需实测
> 标定。默认 4640 面向空气,水下推进器通常需更大值。FF 越准,积分器负担越小。

### 5.2 I-P PID 控制器

I-P 结构(积分在误差,比例在测量值):

```
error = observed_e_com - target_e_com        (e_com↑ = 电机变慢 = 需更多输出)
proportional = (Kp × ags/256) × filtered_value / 100
integral     += (Ki × ags/256) × error / 100  (带 anti-windup)
derivative   = (Kd × ags/256) × Δ(filtered_value) / 100  (|Δ|>10 才更新)
```

- **Anti-windup**: 总输出饱和时停止同向积分
- **积分清零**: `zero_crosses < 100` 时重置(启动/堵转)
- **D 项低通**: `filtered += (observed - filtered) >> 3` (α=1/8)
- **输出**: `input_override = ff + trim`,钳位 `[0, 2047×10000]`

### 5.3 自适应增益

根据观测 e_com 时间动态缩放增益(×256):

```
         ┌ 154,                      obs ≤ adapt_low
ags =    │ 154 + (obs-lo)×230/(hi-lo), adapt_low < obs < adapt_high
         └ 384,                      obs ≥ adapt_high
```

低速(e_com 大)增益高,高速(e_com 小)增益低。默认
`adapt_low=2500`, `adapt_high=6000` (e_com µs)。

### 5.4 状态观测器

α=1/4 低通滤波 + 趋势外推:

```
有新测量:  pred = observed + trend
           observed = pred + (raw - pred)/4
           trend += (observed - pred - trend)/4
无新测量:  observed += trend; trend -= trend/16
```

物理限幅 `observed ∈ [100, 60000]` µs。新鲜度由 `step_count`(每 6 步 fresh)
与 `commutation_interval` 变化驱动。

### 5.5 数据结构 (`Inc/speed_controller.h`)

```c
typedef struct {
    SC_PID pid;
    uint16_t target_e_com_time;      // 目标换向周期 (µs)
    uint16_t k_drag;                 // 阻力系数
    uint16_t adapt_low_ecom, adapt_high_ecom;  // 自适应上下限
    int32_t k1, k2;                  // 前馈系数 (定点)
    uint16_t coeff_vbat, coeff_k_drag, coeff_kv;  // 系数缓存
    int32_t ff_compensated;          // 缓存前馈输出
    int32_t input_override;          // 控制器输出 (×10000)
    int32_t observed_e_com_time, trend;  // 观测器状态
    uint8_t obs_init, step_count;    // 观测器控制
    int32_t last_step_interval;
    uint8_t coeff_dirty;
} SpeedController;

typedef struct {
    uint16_t Kp, Ki, Kd;
    int32_t error, integral, integral_limit;
    int32_t filtered_value, last_filtered, derivative;
    int32_t output_limit, pid_output;
} SC_PID;
```

### 5.6 EEPROM 布局 (v4, `eepromBuffer.can.spd`)

| 偏移 | 字段 | 类型 | 默认 | 说明 |
|------|------|------|------|------|
| 192 | `speed_kp` | uint8 | 8 | I-P 比例增益 |
| 193 | `speed_ki` | uint8 | 3 | I-P 积分增益 |
| 194 | `speed_kd` | uint8 | 60 | I-P 微分增益 |
| 195-196 | `speed_ff_kdrag` | uint16 | 4640 | 阻力系数 |
| 197 | `speed_adapt_lo` | uint8 | 25 | 自适应下限(×100→e_com µs) |
| 198 | `speed_adapt_hi` | uint8 | 60 | 自适应上限(×100→e_com µs) |
| 199 | `spd_pad` | uint8 | — | 对齐填充 |

> 改动布局须在 `Inc/version.h` 升 `EEPROM_VERSION`。`SC_RestoreFromEeprom`
> 在 `eeprom_version >= 4` 时调用。

## 6. RPM 闭环状态机

### 6.1 状态定义

```c
typedef enum { RPM_IDLE, RPM_BRAKE, RPM_RUNNING } RPM_State;
#define REVERSE_SPEED_THRESHOLD_ERPM 5000U
#define RPM_BRAKE_LEVEL 500
```

### 6.2 状态转移图

```
                rpm=0 / rpm≤MIN
  ┌──────────┐ ─────────────────► ┌──────────┐
  │ RUNNING  │                    │   IDLE   │
  │ (驱动+)  │ ◄───────────────── │ (滑行 0) │
  └────┬─────┘   rpm>MIN 且        └────┬─────┘
       │         can_start              │ rpm>MIN
       │ 方向冲突                        │ (forward=desired_dir)
       │ 且 !can_reverse_speed           │
       ▼                                 │
  ┌──────────┐   can_reverse_speed()    │
  │  BRAKE   │ ─────────────────────────┘
  │ (制动−)  │   (低速后 forward=desired_dir, 转 RUNNING)
  └──────────┘
```

### 6.3 转移逻辑

| 当前 | 条件 | 动作 → 下一态 |
|------|------|----------------|
| IDLE | `rpm_abs > MIN` | `forward=desired_dir; SC_Reset; apply_sc_target` → RUNNING |
| BRAKE | `desired_dir==forward` | `SC_Reset; apply_sc_target` → RUNNING |
| BRAKE | `can_reverse_speed(erpm)` | `forward=desired_dir; SC_Reset; apply_sc_target` → RUNNING |
| RUNNING | `rpm_abs ≤ MIN` | → IDLE |
| RUNNING | `desired_dir≠forward && !can_reverse_speed` | → BRAKE |
| RUNNING | `desired_dir≠forward && can_reverse_speed` | `forward=desired_dir; SC_Reset; apply_sc_target` (留 RUNNING) |

### 6.4 输出(mode-sign `duty_cycle_setpoint`)

| 状态 | duty_cycle_setpoint | input | prop_brake_active |
|------|---------------------|-------|-------------------|
| RUNNING | `+map(SC_GetOutput)` | SC raw | 0 |
| BRAKE | `-RPM_BRAKE_LEVEL` | 0 | 0 |
| IDLE | 0 | 0 | 0 |

> RUNNING 中 `input = SC_GetOutput(&sc)`(≥48),使 tenKhzRoutine 的单点
> 启动钳位(`input>=47 && zero_crosses<...`)对 RPM 路径生效。

### 6.5 `setRPM()` — 事件驱动目标 setter

新命令到达时调用(由 `processDroneCANRPM` 或 legacy `rpmSet`):
- `rpm==0` 或 `rpm_abs ≤ MIN` → `rpm_active=0; rpm_state=IDLE; SC_Reset`
- 否则 → `apply_sc_target(rpm_abs); rpm_active=1`

## 7. DroneCAN 路由与超时

### 7.1 路由 (`processDroneCANInput`, 1kHz)

```c
void processDroneCANInput(void) {
    if (!dronecan_input.ready) return;
    dronecan_input.ready = false;
    switch (dronecan_input.mode) {
    case DRONECAN_MODE_RAW:  processDroneCANRaw(...); break;
    case DRONECAN_MODE_RPM:  processDroneCANRPM(...); break;
    }
}
```

> `processDroneCANRaw` 当前走 `setInput()` 3D 扩采路径(过渡实现)。
> `processDroneCANRPM` → `setRPM()`。

### 7.2 命令超时(非看门狗)

| 属性 | 说明 |
|------|------|
| 变量 | `canstats.last_command_us` (RAW/RPM 共用) |
| 阈值 | 250ms |
| 行为 | `set_input(0)` + 若 `rpm_active` 则 `rpm_active=0; SC_Reset` |
| 复位 MCU | ❌ 否 |

> ⚠️ 技术债: `set_input`/`last_can_input`/1kHz 重发机制为旧 setInput 路径
> 设计,中层重构时清理。

### 7.3 看门狗(全局信号超时,真看门狗)

| 属性 | 说明 |
|------|------|
| 变量 | `signaltimeout` (uint16) |
| 阈值 | 0.5s (armed) / 2s (未 armed) |
| 行为 | `allOff; armed=0; NVIC_SystemReset()` — **完全复位 MCU** |

喂狗源: PWM/Servo(`signal.c`)、DShot(`dshot.c`)、**任何 DroneCAN 帧**
(`onTransferReceived` 父回调)、启动音效(`sounds.c`)。

| 属性 | 命令超时 (§7.2) | 看门狗 (§7.3) |
|------|-----------------|--------------|
| 变量 | `canstats.last_command_us` | `signaltimeout` |
| 时间 | 250ms | 0.5s / 2s |
| 行为 | 停电机 | `NVIC_SystemReset()` |
| 复位 MCU | ❌ | ✅ |

## 8. 主循环集成

### 8.1 初始化 (`main.c`)

```c
#ifndef MCU_GDE23
SC_Init(&sc);
if (eepromBuffer.eeprom_version >= 4) SC_RestoreFromEeprom(&sc);
#endif
```

`FIXED_SPEED_MODE` 经 `rpm = FIXED_SPEED_MODE_RPM; rpmSet = 1;` 进入闭环。

### 8.2 `tenKhzRoutine()` 1kHz 段

```c
if (one_khz_loop_counter > PID_LOOP_DIVIDER) {  // 1kHz
    one_khz_loop_counter = 0;
#if DRONECAN_SUPPORT
    processDroneCANInput();          // 路由 RAW/RPM
#endif
#ifndef MCU_GDE23
    if (rpmSet) { rpm = newrpm; rpmSet = 0; setRPM(); }  // legacy
    rpm_control_loop();              // IDLE/BRAKE/RUNNING 状态机
#endif
    ... // current_limit, stall_protection (官方逻辑)
}
```

### 8.3 `tenKhzRoutine()` ramp + 钳位 + 底层 (10kHz, 共享)

```c
duty_cycle = duty_cycle_setpoint;
if (duty_cycle > 0) { ...ramp... }          // 仅驱动限速变化率
if (duty_cycle > 0) { ...单点钳位... }      // 启动/最大/电流/堵转
motor_apply_duty(duty_cycle);                // 底层执行
last_duty_cycle = duty_cycle;
SET_DUTY_CYCLE_ALL(adjusted_duty_cycle);
```

> ramp 与钳位只对 `duty_cycle > 0` 生效,故 BRAKE(`-RPM_BRAKE_LEVEL`)与
> COAST(0)绕过 ramp,响应即时。`main.c` 的 `apply_duty_to_hardware()`
> 现为 `motor_apply_duty()` 的薄包装。

---

# Part IV — 参考

## 9. 配置参数总表

### 9.1 控制器默认值 (`Inc/speed_controller.h`)

```c
#define SC_DEFAULT_KDRAG      4640   // 阻力系数 (空气; 水下需标定)
#define SC_DEFAULT_ADAPT_LOW  2500   // e_com µs
#define SC_DEFAULT_ADAPT_HIGH 6000   // e_com µs
#define SC_DEFAULT_KP         8
#define SC_DEFAULT_KI         3
#define SC_DEFAULT_KD         60
```

### 9.2 RPM 闭环参数 (DroneCAN 可配, EEPROM v4)

| 参数 | EEPROM 字段 | 类型 | 默认 | 说明 |
|------|-------------|------|------|------|
| `SPEED_KP` | `speed_kp` | uint8 | 8 | I-P 比例增益 |
| `SPEED_KI` | `speed_ki` | uint8 | 3 | I-P 积分增益 |
| `SPEED_KD` | `speed_kd` | uint8 | 60 | I-P 微分增益 |
| `SPEED_FF_KDRAG` | `speed_ff_kdrag` | uint16 | 4640 | 前馈阻力系数 |
| `SPEED_ADAPT_LO` | `speed_adapt_lo` | uint8 | 25 | 自适应下限(×100→µs) |
| `SPEED_ADAPT_HI` | `speed_adapt_hi` | uint8 | 60 | 自适应上限(×100→µs) |

> 无 `SPEED_FF_K1/K2` 参数 —— k1 由 `motor_kv`×`Vbat` 自动算,k2 由
> `speed_ff_kdrag`×`Vbat` 自动算。

### 9.3 输入模式与电机参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `ESC_RAW_MODE` | 0 | 0=RPMCommand, 1=RawCommand |
| `MOTOR_DIRECTION` (`dir_reversed`) | 0 | 0=正转, 1=反转 |
| `MOTOR_POLES` | 14 | 电机极数(e_com↔RPM 换算) |
| `MOTOR_KV` | — | k1 前馈计算用 |

### 9.4 全局变量与常量

| 变量/常量 | 类型 | 说明 |
|-----------|------|------|
| `sc` | `SpeedController` | 控制器实例 |
| `newrpm` | `volatile int32_t` | 新 RPM 目标(legacy 原子写入) |
| `rpmSet` | `volatile uint8_t` | RPM 目标待处理标志(legacy) |
| `rpm` | `int32_t` | 当前 RPM 目标(带符号) |
| `rpm_active` | `uint8_t` | 闭环使能(setRPM 置位, 超时清除) |
| `rpm_state` | `static RPM_State` | 内部状态机(main.c 私有) |
| `MAXIMUM_RPM_SPEED_CONTROL` | `uint32_t` | 最大 RPM 限制(默认 10000) |
| `MINIMUM_RPM_SPEED_CONTROL` | `uint32_t` | 最小 RPM 限制(默认 1000) |
| `REVERSE_SPEED_THRESHOLD_ERPM` | `5000U` | 允许换向最高 ERPM |
| `RPM_BRAKE_LEVEL` | `500` | 制动力度(duty/2000=25%) |

## 10. 安全机制汇总

| 机制 | 层 | 说明 |
|------|----|------|
| 积分限幅 | 中层 | `integral_limit = 3000000` |
| 输出限幅 | 中层 | `input_override ∈ [0, 2047×10000]` |
| Anti-windup | 中层 | 输出饱和时停止同向积分 |
| 零交叉清零 | 中层 | `zero_crosses < 100` 清积分 |
| e_com 物理限幅 | 中层 | `[100, 60000]` µs |
| 启动钳位 | 中层 | tenKhzRoutine 单点,`zero_crosses<30>>stall` |
| 制动换向保护 | 中层 | 高于 5000 ERPM 禁止直接换向,先 BRAKE |
| 命令超时 | 中层 | 250ms 无更新停电机 |
| duty 物理限幅 | 底层 | `motor_apply_duty` ±2000 |
| comStep 临界区 | 底层 | `motor_commutate`/`motor_comstep` |
| step ∈ [1,6] | 底层 | `motor_comstep` |
| motor_start 原子 | 底层 | 完整启动序列 |
| 看门狗 | 系统 | 0.5s/2s 无输入 `NVIC_SystemReset()` |

## 11. 水下机器人应用说明

### 11.1 场景特征

- 频繁换向、低速机动多、需推力精确控制
- 螺旋桨水阻 ≈ `k_drag × RPM²`,阻力远大于空气 → FF `k2` 项主导
- 宽速域:低速机动(几百 RPM)至高速巡航(数千 RPM)

### 11.2 调参建议

1. **标定 `SPEED_FF_KDRAG`**: 稳态记录不同 RPM 所需 duty,反推 k_drag。
   水下值通常远大于默认 4640。FF 越准,积分器稳态误差越小,响应越快。
2. **`SPEED_KP/KI/KD`**: 水下惯量大,I-P 增益可比空气应用略高;低速域依赖
   自适应增益提高(ags=384@高 e_com)。
3. **`REVERSE_SPEED_THRESHOLD_ERPM`**: 水下螺旋桨带水阻,自然减速快,5000
   ERPM 通常合适;惯量大可下调保护机构。
4. **`RPM_BRAKE_LEVEL`**: 默认 500(25%)。需更快换向可上调至 800-1000,
   注意电流冲击。
5. **`MINIMUM_RPM_SPEED_CONTROL`**: 水下低速机动需此值足够低(200-500),
   否则低速域退出闭环。过低则 e_com 很大,观测器更新慢。

### 11.3 换向行为

目标 RPM 符号翻转时:
1. 高速(>5000 ERPM): 进 BRAKE,`duty=-500` 低侧制动,等减速。
2. 低于阈值: `forward=desired_dir` 翻转换向序列,`SC_Reset`,转 RUNNING。
3. 全程 `rpm_control_loop` 1kHz 状态机自动完成,飞控只需发新 RPM。

### 11.4 推力线性化(超出本期范围)

水下推进器推力 ∝ RPM² × sign(RPM)。若飞控需推力线性控制,应在飞控侧做
`thrust → rpm` 映射(`rpm = sign(thrust) × sqrt(|thrust|/k_thrust)`),
ESC 只负责 RPM 跟踪。本固件不内置推力线性化。

## 12. 文件变更清单

| 文件 | 变更 | 说明 |
|------|------|------|
| `Inc/motor_control.h` | 新增/扩展 | 底层控制原语接口(§4) |
| `Src/motor_control.c` | 新增/扩展 | 底层原语实现 + 安全契约 |
| `Inc/speed_controller.h` | 新增 | 控制器头文件 + `#ifndef MCU_GDE23` |
| `Src/speed_controller.c` | 新增 | FF + I-P + 观测器 + 自适应 (240 行) |
| `Inc/common.h` | 修改 | `DroneCANInput`/RPM 全局/底层状态 extern |
| `Inc/eeprom.h` | 修改 | `can.spd` 结构 (v4, 偏移 192-199) |
| `Inc/version.h` | 修改 | `EEPROM_VERSION → 4` |
| `Src/main.c` | 修改 | DroneCAN 路由 + `setRPM` + `rpm_control_loop` + tenKhzRoutine 集成 + 删 `target_e_com_time_high/low` |
| `Src/DroneCAN/DroneCAN.c` | 修改 | ISR 仅赋值 + 统一 `last_command_us` |
| `Src/DroneCAN/DroneCAN.h` | 修改 | include `sys_can.h` |
| `Src/DroneCAN/sys_can.h` | 修改 | `last_raw_command_us` → `last_command_us` |

## 13. 测试建议

1. **构建矩阵**: F421(无 DroneCAN) / F415_CAN(DroneCAN) / E230(排除) / G071 均通过
2. **RAW 回归**: DShot/Servo + DroneCAN RawCommand 与官方分支行为一致
3. **单向闭环**: 正转启动、阶跃响应、稳态精度、负载扰动恢复
4. **双向换向**: 高速反转命令 → BRAKE(制动)→ 低速翻转 → RUNNING
5. **低速域**: 接近 `MINIMUM_RPM` 的跟踪与退出
6. **超时**: 250ms 无命令停电机;0.5s/2s 无任何输入 MCU 复位
7. **EEPROM v4**: 参数写入→断电→恢复生效
8. **FF 标定**: 稳态积分器输出小 → FF 准确
