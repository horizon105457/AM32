# RPM Closed-Loop Control — Architecture Spec

## 1. 设计目标

- 在 DroneCAN 框架下实现双向 RPM 闭环速度控制
- 不破坏现有的 DShot/PWM/Servo RAW 输入路径
- 消除 RPM 模式下对 `setInput()` 的依赖
- 明确输入仲裁规则：RAW 和 RPM 互斥，RAW 有最高抢占权

---

## 2. 总体架构

```
┌──────────────────────────────────────────────────────────────┐
│                    CAN IRQ 层 (写原子变量)                    │
│                                                              │
│  handle_RawCommand ──→ setInput(val)    (输入油门值)          │
│  handle_RPMCommand ──→ setRPM()         (输入目标 RPM)       │
│  ├─ rpm_target=0 → 停车 + SC_Reset + duty=0 直接完成        │
│  └─ rpm_target≠0 → SC_SetTarget + 方向预判                  │
│      ├─ 同方向 → rpm_target_pending=0                        │
│      └─ 方向冲突 → rpm_target_pending=1                      │
├──────────────────────────────────────────────────────────────┤
│                    1kHz ISR 块 (处理层)                      │
│                                                              │
│  if rpm_target_pending:   (仅方向冲突时进入)                 │
│      └─ 换向安全处理 (maskPhaseInterrupts / SC_Reset)        │
│                                                              │
│  if rpm_command_active:   (每次执行)                         │
│      ├─ SC_Update(&sc, ...)                                  │
│      └─ rpm_update_throttle() ← 内部函数                      │
│          ├─ startMotor (if !running)                          │
│          ├─ input = raw                                       │
│          └─ duty_cycle_setpoint + 钳位                        │
├──────────────────────────────────────────────────────────────┤
│                    20kHz PWM 输出层                           │
│                                                              │
│  tenKhzRoutine:                                              │
│      duty_cycle = duty_cycle_setpoint                         │
│      斜坡限幅 → adjusted_duty_cycle → SET_DUTY_CYCLE_ALL()   │
└──────────────────────────────────────────────────────────────┘
```

---

## 3. 输入仲裁层 (DroneCAN.c)

### 3.1 input_mode 状态机

```
          第一条 RawCommand             第一条 RPMCommand
  NONE ──────────────────→ RAW     NONE ──────────────────→ RPM
    ↑                        │       ↑                        │
    │  250ms 无消息           │       │  250ms 无消息           │
    └─────────────────────────┘       └────────────────────────┘
```

**规则：**
- `input_mode == RAW`：`handle_RPMCommand` 静默丢弃
- `input_mode == RPM`：`handle_RawCommand` 静默丢弃
- `input_mode == NONE`：两条都接受，第一条锁定模式
- 250ms 无消息 → `input_mode = NONE`，停止电机
- `handle_RPMCommand` 只写 `rpm_target_value`，**不调 `setInput()`**

### 3.2 通用超时与停车路径

**所有超时和外部停车请求统一走 `set_input(0)` → `setInput()`：**

```
set_input(0) → setInput()
  └─ 头部: rpm_command_active → SC_Reset(&sc)   ← 如果 RPM 激活，自动清理
  └─ stuck rotor / 正弦映射 / input=0 / duty_cycle_setpoint=0
```

**涉及路径：**
- RAW 超时（250ms 无 RawCommand）
- RPM 超时（250ms 无 RPMCommand）
- `handle_ArmingStatus`: 解锁时停车
- `handle_param_GetSet`: 方向/双向模式变更时停车

`drive_by_rpm` 已删除，`setInput()` 头部现检查 `rpm_command_active`。  
任何时候调 `setInput()`，自动夺取控制权，RPM 模式干净退出。

---

## 4. 两个入口函数

### 4.1 `setInput()` — RAW 油门入口（CAN IRQ 上下文）

**保持不变，仅头部增加 RPM 清理。**

```c
void setInput(void)
{
    if (rpm_command_active) {
        rpm_command_active = 0;
        SC_Reset(&sc);
    }
    // 以下全部不变：方向/死区 → input → duty_cycle_setpoint
    ...
}
```

`drive_by_rpm` 已删除，`setInput()` 体内原 `if (drive_by_rpm) { input = SC_GetOutput(&sc); }` 分支也已移除。
**行为不变：** 方向仲裁、正弦映射、RC Car、急停、启动、钳位。

### 4.2 `setRPM()` — RPM 目标入口（CAN IRQ 上下文）

**扩写：将可预计算的工作全部在 IRQ 中完成，减少 1kHz 开销。**

```c
void setRPM(void)
{
    if (rpm_target_value == 0) {
        /* 停车 — 全部在 IRQ 中完成 */
        rpm_command_active = 0;
        SC_Reset(&sc);
        duty_cycle_setpoint = 0;
        rpm_target_pending = 0;
    } else {
        int8_t desired_dir;
        uint32_t rpm_abs;
        if (rpm_target_value > 0) {
            desired_dir = 1 - eepromBuffer.dir_reversed;
            rpm_abs = (uint32_t)rpm_target_value;
        } else {
            desired_dir = eepromBuffer.dir_reversed;
            rpm_abs = (uint32_t)(-rpm_target_value);
        }

        if (rpm_abs < MINIMUM_RPM_SPEED_CONTROL) {
            /* 低于最低转速当作停车 */
            rpm_command_active = 0;
            SC_Reset(&sc);
            duty_cycle_setpoint = 0;
            rpm_target_pending = 0;
        } else {
            if (rpm_abs > MAXIMUM_RPM_SPEED_CONTROL)
                rpm_abs = MAXIMUM_RPM_SPEED_CONTROL;

            uint8_t pp_half = eepromBuffer.motor_poles / 2;
            if (pp_half < 1) pp_half = 1;
            uint16_t target = 60000000U / rpm_abs / pp_half;
            SC_SetTarget(&sc, target);

            rpm_command_active = 1;
            /* 方向一致时无需 1kHz 处理 */
            rpm_target_pending = (desired_dir != forward);
        }
    }
}
```

| 依赖 | rpm_target_value, forward, battery_voltage, motor_kv 等 |
|---|---|
| **输出** | rpm_command_active, rpm_target_pending, duty_cycle_setpoint(停车时), SC 内部状态 |

---

## 5. 内部处理层 (1kHz ISR 块)

### 5.1 `rpm_target_pending` — 方向冲突处理

仅方向冲突时进入（`rpm_target_pending` 在 `setRPM()` 中已预判）。  
`SC_SetTarget` 已在 `setRPM()` 中完成，此处只需换向安全处理。

```c
if (rpm_target_pending) {
    rpm_target_pending = 0;

    int8_t desired_dir;
    uint32_t rpm_abs;
    if (rpm_target_value > 0) {
        desired_dir = 1 - eepromBuffer.dir_reversed;
        rpm_abs = (uint32_t)rpm_target_value;
    } else {
        desired_dir = eepromBuffer.dir_reversed;
        rpm_abs = (uint32_t)(-rpm_target_value);
    }

    if (desired_dir != forward) {
        if ((commutation_interval > reverse_speed_threshold && duty_cycle < 200)
            || stepper_sine) {
            forward = desired_dir;
            zero_crosses = 0;
            old_routine = 1;
            maskPhaseInterrupts();
            SC_Reset(&sc);          /* PID 重置，observer 重启 */
        } else {
            rpm_target_pending = 1; /* 下次重试 */
        }
    }
}
```

### 5.2 `rpm_update_throttle()` — 内部油门写入

每次 `SC_Update()` 后调用，将 PID 输出写入油门链。  
电机启动自包含（`!running → startMotor()`），`rpm_apply_target` 已合并至此。

```c
static void rpm_update_throttle(void)
{
    uint16_t raw = SC_GetOutput(&sc);
    input = raw;

    if (raw >= 47) {
        /* 首次或换向复位后启动电机 */
        if (!running) {
            allOff();
            startMotor();
            running = 1;
            last_duty_cycle = min_startup_duty;
        }

        duty_cycle_setpoint = map(raw, 47, 2047,
                                  minimum_duty_cycle, 2000);
        prop_brake_active = 0;

        /* 保护钳位（与 setInput() 下半段保持同源） */
        if (zero_crosses < (uint32_t)(30 >> eepromBuffer.stall_protection)) {
            if (duty_cycle_setpoint < min_startup_duty)
                duty_cycle_setpoint = min_startup_duty;
            if (duty_cycle_setpoint > startup_max_duty_cycle)
                duty_cycle_setpoint = startup_max_duty_cycle;
        }
        if (duty_cycle_setpoint > duty_cycle_maximum)
            duty_cycle_setpoint = duty_cycle_maximum;
        if (use_current_limit &&
            duty_cycle_setpoint > use_current_limit_adjust)
            duty_cycle_setpoint = use_current_limit_adjust;
        if (stall_protection_adjust > 0)
            duty_cycle_setpoint += (uint16_t)(stall_protection_adjust / 10000);
    } else {
        duty_cycle_setpoint = 0;
    }
}
```

### 5.3 1kHz 调用框架

```c
if (rpm_target_pending) {         /* 方向冲突时进入 */
    /* 仅换向安全处理，目标已在 setRPM 中预置 */
    ...
}

if (rpm_command_active) {         /* 每次执行 */
    SC_Update(&sc, e_com_time, (int)commutation_interval, running, zero_crosses);
    rpm_update_throttle();          /* 内部函数 */
}
```

### 职责对比

| 函数 | 调用上下文 | 触发 | 职责 |
|---|---|---|---|
| `setInput()` | CAN IRQ | 每次 RawCommand | 方向仲裁 + 油门写入（旧的全部逻辑）|
| `setRPM()` | CAN IRQ | 每次 RPMCommand | 停车/SC_SetTarget/方向预判 |
| `rpm_target_pending 块` | 1kHz ISR | 仅方向冲突时 | 换向安全处理 (maskPhaseInterrupts) |
| `rpm_update_throttle()` | 1kHz ISR | `rpm_command_active` 时每次 | 电机启动 + 油门写入 + 钳位 |
| `tenKhzRoutine` | 20kHz ISR | 每次中断 | duty_cycle 斜坡 + PWM 输出 |

---

## 6. PWM 输出段 (tenKhzRoutine 下半段)

**完全不变。**

```c
if ((armed && running) && input > 47) {
    adjusted_duty_cycle = ((duty_cycle * tim1_arr) / 2000) + 1;
} else {
    // 刹车/空闲路径（brake_on_stop 等）
}
```

`rpm_update_throttle()` 写 `input = raw` 且 `startMotor()` 设 `running=1`，**门控条件自然满足。**

---

## 7. DroneCAN.c 修改清单

| 修改 | 描述 |
|---|---|
| `set_input()` | 删除 `drive_by_rpm` 注释和 RPM guard |
| `handle_RawCommand` | 不变 |
| `handle_RPMCommand` | 不变（`setRPM` 已扩写） |
| `handle_ArmingStatus` | `set_input(0)` → `setInput()` 自动清理 RPM |
| RAW 超时 | `set_input(0)` → 自动清理 |
| RPM 超时 | `set_input(0)` → 自动清理 |

---

## 8. main.c 修改清单

| 修改 | 区域 | 描述 |
|---|---|---|
| 新函数 `rpm_update_throttle()` | 全局 static | ~40 行，内部函数 |
| 修改 | 区域 | 描述 |
|---|---|---|
| 新函数 `rpm_update_throttle()` | 全局 static | 油门写入 + 电机启动 |
| `setInput()` 头部 | `setInput()` 函数 | 添加 `rpm_command_active` 清理 |
| `setInput()` 体 | `setInput()` 函数 | 删除 `if (drive_by_rpm)` 死分支 |
| `setRPM()` 重写 | 全局 | 扩写为完整入口（SC_SetTarget + 停车 + 方向预判） |
| `drive_by_rpm` 变量 | 全局 | 删除 |
| `common.h` extern | `Inc/common.h` | 删除 `drive_by_rpm` |
| 1kHz pending 块 | tenKhzRoutine | 精简为仅换向处理 |
| 守卫 | `running && rpm_command_active` | 改为 `rpm_command_active` |
| 旧 `duty_cycle_setpoint = map(...)` | 1kHz 块 | 替换为 `rpm_update_throttle()` |
| `#if DRONECAN_SUPPORT setInput()` | 主循环 | **已移除 (之前完成)** |

---

## 9. 边界行为推演

### 9.1 0 → 2000 RPM

```
handle_RPMCommand → setRPM()
  ├─ SC_SetTarget(&sc, 4286)    ← 预计算目标
  ├─ desired_dir == forward → rpm_target_pending=0
  └─ rpm_command_active=1
下一个 1kHz 节拍:
  rpm_command_active → SC_Update → 返回 ff_baseline=48
  → rpm_update_throttle(48):
      !running → allOff() + startMotor() ✓
      duty_cycle_setpoint = map(48)
20kHz: (running=1 && input=48 > 47) → PWM 输出 ✓
```

### 9.2 2000 → -2000 RPM (高速换向)

```
handle_RPMCommand → setRPM()
  ├─ SC_SetTarget(&sc, 4286)    ← 反向目标
  ├─ desired_dir ≠ forward → rpm_target_pending=1
  └─ rpm_command_active=1
下一个 1kHz 节拍:
  rpm_target_pending → desired_dir ≠ forward
  → commutation_interval 小, duty 大 → 不满足换向 → rpm_target_pending=1 (重试)
  → N 次重试后转速降至安全阈值:
    forward=rev, maskPhaseInterrupts, SC_Reset, old_routine=1
  → rpm_command_active → SC_Update → rpm_update_throttle:
      !running → startMotor() ✓
```

### 9.3 2000 → 0 RPM

```
handle_RPMCommand → setRPM()
  ├─ rpm_target_value=0
  ├─ SC_Reset(&sc) + duty_cycle_setpoint=0
  ├─ rpm_command_active=0 → SC_Update 停止
  └─ rpm_target_pending=0     ← 完全无 1kHz 开销
tenKhzRoutine: !(running && input > 47) → 刹车/空闲路径 ✓
```

### 9.4 RPM 超时

```
DroneCAN 超时 → set_input(0) → setInput()
  └─ 头部: rpm_command_active → SC_Reset(&sc)  ← 自动清理
  └─ input=0, duty_cycle_setpoint=0
  └─ tenKhzRoutine: 刹车/空闲路径 ✓
```

### 9.5 RawCommand 到达时 RPM 激活

```
handle_RawCommand → set_input(val) → setInput()
  └─ 头部: rpm_command_active → SC_Reset  ← 自动夺取控制权
  └─ 正常 RAW 路径 ✓
```

### 9.6 同方向连续 RPM 更新 (2000 → 5000, 最频繁场景)

```
handle_RPMCommand → setRPM()
  ├─ SC_SetTarget(&sc, 1714)    ← 新目标
  ├─ desired_dir == forward → rpm_target_pending=0
  └─ rpm_command_active=1
1kHz: rpm_command_active → SC_Update PID 收敛
  → rpm_update_throttle: running=1, 正常写入 duty_cycle_setpoint ✓
  ← 零 rpm_target_pending 开销
```

---

## 10. 未解决的问题 / 未来可能的改进

1. **`drive_by_rpm` 已删除（`rpm_command_active` 替代）：** 原始 RPM guard 已被 `setInput()` 头部的直接清理替代。
