# RPM 转速闭环控制 — 设计与实现说明

> 目标靶板：`AM32_VIMDRONES_NANO_L431_CAN`（`Inc/targets.h` 第 122–135 行）
> 功能分支：`feat/rpm-closed-loop-control`
> 上游（upstream）：`https://github.com/am32-firmware/AM32.git`
> 云端 fork（origin）：`https://github.com/horizon105457/AM32.git`

---

## 一、整体架构

```
DroneCAN 总线
     │
     ├─ UAVCAN_EQUIPMENT_ESC_RAWCOMMAND (ID = 0)
     │      └→ set_input(油门) ─→ 油门→RPM 换算 ─→ PID 速度环 ─→ 电机
     │
     └─ UAVCAN_EQUIPMENT_ESC_RPMCOMMAND (ID = 1031)
            └→ set_input(48)  ──┐
            └→ target_e_com_time ← RPM 换算  ─┬→ 跳过油门→RPM 换算 ─→ PID 速度环 ─→ 电机
            └→ rpm_command_active = 1       ┘

   DroneCAN_update()  keepalive & timeout:
   ┌──────────────────────────────────────────────┐
   │  if (rpm_command_active) {                    │
   │    if (ts - last_rpm_msg > 250ms)              │  ← 用消息时间戳（keepalive 不触碰）
   │      → 超时停机                                 │
   │    else if (ts - last_rpm_msg > 1ms)            │
   │      → set_input(48)  // keepalive，不更新时间戳  │
   │  } else {                                      │
   │    // RawCommand keepalive & timeout            │
   │    // (keepalive 更新时间戳，250ms 超时实际被 bypass)│
   │  }                                              │
   └──────────────────────────────────────────────┘
```

---

## 二、关键文件与位置

### 2.1 变量定义与初始化

| 变量 | 文件:行 | 类型 | 含义 |
|---|---|---|---|
| `drive_by_rpm` | `Src/main.c:278` | `uint8_t` | RPM 模式总开关（runtime，不存 EEPROM） |
| `rpm_command_active` | `Src/main.c:279` | `uint8_t` | DroneCAN 正通过 RPMCommand 调速（阻断油门→RPM 换算） |
| `use_speed_control_loop` | `Src/main.c:358` | `char` | 速度 PID 环路使能 |
| `target_e_com_time` | `Src/main.c:355` | `uint16_t` | 目标换相间隔（微秒） |
| `e_com_time` | `Inc/common.h`(extern) | `int` | 实际换相间隔（微秒），由底层换相 ISR 更新 |
| `input_override` | `Src/main.c:359` | `int32_t` | PID 输出累积量（×10000 缩放） |
| `speedPid` | `Src/main.c:285` | `fastPID` | 速度 PID 控制器实例（Kp=10, Ki=0, Kd=100） |
| `adjusted_input` | `Inc/common.h`(extern) | `uint16_t` | 最终油门值，范围 0–2047 |
| `last_can_input` | `Src/DroneCAN/DroneCAN.c:112` | `static uint16_t` | 最近一次 CAN 收到的原始油门 |
| `last_rpm_command_us` | `Src/DroneCAN/sys_can.h:26` | `uint64_t` | 上次 RPMCommand 时间戳（超时检测用） |
| `canstats` | `Src/DroneCAN/DroneCAN.c:44` | `struct CANStats` | CAN 统计与时间戳结构体 |

### 2.2 函数关键位置

| 函数 | 文件:行 | 作用 |
|---|---|---|
| `handle_RPMCommand()` | `DroneCAN.c:692` | RPMCommand 消息处理：换算 RPM→`target_e_com_time`，设标志 |
| `onTransferReceived()` | `DroneCAN.c:868` | CAN 消息分发入口，RPMCommand 在此路由到 handler |
| `shouldAcceptTransfer()` | `DroneCAN.c:973` | CAN 消息过滤器，需接受 `RPMCOMMAND_ID` 1031 |
| `DroneCAN_update()` | `DroneCAN.c:1220` | DroneCAN 主循环，含 RPM 超时检测（第 1315 行起） |
| `set_input()` | `DroneCAN.c:606` | 将 CAN 输入的油门值注入主信号链路 |
| `setInput()` | `main.c:969` | 根据输入类型（DShot/Servo/CAN）换算 `newinput`→`adjusted_input` |
| `doPidCalculations()` | `main.c:576` | PID 计算：`error = actual - target` |
| 油门处理段 | `main.c:1140` | 根据 `use_speed_control_loop` 和 `drive_by_rpm` 分支处理 |
| 速度 PID 环 | `main.c:1446` | 运行时调用 `doPidCalculations`，累积 `input_override` |

### 2.3 DSDL 生成的消息定义

| 文件 | 内容 |
|---|---|
| `Src/DroneCAN/dsdl_generated/include/uavcan.equipment.esc.RPMCommand.h` | 消息结构：`rpm[]` 数组，最多 20 个 ESC |
| `Src/DroneCAN/dsdl_generated/src/uavcan.equipment.esc.RPMCommand.c` | encode/decode 实现 |

---

## 三、核心控制链路详解

### 3.1 RPM 换算公式

```c
// RPM → 换相间隔（微秒）
target_e_com_time = 60,000,000 / RPM / (motor_poles / 2)
```

- 60,000,000 µs = 1 分钟
- `motor_poles / 2` = 极对数（电气旋转圈数/机械旋转圈数）
- `target_e_com_time` 是**单次换相**的目标时间

### 3.2 PID 控制逻辑（`Src/main.c:576–602`）

```c
error = actual - target              // e_com_time - target_e_com_time
integral += error * Ki               // Ki=0 → 积分项被禁用
derivative = Kd * (error - last_error)
output = error * Kp + integral + derivative
```

- **Kp=10**、**Ki=0**、**Kd=100** → 目前仅 PD 控制
- 输出范围限制在 ±`output_limit`（50000）
- 在 `main.c:1446` 中 `input_override` 累积 PID 输出，限制在 0 ~ 2047×10000

### 3.3 Keepalive & 超时设计（`Src/DroneCAN/DroneCAN.c:1296–1330`）

**RPM 模式的 keepalive 与 RawCommand 的设计差异（改进点）：**

RawCommand 路径 keepalive：
```c
if (ts - last_raw_command_us > TARGET_PERIOD_US) {
    set_input(last_can_input);
    last_raw_command_us = ts;  // ← keepalive 覆盖了时间戳
}
```

→ 隐患：keepalive 每次触发都更新 `last_raw_command_us`，导致 `ts - last_raw_command_us` 永远 ≈ 1ms，**250ms 超时永远无法触发**。

RPM 路径 keepalive（改进版）：
```c
if (rpm_command_active) {
    if (ts - last_rpm_command_us > 250000ULL) {
        // 超时停机
        rpm_command_active = 0; drive_by_rpm = 0; set_input(0);
    } else if (ts - last_rpm_command_us > TARGET_PERIOD_US) {
        set_input(48);  // keepalive，不更新时间戳
    }
}
```

→ `last_rpm_command_us` **仅由 `handle_RPMCommand()` 更新**，keepalive 不碰它。250ms 超时真实有效。

### 3.4 RPMCommand 与 RawCommand 互斥逻辑

- `!rpm_command_active` 时 RawCommand keepalive 才运行；RPM 模式开启时 RawCommand keepalive 被抑制
- 两种控制源在飞控层面互斥，防止 `last_can_input` 交叉污染
- `signaltimeout` 看门狗：RPMCommand 到达 `onTransferReceived` 时清零（同 RawCommand 机制）

---

## 四、重点关注与潜在风险

### ⚠️ 4.1 RawCommand 的 250ms 超时是死代码

- RawCommand keepalive 写入 `last_raw_command_us = ts`，导致 `ts - last_raw_command_us` 永远 ≈ 1ms
- 250ms timeout 条件 `last_raw_command_us != 0 && ts - last_raw_command_us > 250ms` **永不满足**
- **实际安全网是 `signaltimeout`（~2s，`onTransferReceived` 清零）**，不是 DroneCAN 层的 250ms 超时
- RPMCommand 修正了此问题——`last_rpm_command_us` 只由消息 handler 更新，250ms 超时真实有效

### ⚠️ 4.2 `signaltimeout` 看门狗（上层安全网）

- `signaltimeout` 每主循环迭代递增（`main.c:1530`），**仅** `onTransferReceived()` 清零（`DroneCAN.c:871`）
- RPMCommand 和 RawCommand 都通过 `onTransferReceived` 进入，都会清零它
- 飞控通常以 ≥50Hz 发送 RPMCommand（远超 2Hz 阈值），不会触发

### ⚠️ 4.3 多 ESC 索引

- RPMCommand `rpm[20]` 数组 × `eepromBuffer.can.esc_index` 决定槽位
- 与 RawCommand 的 `cmd[20]` 索引机制一致，飞控配置需匹配

### ⚠️ 4.4 偏航/差速场景

- 目前仅实现单一 ESC 的 RPMCommand 响应
- 差速/偏航需要额外逻辑映射多个槽位到多个 ESC

### ⚠️ 4.5 当前控制限制

- **Ki=0**：积分项未启用，可能存在稳态误差
- **无电流前馈**：负载突变时响应可能不够快
- **启动阶段**：`zero_crosses < 100` 时积分器强制归零

---

## 五、修改汇总（自上游提交 `32d7dd0` 起的改动）

### 新增 / 修改的文件

| 文件 | 改动类型 | 说明 |
|---|---|---|
| `Src/DroneCAN/DroneCAN.c` | 修改 | 新增 `handle_RPMCommand()`；`onTransferReceived`/`shouldAcceptTransfer` 新增 RPMCommand 分支；`DroneCAN_update` 新增 RPM 超时 |
| `Src/DroneCAN/sys_can.h` | 修改 | `CANStats` 新增 `last_rpm_command_us` |
| `Src/main.c` | 修改 | 新增 `rpm_command_active` 变量；油门处理段增加 `!rpm_command_active` 判断 |
| `Inc/common.h` | 修改 | 新增 `drive_by_rpm`、`rpm_command_active`、`target_e_com_time`、`MAXIMUM_RPM_SPEED_CONTROL`、`MINIMUM_RPM_SPEED_CONTROL`、`use_speed_control_loop` 的 extern 声明 |

### 未修改但需关注的现有代码

| 位置 | 说明 |
|---|---|
| `Inc/eeprom.h` | `drive_by_rpm` **不在** EEPROM 中存储——每次上电为 0，需 DroneCAN 消息激活 |
| `Src/main.c:1864` | 启动时若 `drive_by_rpm = 1`（目前不可能，因为未存 EEPROM），自动启用 `use_speed_control_loop` |
| `Src/main.c:1829` | `FIXED_SPEED_MODE` 方式直接硬编码 RPM，与 RPMCommand 互斥 |

---

## 六、调试与验证

### 6.1 构建

```bash
make l431   # 编译所有 L431 目标
# 或仅编译 Nano CAN：
make obj/AM32_VIMDRONES_NANO_L431_CAN_2.20.elf
```

### 6.2 典型 DroneCAN 消息流

```
飞控 → CAN 总线 → RPMCommand(ID=1031) → ESC 索引 0 → rpm=5000
  → target_e_com_time = 60000000/5000/(14/2) = 1714 µs
  → PID 以 1714 µs 为目标调节 e_com_time
  → 电机稳定在 5000 RPM
```

### 6.3 超时行为

- 飞控停止发送 RPMCommand → 250ms 后 `drive_by_rpm = 0`，电机停机
- 恢复发送 RPMCommand → 重新设定 RPM 并启动

---

## 七、后续可能优化方向

1. **EEPROM 存储 `drive_by_rpm` 开关** — 允许用户在配置中默认启用 RPM 模式
2. **启用 Ki 项** — 消除稳态误差
3. **电流前馈** — 结合 `currentPid` 提高动态响应
4. **多 ESC 同步** — 协调多个 RPMCommand 槽位的控制
5. **转速爬坡/限幅** — 防止 RPM 阶跃变化导致过流
