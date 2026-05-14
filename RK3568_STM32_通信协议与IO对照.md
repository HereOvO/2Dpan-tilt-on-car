# RK3568 <-> STM32 通信协议与 IO 对照

本文基于 `D:\projects_D\code\嵌赛\2Dpan-tilt` 当前固件实现整理，描述的是“当前代码已经实现的协议与接口”，不是理想化草案。

## 1. 范围与边界

- 主通信链路是 `USB FS CDC`，即 RK3568 作为 USB Host，STM32 作为 USB Device。
- 双舵机控制由 STM32 输出两路 PWM 完成。
- `USART1` 是独立调试串口，只用于日志，不承载 RK3568 业务协议。
- 当前已实现：
  - 跟踪误差输入
  - 双轴一维 Kalman 观测滤波
  - 参数在线读写
  - 控制命令
  - 状态回传
  - ACK 应答
- 当前未实现：
  - IMU 融合
  - 参数掉电保存
  - CRC16/CRC32
  - 自动重传

## 2. IO 对照

### 2.1 物理接口

| 功能 | STM32 引脚 | 外设 | 方向 | 当前用途 |
|---|---|---|---|---|
| 水平舵机控制 | `PE9` | `TIM1_CH1` | 输出 | `Servo_herizon_ctl` PWM |
| 垂直舵机控制 | `PE11` | `TIM1_CH2` | 输出 | `Servo_vert_ctl` PWM |
| USB D- | `PA11` | `USB_OTG_FS_DM` | 双向 | USB CDC 主通信链路 |
| USB D+ | `PA12` | `USB_OTG_FS_DP` | 双向 | USB CDC 主通信链路 |
| 调试串口 TX | `PA9` | `USART1_TX` | 输出 | `DBG_UART1_TX`，115200 8N1 |
| 调试串口 RX | `PA10` | `USART1_RX` | 输入 | `DBG_UART1_RX`，115200 8N1 |
| SWDIO | `PA13` | `SYS_JTMS-SWDIO` | 双向 | ST-LINK 下载调试 |
| SWCLK | `PA14` | `SYS_JTCK-SWCLK` | 输入 | ST-LINK 下载调试 |
| HSE_IN | `PH0` | `RCC_OSC_IN` | 输入 | 外部 8 MHz 晶振 |
| HSE_OUT | `PH1` | `RCC_OSC_OUT` | 输出 | 外部 8 MHz 晶振 |

### 2.2 舵机 PWM 实际配置

- `TIM1` 用于两路舵机 PWM。
- 当前实际运行参数：
  - `Prescaler = 167`
  - `Period = 19999`
- 在 `168 MHz` 定时器时钟下，得到约 `1 MHz` 计数频率，即 `1 count = 1 us`。
- PWM 周期约 `20 ms`，即 `50 Hz`。
- 上电默认输出：
  - Pan: `1500 us`
  - Tilt: `501 us`
  - 该 `Tilt home_us` 以当前云台机械安装为准，定义为“回中后朝小车正前方”
  - 已于 `2026-05-14` 完成实机联调确认

### 2.3 调试串口

- 当前 `USART1` 已切到 `CubeMX/HAL` 风格初始化。
- 波特率：`115200`
- 数据格式：`8N1`
- 典型日志包括：
  - `tim1 servo pwm psc=... arr=...`
  - `gimbal boot`
  - `usb cdc init`
  - `gimbal task start`
  - `gimbal state=...`
  - `usb rx overflow`
  - `ack queue overflow`

## 3. USB CDC 帧格式

### 3.1 传输说明

- STM32 作为 USB CDC 虚拟串口设备。
- 上位机发送的是二进制字节流，不要依赖 USB 回调天然按帧切分。
- STM32 侧先把字节写入 RX FIFO，再由 `gimbalTask` 在 `100 Hz` 节拍内解析。

### 3.2 帧结构

| 字段 | 长度 | 类型 | 说明 |
|---|---:|---|---|
| `SOF0` | 1 | 固定值 | `0xAA` |
| `SOF1` | 1 | 固定值 | `0x55` |
| `msg_type` | 1 | `uint8` | 消息类型 |
| `flags` | 1 | `uint8` | 标志位 |
| `payload_len` | 1 | `uint8` | 负载长度 |
| `frame_id` | 2 | `uint16_le` | 帧序号 |
| `capture_ts_ms` | 4 | `uint32_le` | 图像采集时刻，单位 ms |
| `payload` | N | bytes | 最大 `160` 字节 |
| `checksum` | 1 | XOR | 从 `msg_type` 到 `payload` 的逐字节异或 |
| `tail` | 1 | 固定值 | `0x0D` |

### 3.3 长度限制

- 最大 payload：`160` 字节
- 固定开销：`13` 字节
- 最大整帧：`173` 字节

### 3.4 字节序与校验

- 多字节字段全部使用小端。
- 校验不包含 `SOF0`、`SOF1`、`tail`。
- 校验失败时当前实现直接丢帧，不自动回包。

## 4. 消息类型

| 名称 | 值 | 方向 | 说明 |
|---|---:|---|---|
| `TRACK` | `0x01` | RK3568 -> STM32 | 跟踪误差输入 |
| `PARAM_SET` | `0x02` | RK3568 -> STM32 | 参数写入 |
| `PARAM_GET` | `0x03` | RK3568 -> STM32 | 参数读取 |
| `CONTROL_CMD` | `0x04` | RK3568 -> STM32 | 控制命令 |
| `STATUS` | `0x81` | STM32 -> RK3568 | 状态上报 |
| `ACK` | `0x82` | STM32 -> RK3568 | 应答帧 |

## 5. flags 定义

### 5.1 请求帧 flags

| 位 | 宏名 | 值 | 含义 |
|---|---|---:|---|
| bit0 | `TARGET_VALID` | `0x01` | 目标有效 |
| bit1 | `ACK_REQUEST` | `0x02` | 请求 ACK |
| bit2 | `STATUS_REQUEST` | `0x04` | 请求额外状态帧 |

### 5.2 状态帧 `status_flags`

| 位 | 宏名 | 值 | 含义 |
|---|---|---:|---|
| bit0 | `TARGET_VALID` | `0x01` | 当前目标有效 |
| bit1 | `TARGET_SEEN` | `0x02` | 曾经接收过有效/无效目标帧 |
| bit2 | `ACK_PENDING` | `0x04` | ACK 队列非空 |
| bit3 | `STATUS_PENDING` | `0x08` | 待发状态帧存在 |
| bit4 | `RX_OVERFLOW` | `0x10` | RX FIFO 溢出 |
| bit5 | `ACK_OVERFLOW` | `0x20` | ACK 队列溢出 |

## 6. 各消息 payload

### 6.1 TRACK (`0x01`)

payload 固定 4 字节：

| 偏移 | 长度 | 类型 | 含义 |
|---|---:|---|---|
| 0 | 2 | `int16_le` | `err_x` |
| 2 | 2 | `int16_le` | `err_y` |

行为：

- `payload_len` 必须等于 `4`
- 原始 `err_x / err_y` 在 STM32 侧先进入 Kalman 观测滤波，再进入控制器
- `TARGET_VALID=1`：更新目标并切到 `TRACKING`
- `TARGET_VALID=0`：目标无效，切到 `HOLD_LAST`
- 如果请求了 `ACK_REQUEST`，合法帧回 `ACK OK`
- 如果请求了 `STATUS_REQUEST`，追加一帧状态

建议误差定义：

- `err_x > 0`：目标在画面中心右侧，需要云台水平向“正方向”修正
- `err_y > 0`：目标在画面中心下侧，需要云台垂直向“正方向”修正
- 实际电机方向如果反了，优先改参数 `invert`，不要改协议正负号

### 6.2 PARAM_SET (`0x02`)

payload 按 6 字节一组：

| 偏移 | 长度 | 类型 | 含义 |
|---|---:|---|---|
| 0 | 1 | `uint8` | `param_id` |
| 1 | 1 | `uint8` | `axis_id` |
| 2 | 4 | `int32_le` | `value` |

行为：

- `payload_len` 必须非 0，且是 `6` 的整数倍
- 所有参数先写入临时结构，全部合法后再整体提交
- 提交后会执行参数整理和限幅
- 成功时 ACK detail 返回“实际生效值”
- 对全局参数，ACK detail 中的 `axis_id` 统一回 `0xFF`
- 若修改了 `KALMAN_ENABLE / KALMAN_Q_MILLI / KALMAN_R_MILLI`，滤波内部状态会立即复位

### 6.3 PARAM_GET (`0x03`)

- 空 payload：返回全量参数
- 非空 payload：按 2 字节一组请求

请求格式：

| 偏移 | 长度 | 类型 | 含义 |
|---|---:|---|---|
| 0 | 1 | `uint8` | `param_id` |
| 1 | 1 | `uint8` | `axis_id` |

响应格式：

- 通过 `ACK(OK)` 返回 detail
- detail 按 6 字节一组组织：`param_id + axis_id + value_le32`

### 6.4 CONTROL_CMD (`0x04`)

payload：

| 偏移 | 长度 | 类型 | 含义 |
|---|---:|---|---|
| 0 | 1 | `uint8` | `cmd_id` |
| 1 | 0/1 | `uint8` | `arg0`，部分命令使用 |

命令定义：

| 命令 | 值 | 行为 |
|---|---:|---|
| `GO_HOME` | `0x01` | 清除目标，回 home，状态设为 `STANDBY` |
| `SET_STANDBY` | `0x02` | 清除目标，回 home，状态设为 `STANDBY` |
| `SET_HOLD_LAST` | `0x03` | 状态设为 `HOLD_LAST` |
| `CLEAR_TARGET` | `0x04` | 清除目标；非开机归中阶段转 `HOLD_LAST` |
| `FORCE_STATUS` | `0x05` | 请求发送状态帧 |
| `SET_STATE` | `0x06` | 用 `arg0` 指定状态 |

`SET_STATE.arg0` 支持：

- `1` -> `STANDBY`
- `2` -> `TRACKING`
- `3` -> `HOLD_LAST`

约束：

- 不允许直接设成 `BOOT_CENTERING`
- 若当前没有有效目标，不能强制切入 `TRACKING`

### 6.5 STATUS (`0x81`)

payload 固定 32 字节：

| 偏移 | 长度 | 类型 | 含义 |
|---|---:|---|---|
| 0 | 1 | `uint8` | `state` |
| 1 | 1 | `uint8` | `status_flags` |
| 2 | 2 | `uint16_le` | `last_frame_id` |
| 4 | 2 | `int16_le` | `last_err_x` |
| 6 | 2 | `int16_le` | `last_err_y` |
| 8 | 2 | `uint16_le` | `pan_us` |
| 10 | 2 | `uint16_le` | `tilt_us` |
| 12 | 2 | `uint16_le` | `pan_compare` |
| 14 | 2 | `uint16_le` | `tilt_compare` |
| 16 | 4 | `uint32_le` | `last_capture_ts_ms` |
| 20 | 4 | `uint32_le` | `last_track_rx_ms` |
| 24 | 4 | `uint32_le` | `status_period_ms` |
| 28 | 4 | `uint32_le` | `target_timeout_ms` |

说明：

- `STATUS` 帧自己的 `frame_id` 是 STM32 自增发送序号，不等于最近一次 TRACK 的 `frame_id`
- `last_err_x / last_err_y` 表示 STM32 当前用于控制的最近误差
  - 对应轴 `KALMAN_ENABLE=1` 时，这里是滤波后的误差
  - 对应轴 `KALMAN_ENABLE=0` 时，这里是原始误差

### 6.6 ACK (`0x82`)

payload：

| 偏移 | 长度 | 类型 | 含义 |
|---|---:|---|---|
| 0 | 1 | `uint8` | `acked_msg_type` |
| 1 | 1 | `uint8` | `ack_code` |
| 2 | N | bytes | `detail` |

`ack_code` 定义：

| 名称 | 值 | 含义 |
|---|---:|---|
| `OK` | `0` | 成功 |
| `BAD_LENGTH` | `1` | 长度错误 |
| `BAD_PARAM` | `2` | 参数错误 |
| `BAD_CMD` | `3` | 控制命令非法 |
| `BAD_STATE` | `4` | 当前状态不允许 |
| `BAD_MSG` | `5` | 消息类型非法 |
| `BUFFER_TOO_SMALL` | `6` | 响应 detail 超出缓冲限制 |

当前 ACK 行为：

- ACK 使用与请求帧相同的 `frame_id`
- ACK 队列长度为 `8`
- 队列满时不再压入新 ACK，同时置位 `ACK_OVERFLOW`

## 7. 状态机

| 状态 | 值 | 含义 |
|---|---:|---|
| `BOOT_CENTERING` | `0` | 上电归中保持期 |
| `STANDBY` | `1` | 待机，持续输出 home |
| `TRACKING` | `2` | 根据误差闭环调整输出 |
| `HOLD_LAST` | `3` | 保持最后一次输出 |

### 7.1 上电流程

1. 加载默认参数
2. 配置舵机定时器
3. 状态进入 `BOOT_CENTERING`
4. 输出先置到 `home_us`
5. 到达 `boot_center_ms` 后切到 `STANDBY`

### 7.2 运行期转移

- `BOOT_CENTERING -> STANDBY`：启动保持时间结束
- `非 BOOT_CENTERING -> TRACKING`：收到合法且 `TARGET_VALID=1` 的 TRACK
- `非 BOOT_CENTERING -> HOLD_LAST`：收到合法且 `TARGET_VALID=0` 的 TRACK
- `TRACKING -> HOLD_LAST`：目标超时
- `任意 -> STANDBY`：`GO_HOME` / `SET_STANDBY` / `SET_STATE(STANDBY)`
- `任意 -> HOLD_LAST`：`SET_HOLD_LAST` / `CLEAR_TARGET` / `SET_STATE(HOLD_LAST)`
- 以下情况会复位 Kalman 内部状态：
  - 收到 `TARGET_VALID=0` 的 TRACK
  - `TRACKING` 状态下目标超时
  - `GO_HOME` / `SET_STANDBY` / `CLEAR_TARGET` / `SET_STATE(STANDBY)`
  - 在线修改 `KALMAN_ENABLE / KALMAN_Q_MILLI / KALMAN_R_MILLI`

## 8. 控制算法

- 控制周期：`100 Hz`
- 每轴独立处理
- 输入：
  - Pan 使用 `err_x`
  - Tilt 使用 `err_y`
- 观测滤波：
  - `TRACK` 有效帧到达时，先对每轴误差做一维标量 Kalman 滤波
  - 当前使用随机游走模型：`P_pred = P_prev + Q`，`K = P_pred / (P_pred + R)`
  - `KALMAN_ENABLE=0` 时，该轴直接旁路原始误差
  - `Q / R` 通过 `PARAM_SET` 在线调整，内部按 `value / 1000.0` 解释为浮点量
- 死区：
  - `abs(err) <= deadband` 时按 `0` 处理
- 增量：
  - `delta_us = err * kp_num / kp_den`
- 限步：
  - `delta_us` 再限幅到 `[-max_step_us, +max_step_us]`
- 方向：
  - `invert=1` 时取反
- 最终输出：
  - `output_us += delta_us`
  - 再按 `min_us/max_us` 限幅

## 9. 参数表

### 9.1 轴参数

| param_id | 名称 | 适用 axis | 说明 |
|---|---|---|---|
| `0x01` | `DEADBAND` | `PAN/TILT/ALL` | 死区 |
| `0x02` | `MAX_STEP_US` | `PAN/TILT/ALL` | 单周期最大脉宽改变量 |
| `0x03` | `KP_NUM` | `PAN/TILT/ALL` | 比例分子 |
| `0x04` | `KP_DEN` | `PAN/TILT/ALL` | 比例分母 |
| `0x05` | `CENTER_US` | `PAN/TILT/ALL` | 中位脉宽 |
| `0x06` | `HOME_US` | `PAN/TILT/ALL` | 回中/待机脉宽 |
| `0x07` | `MIN_US` | `PAN/TILT/ALL` | 脉宽下限 |
| `0x08` | `MAX_US` | `PAN/TILT/ALL` | 脉宽上限 |
| `0x09` | `INVERT` | `PAN/TILT/ALL` | 0 正向，1 反向 |
| `0x0A` | `KALMAN_ENABLE` | `PAN/TILT/ALL` | 0 旁路原始误差，1 启用该轴 Kalman 滤波 |

### 9.2 全局参数

| param_id | 名称 | axis_id | 说明 |
|---|---|---|---|
| `0x20` | `STATUS_PERIOD_MS` | `0xFF` | 周期状态上报间隔 |
| `0x21` | `TARGET_TIMEOUT_MS` | `0xFF` | 目标超时阈值 |
| `0x22` | `BOOT_CENTER_MS` | `0xFF` | 上电归中保持时间 |
| `0x23` | `KALMAN_Q_MILLI` | `0xFF` | Kalman 过程噪声参数，内部按 `value / 1000.0` 使用 |
| `0x24` | `KALMAN_R_MILLI` | `0xFF` | Kalman 观测噪声参数，内部按 `value / 1000.0` 使用 |

### 9.3 axis_id

| 名称 | 值 |
|---|---:|
| `PAN` | `0` |
| `TILT` | `1` |
| `ALL` | `0xFF` |

### 9.4 默认值

| 参数 | PAN | TILT |
|---|---:|---:|
| `deadband` | 4 | 4 |
| `max_step_us` | 20 | 20 |
| `kp_num` | 1 | 1 |
| `kp_den` | 8 | 8 |
| `center_us` | 1500 | 1500 |
| `home_us` | 1500 | 501 |
| `min_us` | 500 | 500 |
| `max_us` | 2500 | 2500 |
| `invert` | 0 | 0 |
| `kalman_enable` | 1 | 1 |

全局默认值：

- `status_period_ms = 100`
- `target_timeout_ms = 250`
- `boot_center_ms = 300`
- `kalman_q_milli = 16000`
- `kalman_r_milli = 64000`

### 9.5 参数整理规则

- `min_us` 最低限制为 `500`
- `max_us` 最高限制为 `2500`
- 若 `min_us > max_us`，会自动交换
- `kp_den == 0` 时自动修正为 `1`
- `max_step_us == 0` 时自动修正为 `1`
- `center_us` 和 `home_us` 最终按 `min_us/max_us` 再限幅
- `invert` 只允许 `0/1`
- `kalman_enable` 只允许 `0/1`
- `kalman_q_milli` 允许 `0` 及以上
- `kalman_r_milli` 最小为 `1`
- 当前默认机械定义：
  - 水平轴 `PAN.home_us = 1500 us` 保持不变
  - 竖直轴 `TILT.home_us = 501 us` 定义为“正前方回中”
  - 上述 `TILT.home_us = 501 us` 已在当前整车安装状态下实机确认
  - 该值已接近当前 `TILT.min_us = 500 us`，若后续机械安装变化较大，除了重标定 `HOME_US`，也应一并评估 `MIN_US / MAX_US`
  - 若后续机械安装再次调整，可通过 `PARAM_SET(HOME_US, axis=TILT)` 在线改回中位

## 10. 上位机联调注意事项

- 当前协议是二进制协议，不是文本协议。
- CDC 接收不保证一包就是一帧，上位机不要按“串口一读一写对应一帧”设计。
- `capture_ts_ms` 当前主要用于透传和状态观测，STM32 不拿它做超时判断。
- 真正的目标超时依据是 STM32 本地收到 TRACK 的时刻。
- ACK 优先于 STATUS 发送。
- 当前内部 RX FIFO 为 `512` 字节，持续突发发送可能触发 `RX_OVERFLOW`。
- 当前 ACK 队列长度为 `8`，若短时间大量需要 ACK 的请求同时涌入，可能触发 `ACK_OVERFLOW`。
- 如果机械方向与协议正负号相反，优先通过 `INVERT` 参数修正，不建议改协议定义。
- 若要强制切入 `TRACKING`，必须先有一帧合法且 `TARGET_VALID=1` 的 TRACK。
- 调 `KALMAN_Q_MILLI` 时：
  - 数值越大，误差跟随越快，平滑性越弱
- 调 `KALMAN_R_MILLI` 时：
  - 数值越大，误差更平滑，但响应更慢

## 11. 推荐联调顺序

1. 上电后等待 `BOOT_CENTER_MS` 结束
2. 发送 `CONTROL_CMD/FORCE_STATUS`
3. 校验 `STATUS` 中的当前状态、输出脉宽和参数
4. 发送一帧 `TARGET_VALID=1` 的 TRACK，观察是否进入 `TRACKING`
5. 再发送 `TARGET_VALID=0` 的 TRACK，观察是否进入 `HOLD_LAST`
6. 用 `PARAM_GET` 读取 `KALMAN_ENABLE / KALMAN_Q_MILLI / KALMAN_R_MILLI`
7. 用 `PARAM_SET` 调 `kp`、`deadband`、`max_step_us`、`KALMAN_Q_MILLI / KALMAN_R_MILLI`
8. 解析 `ACK.detail`，确认返回值已经生效

## 12. 建议的 TRACK 误差定义

建议 RK3568 直接发送“归一化后的有符号误差”，而不是原始像素值：

- `err_x`：目标中心相对画面中心的水平误差
- `err_y`：目标中心相对画面中心的垂直误差
- 推荐先把误差压到一个稳定的小范围，例如 `[-1000, 1000]` 或 `[-500, 500]`

这样 STM32 不需要知道画面分辨率，也不用再把像素误差换算成控制尺度，只要通过 `kp_num/kp_den/max_step_us` 调整手感即可。
