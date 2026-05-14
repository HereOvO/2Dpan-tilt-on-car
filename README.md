# 2Dpan-tilt 工程说明

## 1. 工程简介

`2Dpan-tilt` 是一个基于 `STM32F407VET6` 的双舵机二维云台控制工程。

当前版本的核心目标是：

- 由 `RK3568` 通过 `USB CDC` 向 `STM32` 发送目标误差
- 由 `STM32` 以约 `100 Hz` 的频率完成双舵机控制
- 支持在线调参、状态上报、回中待机和调试日志输出

本工程基于以下技术栈：

- `STM32CubeMX`
- `HAL`
- `FreeRTOS`
- `Keil MDK-ARM`
- `USB Device CDC`

## 2. 当前功能

当前已经实现的主要功能如下：

- 双舵机云台控制
  - 水平轴 `PAN`
  - 竖直轴 `TILT`
- `USB FS CDC` 二进制通信协议
  - `TRACK`
  - `PARAM_SET`
  - `PARAM_GET`
  - `CONTROL_CMD`
  - `STATUS`
  - `ACK`
- 状态机
  - `BOOT_CENTERING`
  - `STANDBY`
  - `TRACKING`
  - `HOLD_LAST`
- 在线参数调试
  - 死区
  - 最大单步变化
  - 比例参数
  - Kalman 滤波开关
  - Kalman Q/R 参数
  - 回中位
  - 舵机限位
  - 方向反转
- 双轴一维 Kalman 观测滤波
  - 在 `TRACK` 有效帧进入控制器前执行
  - `TARGET_INVALID`、目标超时、回中类命令和滤波参数变更时自动复位
- 调试串口日志输出
  - 使用 `USART1`
- 联调脚本
  - 通过 `USB CDC + 调试串口` 做集成测试

当前未实现或暂未启用的内容：

- IMU 融合
- 参数掉电保存
- 更强校验方式（如 CRC16/CRC32）

## 3. 当前硬件定义

### 3.1 MCU

- 型号：`STM32F407VET6`

### 3.2 IO 分配

| 功能 | 引脚 | 外设 | 说明 |
|---|---|---|---|
| 水平舵机 PWM | `PE9` | `TIM1_CH1` | `PAN` 轴 |
| 竖直舵机 PWM | `PE11` | `TIM1_CH2` | `TILT` 轴 |
| USB D- | `PA11` | `USB_OTG_FS_DM` | 主通信链路 |
| USB D+ | `PA12` | `USB_OTG_FS_DP` | 主通信链路 |
| 调试串口 TX | `PA9` | `USART1_TX` | 调试日志输出 |
| 调试串口 RX | `PA10` | `USART1_RX` | 调试串口输入 |
| SWDIO | `PA13` | `SYS_JTMS-SWDIO` | 下载调试 |
| SWCLK | `PA14` | `SYS_JTCK-SWCLK` | 下载调试 |

### 3.3 当前默认舵机参数

- `PAN.center_us = 1500`
- `PAN.home_us = 1500`
- `TILT.center_us = 1500`
- `TILT.home_us = 501`

说明：

- 当前 `TILT.home_us = 501 us` 已在现有整车机械安装状态下实机确认，对应“回中后朝小车正前方”
- 该值已经非常接近当前 `TILT.min_us = 500 us`
- 如果后续更换舵盘安装角度、云台支架或摄像头姿态，建议重新标定 `HOME_US`，必要时连同 `MIN_US / MAX_US` 一起调整

## 4. 目录结构

工程主目录结构如下：

```text
2Dpan-tilt/
├─ Core/
│  ├─ Inc/
│  └─ Src/
├─ Drivers/
├─ MDK-ARM/
│  ├─ 2Dpan-tilt/
│  ├─ DebugConfig/
│  └─ RTE/
├─ Middlewares/
├─ tools/
├─ USB_DEVICE/
├─ .mxproject
├─ 2Dpan-tilt.ioc
├─ README.md
├─ RK3568_STM32_通信协议与IO对照.md
└─ RK3568_上位机接口文档.md
```

各目录作用如下：

- `Core/`
  - 工程主逻辑代码
  - 包括 `main.c`、`freertos.c`、`gimbal_app.c`、`usart.c`、`tim.c` 等
- `Drivers/`
  - `CMSIS` 和 `STM32 HAL` 驱动
- `MDK-ARM/`
  - `Keil` 工程文件与构建输出目录
  - `2Dpan-tilt.uvprojx` 位于此目录
- `Middlewares/`
  - `FreeRTOS` 与 `USB Device` 中间件
- `USB_DEVICE/`
  - `USB CDC` 设备层配置与接口实现
- `tools/`
  - 工具脚本
  - 当前包括串口联调脚本 `serial_integration_test.ps1`
- `2Dpan-tilt.ioc`
  - `CubeMX` 配置文件
- `RK3568_STM32_通信协议与IO对照.md`
  - 协议与 IO 对照说明
- `RK3568_上位机接口文档.md`
  - 面向 `RK3568` 上位机开发的接口说明

## 5. 关键源码说明

如果你要快速理解本工程，建议优先阅读以下文件：

- `Core/Src/gimbal_app.c`
  - 云台协议解析
  - 状态机
  - 参数管理
  - 舵机输出控制
- `Core/Inc/gimbal_app.h`
  - 协议常量、状态、数据结构定义
- `Core/Src/freertos.c`
  - `gimbalTask` 入口和周期调度
- `USB_DEVICE/App/usbd_cdc_if.c`
  - `USB CDC` 收发接口
- `Core/Src/usart.c`
  - `USART1` 初始化
- `Core/Src/debug_uart.c`
  - 调试串口输出封装

## 6. 通信方式

### 6.1 主通信链路

- `RK3568 <-> STM32`：`USB FS CDC`

### 6.2 调试链路

- `USART1`
- 波特率：`115200`
- 格式：`8N1`

### 6.3 协议特点

- 二进制帧协议
- 小端字节序
- 异或校验
- 支持字节流拆包，不依赖 USB 一次回调等于一整帧
- `PARAM_SET / PARAM_GET` 支持在线调试 Kalman 参数，成功后立即生效

帧格式：

```text
AA 55 | msg_type | flags | payload_len | frame_id_le(2) | capture_ts_ms_le(4) | payload | xor | 0D
```

详细协议请查看：

- [RK3568_STM32_通信协议与IO对照.md](./RK3568_STM32_通信协议与IO对照.md)
- [RK3568_上位机接口文档.md](./RK3568_上位机接口文档.md)

## 7. 构建与烧录

### 7.1 使用 Keil 打开工程

工程文件：

- `MDK-ARM/2Dpan-tilt.uvprojx`

### 7.2 命令行构建

示例命令：

```powershell
D:\Keil_v5\UV4\UV4.exe -b D:\projects_D\code\嵌赛\2Dpan-tilt\MDK-ARM\2Dpan-tilt.uvprojx -j0
```

构建输出一般位于：

- `MDK-ARM/2Dpan-tilt/`

常见产物包括：

- `2Dpan-tilt.axf`
- `2Dpan-tilt.hex`
- `2Dpan-tilt.map`

### 7.3 命令行烧录

示例命令：

```powershell
"D:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -c port=SWD -w "D:\projects_D\code\嵌赛\2Dpan-tilt\MDK-ARM\2Dpan-tilt\2Dpan-tilt.hex" -v -rst
```

### 7.4 CubeMX 使用注意事项

如果需要改外设配置，建议遵守以下规则：

- 优先修改 `2Dpan-tilt.ioc`
- 重新生成代码后，再补应用逻辑
- 自定义代码尽量保留在 `USER CODE` 区域

## 8. 快速使用流程

### 8.1 硬件连接

至少需要连接：

- `STM32F407VET6`
- 两路舵机
- `ST-LINK`
- `USB FS` 到 `RK3568` 或 PC
- 可选：`USART1` 调试串口

### 8.2 上电后的默认行为

上电后流程如下：

1. 进入 `BOOT_CENTERING`
2. 输出回中位
3. 保持 `boot_center_ms`
4. 自动转入 `STANDBY`

当前默认：

- `boot_center_ms = 300`

### 8.3 上位机最小联调步骤

建议按下面顺序联调：

1. 上电，等待 `300 ms`
2. 发送 `CONTROL_CMD(FORCE_STATUS)`
3. 读取一帧 `STATUS`
4. 确认当前状态为 `STANDBY`
5. 发送一次 `PARAM_GET(all)`，确认当前 `KALMAN_ENABLE / KALMAN_Q_MILLI / KALMAN_R_MILLI`
6. 如需调试滤波手感，发送 `PARAM_SET`
7. 解析 `ACK.detail`，确认返回的是实际生效值
8. 发送 `TRACK(valid=1)`
9. 观察是否进入 `TRACKING`
10. 丢目标时发送 `TRACK(valid=0)`
11. 观察是否进入 `HOLD_LAST`
12. 发送 `GO_HOME`
13. 观察是否回到 `STANDBY`

## 9. 当前默认控制参数

轴参数默认值：

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

全局参数默认值：

- `status_period_ms = 100`
- `target_timeout_ms = 250`
- `boot_center_ms = 300`
- `kalman_q_milli = 16000`
- `kalman_r_milli = 64000`

## 10. 调试方法

### 10.1 串口日志

`USART1` 日志中常见的关键输出包括：

- `tim1 servo pwm psc=... arr=...`
- `gimbal boot`
- `gimbal task start`
- `usb cdc init`
- `gimbal state=...`
- `usb rx overflow`
- `ack queue overflow`

### 10.2 联调脚本

工程内提供了联调脚本：

- `tools/serial_integration_test.ps1`

示例命令：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\serial_integration_test.ps1 -DebugPort COM19 -CdcPort COM18
```

说明：

- `COM19` 和 `COM18` 只是示例
- 实际端口号请根据本机枚举结果修改
- 脚本当前会解析 `ACK.detail` 中的参数记录，并验证 `PARAM_SET -> PARAM_GET` 回读
- 默认覆盖 `DEADBAND` 与 `KALMAN_Q_MILLI` 两组在线参数测试

## 11. 文档说明

根目录下的主要文档如下：

- `README.md`
  - 工程总体说明
- `RK3568_STM32_通信协议与IO对照.md`
  - 从工程和硬件角度描述通信协议与 IO
- `RK3568_上位机接口文档.md`
  - 面向 `RK3568` 程序编写的接口文档

## 12. Git 上传说明

根目录下已提供：

- `.ignore`
- `.gitignore`

两者使用相同的忽略规则，主要用于过滤以下无关文件：

- `Keil` 构建产物
- 用户本地调试配置
- 日志文件
- 临时文件
- Python 缓存

如果你要把工程上传到 `GitHub`，建议保留以下内容：

- `Core/`
- `Drivers/`
- `Middlewares/`
- `USB_DEVICE/`
- `tools/`
- `MDK-ARM/2Dpan-tilt.uvprojx`
- `MDK-ARM/RTE/_2Dpan-tilt/RTE_Components.h`
- `2Dpan-tilt.ioc`
- 根目录文档

建议忽略以下内容：

- `MDK-ARM/2Dpan-tilt/` 下的构建输出
- `MDK-ARM/build.log`
- `MDK-ARM/DebugConfig/`
- `*.uvoptx`

## 13. 后续建议

如果后面继续扩展这个工程，推荐优先按以下顺序推进：

1. 完成 `RK3568` 上位机正式程序
2. 做 `HOME_US / MIN_US / MAX_US / INVERT` 的机械标定
3. 完成双舵机跟随与 `Kalman Q/R` 参数整定
4. 评估是否需要在 RK3568 侧加预测补偿或更高阶滤波
5. 最后再考虑 IMU 融合
