# 单摄像头静止搬运工程 v1

独立 STM32F407ZG 工程：车不动，依次从三槽料盘取出物料，放到指定的三个空圆环。使用顶部 MaixCAM2，同一摄像头切换物料检测、圆环定位、抬起检查和落位检查。没有底盘运动、二维码扫描、原料转盘抓取或第二层码垛代码。

**交付状态：ARM GCC 已构建 ELF/HEX/BIN；C 状态机与 Python 协议/检测辅助逻辑已做离线测试。未烧录、未在 MaixCAM2 或实车上运行。默认禁止自动搬运，必须标定后启用。** 不把编译通过等同于实车成功。

## 文件入口

| 文件 | 用途 |
|---|---|
| `StationaryTransfer.uvprojx` | Keil 工程，基于原 STM32F407ZG/ARMCC5 项目设置生成，未在 Keil 实编译 |
| `tools/build.py` | 已验证的 ARM GNU 编译入口；不依赖旧工程绝对路径 |
| `build/stationary_transfer.hex`、`.bin` | 默认关闭自动模式的固件；Flash 起始地址 0x08000000 |
| `firmware/main.c` | 初始化、调试命令、主循环 |
| `firmware/transfer.c` | 独立非阻塞搬运状态机 |
| `firmware/platform.c` | UART4、USART1、CAN、电机、舵机、软件位置与停止 |
| `firmware/config.c` | 必须实测的机械、姿态、ROI、视觉修正参数 |
| `camera/main.py` | MaixVision 项目入口；打开并运行整个 camera 目录 |
| `camera/顶部搬运摄像头.py` | MaixCAM2 主程序实现 |
| `camera/标定预览.py` | 独立观察色块中心和宽高，不发送执行器动作 |
| `camera/settings.py` | 六色 LAB、尺寸范围、识别阈值与标定开关 |
| `CALIBRATION.md` | 按顺序完成的标定与首轮运行步骤 |
| `PROTOCOL.md` | 两端完整通信协议和状态含义 |
| `tools/test.py` | 可重复离线验证 |

原主控工程、原两份摄像头程序均保留。新 camera 程序重用原六色阈值及圆环检测思路，重写为带通信和状态检查的独立版本；不用覆盖原文件才能运行。

## 接线与已有硬件

| 主控 | 连接/功能 |
|---|---|
| PC10 / UART4_TX | MaixCAM2 A22 / UART4_RX |
| PC11 / UART4_RX | MaixCAM2 A21 / UART4_TX |
| GND | MaixCAM2 共地；逻辑电平和供电按设备实际规格连接 |
| PA1 / TIM2_CH2 | 夹爪，指令角度 80°张开、30°夹紧 |
| PA2 / TIM2_CH3 | 料盘舵机 |
| PA3 / TIM2_CH4 | 旋转舵机 |
| PA11 RX / PA12 TX | 沿用原工程 CAN1，500 kbit/s，接既有 CAN 收发电路 |
| 电机地址 5 / 6 | 升降 / 平推；程序明确拒绝发送其它电机地址 |
| PA9 TX / PA10 RX | 新调试控制台 USART1，115200、8N1，外接 USB-TTL |
| PC9 | 沿用旧工程按钮引脚，低电平持续 30 ms 为软件停止；应核对实际按钮接线 |

相机 UART4 固定 `/dev/ttyS4`，A21/A22 映射按 Sipeed 官方文档实现：[UART 使用与 MaixCAM2 映射](https://github.com/sipeed/MaixPy/blob/main/docs/doc/en/peripheral/uart.md)，[UART API](https://wiki.sipeed.com/maixpy/api/maix/peripheral/uart.html)。实际固件仍需验证 MaixPy 版本和 OpenCV 可用性。

主控沿用原工程 8 MHz 外部晶振、168 MHz 系统时钟。上电检查系统时钟；CAN 沿用原工程 Emm_V5 指令格式，新增分包发送结果与超时检查。CAN 发送成功只代表帧已发出，不证明电机已到位。

## 实际执行流程

人工摆到机械基准并执行 ZERO → 输入颜色/目标环计划 → 抬到安全高度 → 平推收回 → 料盘槽 1 转到取料位置 → 转到取料观察角 → 张爪/伸出 → 视觉对准 → 下降/夹紧/抬起 → 视觉检查抬起物料 → 收回/转到目标环观察角/伸出 → 空环视觉修正 → 下降/松开/抬升/收回 → 视觉检查落位 → 槽 2、槽 3 重复。

任何未确认、超时、超出软件范围、CAN 错误、串口溢出或 STOP 都进入 FAILED。保持夹爪最后指令，不故障自动张爪；发送 5、6 号停止指令并使软件位置失效。恢复需人工核对现场并重新 ZERO，不自动续做，不重放抓取。

程序不调用电机硬件回零，也不会让电机撞到机械端点找零。舵机通过位置 PWM 给定角度，**没有臆造舵机厂商的回零协议**。用户所述回零能力须体现为可靠的工作角度映射；若实际为另一种控制方式，应先修改驱动。

## 控制台命令

命令末尾发送换行；先设置串口为 115200、8N1。命令中的位置是当前配置坐标，不是从照片推测的尺寸。

| 命令 | 含义 |
|---|---|
| `STATUS` | 状态编号、当前项、已确认放置数、持料状态、软件基准、故障和指令位置 |
| `ZERO x z theta tray grip` | 声明已经人工摆到这些实际位置，并从此建立两轴软件基准；同时输出给定舵机 PWM 并使能电机。舵机必须已处于对应姿态，否则会动 |
| `AXIS 0 value` | 平推到绝对软件位置，单次最多改变 2 mm |
| `AXIS 1 value` | 升降到绝对软件位置，单次最多改变 2 mm；本工程 Z 正向定义为下降，需标定驱动方向 |
| `SERVO 0 angle` | 夹爪，单次最多改变 5° |
| `SERVO 1 angle` | 料盘，单次最多改变 5° |
| `SERVO 2 angle` | 旋转，单次最多改变 5° |
| `SNAP mode color ring` | 静止时请求一次视觉结果并显示 u/v/质量/宽高；模式 1 物料、2 空环、3 已抬起、4 已放置 |
| `START c1 r1 c2 r2 c3 r3` | 三个料槽分别装入 c1/c2/c3 颜色，按顺序放到 r1/r2/r3 环；仅校准和基准均有效时接受 |
| `STOP` | 软件停止，作业失效，重新核对并 ZERO 后才可再开始 |

例如 `START 1 1 5 2 6 3` 表示槽 1 红到环 1、槽 2 黑到环 2、槽 3 浅蓝到环 3。颜色 ID：1 红、2 黄、3 蓝、4 绿、5 黑、6 浅蓝。此次不解析四组完整赛题任务码。

手动 AXIS/SERVO 是标定工具，只做单次位移和行程检查，不代替操作者检查机构干涉。自动模式才保证先抬升、再收回、再旋转。运行中只接收 STOP/STATUS；完成后也须确认库存并重新 ZERO 才能新开一轮。STOP 是软件停止，不能在 MCU 死机、断电或 CAN 故障时替代硬件急停。

## 构建与测试

安装 Python 3 和 ARM GNU 工具链后，在本目录执行：

```powershell
python tools/build.py
python tools/test.py
```

若编译器不在 PATH：

```powershell
python tools/build.py --cc "C:\Program Files\DevEnv\DevEnv\GNU-tools-for-STM32\bin\arm-none-eabi-gcc.exe"
```

测试另需主机 GCC（如 MinGW）。Keil 用户打开 `.uvprojx`，安装原工程使用的 STM32F4 DFP 与 ARMCC5，构建输出到 `build/keil`；GCC 与 Keil 分别使用自己的启动汇编，不能把两份启动文件一起编译。

依赖在 vendor 中随工程附带，保留原文件许可与作者说明。工程不需要电机 1—4、旧底盘、扫码、IMU 或原 main.c。

## 当前边界

- 固定停车、平面单层、三种不同颜色、三个空环。不是完整比赛两批码垛程序。
- 两轴软件位置和等待时间是估计，不是编码器或限位反馈。要通过单轴实测保证时序有余量，并做视觉抓放验证。
- 视觉确认基于颜色、位置、尺寸及连续稳定帧，无法证明夹持力或物体所有三维姿态；如果顶视下“没抓起来”和“已抓起来”不可区分，不能打开自动抓取确认，应先改观察姿态或反馈方案。
- 圆环 ID 来自标定后的 `place[ring_id-1]` 与 ROI；不做 OCR，不把左右排序当编号。停车偏移必须在该 ROI 的有效范围内，不能保证任意停车位置都能识别身份。
- 对旋转随动相机采用各姿态局部标定，视觉修正有单步和累计上限；不把同一像素换算用于所有角度。
