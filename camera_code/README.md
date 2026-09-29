# MaixCAM 车辆视觉微调

## K230 二维码串口

在 K230 CanMV 上单独运行 `扫码.py`，与本目录的 MaixCAM `main.py` 分别用于扫码和物料/圆环对准。K230 IO9 配为 UART1_TXD，接 STM32 PB11/USART3_RX；K230 IO10 配为 UART1_RXD，接 STM32 PB10/USART3_TX；两板共地，并核对实际板卡的 IO 电平。串口为 115200、8N1。

K230 发现二维码后发送 `QR:<UTF-8内容>\n`，内容最长 48 字节，换行和控制字符替换为空格；持续识别同一内容时约每秒重发一次。STM32 验证行格式后在蓝牙串口输出 `QR RESULT value=<内容>`，上位机“场地定位”页显示结果。扫码结果目前不决定物料顺序，`route mission` 在二维码点仍至少停车 1 秒，到时按固定顺序继续。

将本目录全部文件部署到MaixCAM，运行 `main.py`。默认使用MaixCAM2 UART4：A21/TX、A22/RX、9600 baud；分别交叉连接STM32 PC11/UART4_RX、PC10/UART4_TX，并共地。若实际板型引脚不同，只修改 `vision_settings.py` 的设备和引脚映射。

程序只响应STM32请求，不主动发送运动命令。每个请求先丢弃2帧，再要求至少5帧、跨度180ms、整个窗口中心最大波动不超过2px。1秒内未得到稳定唯一目标时返回无效结果。

标定步骤：

1. 架空验证车体前后、左右方向，确认急停有效。
2. 分别对物料和三个物理圆环设置不会互相重叠的ROI与目标锚点。
3. 在中心、上下左右和四角至少九个位置记录像素误差及车辆实际毫米偏差。
4. 拟合 `forward_mm=a*du+b*dv`、`right_mm=c*du+d*dv`。
5. 将ROI、锚点和矩阵写入上级目录 `vision_config.h`，最后才把对应 `calibrated` 置1。
6. 先用5mm人工点动验证符号，再从小偏差开始自动闭环测试。

固定30字节帧定义见 `vision_protocol.py`。CRC为CRC-16/CCITT-FALSE，测试向量 `123456789 → 0x29B1`。
