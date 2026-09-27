# MaixCAM 车辆视觉微调

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
