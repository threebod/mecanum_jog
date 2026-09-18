# 相机协议 v1

115200、8 数据位、无校验、1 停止位，二进制固定 30 字节。主控发一个请求，相机最多返回一个结果；不使用 `print` 文本控制机械手。所有多字节字段小端。

| 偏移 | 长度 | 内容 |
|---|---|---|
| 0 | 2 | A5 5A |
| 2 | 1 | 版本 1 |
| 3 | 1 | 1 请求；2 结果 |
| 4 | 4 | token，请求序号，结果必须原样回显 |
| 8 | 1 | mode：1 取料识别，2 空环，3 抬起确认，4 落位确认 |
| 9 | 1 | color：1 红、2 黄、3 蓝、4 绿、5 黑、6 浅蓝 |
| 10 | 1 | target：物理环号 1—3；不是检测结果排序 |
| 11 | 1 | flags：bit0 有效；bit1 连续帧稳定；自动执行要求两者都为 1 |
| 12 | 16 | 八个有符号 int16 |
| 28 | 2 | CRC16/CCITT-FALSE，init FFFF，poly 1021，refin/refout false，xorout 0，计算字节 0—27 |

请求八个数：ROI 的 x/y/w/h、目标锚点 u/v、0、0。结果八个数：u/v、质量 0—100、目标宽/高、0、0、0。所有图像坐标右正下正、320×240。invalid 结果可附候选坐标用于诊断，但不能驱动动作。

CRC 测试向量：ASCII `123456789` → `29B1`。C/Python 实现通过同一个完整帧交叉核对。

每次换姿态或修正关节后发新 token，相机重新建立稳定窗口、丢弃最初两帧；连续至少 5 帧且跨度至少 180 ms，中心相对首帧波动不超过 2 px。相机约 1000 ms 内返回未确认结果，主控请求超时默认 1800 ms；接收时间按主控本地时钟判断，不混用设备时钟。

三次有效请求仍看不到稳定目标就退出；对准最多 12 次修正，另受累计修正距离限制。只验证当前等待请求的 token/mode/color/target，重复帧不重复推进状态或累计完成数。每个状态总超时 60 秒。

本协议没有掉电持久化任务恢复：任一设备重启后应停止并重新建立基准；主控禁止自动重启任务。诊断 SNAP 使用高位 token，与自动流程序号分离。

状态枚举按 `firmware/transfer.h` 从 0 编号：0 IDLE，1 CLEAR_Z，2 CLEAR_X，3 TRAY，4 TURN_PICK，5 EXTEND_PICK，6 ALIGN_PICK，7 LOWER_PICK，8 CLOSE_GRIP，9 LIFT_PICK，10 VERIFY_HELD，11 RETRACT_PICK，12 TURN_PLACE，13 EXTEND_PLACE，14 ALIGN_PLACE，15 LOWER_PLACE，16 OPEN_GRIP，17 LIFT_PLACE，18 RETRACT_PLACE，19 VERIFY_PLACED，20 NEXT_ITEM，21 FINISH，22 FAILED。
