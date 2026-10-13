"""
MaixCAM2 色块识别 + LAB 阈值标定工具
在 MaixVision 中运行。

功能:
  1. 画面中央有一个采样框, 实时统计框内像素的 LAB 值,
     在串口输出"建议阈值范围"(均值 ± 余量), 可直接填到 THRESHOLDS 使用。
  2. 用 THRESHOLDS 里配置好的阈值运行 find_blobs,
     实时框出色块、标出中心点, 验证阈值是否合适。

用法:
  1) 把要识别的颜色对准中央方框, 保持稳定;
  2) 看串口打印的 建议阈值: (L_min, L_max, A_min, A_max, B_min, B_max);
  3) 复制到下方 THRESHOLDS 列表, 重新运行即可看到识别框。
"""
from maix import camera, display, image, app, time, pinmap, gpio

# ==================== 可调参数 ====================

# --- 摄像头 ---
CAM_W = 320
CAM_H = 240
WARMUP_FRAMES = 20       # 上电先丢弃的帧数, 等自动曝光/白平衡稳定再采样

# --- 板载补光灯 (MaixCAM2: B25, 高电平亮; MaixCAM-Pro 用 B3) ---
FILL_LIGHT = False        # 是否打开补光灯
LIGHT_PIN = "B25"        # 引脚名
LIGHT_GPIO = "GPIOB25"   # GPIO 功能名

# --- 采样框(用于标定阈值) ---
SAMPLE_SIZE = 50         # 中央采样框边长(像素), 越小采样越"纯"
MARGIN_L = 10            # 生成建议阈值时 L 均值两侧的余量
MARGIN_A = 12            # A 余量
MARGIN_B = 12            # B 余量
# 提示: 环境光变化大时把余量调大; 颜色相近难区分时调小

# --- 色块识别阈值 (L_min, L_max, A_min, A_max, B_min, B_max) ---
# MaixPy LAB 范围: L: 0~100, A/B: -128~127
# 把串口输出的"建议阈值"复制到这里, 可同时放多组(多种颜色)
THRESHOLDS = [
    # 示例(红色): (20, 80, 20, 127, 0, 127),
]

# --- find_blobs 参数 ---
PIXELS_THRESHOLD = 60    # 色块最少像素数, 滤噪点
AREA_THRESHOLD = 60      # 色块最小外接框面积
MERGE = True             # 合并相邻色块, 减少碎片化
MERGE_MARGIN = 10        # 合并时允许的最大间隙(像素)

# --- 打印节流 ---
PRINT_INTERVAL = 0.5     # 建议阈值打印间隔(秒), 避免刷屏

# ==================== 主程序 ====================

def clamp(v, lo, hi):
    return max(lo, min(hi, v))


cam = camera.Camera(CAM_W, CAM_H)
disp = display.Display()

# 打开补光灯(在预热之前, 让自动曝光/白平衡适应开灯后的亮度)
if FILL_LIGHT:
    pinmap.set_pin_function(LIGHT_PIN, LIGHT_GPIO)
    fill_led = gpio.GPIO(LIGHT_GPIO, gpio.Mode.OUT)
    fill_led.value(1)    # 高电平点亮, 0 熄灭

# 预热: 丢弃若干帧, 让自动曝光/自动白平衡收敛
for _ in range(WARMUP_FRAMES):
    cam.read()

# 如需锁定曝光/白平衡防止阈值漂移, 可取消下面注释(不同固件接口可能有差异, 失败会忽略)
# try:
#     cam.awb_mode(1)        # 关闭自动白平衡
#     cam.exposure(value=1000)  # 固定曝光值, 数值按现场亮度调
# except Exception as e:
#     print("锁曝光/白平衡失败(可忽略):", e)

# 采样框位置(画面中心)
sx = (CAM_W - SAMPLE_SIZE) // 2
sy = (CAM_H - SAMPLE_SIZE) // 2
sample_roi = (sx, sy, SAMPLE_SIZE, SAMPLE_SIZE)

last_print = time.ticks_ms()

while not app.need_exit():
    img = cam.read()

    # ---- 1. 采样框统计, 输出建议阈值 ----
    st = img.get_statistics(roi=sample_roi)
    now = time.ticks_ms()
    if now - last_print > PRINT_INTERVAL * 1000:
        l_min = clamp(int(st.l_mean() - MARGIN_L), 0, 100)
        l_max = clamp(int(st.l_mean() + MARGIN_L), 0, 100)
        a_min = clamp(int(st.a_mean() - MARGIN_A), -128, 127)
        a_max = clamp(int(st.a_mean() + MARGIN_A), -128, 127)
        b_min = clamp(int(st.b_mean() - MARGIN_B), -128, 127)
        b_max = clamp(int(st.b_mean() + MARGIN_B), -128, 127)
        print("LAB均值: L={:.1f} A={:.1f} B={:.1f}".format(
            st.l_mean(), st.a_mean(), st.b_mean()))
        print("建议阈值: ({}, {}, {}, {}, {}, {})".format(
            l_min, l_max, a_min, a_max, b_min, b_max))
        last_print = now

    img.draw_rect(sx, sy, SAMPLE_SIZE, SAMPLE_SIZE, image.COLOR_WHITE, 1)
    img.draw_string(sx, sy - 14, "sample", image.COLOR_WHITE, 1.0)

    # ---- 2. 色块识别 ----
    if THRESHOLDS:
        blobs = img.find_blobs(
            THRESHOLDS,
            pixels_threshold=PIXELS_THRESHOLD,
            area_threshold=AREA_THRESHOLD,
            merge=MERGE, margin=MERGE_MARGIN,
        )
        for b in blobs:
            img.draw_circle(b.cx(), b.cy(), max(b.w(), b.h()) // 2,
                            image.COLOR_GREEN, 2)
            img.draw_cross(b.cx(), b.cy(), image.COLOR_RED, 6, 2)
            img.draw_string(b.x(), b.y() - 14,
                            "({},{}) px={}".format(b.cx(), b.cy(), b.pixels()),
                            image.COLOR_YELLOW, 1.0)
        img.draw_string(4, 4, "blobs: {}".format(len(blobs)),
                        image.COLOR_YELLOW, 1.0)
    else:
        img.draw_string(4, 4, "THRESHOLDS is empty, sampling only",
                        image.COLOR_YELLOW, 1.0)

    disp.show(img)
