from maix import camera, display, image, app

# 六种颜色的 LAB 阈值，格式 [L_min, L_max, A_min, A_max, B_min, B_max]
COLOR_THRESHOLDS = {
    "red":        [30, 80,  55, 105, 25, 80],   # 红色：A 轴正向
    "yellow":     [50, 105, -30,  25,   35, 100],   # 黄色：高亮度 + B 轴正向
    "green":      [50, 100, -100, -50,    20, 80],   # 绿色：A 轴负向
    "light_blue": [33, 75, -20, 5, -32, -8],   # 浅蓝：高亮度 + A、B 略负
    "blue":       [0, 40, 8, 35, -60, -30],  # 深蓝：B 轴强负向
    "black":      [ 0, 20, -13, 11, -11, 13],  # 黑色：低亮度
}

# 每种颜色画框用的显示颜色
DRAW_COLOR = {
    "red":        image.COLOR_RED,
    "yellow":     image.COLOR_YELLOW,
    "green":      image.COLOR_GREEN,
    "light_blue": image.Color.from_rgb(0, 255, 255),  # 浅蓝(青色)用 RGB 自定义
    "blue":       image.COLOR_BLUE,
    "black":      image.COLOR_WHITE,   # 黑色物块用白框标出
}

# 面积限幅：只保留"外接矩形面积"在 [min_area, max_area] 内的色块
AREA_LIMIT = {
    "red":        (1500, 30000),
    "yellow":     (1500, 30000),
    "green":      (1500, 30000),
    "light_blue": (1500, 30000),
    "blue":       (1500, 30000),
    "black":      (1500, 30000),
}

CAM_W, CAM_H = 320, 240                       # 分辨率常量
SCREEN_CX, SCREEN_CY = CAM_W // 2, CAM_H // 2  # 屏幕中心(原点) = (160, 120)

cam = camera.Camera(CAM_W, CAM_H)
disp = display.Display()

while not app.need_exit():
    img = cam.read()
    if img is None:
        continue

    # 屏幕中心画个十字，标出原点
    img.draw_cross(SCREEN_CX, SCREEN_CY, image.COLOR_WHITE, 10, 1)

    for name, thr in COLOR_THRESHOLDS.items():
        blobs = img.find_blobs([thr], area_threshold=50, pixels_threshold=50)
        if not blobs:
            continue
        b = max(blobs, key=lambda b: b.area())   # 取该颜色里面积最大的色块

        # 面积限幅判断（相当于原来的 size_code）
        min_area, max_area = AREA_LIMIT[name]
        area = b.area()
        if area < min_area or (max_area > 0 and area > max_area):
            continue

        cx, cy = b.cx(), b.cy()                  # 中心点坐标

        # 中心点相对屏幕中心的位置：右为正 x，上为正 y
        rel_x = cx - SCREEN_CX
        rel_y = SCREEN_CY - cy

        img.draw_rect(b.x(), b.y(), b.w(), b.h(), DRAW_COLOR[name], 2)
        img.draw_circle(cx, cy, 3, DRAW_COLOR[name], -1)
        # 只显示相对坐标
        img.draw_string(b.x(), max(0, b.y() - 18),
                        f"{name} rel({rel_x},{rel_y})", DRAW_COLOR[name])
        print(f"{name}: rel=({rel_x}, {rel_y})")

    disp.show(img)