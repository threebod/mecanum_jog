'''
    MaixCAM2 圆环中心识别
    流程：灰度 -> 自适应二值化 -> 查找轮廓 -> 同心轮廓聚类 -> 圆环中心。
    控制云台可以基于 err_centers（圆心与画面中心的误差）进行 PID 控制。
'''

from maix import camera, display, image, app, time, gpio, pinmap, err
import cv2
import numpy as np
import atexit


class FindRingCenter:
    DEBUG = False                   # 显示二值化中间调试图
    PRINT_TIME = False              # 打印每一步消耗的时间
    PRINT_RESULT = True             # 串口打印识别到的圆心坐标

    debug_draw_circle = True        # 画出检出的同心圆轮廓
    debug_draw_err_line = True      # 画面中心到圆心的误差线
    debug_draw_err_msg = True       # 显示误差值和 FPS

    ############################# config #############################

    cam_res = [320, 240]            # 摄像头分辨率，越小越快
    contrast = 80                   # 对比度，影响二值化效果
    auto_awb = True                 # 自动白平衡，False 时用下面的手动值
    awb_gain = [0.134, 0.0625, 0.0625, 0.1139]  # 手动白平衡 R GR GB B

    # 自适应二值化参数，光照不均时调大 block，线条太淡时调小 c
    binary_block = 21               # 邻域大小（奇数）
    binary_c = 10                   # 阈值偏移量

    # 有效轮廓筛选（按外接圆半径和圆度过滤）
    min_radius = 50                  # 最小半径(像素)，小于图中最内圈半径
    max_radius = 160                # 最大半径(像素)，大于图中最外圈半径
    min_circularity = 0.5           # 圆度 4*pi*A/P^2，斜视透视变形时调低

    # 同心聚类：一个圆环靶标由多个同心轮廓组成（每个墨环有内外两条边）
    center_dist = 8                 # 圆心距离小于该值视为同心
    min_rings = 2                   # 至少几个同心轮廓才算有效圆环
    target_count = 3                # 期望的靶标个数，多余的按强度丢弃

    ###################################################################

    def __init__(self, disp):
        # 打开 MaixCAM2 板载补光灯（照明 LED 接在 B25，高电平点亮）
        # 其它板型：MaixCAM-Pro 为 B3 / GPIOB3
        err.check_raise(pinmap.set_pin_function("B25", "GPIOB25"),
                        "set illumination pin failed")
        self.illuminator = gpio.GPIO("GPIOB25", gpio.Mode.OUT)
        self.illuminator.value(1)
        atexit.register(self.illumination_off)   # 程序正常退出时自动关灯

        self.disp = disp
        self.cam = camera.Camera(self.cam_res[0], self.cam_res[1])
        if not self.auto_awb:
            self.cam.awb_mode(camera.AwbMode.Manual)
            self.cam.set_wb_gain(self.awb_gain)
        self.cam.constrast(self.contrast)

        self._t = time.ticks_ms()
        self.center_pos = [self.cam.width() // 2, self.cam.height() // 2]  # 画面中心
        self.targets = []           # 圆环中心列表 [[x, y, r], ...]，按 x 从左到右排序
        self.err_centers = []       # 各圆心相对画面中心的误差 [[dx, dy], ...]
        self.updated = False        # 本帧是否识别到圆环

    def illumination_off(self):
        '''关闭补光灯，可重复调用'''
        led = getattr(self, "illuminator", None)
        if led is not None:
            led.value(0)

    def debug_time(self, msg):
        if self.PRINT_TIME:
            print("t: {:4d} {}".format(time.ticks_ms() - self._t, msg))
            self._t = time.ticks_ms()

    def cluster_circles(self, circles):
        '''把圆心距离近的轮廓聚成一组（同心圆环），返回有效靶标列表'''
        groups = []  # [x_sum, y_sum, count, max_r]
        for cx, cy, r in circles:
            for g in groups:
                gx, gy = g[0] / g[2], g[1] / g[2]
                if abs(cx - gx) < self.center_dist and abs(cy - gy) < self.center_dist:
                    g[0] += cx; g[1] += cy; g[2] += 1
                    if r > g[3]:
                        g[3] = r
                    break
            else:
                groups.append([cx, cy, 1, r])
        # 同心轮廓数量不足的视为误检，按强度排序后取前 target_count 个
        targets = [g for g in groups if g[2] >= self.min_rings]
        targets.sort(key=lambda g: g[2], reverse=True)
        targets = targets[:self.target_count]
        targets.sort(key=lambda g: g[0])    # 按 x 从左到右，对应靶 1/2/3
        return [[g[0] / g[2], g[1] / g[2], g[3]] for g in targets]

    def run(self):
        '''
            Return: [圆环中心列表[[x,y,r],...], 画面中心坐标, 误差列表[[dx,dy],...], 本帧是否更新]
        '''
        self.updated = False
        self.debug_time("start")
        img = self.cam.read()
        self.debug_time("cam read")

        gray = img.to_format(image.Format.FMT_GRAYSCALE)
        gray_cv = image.image2cv(gray, False, False)
        gray_cv = cv2.GaussianBlur(gray_cv, (3, 3), 0)

        # 自适应二值化，黑色圆环线条为白
        binary = cv2.adaptiveThreshold(gray_cv, 255,
                    cv2.ADAPTIVE_THRESH_MEAN_C,
                    cv2.THRESH_BINARY_INV, self.binary_block, self.binary_c)
        self.debug_time("binary")

        contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        circles = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < 3.14 * self.min_radius * self.min_radius * 0.5:
                continue
            (cx, cy), r = cv2.minEnclosingCircle(c)
            if r < self.min_radius or r > self.max_radius:
                continue
            perimeter = cv2.arcLength(c, True)
            if perimeter <= 0:
                continue
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            if circularity < self.min_circularity:
                continue
            # 用矩求轮廓中心，比外接圆圆心更稳
            m = cv2.moments(c)
            if m['m00'] > 0:
                cx = m['m10'] / m['m00']
                cy = m['m01'] / m['m00']
            circles.append((cx, cy, r))
        self.debug_time("find contours")

        self.targets = self.cluster_circles(circles)
        self.err_centers = []
        self.debug_time("cluster")

        # 画出所有参与聚类的圆
        if self.debug_draw_circle:
            for cx, cy, r in circles:
                img.draw_circle(int(cx), int(cy), int(r), image.COLOR_GREEN, 1)

        # 画画面中心
        img.draw_cross(self.center_pos[0], self.center_pos[1], image.COLOR_GREEN, 8, 2)

        for i, t in enumerate(self.targets):
            cx, cy, r = int(t[0]), int(t[1]), int(t[2])
            err = [t[0] - self.center_pos[0], t[1] - self.center_pos[1]]
            self.err_centers.append(err)
            self.updated = True
            img.draw_cross(cx, cy, image.COLOR_RED, 6, 2)
            img.draw_circle(cx, cy, 3, image.COLOR_RED, -1)
            img.draw_circle(cx, cy, r, image.COLOR_RED, 2)
            img.draw_string(cx + 8, cy - 20, "#{} ({},{})".format(i + 1, cx, cy),
                            image.COLOR_RED, 1.0, 2)
            if self.debug_draw_err_line:
                img.draw_line(self.center_pos[0], self.center_pos[1], cx, cy,
                              image.COLOR_RED, 2)
            if self.PRINT_RESULT:
                print("ring#{} center=({}, {}) err=({}, {}) r={}".format(
                    i + 1, cx, cy, err[0], err[1], r))
        self.debug_time("draw")

        if self.debug_draw_err_msg:
            msg = "targets: {}/{} fps: {:2.0f}".format(len(self.targets), self.target_count, time.fps())
            img.draw_string(2, img.height() - 20, msg, image.COLOR_YELLOW, 1.0, 2)

        if self.DEBUG:
            # 左下角显示二值化调试图
            img_bin = image.cv2image(binary, False, False)
            img.draw_image(0, img.height() - img_bin.height(), img_bin)

        self.disp.show(img)
        self.debug_time("display")
        return [self.targets, self.center_pos, self.err_centers, self.updated]


if __name__ == "__main__":
    disp = display.Display()
    finder = FindRingCenter(disp)
    try:
        while not app.need_exit():
            targets, screen_center, err_centers, updated = finder.run()
    finally:
        finder.illumination_off()   # 程序结束（含异常/Ctrl+C）时关闭补光灯
