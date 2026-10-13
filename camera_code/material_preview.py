"""Standalone MaixCAM2 preview; production detection with missed-frame tolerance."""
import math
import time

from maix import app, camera, display, err, image, pinmap, pwm

# Snapshot of current production thresholds; update here after recalibration.
COLOR_THRESHOLDS = {
    1: [38, 58, 56, 80, 38, 62],
    2: [73, 93, -32, -8, 71, 95],
    3: [27, 47, 4, 28, -56, -32],
    4: [50, 100, -100, -50, 20, 80],
    5: [17, 37, -12, 12, -11, 13],
    6: [58, 78, -29, -5, -35, -11],
}
MATERIAL_AREA = (1500, 30000)
MIN_QUALITY = {1: 60, 2: 60, 3: 60, 4: 60, 5: 30, 6: 60}


COLOR = 0  # 0=all, 1=red, 2=yellow, 3=blue, 4=green, 5=black, 6=light blue.
COLOR_MASK = 0  # Same as production: 0 allows every color.
ROI = (0, 0, 320, 240)
ANCHOR = (160, 120)
TARGET = 0  # 0=pickup (8px tolerance); 1..3=storage (2px tolerance).
MAX_MISSING_MS = 200  # Reset after this long without a valid center.
COLOR_NAMES = {1: "red", 2: "yellow", 3: "blue", 4: "green",
               5: "black", 6: "light_blue"}


def _unique_near(candidates, anchor, radius):
    selected = [item for item in candidates
                if math.hypot(item[0] - anchor[0], item[1] - anchor[1]) <= radius]
    return selected[0] if len(selected) == 1 else None


def find_material(img, request):
    x, y, width, height, anchor_u, anchor_v, _, _ = request["values"]
    candidates = []
    mask = request["values"][6]
    for color, threshold in COLOR_THRESHOLDS.items():
        if request["selector"] not in (0, color) or \
                (mask and not mask & (1 << (color - 1))):
            continue
        for blob in img.find_blobs([threshold], roi=(x, y, width, height),
                                   area_threshold=50, pixels_threshold=50):
            area = blob.w() * blob.h()
            if not MATERIAL_AREA[0] <= area <= MATERIAL_AREA[1]:
                continue
            if blob.x() <= x or blob.y() <= y or \
                    blob.x() + blob.w() >= x + width or \
                    blob.y() + blob.h() >= y + height:
                continue
            quality = min(100, int(blob.pixels() * 100 / max(area, 1)))
            candidates.append((blob.cx(), blob.cy(), quality, blob.w(), blob.h(), color,
                               blob.x(), blob.y()))
    return _unique_near(candidates, (anchor_u, anchor_v), max(width, height))


class StableWindow:
    def __init__(self, count=5, tolerance=2, minimum_ms=180):
        self.count = count
        self.tolerance = tolerance
        self.minimum_ms = minimum_ms
        self.reset()

    def reset(self):
        self.samples = []
        self.sample_times = []
        self.started_ms = None

    def update(self, point, now_ms):
        if self.sample_times and now_ms - self.sample_times[-1] > MAX_MISSING_MS:
            self.reset()
        if point is None:
            return False
        # Keep the sample just before the window boundary to measure its full span.
        while len(self.sample_times) > 1 and \
                self.sample_times[1] <= now_ms - self.minimum_ms:
            del self.samples[0]
            del self.sample_times[0]
        next_samples = self.samples + [point]
        if self.samples and any(
                max(sample[i] for sample in next_samples) -
                min(sample[i] for sample in next_samples) > self.tolerance
                for i in (0, 1)):
            self.reset()
        if self.samples and len(point) > 5 and point[5] != self.samples[-1][5]:
            self.reset()
        self.samples.append(point)
        self.sample_times.append(now_ms)
        self.started_ms = self.sample_times[0]
        return len(self.samples) >= self.count and now_ms - self.started_ms >= self.minimum_ms

    def center(self):
        centers = []
        for axis in (0, 1):
            values = sorted(sample[axis] for sample in self.samples)
            middle = len(values) // 2
            centers.append(int(round((values[middle] + values[~middle]) / 2)))
        return centers


def main():
    request = {"selector": COLOR, "target": TARGET,
               "values": [*ROI, *ANCHOR, COLOR_MASK, 0]}
    x, y, width, height = ROI
    u, v = ANCHOR
    if not (0 <= COLOR <= 6 and 0 <= COLOR_MASK <= 63 and TARGET in (0, 1, 2, 3)
            and 0 <= x < 320 and 0 <= y < 240 and width > 0 and height > 0
            and x + width <= 320 and y + height <= 240
            and x <= u < x + width and y <= v < y + height):
        raise ValueError("Invalid COLOR/COLOR_MASK/ROI/ANCHOR/TARGET")
    err.check_raise(pinmap.set_pin_function("B25", "PWM6"), "illumination PWM mapping")
    lamp = pwm.PWM(6, freq=100000, duty=0, enable=True)
    cam = None
    try:
        cam = camera.Camera(320, 240)
        screen = display.Display()
        stable = StableWindow(tolerance=8 if TARGET == 0 else 2, minimum_ms=500)
        started = int(time.monotonic() * 1000)
        discard_frames = 2
        last_result = "waiting"
        print("MATERIAL PREVIEW color={} mask={} ROI={} anchor={} thresholds={}".format(
            COLOR, COLOR_MASK, ROI, ANCHOR, COLOR_THRESHOLDS))
        while not app.need_exit():
            now = int(time.monotonic() * 1000)
            frame = cam.read()
            if frame is None:
                stable.update(None, now)
                continue
            found = None
            if discard_frames:
                discard_frames -= 1
            else:
                found = find_material(frame, request)
            confirmed = stable.update(found, now)
            if confirmed or now - started >= 1200:
                if confirmed:
                    center = stable.center() if TARGET == 0 else found[:2]
                    minimum_quality = MIN_QUALITY[found[5]]
                    last_result = "valid {} ({},{}) q={} min={} usable={}".format(
                        COLOR_NAMES[found[5]], center[0], center[1], found[2],
                        minimum_quality, int(found[2] >= minimum_quality))
                else:
                    last_result = "invalid: timeout"
                print("MATERIAL RESULT " + last_result)
                stable.reset()
                started = now
                discard_frames = 2
            # Detect before drawing, as in the production entry point.
            frame.draw_rect(*ROI, image.COLOR_YELLOW, 1)
            frame.draw_cross(*ANCHOR, image.COLOR_RED, 8, 2)
            if found is not None:
                u, v, quality, width, height, color, box_x, box_y = found
                frame.draw_rect(box_x, box_y, width, height, image.COLOR_GREEN, 2)
                frame.draw_circle(u, v, max(width, height) // 2, image.COLOR_GREEN, 2)
                frame.draw_cross(u, v, image.COLOR_GREEN, 8, 2)
                frame.draw_string(2, 2, "{} center=({},{}) q={} min={} stable={}".format(
                    COLOR_NAMES[color], u, v, quality, MIN_QUALITY[color], int(confirmed)),
                    image.COLOR_GREEN)
            else:
                frame.draw_string(2, 2, "No unique material / settling", image.COLOR_YELLOW)
            frame.draw_string(2, 220, last_result, image.COLOR_YELLOW)
            screen.show(frame)
    finally:
        lamp.duty(0)
        if cam is not None:
            cam.close()


if __name__ == "__main__":
    main()
