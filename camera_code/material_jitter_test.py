"""Standalone MaixCAM2 material diagnostic; lamp off, no UART commands."""
import math
import time

COLOR = 5  # 5=black, 6=light blue. Test one color at a time.
ROI = (0, 0, 320, 240)  # Match the vehicle request; include one whole material.
ANCHOR = (160, 120)
COLOR_THRESHOLDS = {5: [0, 20, -13, 11, -11, 13],
                    6: [33, 75, -20, 5, -32, -8]}
MATERIAL_AREA = (1500, 30000)
WARMUP_MS = 2000
TEST_MS = 10000


def detect_material(img):
    x, y, width, height = ROI
    candidates, records = [], []
    for blob in img.find_blobs([COLOR_THRESHOLDS[COLOR]], roi=ROI,
                               area_threshold=50, pixels_threshold=50):
        area = blob.w() * blob.h()
        quality = min(100, int(blob.pixels() * 100 / max(area, 1)))
        if not MATERIAL_AREA[0] <= area <= MATERIAL_AREA[1]:
            reason = "area"
        elif (blob.x() <= x or blob.y() <= y or
              blob.x() + blob.w() >= x + width or
              blob.y() + blob.h() >= y + height):
            reason = "clipped"
        elif math.hypot(blob.cx() - ANCHOR[0], blob.cy() - ANCHOR[1]) > max(width, height):
            reason = "outside"
        else:
            reason = "candidate"
            candidates.append((blob.cx(), blob.cy(), quality, blob.w(), blob.h()))
        records.append((blob, quality, reason))
    return (candidates[0] if len(candidates) == 1 else None), records


class StableWindow:
    def __init__(self, count=5, tolerance=2, minimum_ms=180):
        self.count = count
        self.tolerance = tolerance
        self.minimum_ms = minimum_ms
        self.reset()

    def reset(self):
        self.samples = []
        self.started_ms = None

    def update(self, point, now_ms):
        if point is None:
            self.reset()
            return False
        next_samples = self.samples + [point]
        if self.samples and any(
                max(sample[i] for sample in next_samples) -
                min(sample[i] for sample in next_samples) > self.tolerance
                for i in (0, 1)):
            self.reset()
        if not self.samples:
            self.started_ms = now_ms
        self.samples.append(point)
        return len(self.samples) >= self.count and now_ms - self.started_ms >= self.minimum_ms


def median(values):
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return (ordered[middle - 1] + ordered[middle]) / 2


def statistics(points):
    if not points:
        return None
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    cx, cy = median(xs), median(ys)
    mean_x, mean_y = sum(xs) / len(xs), sum(ys) / len(ys)
    sx = math.sqrt(sum((x - mean_x) ** 2 for x in xs) / len(xs))
    sy = math.sqrt(sum((y - mean_y) ** 2 for y in ys) / len(ys))
    return cx, cy, max(xs) - min(xs), max(ys) - min(ys), sx, sy

def main():
    from maix import app, camera, display, err, image, pinmap, pwm

    x, y, width, height = ROI
    u, v = ANCHOR
    if COLOR not in COLOR_THRESHOLDS or not (
            0 <= x < 320 and 0 <= y < 240 and width > 0 and height > 0 and
            x + width <= 320 and y + height <= 240 and
            x <= u < x + width and y <= v < y + height):
        raise ValueError("Invalid COLOR/ROI/ANCHOR")
    # Sampling box must lie entirely on the material, away from glare/background.
    sample_roi = (max(x, min(u - 10, x + width - 20)),
                  max(y, min(v - 10, y + height - 20)), min(20, width), min(20, height))
    err.check_raise(pinmap.set_pin_function("B25", "PWM6"), "illumination mapping")
    lamp = pwm.PWM(6, freq=100000, duty=0, enable=True)
    cam = None
    try:
        cam = camera.Camera(320, 240)
        screen = display.Display()
        stable = StableWindow(minimum_ms=500)
        start = int(time.monotonic() * 1000)
        points, labs = [], []
        frames = valid = stable_frames = usable = 0
        failures = {"none": 0, "ambiguous": 0, "low_quality": 0}
        rejections = {"area": 0, "clipped": 0, "outside": 0}
        print("MATERIAL color={} lamp=OFF threshold={} ROI={} anchor={}".format(
            COLOR, COLOR_THRESHOLDS[COLOR], ROI, ANCHOR))
        print("Keep material still; white sample box must contain only material")
        while not app.need_exit():
            frame = cam.read()
            now = int(time.monotonic() * 1000)
            if now - start >= WARMUP_MS + TEST_MS:
                print("RESULT frames={} valid={} ({:.1f}%) stable={} usable={} ({:.1f}%)".format(
                    frames, valid, valid * 100 / max(frames, 1), stable_frames,
                    usable, usable * 100 / max(frames, 1)))
                print("frame failures={} rejected blob counts={}".format(failures, rejections))
                if points:
                    print("center median=({:.2f},{:.2f}) span=({:.2f},{:.2f}) std=({:.2f},{:.2f}) px".format(
                        *statistics(points)))
                    print("quality min={} median={:.1f}; STM32 requires >=60".format(
                        min(p[2] for p in points), median([p[2] for p in points])))
                if labs:
                    print("sample LAB median=({:.1f},{:.1f},{:.1f})".format(
                        *(median([sample[i] for sample in labs]) for i in range(3))))
                break
            if frame is None:
                stable.reset()
                continue
            if now - start < WARMUP_MS:
                frame.draw_string(2, 2, "Warming up, lamp OFF", image.COLOR_YELLOW)
                screen.show(frame)
                continue
            found, records = detect_material(frame)
            # Read LAB before adding any drawing to the image.
            lab = frame.get_statistics(roi=sample_roi)
            labs.append((lab.l_mean(), lab.a_mean(), lab.b_mean()))
            confirmed = stable.update(found, now)
            frames += 1
            stable_frames += int(confirmed)
            candidates = sum(reason == "candidate" for _, _, reason in records)
            if found:
                valid += 1
                points.append(found)
                failures["low_quality"] += int(found[2] < 60)
                usable += int(confirmed and found[2] >= 60)
            else:
                failures["ambiguous" if candidates > 1 else "none"] += 1
            for blob, quality, reason in records:
                if reason in rejections:
                    rejections[reason] += 1
                color = image.COLOR_GREEN if reason == "candidate" else image.COLOR_RED
                frame.draw_rect(blob.x(), blob.y(), blob.w(), blob.h(), color, 1)
                frame.draw_cross(blob.cx(), blob.cy(), color, 5, 1)
                frame.draw_string(blob.x(), max(0, blob.y() - 12),
                                  "{} q={}".format(reason, quality), color)
            if found:
                frame.draw_cross(found[0], found[1], image.COLOR_YELLOW, 8, 2)
            frame.draw_rect(*sample_roi, image.COLOR_WHITE, 1)
            frame.draw_string(2, 2, "color={} candidates={} stable={} {:.1f}/10s".format(
                COLOR, candidates, int(confirmed), (now - start - WARMUP_MS) / 1000), image.COLOR_YELLOW)
            screen.show(frame)
    finally:
        lamp.duty(0)
        if cam is not None:
            cam.close()


if __name__ == "__main__":
    main()
