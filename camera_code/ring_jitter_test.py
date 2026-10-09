"""Run on MaixCAM2: measure ring jitter without UART or motor commands."""
import math
import time

# Restrict this rectangle to ONE complete ring target. Match the vehicle ROI
# when reproducing an alignment failure; coordinates use the 320x240 image.
ROI = (0, 0, 320, 240)
ANCHOR = (160, 120)
WINDOW_MS = 500
WARMUP_MS = 2000
TEST_MS = 10000
LIGHT_LEVELS = (0, 20, 40, 60)  # PWM duty percentage; PWM6 must not drive A30.
CSV_PATH = "ring_jitter.csv"  # Saved in the camera's working directory.


# Snapshot of vehicle ring detection/stability for standalone MaixVision runs.
RING_CENTER_TOLERANCE = 8
RING_MIN_RADIUS = 8
RING_MAX_RADIUS = 150
RING_MIN_DISTINCT_RADII = 2


def _unique_near(candidates, anchor, radius):
    selected = [item for item in candidates
                if math.hypot(item[0] - anchor[0], item[1] - anchor[1]) <= radius]
    return selected[0] if len(selected) == 1 else None


def _cluster_rings(circles):
    groups = []
    for circle in circles:
        for group in groups:
            center_x = sum(item[0] for item in group) / len(group)
            center_y = sum(item[1] for item in group) / len(group)
            if math.hypot(circle[0] - center_x, circle[1] - center_y) < \
                    RING_CENTER_TOLERANCE:
                group.append(circle)
                break
        else:
            groups.append([circle])
    targets = []
    for group in groups:
        radii = []
        for radius in sorted(item[2] for item in group):
            if not radii or radius - radii[-1] > 3:
                radii.append(radius)
        if len(radii) < RING_MIN_DISTINCT_RADII:
            continue
        center_x = int(round(sum(item[0] for item in group) / len(group)))
        center_y = int(round(sum(item[1] for item in group) / len(group)))
        diameter = int(max(radii) * 2)
        targets.append((center_x, center_y, 80, diameter, diameter))
    return targets


def find_ring(img, request, diagnostics=None):
    import cv2
    from maix import image

    if diagnostics is not None:
        diagnostics.clear()
        diagnostics.update(circles=[], targets=[])
    x, y, width, height, anchor_u, anchor_v, _, _ = request["values"]
    gray = image.image2cv(img.to_format(image.Format.FMT_GRAYSCALE), False, False)
    roi = gray[y:y + height, x:x + width]
    if min(roi.shape[:2]) < 23:
        return None
    binary = cv2.adaptiveThreshold(cv2.GaussianBlur(roi, (3, 3), 0), 255,
                                   cv2.ADAPTIVE_THRESH_MEAN_C,
                                   cv2.THRESH_BINARY_INV, 21, 10)
    contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE,
                                            cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is None:
        return None
    circles = []
    for index, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        if area <= 0 or perimeter <= 0 or 4 * math.pi * area / perimeter ** 2 < 0.5:
            continue
        if hierarchy[0][index][2] < 0 and hierarchy[0][index][3] < 0:
            continue
        (center_x, center_y), radius = cv2.minEnclosingCircle(contour)
        if not RING_MIN_RADIUS <= radius <= RING_MAX_RADIUS:
            continue
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            continue
        circles.append((x + moments["m10"] / moments["m00"],
                        y + moments["m01"] / moments["m00"], radius))
    targets = _cluster_rings(circles)
    if diagnostics is not None:
        diagnostics.update(circles=circles, targets=targets)
    return _unique_near(targets, (anchor_u, anchor_v), max(width, height))


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
    from maix import app, camera, display, err, pwm, image, pinmap

    x, y, width, height = ROI
    u, v = ANCHOR
    if not (0 <= x < 320 and 0 <= y < 240 and width >= 23 and height >= 23
            and x + width <= 320 and y + height <= 240
            and x <= u < x + width and y <= v < y + height):
        raise ValueError("ROI/ANCHOR must be inside the 320x240 image")
    request = {"values": [x, y, width, height, u, v, 0, 0]}
    err.check_raise(pinmap.set_pin_function("B25", "PWM6"), "illumination PWM mapping")
    lamp = pwm.PWM(6, freq=100000, duty=0, enable=True)
    cam = None
    log = None
    try:
        cam = camera.Camera(320, 240)
        screen = display.Display()
        log = open(CSV_PATH, "w")
        log.write("light_percent,ms,valid,cx,cy,contours,targets,radii,contour_centers,"
                  "old_stable,window_samples,median_x,median_y,span_x,span_y,std_x,std_y\n")
        print("CSV: {} (overwritten each run); keep camera and ring still".format(CSV_PATH))
        start = int(time.monotonic() * 1000)
        level_index = 0
        level = LIGHT_LEVELS[level_index]
        lamp.duty(level)
        print("LIGHT {}%: settle {}ms, sample {}ms".format(level, WARMUP_MS, TEST_MS))
        stable = StableWindow()
        window = []
        period_points = []
        frames = valid_frames = stable_frames = 0
        while not app.need_exit():
            frame = cam.read()
            now = int(time.monotonic() * 1000)
            if now - start >= WARMUP_MS + TEST_MS:
                overall = statistics(period_points)
                print("RESULT light={}% frames={} valid={} ({:.1f}%) old_stable={} ({:.1f}%)".format(
                    level, frames, valid_frames, valid_frames * 100 / max(frames, 1),
                    stable_frames, stable_frames * 100 / max(frames, 1)))
                if overall:
                    print("median=({:.2f},{:.2f}) span=({:.2f},{:.2f}) std=({:.2f},{:.2f}) px".format(*overall))
                else:
                    print("No valid center at this light level")
                log.flush()
                level_index += 1
                if level_index == len(LIGHT_LEVELS):
                    print("All four light levels finished; CSV: {}".format(CSV_PATH))
                    break
                level = LIGHT_LEVELS[level_index]
                lamp.duty(level)
                start = int(time.monotonic() * 1000)
                stable.reset()
                window = []
                period_points = []
                frames = valid_frames = stable_frames = 0
                print("LIGHT {}%: settle {}ms, sample {}ms".format(level, WARMUP_MS, TEST_MS))
                continue
            if frame is None:
                stable.reset()
                window = []
                continue
            if now - start < WARMUP_MS:
                frame.draw_string(2, 2, "Light {}% warming up...".format(level), image.COLOR_YELLOW)
                screen.show(frame)
                continue
            diagnostics = {}
            found = find_ring(frame, request, diagnostics)
            confirmed = stable.update(found, now)
            frames += 1
            stable_frames += int(confirmed)
            window = [sample for sample in window if now - sample[0] <= WINDOW_MS]
            if found is None:
                window = []
            else:
                valid_frames += 1
                window.append((now, found))
                period_points.append(found)
            stats = statistics([sample[1] for sample in window])
            circles = diagnostics["circles"]
            targets = diagnostics["targets"]
            radii = ";".join("{:.1f}".format(c[2]) for c in circles)
            centers = ";".join("{:.2f}:{:.2f}".format(c[0], c[1]) for c in circles)
            coordinates = "{:.2f},{:.2f}".format(*found[:2]) if found else ","
            stats_csv = ",".join("{:.3f}".format(value) for value in stats) if stats else ",,,,,"
            log.write("{},{},{},{},{},{},{},{},{},{},{}\n".format(
                level, now - start - WARMUP_MS, int(found is not None), coordinates,
                len(circles), len(targets), radii, centers,
                int(confirmed), len(window), stats_csv))
            frame.draw_rect(x, y, width, height, image.COLOR_YELLOW, 1)
            for cx, cy, radius in circles:
                frame.draw_circle(int(cx), int(cy), int(radius), image.COLOR_GREEN, 1)
            if found:
                frame.draw_cross(int(found[0]), int(found[1]), image.COLOR_RED, 8, 2)
                frame.draw_cross(int(round(stats[0])), int(round(stats[1])), image.COLOR_YELLOW, 5, 1)
            frame.draw_string(2, 2, "valid={} old_stable={} contours={} targets={}".format(
                int(found is not None), int(confirmed), len(circles), len(targets)), image.COLOR_YELLOW)
            if stats:
                frame.draw_string(2, 20, "500ms span={:.1f}/{:.1f}px n={}".format(
                    stats[2], stats[3], len(window)), image.COLOR_YELLOW)
            frame.draw_string(2, 38, "Light {}% sample {:.1f}/{}s".format(
                level, (now - start - WARMUP_MS) / 1000, TEST_MS // 1000), image.COLOR_YELLOW)
            screen.show(frame)
    finally:
        lamp.duty(0)
        if log is not None:
            log.close()
        if cam is not None:
            cam.close()


if __name__ == "__main__":
    main()
