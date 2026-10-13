import math

import vision_settings as settings
from vision_protocol import FLAG_RING_NO_DIGIT


def _unique_near(candidates, anchor, radius):
    selected = [item for item in candidates
                if math.hypot(item[0] - anchor[0], item[1] - anchor[1]) <= radius]
    return selected[0] if len(selected) == 1 else None


def find_material(img, request, scale=1):
    x, y, width, height = request["values"][:4]
    # Pickup uses the top 3/4 of the 320x240 protocol image to exclude the gripper.
    if request.get("target", 0) == 0:
        height = min(y + height, 180) - y
        if height <= 0:
            return None
    candidates = []
    mask = request["values"][6]
    for color, threshold in settings.COLOR_THRESHOLDS.items():
        if request["selector"] not in (0, color) or \
                (mask and not mask & (1 << (color - 1))):
            continue
        blobs = img.find_blobs([threshold],
                               roi=tuple(int(value * scale) for value in (x, y, width, height)),
                               area_threshold=50, pixels_threshold=50)
        if not blobs:
            continue
        blob = max(blobs, key=lambda blob: blob.area())
        area = blob.area()
        if not settings.MATERIAL_AREA[0] * scale ** 2 <= area <= \
                settings.MATERIAL_AREA[1] * scale ** 2:
            continue
        if max(blob.w(), blob.h()) / min(blob.w(), blob.h()) > 1.4:
            continue
        quality = min(100, int(blob.pixels() * 100 / max(area, 1)))
        candidates.append((area, (int(round(blob.cx() / scale)), int(round(blob.cy() / scale)),
                                  quality, int(round(blob.w() / scale)),
                                  int(round(blob.h() / scale)), color)))
    return max(candidates, key=lambda item: item[0])[1] if candidates else None


def _cluster_rings(circles):
    groups = []
    for circle in circles:
        for group in groups:
            center_x = sum(item[0] for item in group) / len(group)
            center_y = sum(item[1] for item in group) / len(group)
            if math.hypot(circle[0] - center_x, circle[1] - center_y) < \
                    settings.RING_CENTER_TOLERANCE:
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
        if len(radii) < settings.RING_MIN_DISTINCT_RADII:
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
        diagnostics.update(circles=[], targets=[], digits=[])
    if request["target"] != 2:
        return None
    x, y, width, height, anchor_u, anchor_v, _, _ = request["values"]
    gray = image.image2cv(img.to_format(image.Format.FMT_GRAYSCALE), False, False)
    # Requests and results remain in the vehicle's 320x240 calibration coordinates.
    scale = gray.shape[1] / 320
    roi_x, roi_y = int(x * scale), int(y * scale)
    roi = gray[roi_y:roi_y + int(height * scale),
               roi_x:roi_x + int(width * scale)]
    if min(roi.shape[:2]) < 23:
        return None
    binary = cv2.adaptiveThreshold(cv2.GaussianBlur(roi, (3, 3), 0), 255,
                                   cv2.ADAPTIVE_THRESH_MEAN_C,
                                   cv2.THRESH_BINARY_INV, int(20 * scale) + 1, 10)
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
        radius /= scale
        if not settings.RING_MIN_RADIUS <= radius <= settings.RING_MAX_RADIUS:
            continue
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            continue
        circles.append((x + moments["m10"] / moments["m00"] / scale,
                        y + moments["m01"] / moments["m00"] / scale, radius))
    targets = _cluster_rings(circles)
    if request.get("flags", 0) & FLAG_RING_NO_DIGIT:
        if diagnostics is not None:
            diagnostics.update(circles=circles, targets=targets)
        if len(targets) > 1:
            material = find_material(img, request, scale=scale)
            if material is None:
                return None
            return min(targets, key=lambda target: math.hypot(
                target[0] - material[0], target[1] - material[1]))
        return _unique_near(targets, (anchor_u, anchor_v), max(width, height))
    from ring_digits import read_ring_digit

    numbered_targets = []
    digits = []
    for target in targets:
        digit, score = read_ring_digit(gray, target, scale)
        digits.append((target[0], target[1], digit, score))
        if digit == 2:
            numbered_targets.append(target)
    if diagnostics is not None:
        diagnostics.update(circles=circles, targets=numbered_targets, digits=digits)
    return _unique_near(numbered_targets, (anchor_u, anchor_v), max(width, height))
