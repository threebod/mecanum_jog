import math

import vision_settings as settings


def _unique_near(candidates, anchor, radius):
    selected = [item for item in candidates
                if math.hypot(item[0] - anchor[0], item[1] - anchor[1]) <= radius]
    return selected[0] if len(selected) == 1 else None


def find_material(img, request):
    x, y, width, height, anchor_u, anchor_v, _, _ = request["values"]
    candidates = []
    mask = request["values"][6]
    for color, threshold in settings.COLOR_THRESHOLDS.items():
        if request["selector"] not in (0, color) or \
                (mask and not mask & (1 << (color - 1))):
            continue
        for blob in img.find_blobs([threshold], roi=(x, y, width, height),
                                   area_threshold=50, pixels_threshold=50):
            area = blob.w() * blob.h()
            if not settings.MATERIAL_AREA[0] <= area <= settings.MATERIAL_AREA[1]:
                continue
            if blob.x() <= x or blob.y() <= y or \
                    blob.x() + blob.w() >= x + width or \
                    blob.y() + blob.h() >= y + height:
                continue
            quality = min(100, int(blob.pixels() * 100 / max(area, 1)))
            candidates.append((blob.cx(), blob.cy(), quality, blob.w(), blob.h(), color))
    return _unique_near(candidates, (anchor_u, anchor_v), max(width, height))


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
        diagnostics.update(circles=[], targets=[])
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
    if diagnostics is not None:
        diagnostics.update(circles=circles, targets=targets)
    return _unique_near(targets, (anchor_u, anchor_v), max(width, height))
