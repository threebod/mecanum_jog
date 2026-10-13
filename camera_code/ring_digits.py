"""Identify the central 1/2/3 glyph with OpenCV templates, without an OCR model."""
import math

import vision_settings as settings


# Central glyphs sampled from the user-provided photograph.
_REFERENCE_DIGITS = {
    1: (0.39130435, (
        '..............######',
        '..............######',
        '.............#######',
        '...........#########',
        '.........###########',
        '####################',
        '####################',
        '####################',
        '............########',
        '.............#######',
        '.............#######',
        '.............#######',
        '.............#######',
        '.............#######',
        '.............#######',
        '.............#######',
        '.............#######',
        '.............#######',
        '.............#######',
        '............########',
        '.............#######',
        '.............#######',
        '............########',
        '.............#######',
        '.............#######',
        '............########',
        '............########',
        '............########',
    )),
    2: (0.69642857, (
        '.......#######......',
        '.....###########....',
        '...##############...',
        '...###############..',
        '..#####.......#####.',
        '.#####.........#####',
        '.#####.........#####',
        '.####...........####',
        '.####...........####',
        '.####...........####',
        '...............#####',
        '...............#####',
        '..............#####.',
        '.............######.',
        '...........#######..',
        '.........#######....',
        '.......########.....',
        '.....########.......',
        '....#######.........',
        '...######...........',
        '..#####.............',
        '..####..............',
        '.####...............',
        '.####...............',
        '#################...',
        '####################',
        '####################',
        '####################',
    )),
    3: (0.68468468, (
        '......########......',
        '....############....',
        '...##############...',
        '..#######.########..',
        '..####.......######.',
        '.#####........#####.',
        '.####..........####.',
        '.####..........####.',
        '.####..........####.',
        '..............#####.',
        '..............####..',
        '............######..',
        '........#########...',
        '.......#########....',
        '........##########..',
        '............#######.',
        '..............######',
        '...............#####',
        '...............#####',
        '####...........#####',
        '####...........#####',
        '####...........#####',
        '#####..........####.',
        '.####.........#####.',
        '.######.....######..',
        '..###############...',
        '...#############....',
        '.....#########......',
    )),
}
_templates = None


def _normalize(mask):
    import cv2

    points = cv2.findNonZero(mask)
    if points is None:
        return None
    x, y, width, height = cv2.boundingRect(points)
    if min(width, height) < 2:
        return None
    normalized = cv2.resize(mask[y:y + height, x:x + width], (20, 28),
                            interpolation=cv2.INTER_AREA)
    return normalized > 90, width / height


def _get_templates():
    import cv2
    import numpy as np

    global _templates
    if _templates is None:
        _templates = []
        for digit in (1, 2, 3):
            for font in (cv2.FONT_HERSHEY_SIMPLEX, cv2.FONT_HERSHEY_DUPLEX,
                         cv2.FONT_HERSHEY_COMPLEX, cv2.FONT_HERSHEY_TRIPLEX):
                for thickness in (1, 2, 3):
                    mask = np.zeros((64, 64), dtype=np.uint8)
                    cv2.putText(mask, str(digit), (8, 50), font, 1.5, 255, thickness)
                    normalized, aspect = _normalize(mask)
                    _templates.append((digit, normalized, aspect))
        for digit, (aspect, rows) in _REFERENCE_DIGITS.items():
            reference = np.array([[pixel == "#" for pixel in row] for row in rows])
            _templates.append((digit, reference, aspect))
    return _templates


def classify_digit(mask):
    import cv2
    import numpy as np

    scores = {1: 0.0, 2: 0.0, 3: 0.0}
    # Padding prevents glyph clipping while testing camera tilt and quarter turns.
    padding = max(mask.shape) // 2
    padded = np.pad(mask, padding)
    for turn in range(4):
        rotated = np.ascontiguousarray(np.rot90(padded, turn))
        height, width = rotated.shape
        for angle in (-30, -15, 0, 15, 30, 45):
            matrix = cv2.getRotationMatrix2D((width / 2, height / 2), angle, 1)
            normalized = _normalize(cv2.warpAffine(rotated, matrix, (width, height)))
            if normalized is None:
                continue
            glyph, aspect = normalized
            for digit, template, template_aspect in _get_templates():
                intersection = np.count_nonzero(glyph & template)
                union = np.count_nonzero(glyph | template)
                score = intersection / max(union, 1)
                score *= min(aspect, template_aspect) / max(aspect, template_aspect)
                scores[digit] = max(scores[digit], score)
    ordered = sorted(scores, key=scores.get, reverse=True)
    digit = ordered[0]
    if scores[digit] < settings.RING_DIGIT_MIN_SCORE or \
            scores[digit] - scores[ordered[1]] < settings.RING_DIGIT_SCORE_MARGIN:
        return None, scores[digit]
    return digit, scores[digit]


def read_ring_digit(gray, target, scale):
    import cv2
    import numpy as np

    center_x, center_y = target[0] * scale, target[1] * scale
    radius = target[3] * scale / 2
    # The provided target has a white center about half the outer radius.
    inner_radius = radius * settings.RING_DIGIT_ROI_RATIO
    half = int(math.ceil(inner_radius))
    x, y = int(round(center_x)) - half, int(round(center_y)) - half
    side = half * 2 + 1
    if half < 8 or x < 0 or y < 0 or \
            x + side > gray.shape[1] or y + side > gray.shape[0]:
        return None, 0.0
    patch = gray[y:y + side, x:x + side]
    _, mask = cv2.threshold(patch, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    yy, xx = np.ogrid[:side, :side]
    mask[(xx - half) ** 2 + (yy - half) ** 2 > inner_radius ** 2] = 0
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    clean = np.zeros_like(mask)
    for index in range(1, count):
        if stats[index, cv2.CC_STAT_AREA] >= max(4, side * side * 0.005):
            clean[labels == index] = 255
    return classify_digit(clean)
