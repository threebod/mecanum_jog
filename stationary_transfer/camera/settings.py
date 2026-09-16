"""Read ../CALIBRATION.md. Defaults intentionally do not authorize verification."""
VISION_CALIBRATED = False
UART_DEVICE = '/dev/ttyS4'
UART_PINS = {'A21': 'UART4_TX', 'A22': 'UART4_RX'}
BAUD = 115200
USE_DOWNWARD_LIGHT = False  # Enable only after checking the installed light points down.
THRESHOLDS = {
    1: [30, 80, 55, 105, 25, 80],
    2: [50, 100, -30, 25, 35, 100],
    3: [0, 40, 8, 35, -60, -30],
    4: [50, 100, -100, -50, 20, 80],
    5: [0, 20, -13, 11, -11, 13],
    6: [33, 75, -20, 5, -32, -8],
}
MATERIAL_AREA = (500, 30000)
MIN_FILL = 0.55
ALIGN_SEARCH_RADIUS = 35
VERIFY_TOLERANCE = 6
# Pixel width/height ranges measured at each observation pose. None means reject.
# MODE 3 must distinguish a LIFTED object from an object still in its slot.
# If top view cannot distinguish them, do not enable automatic held verification.
SIZE_BOUNDS = {1: None, 3: None, 4: None}  # each: (min_w,max_w,min_h,max_h)
RING_MIN_RADIUS = 8
RING_MAX_RADIUS = 110
RING_MIN_DISTINCT_RADII = 3
RING_CENTER_TOLERANCE = 4
# Calibrated gain is optional; unset retains camera default. Measure LAB after setup.
MANUAL_WB_GAINS = None
