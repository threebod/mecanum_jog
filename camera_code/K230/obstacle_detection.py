"""Black blob selection and consecutive-frame confirmation; no board imports."""

STOP_LINE = b"OB:STOP\n"


def largest_black_blob(blobs, minimum_pixels):
    valid = [blob for blob in blobs if blob.pixels() >= minimum_pixels]
    return max(valid, key=lambda blob: blob.pixels()) if valid else None


class Confirmation:
    def __init__(self, frames=3, tolerance=40):
        self.frames = frames
        self.tolerance = tolerance
        self.center = None
        self.count = 0

    def update(self, blob):
        if blob is None:
            self.center = None
            self.count = 0
            return False
        center = (blob.cx(), blob.cy())
        if self.center is None or any(abs(a - b) > self.tolerance
                                      for a, b in zip(center, self.center)):
            self.center = center
            self.count = 1
        else:
            self.count = min(self.count + 1, self.frames)
        return self.count >= self.frames
