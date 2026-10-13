"""Run once through USB CanMV IDE; no camera, motor or network changes."""
import os
import sys

print("SYSTEM:", os.uname())
print("PYTHON:", sys.version)
for name in ("network", "socket", "select", "image", "media.mjpeg", "media.rtsp", "ybUtils"):
    try:
        module = __import__(name, None, None, ["*"])
        print("MODULE:", name, "OK")
        if name in ("network", "image"):
            print("API:", name, dir(module))
        if name == "image":
            print("IMAGE API:", dir(module.Image))
    except ImportError as error:
        print("MODULE:", name, "UNAVAILABLE", error)
