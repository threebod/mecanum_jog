"""MaixVision project entry point. Keep all files in this directory together."""
CALIBRATION_PREVIEW = False


if __name__ == "__main__":
    if CALIBRATION_PREVIEW:
        import 标定预览  # This module runs its preview loop when imported.
    else:
        from 顶部搬运摄像头 import main
        main()
