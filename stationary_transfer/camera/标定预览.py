"""Run separately on MaixCAM2. No serial commands, no actuator motion.
Shows raw candidate dimensions even before SIZE_BOUNDS are calibrated.
"""
from maix import camera, display, image, app
import settings as cfg

cam=camera.Camera(320,240)
disp=display.Display()
while not app.need_exit():
    img=cam.read()
    if img is None:
        continue
    records=[]
    for color,threshold in cfg.THRESHOLDS.items():
        for b in img.find_blobs([threshold],area_threshold=50,pixels_threshold=50):
            if 300<=b.area()<=30000:
                records.append((color,b.x(),b.y(),b.w(),b.h(),b.cx(),b.cy(),b.pixels()))
    for color,x,y,w,h,u,v,pixels in records:
        img.draw_rect(x,y,w,h,image.COLOR_GREEN,1)
        img.draw_string(x,max(0,y-14),'{} {},{} {}x{}'.format(color,u,v,w,h),image.COLOR_WHITE)
    img.draw_cross(160,120,image.COLOR_RED,8,1)
    disp.show(img)
