"""MaixCAM2 stationary transfer application. Deploy entire camera directory.

UART4: A21 TX -> STM32 PC11, A22 RX <- STM32 PC10, common GND.
No remote motion commands originate here; only request-scoped observations.
"""
from maix import camera, display, image, app, uart, pinmap, err
import time
import settings as cfg
from protocol import Parser, Stable, encode, validate_request
from detectors import material, ring


def main():
    for pin,function in cfg.UART_PINS.items():
        err.check_raise(pinmap.set_pin_function(pin,function), 'UART4 pin mapping')
    serial = uart.UART(cfg.UART_DEVICE,cfg.BAUD)
    cam = camera.Camera(320,240)
    disp = display.Display()
    led = None
    if cfg.MANUAL_WB_GAINS is not None:
        cam.awb_mode(camera.AwbMode.Manual)
        cam.set_wb_gain(cfg.MANUAL_WB_GAINS)
    if cfg.USE_DOWNWARD_LIGHT:
        from maix import gpio
        err.check_raise(pinmap.set_pin_function('B25','GPIOB25'),'light pin')
        led=gpio.GPIO('GPIOB25',gpio.Mode.OUT);led.value(1)
    parser=Parser();stable=Stable();request=None;started=0;discard=0
    last_status='UNCALIBRATED' if not cfg.VISION_CALIBRATED else 'READY'
    last_packet=None
    try:
        while not app.need_exit():
            now=int(time.monotonic()*1000)
            for p in parser.feed(serial.read(-1,0) or b''):
                if validate_request(p):
                    request=p;started=now;discard=2;stable.reset()
            img=cam.read()
            if request is not None:
                found=None
                if img is not None and discard:
                    discard-=1
                elif img is not None:
                    found=ring(img,request) if request['mode']==2 else material(img,request)
                good=stable.update(found,now)
                if good or now-started>=1000:
                    approved=good and cfg.VISION_CALIBRATED
                    values=[0]*8
                    if found is not None:
                        values[:5]=found  # u,v,quality,width,height
                    response=encode(request['token'],kind=2,mode=request['mode'],color=request['color'],
                                    target=request['target'],flags=3 if approved else 0,values=values)
                    if serial.write(response)!=len(response):
                        raise RuntimeError('UART short write')
                    last_packet=request
                    last_status='OK' if approved else ('UNCALIBRATED' if not cfg.VISION_CALIBRATED else 'NO_STABLE_TARGET')
                    request=None
                if img is not None and found is not None:
                    img.draw_cross(int(found[0]),int(found[1]),image.COLOR_GREEN,8,2)
            if img is not None:
                p=request or last_packet
                if p:
                    x,y,w,h,u,v,_,_=p['values']
                    img.draw_rect(x,y,w,h,image.COLOR_YELLOW,1)
                    img.draw_cross(u,v,image.COLOR_RED,6,1)
                    img.draw_string(2,2,'mode={} color={} ring={}'.format(p['mode'],p['color'],p['target']),image.COLOR_WHITE)
                img.draw_string(2,220,last_status,image.COLOR_WHITE)
                disp.show(img)
    finally:
        if led is not None:led.value(0)
        serial.close()


if __name__=='__main__':
    main()
