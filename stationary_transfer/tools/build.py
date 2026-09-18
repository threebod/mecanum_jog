"""Reproducible ARM GCC build. python tools/build.py [--cc path/to/arm-none-eabi-gcc]."""
from pathlib import Path
import argparse
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--cc', default=shutil.which('arm-none-eabi-gcc'))
args = parser.parse_args()
if not args.cc:
    raise SystemExit('Install ARM GNU toolchain or pass --cc')
cc=Path(args.cc)
out=ROOT/'build';out.mkdir(exist_ok=True)
flags=['-mcpu=cortex-m4','-mthumb','-mfpu=fpv4-sp-d16','-mfloat-abi=hard','-std=c99','-Os','-g3',
       '-ffunction-sections','-fdata-sections','-Wall','-Wextra','-Werror=implicit-function-declaration',
       '-DUSE_STDPERIPH_DRIVER','-DSTM32F40_41xxx','-DHSE_VALUE=8000000']
for include in ['firmware','vendor/CORE','vendor/FWLIB/inc','vendor/Emm']:
    flags.extend(['-I',str(ROOT/include)])
sources=list((ROOT/'firmware').glob('*.c'))+[ROOT/'firmware/startup_gcc.s',ROOT/'vendor/Emm/Emm_V5.c']
sources.extend(ROOT/'vendor/FWLIB/src'/name for name in ['misc.c','stm32f4xx_rcc.c','stm32f4xx_gpio.c',
                                                    'stm32f4xx_tim.c','stm32f4xx_usart.c','stm32f4xx_can.c'])
objects=[]
for src in sources:
    obj=out/(src.stem+'.o');objects.append(obj)
    subprocess.run([str(cc),*flags,'-c',str(src),'-o',str(obj)],check=True)
elf=out/'stationary_transfer.elf'
subprocess.run([str(cc),*flags,*map(str,objects),'-T',str(ROOT/'firmware/stm32f407.ld'),
                '-nostartfiles','--specs=nano.specs','--specs=nosys.specs','-Wl,--gc-sections',
                '-Wl,-Map='+str(out/'stationary_transfer.map'),'-Wl,-u,_printf_float',
                '-Wl,-u,_scanf_float','-lm','-o',str(elf)],check=True)
suffix=cc.suffix
for fmt,extension in [('ihex','hex'),('binary','bin')]:
    subprocess.run([str(cc.with_name('arm-none-eabi-objcopy'+suffix)),'-O',fmt,str(elf),
                    str(out/('stationary_transfer.'+extension))],check=True)
subprocess.run([str(cc.with_name('arm-none-eabi-size'+suffix)),str(elf)],check=True)
print('Build complete. Default firmware refuses AUTO until calibrated. No hardware was flashed.')
