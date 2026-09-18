from pathlib import Path
import subprocess
import shutil
import sys
import compileall
ROOT=Path(__file__).resolve().parents[1]
gcc=shutil.which('gcc')
if not gcc:raise SystemExit('Host GCC required')
out=ROOT/'build';out.mkdir(exist_ok=True)
flags=['-std=c99','-Wall','-Wextra','-Werror','-I',str(ROOT/'firmware')]
subprocess.run([gcc,*flags,'-Dconfig=default_config','-c',str(ROOT/'firmware/config.c'),'-o',str(out/'test_config.o')],check=True)
exe=out/('test_controller.exe' if sys.platform=='win32' else 'test_controller')
subprocess.run([gcc,*flags,str(ROOT/'tests/test_controller.c'),str(ROOT/'firmware/transfer.c'),
                str(ROOT/'firmware/protocol.c'),str(out/'test_config.o'),'-lm','-o',str(exe)],check=True)
result=subprocess.run([str(exe)],check=True,capture_output=True,text=True)
print(result.stdout)
sys.path.insert(0,str(ROOT/'camera'))
from protocol import encode
expected=encode(0x12345678,kind=2,mode=3,color=5,target=2,flags=3,values=[-20,120,80,0,0,0,0,0]).hex()
assert 'GOLDEN:'+expected in result.stdout, 'C/Python wire format mismatch'
assert compileall.compile_dir(str(ROOT/'camera'),quiet=1)
subprocess.run([sys.executable,str(ROOT/'tests/test_camera.py')],check=True)
print('PASS cross-language wire format and camera syntax. Hardware/SDK not exercised.')
