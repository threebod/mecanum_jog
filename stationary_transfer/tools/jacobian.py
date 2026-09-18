"""Compute signed visual correction matrix from measured finite differences.
Example: python tools/jacobian.py --base 160 120 --x-sample 164 121 --dx 1
         --theta-sample 162 115 --dtheta 1 --anchor-slope 2 0 0 0
Values in example are illustrative only.
"""
import argparse
import math
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--base',type=float,nargs=2,required=True)
p.add_argument('--x-sample',type=float,nargs=2,required=True)
p.add_argument('--theta-sample',type=float,nargs=2,required=True)
p.add_argument('--dx',type=float,required=True)
p.add_argument('--dtheta',type=float,required=True)
p.add_argument('--anchor-slope',type=float,nargs=4,required=True,
               help='TCP anchor derivatives: du/dx du/dtheta dv/dx dv/dtheta')
a=p.parse_args()
values=a.base+a.x_sample+a.theta_sample+[a.dx,a.dtheta]+a.anchor_slope
if not all(map(math.isfinite,values)) or a.dx==0 or a.dtheta==0:
    raise SystemExit('Invalid finite difference measurements')
j00=(a.x_sample[0]-a.base[0])/a.dx-a.anchor_slope[0]
j10=(a.x_sample[1]-a.base[1])/a.dx-a.anchor_slope[2]
j01=(a.theta_sample[0]-a.base[0])/a.dtheta-a.anchor_slope[1]
j11=(a.theta_sample[1]-a.base[1])/a.dtheta-a.anchor_slope[3]
det=j00*j11-j01*j10
if abs(det)<0.01 or abs(det)/(math.hypot(j00,j10)*math.hypot(j01,j11))<0.1:
    raise SystemExit('Ill-conditioned pose; change observation geometry')
print('correction = {%+.6ff, %+.6ff, %+.6ff, %+.6ff};' % (-j11/det,j01/det,j10/det,-j00/det))
