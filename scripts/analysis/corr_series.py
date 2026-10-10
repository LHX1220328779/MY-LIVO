import sys, pathlib, numpy as np
from scipy.spatial.transform import Rotation as Rot
run = pathlib.Path(sys.argv[1]); 
if not (run/"frames.csv").exists(): run = sorted((run/"Log"/"fusion").glob("*"))[-1]
a = np.loadtxt(run/"trajectory_lio.txt"); b = np.loadtxt(run/"trajectory_optimized.txt")
n=min(len(a),len(b)); a=a[:n]; b=b[:n]
t=a[:,0]-a[0,0]
Rn=Rot.from_quat(a[:,4:8]); Ro=Rot.from_quat(b[:,4:8])
dR=(Ro*Rn.inv()).as_rotvec()*180/np.pi   # world-frame correction rotation, deg
dp=b[:,1:4]-a[:,1:4]
print(" t    corr_rot_world(x,y,z deg)      corr_pos(x,y,z cm)   |  yaw_nat yaw_opt (deg)")
for k in range(0,int(t[-1])+1,int(sys.argv[2]) if len(sys.argv)>2 else 2):
    i=np.searchsorted(t,k)
    if i>=n: break
    print(f"{t[i]:5.1f}  {dR[i,0]:7.3f} {dR[i,1]:7.3f} {dR[i,2]:7.3f}     {dp[i,0]*100:7.1f} {dp[i,1]*100:7.1f} {dp[i,2]*100:7.1f}")
