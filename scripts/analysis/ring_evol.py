import sys, pathlib, numpy as np
sys.path.insert(0,"/home/project/MY-LIVO/scripts"); sys.path.insert(0,"/home/project/MY-LIVO")
from evaluate_wuhu_lio import read_pcd, ring_fit, POLE_RING_RADIUS
from scripts.analysis.round_summary import POLE
cx,cy,r,sh0,sh1=POLE['cx'],POLE['cy'],POLE['r'],POLE['shaft'][0],POLE['shaft'][1]
for run_dir in sys.argv[1:]:
    fus=sorted(pathlib.Path(run_dir+'/Log/fusion').glob('*'))[-1]
    prev=None
    print('---',run_dir.split('/')[-1])
    for f in sorted(fus.glob('map_*.pcd')):
        k=int(f.stem.split('_')[1])
        if k<3 or k>10: continue
        p=read_pcd(f); xyz=np.asarray(p[:,:3],float)
        m=(np.hypot(xyz[:,0]-cx,xyz[:,1]-cy)<=r)&(xyz[:,2]>=sh0)&(xyz[:,2]<=sh1)
        if m.sum()<40: continue
        rr=ring_fit(xyz[m,:2], np.array([cx,cy]), POLE_RING_RADIUS)
        if not rr: continue
        c=rr[0]; n=rr[2]; rms=rr[1]
        dprev = 100*np.linalg.norm(c-prev) if prev is not None else 0
        print(f"  map_{k} n={n:4d} rms={rms*100:.1f}cm cx={c[0]:.3f} cy={c[1]:.3f} d_prev={dprev:.1f}cm")
        prev=c
