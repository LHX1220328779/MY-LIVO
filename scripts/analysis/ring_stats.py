import sys, pathlib, numpy as np
sys.path.insert(0,"/home/project/MY-LIVO/scripts");sys.path.insert(0,"/home/project/MY-LIVO")
from evaluate_wuhu_lio import read_pcd, ring_fit, POLE_RING_RADIUS
from scripts.analysis.round_summary import POLE
cx,cy,r,sh0,sh1=POLE['cx'],POLE['cy'],POLE['r'],POLE['shaft'][0],POLE['shaft'][1]
for run_dir in sys.argv[1:]:
    fus=sorted(pathlib.Path(run_dir+'/Log/fusion').glob('*'))[-1]
    fits={}
    for f in sorted(fus.glob('map_*.pcd')):
        k=int(f.stem.split('_')[1])
        p=read_pcd(f); xyz=np.asarray(p[:,:3],float)
        m=(np.hypot(xyz[:,0]-cx,xyz[:,1]-cy)<=r)&(xyz[:,2]>=sh0)&(xyz[:,2]<=sh1)
        if m.sum()<40: continue
        rr=ring_fit(xyz[m,:2], np.array([cx,cy]), POLE_RING_RADIUS)
        if rr: fits[k]=rr[0]
    keys=sorted(fits)
    print(run_dir,'chunks',keys[:10],'...',keys[-5:])
    def stat(ks):
        ks=[k for k in ks if k in fits]
        if len(ks)<2: return
        ps=[100*np.linalg.norm(fits[a]-fits[b]) for i,a in enumerate(ks) for b in ks[i+1:]]
        print(f'  n={len(ks)} median/P95/max={np.median(ps):.2f}/{np.percentile(ps,95):.2f}/{max(ps):.2f} cm ({len(ps)} pairs)')
    print(' chunks 3-8 (start, t<~18s):'); stat([k for k in keys if 3<=k<=8])
    print(' chunks 3-9 (start, t<~24s):'); stat([k for k in keys if 3<=k<=9])
    print(' all chunks 3-{}:'.format(keys[-1])); stat([k for k in keys if 3<=k])
