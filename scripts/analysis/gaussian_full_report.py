#!/usr/bin/env python3
"""Full-recording Gaussian quality report and saved-map overlap rendering (no pose changes)."""
import argparse
import csv
import html
import json
import os
from pathlib import Path
import subprocess
import sys

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
from livo.map_export import resolve_run


def read_ply(path):
    props = []
    with open(path, "rb") as f:
        while True:
            line = f.readline()
            if not line: raise ValueError(f"Truncated PLY: {path}")
            text = line.decode().strip()
            if text.startswith("element vertex "): n = int(text.split()[-1])
            if text.startswith("property float "): props.append(text.split()[-1])
            if text == "end_header": offset = f.tell(); break
    if path.stat().st_size != offset+n*len(props)*4: raise ValueError("PLY length/count mismatch")
    return np.memmap(path, mode="r", dtype="<f4", offset=offset, shape=(n,len(props))), props


def stats(v):
    a = np.asarray(v, float)
    return dict(mean=float(a.mean()), median=float(np.median(a)), p10=float(np.percentile(a,10)),
                p90=float(np.percentile(a,90)), min=float(a.min()), max=float(a.max()), n=len(a))


def make_previews(table,props,poses,out,model_info=None):
    """Small unchanged spatial regions for an 8 GB GPU; the complete global PLY remains intact."""
    ordered=list(poses.values());views={"start":ordered[min(60,len(ordered)-1)],"middle":ordered[len(ordered)//2],"end":ordered[-8]}
    records=[]
    for name,pose in views.items():
        folder=out/"preview_regions"/name;folder.mkdir(parents=True,exist_ok=True)
        center=np.array(pose['T_world_camera'])[:3,3];payload=folder/'rows.bin';count=0
        with open(payload,'wb') as f:
            for start in range(0,len(table),100000):
                block=table[start:start+100000];delta=block[:,:3]-center
                keep=(delta*delta).sum(1)<=62**2
                if model_info and model_info.get("intrinsics"):
                    k=model_info["intrinsics"];camera=delta@np.array(pose["T_world_camera"])[:3,:3];z=camera[:,2]
                    keep|=(z>.2)&(z<120)&(camera[:,0]>-k["cx"]/k["fx"]*z-1.5)&(camera[:,0]<(k["width"]-k["cx"])/k["fx"]*z+1.5)&(camera[:,1]>-k["cy"]/k["fy"]*z-1.5)&(camera[:,1]<(k["height"]-k["cy"])/k["fy"]*z+1.5)
                chosen=block[keep];chosen.tofile(f);count+=len(chosen)
        model=folder/'point_cloud.ply'
        with open(model,'wb') as f:
            f.write(('ply\nformat binary_little_endian 1.0\nelement vertex '+str(count)+'\n'+''.join('property float '+p+'\n' for p in props)+'end_header\n').encode())
            with open(payload,'rb') as src:
                import shutil
                shutil.copyfileobj(src,f,4*1024*1024)
        payload.unlink();(folder/'camera_poses.json').write_text(json.dumps([pose],indent=2))
        record={'region':name,'gaussians':count,'radius_m':62,'forward_context_depth_m':120 if model_info else None,'stamp':pose['stamp'],'source':'complete global Gaussian map; unchanged selected rows'}
        (folder/'region_info.json').write_text(json.dumps(record,indent=2));records.append(record)
        if model_info:
            (folder/'map_info.json').write_text(json.dumps({k:model_info[k] for k in ['background_rgb','intrinsics'] if k in model_info},indent=2))
    return records


def replay_merged(g, out, props, table, poses):
    saved = out / "replay_inputs"
    views = json.loads((saved / "views.json").read_text())
    rows = []
    for view in views:
        name = view["frame"]; inp = saved/name
        target = out/"merged_views"/name
        if not (target/"metrics.csv").exists():
            header = (inp/"frames.txt").read_text().splitlines()[0]
            w,h,fx,fy,cx,cy = map(float,header.split())
            T = np.array(poses[name]["T_world_camera"])
            tmp = out/"visible_gaussians.bin"
            count = 0
            # Conservative frustum culling includes a 2 m support margin (4 sigma at max scale .5 m).
            # Every retained row remains byte-for-byte in the original map frame.
            with open(tmp,"wb") as f:
                for start in range(0,len(table),100000):
                    block = table[start:start+100000]
                    p = (block[:,:3].astype(float)-T[:3,3]) @ T[:3,:3]
                    z=p[:,2]; near=z>-2
                    loz=np.maximum(z,0)
                    keep = near & \
                        (p[:,0]>=-cx/fx*loz-2) & (p[:,0]<=(w-cx)/fx*loz+2) & \
                        (p[:,1]>=-cy/fy*loz-2) & (p[:,1]<=(h-cy)/fy*loz+2)
                    chosen=block[keep];chosen.tofile(f);count+=len(chosen)
            model=out/"visible_gaussians.ply"
            with open(model,"wb") as f:
                f.write(("ply\nformat binary_little_endian 1.0\nelement vertex "+str(count)+"\n"+
                         "".join("property float "+p+"\n" for p in props)+"end_header\n").encode())
                with open(tmp,"rb") as src:
                    import shutil
                    shutil.copyfileobj(src,f,4*1024*1024)
            tmp.unlink()
            print(f"merged replay {name}: {count} visible Gaussians",flush=True)
            with open(out/(name+"_render.log"),"w") as log:
                subprocess.run([str(REPO/"build/gaussian/my_livo_gaussian"),"--render",str(inp),str(model),str(target)],
                               stdout=log,stderr=subprocess.STDOUT,check=True)
            model.unlink()
        row=next(csv.DictReader(open(target/"metrics.csv")))
        rows.append(dict(row,source_submap=view["source_submap"]))
    (out/"merged_metrics.json").write_text(json.dumps(rows,indent=2))
    return rows


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("run");ap.add_argument("--render-merged",action="store_true")
    a=ap.parse_args();run=Path(a.run).resolve();fusion=resolve_run(run);g=fusion/"gaussian"
    out=run/"full_gaussian_analysis";out.mkdir(exist_ok=True)
    info=json.loads((g/"map_info.json").read_text());native=json.loads((run/"evaluation/metrics.json").read_text())
    poses={p["frame"]:p for p in json.loads((g/"camera_poses.json").read_text())}
    t0=min(p["stamp"] for p in poses.values())
    table,props=read_ply(g/"point_cloud.ply")
    if len(table)!=info['gaussians']:raise ValueError('Saved Gaussian count differs from map_info')
    with open(g/'gaussian_centers.pcd','rb') as f:
        while True:
            line=f.readline()
            if not line:raise ValueError('Invalid centre PCD')
            if line.strip()==b'DATA binary':break
        pcd_offset=f.tell()
    if (g/'gaussian_centers.pcd').stat().st_size!=pcd_offset+16*len(table):raise ValueError('Centre PCD is incomplete')
    centers=np.memmap(g/'gaussian_centers.pcd',mode='r',dtype=[('xyz','<f4',(3,)),('rgb','<u4')],offset=pcd_offset,shape=(len(table),))
    all_finite=True;scale_hist=[];opacity_hist=[]
    for start in range(0,len(table),100000):
        block=table[start:start+100000];all_finite &= bool(np.isfinite(block).all())
        if not np.array_equal(block[:,:3],centers['xyz'][start:start+len(block)]):raise ValueError('PLY and PCD centre coordinates differ')
        sample=block[::25]
        scale_hist.extend(np.exp(sample[:,52:55]).max(1).tolist())
        opacity_hist.extend((1/(1+np.exp(-np.clip(sample[:,51],-50,50)))).tolist())
    if not all_finite: raise ValueError("Non-finite saved Gaussian parameters")
    metrics=info["metrics"];keys=["psnr_lidar_region_db","psnr_full_db","ssim_full","depth_mae_m","depth_30m_mae_m","depth_30m_median_m"]
    summaries={s:{k:stats([float(r[k]) for r in metrics if r['stage']==s and r['split']=='heldout']) for k in keys} for s in ['initial','trained']}
    trained=[r for r in metrics if r['stage']=='trained' and r['split']=='heldout']
    initial={(r['submap'],r['frame']):r for r in metrics if r['stage']=='initial' and r['split']=='heldout'}
    times=np.array([poses[r['frame']]['stamp']-t0 for r in trained])
    delta={k:[float(r[k])-float(initial[r['submap'],r['frame']][k]) for r in trained] for k in keys}
    dark=[];sharp=[]
    for r in trained:
        folder=g/r['submap'];name=r['frame']
        gt=cv2.imread(str(folder/('reference_'+name+'.png')),cv2.IMREAD_GRAYSCALE)
        pred=cv2.imread(str(folder/('trained_'+name+'.png')),cv2.IMREAD_GRAYSCALE)
        dark.append(float(np.mean((pred<5)&(gt>20))))
        sharp.append(float(cv2.Laplacian(pred,cv2.CV_32F).var()/max(cv2.Laplacian(gt,cv2.CV_32F).var(),1e-6)))
    fig,ax=plt.subplots(3,1,figsize=(12,9),sharex=True)
    for i,key in enumerate(['psnr_full_db','ssim_full','depth_30m_mae_m']):
        ax[i].plot(times,[float(initial[r['submap'],r['frame']][key]) for r in trained],alpha=.65,label='initial')
        ax[i].plot(times,[float(r[key]) for r in trained],label='trained');ax[i].set_ylabel(key);ax[i].grid(alpha=.3);ax[i].legend()
    ax[-1].set_xlabel('Time after first Gaussian camera (s)');fig.tight_layout();fig.savefig(out/'quality_over_time.png',dpi=140);plt.close(fig)
    pos=np.array([np.array(poses[r['frame']]['T_world_camera'])[:3,3] for r in trained])
    fig,ax=plt.subplots(figsize=(9,8));sc=ax.scatter(pos[:,0],pos[:,1],c=[float(r['ssim_full']) for r in trained],s=14,cmap='viridis',vmin=0,vmax=1);ax.axis('equal');ax.set_xlabel('Map x (m)');ax.set_ylabel('Map y (m)');fig.colorbar(sc,label='Held-out SSIM');fig.tight_layout();fig.savefig(out/'route_quality.png',dpi=140);plt.close(fig)
    errors=np.genfromtxt(run/'evaluation/error_series.csv',delimiter=',',names=True)
    fig,ax=plt.subplots(2,1,figsize=(12,7),sharex=True)
    ax[0].plot(errors['t_rel_s'],errors['native_rear_m'],label='native LIVO');ax[0].set_ylabel('Distance to RTK (m)');ax[0].legend();ax[0].grid(alpha=.3)
    ax[1].plot(errors['t_rel_s'],100*errors['opt_rear_m'],label='final corrected poses');ax[1].axhline(10,c='orange',ls='--',label='10 cm');ax[1].axhline(20,c='red',ls='--',label='20 cm');ax[1].set_ylabel('Distance to RTK (cm)');ax[1].set_xlabel('Run time (s)');ax[1].legend();ax[1].grid(alpha=.3);fig.tight_layout();fig.savefig(out/'trajectory_consistency.png',dpi=140);plt.close(fig)
    merged=replay_merged(g,out,props,table,poses) if a.render_merged else []
    regions=make_previews(table,props,poses,out)
    record={'status':'done','full_recording_seconds':1020.12,'source_archive_frames':native['run']['frames'],
            'source_distance_m':native['run']['optimized_path_m'],'gaussian_frames':info['frames'],
            'camera_span_sec':max(p['stamp'] for p in poses.values())-t0,'submaps':info['submaps'],
            'gaussians':info['gaussians'],'ply_bytes':(g/'point_cloud.ply').stat().st_size,
            'all_saved_parameters_finite':all_finite,'per_submap_quality':summaries,
            'center_pcd_complete_and_matches_ply':True,
            'paired_improvement_fraction':{k:float(np.mean(np.array(v)<0 if k.startswith('depth') else np.array(v)>0)) for k,v in delta.items()},
            'unsupported_dark_pixel_fraction':stats(dark),'laplacian_variance_ratio':stats(sharp),
            'sampled_max_sigma_m':stats(scale_hist),'sampled_opacity':stats(opacity_hist),
            'native_trajectory':native['trajectory']['optimized'],'native_map':native['map'],
            'native_startup':native.get('startup'),'native_pole_pairs':native.get('pole_pairs'),
            'merged_replay_views':len(merged),'merged_quality':{k:stats([float(r[k]) for r in merged]) for k in keys} if merged else {}}
    record['preview_regions']=regions
    if merged:
        singles={(r['submap'],r['frame']):r for r in trained}
        record['merged_vs_same_single_views']={
            k:{'single':stats([float(singles[r['source_submap'],r['frame']][k]) for r in merged]),
               'merged_minus_single':stats([float(r[k])-float(singles[r['source_submap'],r['frame']][k]) for r in merged])}
            for k in keys}
    for k in ['gpu.csv','execution.json']:
        if k=='execution.json' and (out/k).exists():record['execution']=json.loads((out/k).read_text())
        if k=='gpu.csv' and (out/k).exists():
            gpu=list(csv.DictReader(open(out/k)));mk=next(k for k in gpu[0] if k.strip().startswith('memory.used'));record['gpu_peak_mib']=max(int(r[mk].split()[0]) for r in gpu)
    (out/'quality_summary.json').write_text(json.dumps(record,indent=2))
    chosen=set(np.linspace(0,len(trained)-1,8,dtype=int).tolist())
    chosen.update(np.argsort([float(r['ssim_full']) for r in trained])[:4].tolist())
    rel=lambda p:html.escape(os.path.relpath(p,out))
    gallery=[]
    for i in sorted(chosen):
        r=trained[i];folder=g/r['submap'];name=r['frame']
        images=''.join('<figure><img loading="lazy" src="'+rel(folder/(kind+'_'+name+'.png'))+'"><figcaption>'+kind+'</figcaption></figure>' for kind in ['reference','initial','trained'])
        gallery.append(f"<h3>t+{times[i]:.1f}s · {r['submap']} · PSNR {float(r['psnr_full_db']):.2f} dB · SSIM {float(r['ssim_full']):.3f}</h3><div class='row'>{images}</div>")
    merged_gallery=[]
    for r in merged:
        name=r['frame'];folder=out/'merged_views'/name
        merged_gallery.append('<h3>'+name+f" · merged SSIM {float(r['ssim_full']):.3f}</h3><div class='row'>"+''.join('<figure><img loading="lazy" src="'+rel(p)+'"><figcaption>'+label+'</figcaption></figure>' for p,label in [(folder/('reference_'+name+'.png'),'reference'),(g/r['source_submap']/('trained_'+name+'.png'),'single submap'),(folder/('merged_'+name+'.png'),'actual merged map')])+'</div>')
    doc='''<!doctype html><meta charset="utf-8"><title>Full Gaussian map analysis</title><style>body{max-width:1500px;margin:30px auto;padding:15px;font-family:system-ui}.row{display:flex;gap:8px}figure{margin:0;flex:1}img{max-width:100%;height:auto}pre{white-space:pre-wrap;overflow-wrap:anywhere}h3{margin-top:35px}</style><h1>全程高斯建图实际分析</h1><p>固定原 LIVO/RTK 位姿和 LiDAR 中心。initial 指未训练高斯，不是原生 LIO 定位精度基线。SSIM/PSNR 评价画面；渲染深度含透明混合和遮挡，不能代替独立定位或灯柱顶部真值验收。</p>'''
    doc+='<img src="quality_over_time.png"><img src="route_quality.png"><img src="trajectory_consistency.png"><h2>全程代表视角与低分视角</h2>'+''.join(gallery)+'<h2>合并地图的区域重渲染</h2>'+''.join(merged_gallery)+'<h2>完整统计</h2><pre>'+html.escape(json.dumps(record,indent=2,ensure_ascii=False))+'</pre>'
    (out/'report.html').write_text(doc)
    print(json.dumps({k:record[k] for k in ['gaussian_frames','submaps','gaussians','camera_span_sec','gpu_peak_mib','merged_replay_views']},indent=2))


if __name__=='__main__':main()
