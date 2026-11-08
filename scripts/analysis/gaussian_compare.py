#!/usr/bin/env python3
"""Run the previous mapper and compare actual exported maps at identical heldout cameras."""
import argparse
import csv
import html
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF","expandable_segments:True")
import numpy as np

REPO=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(REPO/"tools"))
from livo.gaussian_global import frames_from, read_ply, write_ply, visible_rows, visibility_index, prepare_batch
from livo.gaussian_map import merge_binary


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("prepared",type=Path);ap.add_argument("candidate",type=Path);ap.add_argument("output",type=Path)
    ap.add_argument("--reuse-candidate-renders",action="store_true",help="Reuse this mapper run's completed final global replay")
    ap.add_argument("--batch-size",type=int,default=2)
    ap.add_argument("--binary",type=Path,default=REPO/"build/gaussian/my_livo_gaussian")
    ap.add_argument("--baseline-model",type=Path,help="Reuse an existing complete baseline PLY instead of retraining it")
    a=ap.parse_args()
    if a.batch_size<1:ap.error("batch-size must be positive")
    a.prepared=a.prepared.resolve();a.candidate=a.candidate.resolve();a.output=a.output.resolve()
    a.output.mkdir(parents=True,exist_ok=True)
    baseline=a.output/"legacy";baseline.mkdir(exist_ok=True)
    header,frames=frames_from(a.prepared)
    started=time.monotonic()
    parts=[]
    if a.baseline_model and not (baseline/"point_cloud.ply").exists():
        (baseline/"point_cloud.ply").symlink_to(a.baseline_model.resolve())
    for manifest in ([] if a.baseline_model else sorted(a.prepared.glob("submap_*/frames.txt"))):
        dest=baseline/manifest.parent.name
        if not (dest/"point_cloud.ply").exists():
            print(f"legacy training {dest.name}",flush=True)
            with open(a.output/"legacy.log","a") as log:
                subprocess.run([str(a.binary),str(manifest.parent),str(dest),"40","15","600000","0.1","0.5","0"],
                               stdout=log,stderr=subprocess.STDOUT,check=True)
        parts.append(dest)
    if not (baseline/"point_cloud.ply").exists():
        merge_binary([p/"point_cloud.ply" for p in parts],baseline/"point_cloud.ply","ply")
        merge_binary([p/"gaussian_centers.pcd" for p in parts],baseline/"gaussian_centers.pcd","pcd")
    heldout=[f for f in frames if f["validation"]]
    info_path=a.candidate/"map_info.json"
    background=json.loads(info_path.read_text()).get("background_rgb",[0,0,0]) if info_path.exists() else [0,0,0]
    metrics={}
    for label,model_path in [("legacy",baseline/"point_cloud.ply"),("shared",a.candidate/"point_cloud.ply")]:
        rows=[]
        if label=="shared" and a.reuse_candidate_renders:
            for base in [a.candidate/"views",a.candidate/"shared/views"]:
                for file in sorted(base.glob("batch_*/metrics.csv")):
                    dest=a.output/"shared"/file.parent.name
                    shutil.copytree(file.parent,dest,dirs_exist_ok=True)
                    rows.extend(dict(r,folder=str(dest)) for r in csv.DictReader(file.open()))
            if sorted(r["frame"] for r in rows)==sorted(f["name"] for f in heldout):
                metrics[label]={r["frame"]:r for r in rows}
                continue
            raise ValueError("Candidate final-map replay is incomplete")
        table=read_ply(model_path);index=None;rows=[]
        for start in range(0,len(heldout),a.batch_size):
            group=heldout[start:start+a.batch_size];dest=a.output/label/f"views_{start:04d}_n{a.batch_size}"
            if not (dest/"metrics.csv").exists():
                work=a.output/"replay_work"
                if work.exists():shutil.rmtree(work)
                prepare_batch(work,header,group,background=background)
                if index is None:index=visibility_index(table[:,:3],header)
                active=visible_rows(table[:,:3],group,header,index)
                write_ply(work/"point_cloud.ply",table[active])
                print(f"{label} final-map replay {start+1}/{len(heldout)}: {len(active)} Gaussians",flush=True)
                with open(a.output/"replay.log","a") as log:
                    subprocess.run([str(a.binary),"--render",str(work),str(work/"point_cloud.ply"),str(dest)],
                                   stdout=log,stderr=subprocess.STDOUT,check=True)
                shutil.rmtree(work)
            rows.extend(dict(row,folder=str(dest)) for row in csv.DictReader(open(dest/"metrics.csv")))
        metrics[label]={r["frame"]:r for r in rows}
    keys=["psnr_full_db","psnr_lidar_region_db","ssim_full","depth_30m_mae_m","depth_mae_m","depth_coverage","depth_30m_median_m"]
    names=sorted(metrics["legacy"])
    if names != sorted(metrics["shared"]):raise ValueError("Camera sets differ")
    summary={label:{k:float(np.mean([float(rows[n][k]) for n in names])) for k in keys} for label,rows in metrics.items()}
    summary["ssim_improved_views"]=sum(float(metrics["shared"][n]["ssim_full"])>float(metrics["legacy"][n]["ssim_full"]) for n in names)
    summary["heldout_views"]=len(names);summary["elapsed_sec"]=time.monotonic()-started
    summary["common_background_rgb"]=background
    (a.output/"comparison.json").write_text(json.dumps(dict(summary=summary,per_view=metrics),indent=2))
    figures=[]
    pose_path=a.candidate/"camera_poses.json"
    if pose_path.is_file():
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        poses={p["frame"]:p for p in json.loads(pose_path.read_text())}
        zero=min(p["stamp"] for p in poses.values())
        times=np.array([poses[n]["stamp"]-zero for n in names])
        fig,axes=plt.subplots(3,1,figsize=(12,8),sharex=True)
        for ax,key in zip(axes,["ssim_full","psnr_lidar_region_db","depth_30m_mae_m"]):
            for label in ["legacy","shared"]:
                ax.plot(times,[float(metrics[label][n][key]) for n in names],label=label,alpha=.85)
            ax.set_ylabel(key);ax.grid(alpha=.3);ax.legend()
        axes[-1].set_xlabel("Time after first camera (s)");fig.tight_layout()
        fig.savefig(a.output/"quality_over_time.png",dpi=140);plt.close(fig);figures.append("quality_over_time.png")
        xy=np.array([np.array(poses[n]["T_world_camera"])[:2,3] for n in names])
        fig,ax=plt.subplots(figsize=(8,7));sc=ax.scatter(xy[:,0],xy[:,1],c=[float(metrics["shared"][n]["ssim_full"]) for n in names],s=16,cmap="viridis",vmin=0,vmax=1)
        ax.axis("equal");ax.set_xlabel("Map x (m)");ax.set_ylabel("Map y (m)");fig.colorbar(sc,label="Final-map heldout SSIM");fig.tight_layout()
        fig.savefig(a.output/"route_quality.png",dpi=140);plt.close(fig);figures.append("route_quality.png")
    rel=lambda p:html.escape(os.path.relpath(p,a.output))
    doc='<!doctype html><meta charset="utf-8"><title>Gaussian mapping comparison</title><style>body{max-width:1600px;margin:30px auto;font:16px system-ui}.row{display:flex;gap:8px}.row figure{margin:0;flex:1}img{max-width:100%}pre{white-space:pre-wrap}</style><h1>固定相同位姿：导出高斯地图逐视角对照</h1><p>所有图像均由实际保存的最终全局 PLY 重渲染。留出 RGB 不参与高斯外观初始化或参数训练；位姿和原生 LiDAR 几何复用已完成的 LIVO。深度指标为渲染深度误差，不能代替独立定位和灯柱几何验收。</p>'
    doc+='<pre>'+html.escape(json.dumps(summary,indent=2))+'</pre>'
    for figure in figures:doc+='<img src="'+figure+'">'
    worst=sorted(names,key=lambda n:float(metrics["shared"][n]["ssim_full"]))[:10]
    doc+='<p>最低分视角：'+' · '.join('<a href="#'+n+'">'+n+'</a>' for n in worst)+'</p>'
    for n in names:
        old=metrics['legacy'][n];new=metrics['shared'][n]
        doc+=f'<h3 id="{n}">{n} · SSIM {float(old["ssim_full"]):.3f} → {float(new["ssim_full"]):.3f}</h3><div class="row">'
        for caption,path in [('真实图像',Path(old['folder'])/('reference_'+n+'.png')),('原独立子图拼接',Path(old['folder'])/('merged_'+n+'.png')),('共享表面地图',Path(new['folder'])/('merged_'+n+'.png'))]:
            doc+='<figure><img loading="lazy" src="'+rel(path)+'"><figcaption>'+caption+'</figcaption></figure>'
        doc+='</div>'
    (a.output/"report.html").write_text(doc)
    print(json.dumps(summary,indent=2),flush=True)


if __name__=="__main__":main()
