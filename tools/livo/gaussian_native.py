"""Read the accepted native surface for Gaussian mapping without exporting PCDs."""
import json
from pathlib import Path
import numpy as np
import cv2

from livo.map_export import read_frames,read_corrections,quat_to_rot,apply_correction


def colour_surface(geometry,tree,frames,header,max_depth=120,sky_masks=True):
    """Colour the full accepted surface, including each scan's LiDAR blind areas."""
    w,h,fx,fy,cx,cy=map(float,header.split());w,h=int(w),int(h)
    radius=max_depth*np.sqrt(1+max(cx/fx,(w-cx)/fx)**2+max(cy/fy,(h-cy)/fy)**2)+1
    best=np.full(len(geometry),np.inf,np.float32);rgb=np.zeros((len(geometry),3),np.uint8);observations=0
    for number,frame in enumerate(frames):
        if frame["validation"]:continue
        T=frame["T"];ids=np.asarray(tree.query_ball_point(T[:3,3],radius,return_sorted=False),np.int64)
        p=(geometry[ids].astype(float)-T[:3,3])@T[:3,:3];z=p[:,2]
        good=(z>.5)&(z<max_depth);ids,p,z=ids[good],p[good],z[good]
        uf=p[:,0]*fx/z+cx;vf=p[:,1]*fy/z+cy;u=np.rint(uf).astype(int);v=np.rint(vf).astype(int)
        good=(u>=0)&(v>=0)&(u<w)&(v<h);ids,z,u,v,uf,vf=[a[good] for a in [ids,z,u,v,uf,vf]]
        order=np.argsort(z,kind="stable");_,first=np.unique((v*w+u)[order],return_index=True);take=order[first]
        ids,z,u,v,uf,vf=[a[take] for a in [ids,z,u,v,uf,vf]]
        depth=np.fromfile(frame["folder"]/(frame["name"]+".bin"),dtype="<f4",count=w*h).reshape(h,w)
        near=cv2.erode(np.where(depth>0,depth,np.inf),np.ones((3,3),np.uint8));measured=near[v,u]
        good=(measured>1e20)|(z<=np.minimum(measured,1e6)*1.02+.4)
        sky_path=frame["folder"]/(frame["name"]+"_sky.png")
        if sky_masks and sky_path.exists():
            sky=cv2.imread(str(sky_path),cv2.IMREAD_GRAYSCALE);good&=(sky[v,u]<128)|(depth[v,u]>0)
        good&=z<best[ids]
        ids,z,uf,vf=[a[good] for a in [ids,z,uf,vf]]
        image=cv2.imread(str(frame["folder"]/(frame["name"]+".png")))
        uf=np.clip(uf,0,w-1);vf=np.clip(vf,0,h-1);u0=uf.astype(int);v0=vf.astype(int);u1=np.minimum(u0+1,w-1);v1=np.minimum(v0+1,h-1)
        dx=(uf-u0)[:,None];dy=(vf-v0)[:,None]
        colour=(1-dy)*((1-dx)*image[v0,u0]+dx*image[v0,u1])+dy*((1-dx)*image[v1,u0]+dx*image[v1,u1])
        rgb[ids]=np.clip(np.rint(colour[:,::-1]),0,255);best[ids]=z;observations+=len(ids)
        if number%40==0:print(f"[native colour] {number+1}/{len(frames)} cameras",flush=True)
    selected=np.flatnonzero(np.isfinite(best))
    points=np.column_stack([geometry[selected],rgb[selected]/255.,best[selected]]).astype(np.float32)
    return points,observations


def native_surface(fusion,voxel):
    fusion=Path(fusion);pcd=fusion/"map/global_map.pcd"
    sources=[fusion/n for n in ["frames.csv","corrections.txt","scans.bin","map_rows.bin"] if (fusion/n).exists()]
    if pcd.exists():sources.append(pcd)
    signature=dict(voxel=voxel,sources=[dict(path=str(p),size=p.stat().st_size,mtime=p.stat().st_mtime_ns) for p in sources])
    cache=fusion/"gaussian_native_cache";cache.mkdir(exist_ok=True)
    name=f"surface_{round(voxel*1000)}mm";path=cache/(name+".npy");metadata=cache/(name+".json")
    if path.exists() and metadata.exists() and json.loads(metadata.read_text())["signature"]==signature:
        info=json.loads(metadata.read_text());return np.load(path,mmap_mode="r"),info
    geometry=np.empty((0,3),np.float32);keys=np.empty(0,np.uint64);scores=np.empty(0,np.float32)
    def merge(points):
        nonlocal geometry,keys,scores
        points=points[np.isfinite(points).all(1)]
        if not len(points):return
        q=np.floor(points.astype(float)/voxel).astype(np.int64)
        score=np.square(points.astype(float)/voxel-q-.5).sum(1).astype(np.float32)
        q=q+(1<<20)
        if np.any(q<0) or np.any(q>=1<<21):raise ValueError("Native map exceeds association grid")
        q=q.astype(np.uint64);k=(q[:,0]<<np.uint64(42))|(q[:,1]<<np.uint64(21))|q[:,2]
        geometry=np.concatenate([geometry,points]);keys=np.concatenate([keys,k]);scores=np.concatenate([scores,score])
        order=np.lexsort((scores,keys));s=keys[order];keep=order[np.r_[True,s[1:]!=s[:-1]]]
        geometry,keys,scores=geometry[keep],keys[keep],scores[keep]
    if pcd.exists():
        header={}
        with pcd.open("rb") as f:
            while True:
                line=f.readline().decode().strip();parts=line.split()
                if parts and not line.startswith("#"):header[parts[0]]=parts[1:]
                if line.startswith("DATA "):offset=f.tell();break
        if header["DATA"]!=["binary"] or header["SIZE"]!=["4"]*len(header["FIELDS"]) or header["TYPE"]!=["F"]*len(header["FIELDS"]):
            raise ValueError("Existing native surface must be a binary float PCD")
        n=int(header["POINTS"][0]);data=np.memmap(pcd,mode="r",dtype="<f4",offset=offset,shape=(n,len(header["FIELDS"])))
        columns=[header["FIELDS"].index(c) for c in ["x","y","z"]]
        for start in range(0,n,1000000):
            merge(np.asarray(data[start:start+1000000,columns]))
            if start%8000000==0:print(f"[native surface] {min(start+1000000,n)}/{n} accepted points; {len(geometry)} anchors",flush=True)
        source="existing accepted native map (read only)"
    else:
        fr,lt,lr=read_frames(fusion);corr=read_corrections(fusion)
        R=quat_to_rot(np.stack([fr[k] for k in ["qx","qy","qz","qw"]],1));t=np.column_stack([fr[k] for k in ["tx","ty","tz"]])
        counts=fr["count"].astype(np.int64);sizes=(counts+7)//8;offsets=8+np.r_[0,np.cumsum(sizes)[:-1]]
        rows=np.memmap(fusion/"map_rows.bin",mode="r",dtype=np.uint8)
        if bytes(rows[:8])!=b"LIVOMSK1" or len(rows)!=offsets[-1]+sizes[-1]:raise ValueError("Invalid native surface row selection")
        scans=np.memmap(fusion/"scans.bin",mode="r",dtype="<f4",offset=8).reshape(-1,5);pending=[]
        for i,count in enumerate(counts):
            offset=(int(fr["offset_bytes"][i])-8)//20;raw=np.asarray(scans[offset:offset+count],float)
            keep=np.unpackbits(rows[offsets[i]:offsets[i]+sizes[i]],bitorder="little")[:count].astype(bool)&np.isfinite(raw).all(1);p=raw[keep]
            world=p[:,:3]@(R[i]@lr).T+R[i]@lt+t[i]
            times=np.clip(fr["begin"][i]+p[:,4]/1000,fr["begin"][i],fr["stamp"][i])
            if len(p):pending.append(apply_correction(world,times,corr).astype(np.float32))
            if (i+1)%100==0 or i==len(counts)-1:
                if pending:merge(np.concatenate(pending));pending.clear()
                if i%1000==999:print(f"[native surface] {i+1}/{len(counts)} scans; {len(geometry)} anchors",flush=True)
        source="accepted archived native rows; no native PCD written"
    temporary=cache/(name+".tmp")
    with temporary.open("wb") as f:np.save(f,geometry)
    temporary.replace(path)
    info=dict(signature=signature,source=source,anchors=len(geometry),voxel_m=voxel)
    metadata.write_text(json.dumps(info,indent=2))
    return np.load(path,mmap_mode="r"),info


def ego_surface_mask(points,fusion,calibration):
    """Reject accepted self returns at their acquisition poses, above the axle.

    The calibrated footprint is inset 1 m in width and 0.5 m in length. The ground stays below the
    axle; only the central vehicle height band is considered. Matching measured
    returns in world coordinates avoids carving an arbitrary world-space box.
    """
    from scipy.spatial import cKDTree
    fusion=Path(fusion).resolve();calibration=Path(calibration).resolve();cal=json.loads(calibration.read_text())
    body=cal.get("body_filter",{});box=body.get("rear_axle_box")
    if not body.get("enabled") or not box:
        return np.ones(len(points),bool),dict(enabled=False,reason="no calibrated vehicle footprint")
    camera=np.array(cal["transforms"]["midrange_camera_to_rear_axle"]["transform_matrix"]).reshape(4,4)
    limits=[box["x_min"]+1.,box["x_max"]-1.,box["y_min"]+.5,box["y_max"]-.5,.1,camera[2,3]+1.]
    files=[calibration]+[fusion/n for n in ["frames.csv","corrections.txt","scans.bin","map_rows.bin"] if (fusion/n).exists()]
    signature=dict(limits=limits,sources=[dict(path=str(p),size=p.stat().st_size,mtime=p.stat().st_mtime_ns) for p in files])
    cache=fusion/"gaussian_native_cache";cache.mkdir(exist_ok=True);path=cache/"ego_returns.npy";metadata=cache/"ego_returns.json"
    if path.exists() and metadata.exists() and json.loads(metadata.read_text())["signature"]==signature:
        ego=np.load(path)
    else:
        fr,lt,lr=read_frames(fusion);corr=read_corrections(fusion)
        R=quat_to_rot(np.stack([fr[k] for k in ["qx","qy","qz","qw"]],1));t=np.column_stack([fr[k] for k in ["tx","ty","tz"]])
        counts=fr["count"].astype(np.int64);sizes=(counts+7)//8;offsets=8+np.r_[0,np.cumsum(sizes)[:-1]]
        rows=np.memmap(fusion/"map_rows.bin",mode="r",dtype=np.uint8) if (fusion/"map_rows.bin").exists() else None
        scans=np.memmap(fusion/"scans.bin",mode="r",dtype="<f4",offset=8).reshape(-1,5);parts=[]
        for i,count in enumerate(counts):
            offset=(int(fr["offset_bytes"][i])-8)//20;raw=np.asarray(scans[offset:offset+count])
            keep=(raw[:,0]>limits[0])&(raw[:,0]<limits[1])&(raw[:,1]>limits[2])&(raw[:,1]<limits[3])&(raw[:,2]>limits[4])&(raw[:,2]<limits[5])
            if rows is not None:keep&=np.unpackbits(rows[offsets[i]:offsets[i]+sizes[i]],bitorder="little")[:count].astype(bool)
            p=raw[keep].astype(float)
            if len(p):
                world=p[:,:3]@(R[i]@lr).T+R[i]@lt+t[i]
                times=np.clip(fr["begin"][i]+p[:,4]/1000,fr["begin"][i],fr["stamp"][i])
                parts.append(apply_correction(world,times,corr).astype(np.float32))
            if i%2000==0:print(f"[ego returns] {i+1}/{len(counts)} scans",flush=True)
        ego=np.concatenate(parts) if parts else np.empty((0,3),np.float32)
        np.save(path,ego);metadata.write_text(json.dumps(dict(signature=signature,returns=len(ego)),indent=2))
    keep=np.ones(len(points),bool)
    if len(ego):
        tree=cKDTree(ego)
        for start in range(0,len(points),100000):
            distance,_=tree.query(points[start:start+100000,:3],distance_upper_bound=.08,workers=2)
            keep[start:start+len(distance)]=~np.isfinite(distance)
    info=dict(enabled=True,body_limits_rear_axle_m=limits,accepted_self_returns=len(ego),matching_radius_m=.08,rejected=int((~keep).sum()))
    print(f"[ego returns] rejected {info['rejected']}/{len(points)} surface seeds",flush=True)
    return keep,info


def traversed_vehicle_mask(points,frames,calibration):
    """A static surface cannot occupy the interior swept by this vehicle.

    Use an inset central body volume, above the axle and below the observed
    cabin height. Ground and roadside structures remain outside this volume.
    Camera poses only provide the already-fixed rigid vehicle trajectory.
    """
    from scipy.spatial import cKDTree
    cal=json.loads(Path(calibration).read_text());body=cal.get("body_filter",{});box=body.get("rear_axle_box")
    if not body.get("enabled") or not box:return np.ones(len(points),bool),dict(enabled=False)
    T_rc=np.array(cal["transforms"]["midrange_camera_to_rear_axle"]["transform_matrix"]).reshape(4,4)
    half=(box["x_max"]-box["x_min"])/2-1.;centre=(box["x_max"]+box["x_min"])/2
    limits=[centre-half,centre+half,max(.5,box["y_min"]+.5),min(T_rc[1,3]-.5,box["y_max"]-.5),.2,T_rc[2,3]+.75]
    if limits[0]>=limits[1] or limits[2]>=limits[3]:return np.ones(len(points),bool),dict(enabled=False,reason="vehicle interior too small")
    inverse=np.linalg.inv(T_rc);tree=cKDTree(points[:,:3]);keep=np.ones(len(points),bool)
    radius=np.sqrt(max(abs(limits[0]),abs(limits[1]))**2+max(abs(limits[2]),abs(limits[3]))**2+max(abs(limits[4]),abs(limits[5]))**2)+.2
    for frame in frames:
        T=frame["T"]@inverse;ids=np.asarray(tree.query_ball_point(T[:3,3],radius,return_sorted=False),np.int64)
        ids=ids[keep[ids]];p=(points[ids,:3].astype(float)-T[:3,3])@T[:3,:3]
        inside=(p[:,0]>limits[0])&(p[:,0]<limits[1])&(p[:,1]>limits[2])&(p[:,1]<limits[3])&(p[:,2]>limits[4])&(p[:,2]<limits[5]);keep[ids[inside]]=False
    info=dict(enabled=True,bounds_rear_axle_m=limits,rejected=int((~keep).sum()),pose_count=len(frames),uses_rgb=False)
    print(f"[vehicle swept interior] rejected {info['rejected']}/{len(points)} surface seeds",flush=True)
    return keep,info
