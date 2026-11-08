#!/usr/bin/env python3
"""Refine one fixed-pose LiDAR-anchored Gaussian map in GPU-sized working sets.

Rows have permanent global identities. Working sets read/write those rows; they
are never concatenated into independent maps. Heldout RGB never seeds appearance
or trains parameters. The pre-existing native LiDAR surface is a geometry input.
"""
import argparse
import csv
from collections import OrderedDict
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF","expandable_segments:True")
import numpy as np
import cv2
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

PROPS = (["x", "y", "z"] + [f"f_dc_{i}" for i in range(3)] +
         [f"f_rest_{i}" for i in range(45)] + ["opacity"] +
         [f"scale_{i}" for i in range(3)] + [f"rot_{i}" for i in range(4)])
C0 = 0.28209479177387814
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(REPO/"tools"))


def read_ply(path, mode="r"):
    props = []
    with open(path, "rb") as f:
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f"Invalid PLY: {path}")
            if line.startswith(b"element vertex "):
                n = int(line.split()[-1])
            if line.startswith(b"property float "):
                props.append(line.split()[-1].decode())
            if line.strip() == b"end_header":
                offset = f.tell()
                break
    if props != PROPS or path.stat().st_size != offset + n * 236:
        raise ValueError(f"Unexpected Gaussian PLY schema/length: {path}")
    return np.memmap(path, dtype="<f4", mode=mode, offset=offset, shape=(n, 59))


def write_ply(path, rows):
    with open(path, "wb") as f:
        f.write((f"ply\nformat binary_little_endian 1.0\nelement vertex {len(rows)}\n" +
                 "".join(f"property float {p}\n" for p in PROPS) + "end_header\n").encode())
        for start in range(0, len(rows), 100000):
            np.asarray(rows[start:start+100000], dtype="<f4").tofile(f)


def frames_from(root):
    frames, header = [], None
    for manifest in sorted(root.glob("submap_*/frames.txt")):
        lines = manifest.read_text().splitlines()
        if header is not None and lines[0] != header:
            raise ValueError("Mixed intrinsics in prepared frames")
        header = lines[0]
        for line in lines[1:]:
            fields = line.split()
            T = np.eye(4)
            T[:3, :3] = np.array(fields[3:12], float).reshape(3, 3)
            T[:3, 3] = np.array(fields[12:15], float)
            frames.append(dict(name=fields[0], validation=bool(int(fields[1])), n=int(fields[2]),
                               T=T, line=line, folder=manifest.parent))
    if not frames:
        raise ValueError(f"No prepared frames: {root}")
    return header, frames


def static_visibility(points, frames, header, protected, tree, sky_masks=True,calibration=None):
    """Remove foreground contradicted by conservative ray cones and clear sky.

    RGB edges and thin measured structures protect silhouettes. Missing depth
    outside clear sky remains unknown. Large contradictions can reject an
    unsupported transient even when sparse LiDAR supplies only one clear ray.
    """
    w,h,fx,fy,cx,cy=map(float,header.split());w,h=int(w),int(h)
    support=np.zeros(len(points),np.uint16);free=np.zeros(len(points),np.uint16);strong=np.zeros(len(points),np.uint16)
    dynamic_hits=np.zeros(len(points),np.uint16)
    vehicle=json.loads(Path(calibration).read_text()) if calibration else None
    rear_camera=np.array(vehicle["transforms"]["midrange_camera_to_rear_axle"]["transform_matrix"]).reshape(4,4) if vehicle else None
    radius=50*np.sqrt(1+max(cx/fx,(w-cx)/fx)**2+max(cy/fy,(h-cy)/fy)**2)+1
    for number,frame in enumerate(frames):
        if frame["validation"]:continue
        T=frame["T"];nearby=np.asarray(tree.query_ball_point(T[:3,3],radius,return_sorted=False),dtype=np.int64)
        if not len(nearby):continue
        depth=np.fromfile(frame["folder"]/(frame["name"]+".bin"),dtype="<f4",count=w*h).reshape(h,w)
        near=cv2.erode(np.where(depth>0,depth,np.inf),np.ones((3,3),np.uint8))
        gray=cv2.imread(str(frame["folder"]/(frame["name"]+".png")),cv2.IMREAD_GRAYSCALE)
        edges=cv2.dilate((np.hypot(cv2.Sobel(gray,cv2.CV_32F,1,0),cv2.Sobel(gray,cv2.CV_32F,0,1))>40).astype(np.uint8),np.ones((5,5),np.uint8))>0
        sky_path=frame["folder"]/(frame["name"]+"_sky.png")
        sky=cv2.imread(str(sky_path),cv2.IMREAD_GRAYSCALE) if sky_masks and sky_path.is_file() else None
        dynamic_path=frame["folder"]/(frame["name"]+"_dynamic.png")
        dynamic=cv2.imread(str(dynamic_path),cv2.IMREAD_GRAYSCALE) if sky_masks and dynamic_path.is_file() and vehicle else None
        if dynamic is not None:dynamic=cv2.dilate(dynamic,np.ones((3,3),np.uint8))
        for start in range(0,len(nearby),100000):
            selected=nearby[start:start+100000];p=(points[selected,:3].astype(float)-T[:3,3])@T[:3,:3]
            ids=np.flatnonzero((p[:,2]>.5)&(p[:,2]<50));z=p[ids,2]
            u=np.rint(p[ids,0]*fx/z+cx).astype(int);v=np.rint(p[ids,1]*fy/z+cy).astype(int)
            inside=(u>=0)&(u<w)&(v>=0)&(v<h);ids,z,u,v=ids[inside],z[inside],u[inside],v[inside]
            measured=near[v,u];known=measured<1e20;measured=np.minimum(measured,1e6)
            transient=dynamic[v,u]>128 if dynamic is not None else np.zeros(len(ids),bool)
            support[selected[ids]]+=known&(np.abs(z-measured)<.15+.01*measured)&~transient
            if dynamic is not None:
                height=p[ids]@rear_camera[2,:3]+rear_camera[2,3]
                dynamic_hits[selected[ids]]+=known&transient&(np.abs(z-measured)<2.+.1*measured)&(height>float(vehicle.get("ground_height",-1))+.5)
            clear=~edges[v,u]
            contradicted=known&(z<measured-.5-.02*measured)&clear
            decisive=known&(z<measured-np.maximum(3.,.1*measured))&clear
            if sky is not None:contradicted|=(depth[v,u]<=0)&(sky[v,u]>128)
            free[selected[ids]]+=contradicted;strong[selected[ids]]+=decisive
        if number%200==0:print(f"[visibility] {number+1}/{len(frames)} cameras",flush=True)
    semantic_reject=(dynamic_hits>=1)&(dynamic_hits>2*support)
    remove=(((free>=3)&(free>3*support))|((strong>=1)&(support==0))|semantic_reject)&~protected
    print(f"[shared] measured visibility: rejected {int(remove.sum())}/{len(points)} inconsistent foreground anchors",flush=True)
    return ~remove


def seed_map(root, output, voxel, static_filter, sky_masks=True):
    header, frames = frames_from(root)
    w, h, fx, fy, _, _ = map(float, header.split())
    source=root.parent/"preparation.json"
    if not source.exists():source=frames[0]["folder"].resolve().parent.parent/"preparation.json"
    native_geometry=None;native_info=None;native_tree=None
    if source.exists():
        from livo.gaussian_native import native_surface
        preparation=json.loads(source.read_text())
        native_geometry,native_info=native_surface(preparation["fusion"],voxel)
        native_tree=cKDTree(native_geometry)
        print(f"[shared] native surface association: {len(native_geometry)} accepted anchors",flush=True)
    source_candidates=sum(f["n"] for f in frames if not f["validation"])
    native_supported=0
    if native_geometry is not None:
        from livo.gaussian_native import colour_surface
        points,native_supported=colour_surface(native_geometry,native_tree,frames,header,
            preparation.get("options",{}).get("max_depth",120),sky_masks)
        del native_tree,native_geometry
        print(f"[shared] {len(points)} visible accepted surface anchors",flush=True)
    else:
        # Keep the closest measured return in each cell, not a displaced centroid.
        points = np.empty((0, 7), np.float32)
        keys = np.empty(0, np.uint64)
        pending = []
        for index, frame in enumerate(frames):
            if not frame["validation"]:
                p = np.fromfile(frame["folder"]/(frame["name"]+".bin"), dtype="<f4",
                                offset=int(w*h)*4).reshape(-1, 7)
                if len(p) != frame["n"]:
                    raise ValueError("Prepared point count mismatch")
                pending.append(p)
            if (index+1) % 80 == 0 or index == len(frames)-1:
                block = np.concatenate(pending) if pending else np.empty((0,7),np.float32)
                q = np.floor(block[:, :3].astype(float)/voxel).astype(np.int64) + (1 << 20)
                if np.any(q < 0) or np.any(q >= 1 << 21):
                    raise ValueError("Map coordinates exceed voxel key range")
                q = q.astype(np.uint64)
                k = (q[:, 0] << np.uint64(42)) | (q[:, 1] << np.uint64(21)) | q[:, 2]
                points = np.concatenate([points, block])
                keys = np.concatenate([keys, k])
                order = np.lexsort((points[:, 6], keys))
                sorted_keys = keys[order]
                keep = order[np.r_[True, sorted_keys[1:] != sorted_keys[:-1]]]
                points, keys = points[keep], keys[keep]
                pending.clear()
                print(f"[shared] seed {index+1}/{len(frames)} frames: {len(points)} unique measured anchors", flush=True)
    if len(points) < 32:
        raise ValueError("Insufficient training LiDAR anchors")
    model = np.lib.format.open_memmap(output/"state.npy", mode="w+", dtype="<f4", shape=(len(points),59))
    bounds = np.lib.format.open_memmap(output/"bounds.npy", mode="w+", dtype="<f4", shape=(len(points),6))
    model[:] = 0
    model[:, :3] = points[:, :3]
    model[:, 3:6] = (points[:, 3:6]-.5)/C0
    model[:, 51] = np.log(.7/.3)
    tree = cKDTree(points[:, :3])
    planar_count = 0
    for start in range(0, len(points), 20000):
        stop = min(start+20000, len(points))
        dist, nn = tree.query(points[start:stop, :3], k=32, workers=2)
        neighbors = points[nn, :3].astype(float)
        delta = neighbors-neighbors.mean(1, keepdims=True)
        cov = np.einsum("nki,nkj->nij", delta, delta)/32
        eig, vectors = np.linalg.eigh(cov)
        planar = ((eig[:, 0] < .08*np.maximum(eig[:, 1], 1e-9)) & (eig[:, 1] > .08*eig[:, 2]) &
                  (dist[:, -1]<np.maximum(.6,4*points[start:stop,6]/((fx+fy)/2))))
        tangent = np.clip(np.maximum(dist[:, 3]*.65, points[start:stop, 6]/((fx+fy)/2)*.7), .012, .18)
        scales = np.repeat(tangent[:, None], 3, axis=1)
        normal = np.clip(np.sqrt(np.maximum(eig[:, 0], 0)), .005, .02)
        scales[planar, :2] = tangent[planar, None]
        scales[planar, 2] = normal[planar]
        linear = (eig[:, 1] < .06*np.maximum(eig[:, 2], 1e-9)) & (eig[:, 2] > 1e-5)
        compact=~(planar|linear)
        footprint=np.clip(points[start:stop,6]/((fx+fy)/2),.008,.18)
        scales[compact]=np.minimum(scales[compact],footprint[compact,None])
        # Thin poles/edges have a reliable long axis, but no reliable plane normal.
        scales[linear, 1:] = np.clip(np.sqrt(np.maximum(eig[linear, 1],0)), .005, .025)[:, None]
        orientation = vectors[:, :, [2, 1, 0]].copy()
        orientation[:, :, 0] *= np.linalg.det(orientation)[:, None]
        quat = Rotation.from_matrix(orientation).as_quat()[:, [3, 0, 1, 2]]
        quat[~(planar|linear)] = [1, 0, 0, 0]
        model[start:stop, 52:55] = np.log(scales)
        model[start:stop, 55:59] = quat
        lower = np.maximum(scales*.4, .002)
        upper = np.minimum(scales*2.5, .3)
        upper[compact]=np.minimum(upper[compact],1.5*footprint[compact,None])
        upper[planar, 2] = np.minimum(normal[planar]*1.25, .025)
        upper[linear, 1:] = np.minimum(scales[linear, 1:]*1.25,.03)
        bounds[start:stop, :3] = np.log(lower)
        bounds[start:stop, 3:] = np.log(upper)
        planar_count += int(planar.sum())
    model.flush(); bounds.flush()
    before_filter=len(model)
    if static_filter:
        protected=(np.abs(bounds[:,4]-bounds[:,5])<1e-6)&(bounds[:,3]>bounds[:,4]+np.log(1.5))
        visibility_frames=frames
        if native_info and (source.parent/"input").is_dir() and (source.parent/"input").resolve()!=root:
            _,visibility_frames=frames_from(source.parent/"input")
        keep=static_visibility(points,visibility_frames,header,protected,tree,sky_masks,preparation["calibration"] if native_info else None)
        if native_info:
            from livo.gaussian_native import ego_surface_mask,traversed_vehicle_mask
            ego_keep,ego_info=ego_surface_mask(points,preparation["fusion"],preparation["calibration"])
            swept_keep,swept_info=traversed_vehicle_mask(points,visibility_frames,preparation["calibration"])
            keep&=ego_keep&swept_keep
            native_info=dict(native_info,ego_filter=ego_info,swept_vehicle_filter=swept_info)
        filtered=np.lib.format.open_memmap(output/"filtered_state.npy",mode="w+",dtype="<f4",shape=(int(keep.sum()),59))
        filtered_bounds=np.lib.format.open_memmap(output/"filtered_bounds.npy",mode="w+",dtype="<f4",shape=(int(keep.sum()),6))
        offset=0
        for start in range(0,len(model),100000):
            mask=keep[start:start+100000];n=int(mask.sum())
            filtered[offset:offset+n]=model[start:start+100000][mask]
            filtered_bounds[offset:offset+n]=bounds[start:start+100000][mask]
            offset+=n
        filtered.flush();filtered_bounds.flush()
        del model,bounds,filtered,filtered_bounds
        (output/"filtered_state.npy").replace(output/"state.npy")
        (output/"filtered_bounds.npy").replace(output/"bounds.npy")
        model=np.load(output/"state.npy",mmap_mode="r+")
        bounds=np.load(output/"bounds.npy",mmap_mode="r")
    (output/"seed_info.json").write_text(json.dumps(dict(anchors=len(model), planar_anchors_before_visibility=planar_count,
        static_visibility_filter=static_filter,visibility_rejected=before_filter-len(model),
        native_surface=native_info,source_candidates=source_candidates,native_supported_observations=native_supported,
        voxel_m=voxel, centres="accepted native surface anchors; never optimized" if native_info else "measured training returns; never optimized",
        heldout_rgb_seeds=0,native_geometry_all_scans=bool(native_info)), indent=2))
    return header, frames, model, bounds


def visibility_index(xyz,header=None):
    cells,inverse=np.unique(np.floor(xyz/32).astype(np.int32),axis=0,return_inverse=True)
    return dict(cells=cells,inverse=inverse,header=header,cache=OrderedDict())


def visible_rows(xyz, frames, header, index=None, margin=2.,far=None):
    # Centres and poses stay fixed. Reuse packed per-camera frusta between
    # temporal/replay batches rather than reprojecting the route for each tile.
    index=visibility_index(xyz,header) if index is None else index
    base=index["header"] or header;cache=index["cache"]
    n=len(xyz);packed=np.zeros((n+7)//8,np.uint8)
    cells=index["cells"];centres=(cells.astype(float)+.5)*32
    def project(ids,T,intrinsics):
        w,h,fx,fy,cx,cy=map(float,intrinsics.split())
        keep=np.zeros(len(ids),bool)
        for start in range(0,len(ids),100000):
            selected=ids[start:start+100000]
            p=(np.asarray(xyz[selected],float)-T[:3,3])@T[:3,:3];z=np.maximum(p[:,2],0)
            keep[start:start+len(selected)]=((p[:,2]>-margin)&
                ((p[:,2]<far+margin) if far is not None else True)&(p[:,0]>-(cx+3)/fx*z-margin)&
                (p[:,0]<(w-cx+3)/fx*z+margin)&(p[:,1]>-(cy+3)/fy*z-margin)&(p[:,1]<(h-cy+3)/fy*z+margin))
        return ids[keep]
    for frame in frames:
        key=(frame["name"],margin,far);T=frame["T"]
        if key not in cache:
            w,h,fx,fy,cx,cy=map(float,base.split())
            p=(centres-T[:3,3])@T[:3,:3];z=np.maximum(p[:,2],0)
            cell_margin=30*(1+max(cx/fx,(w-cx)/fx,cy/fy,(h-cy)/fy))
            coarse=((p[:,2]>-30)&((p[:,2]<far+32) if far is not None else True)&(p[:,0]>-(cx+3)/fx*z-cell_margin)&
                (p[:,0]<(w-cx+3)/fx*z+cell_margin)&(p[:,1]>-(cy+3)/fy*z-cell_margin)&(p[:,1]<(h-cy+3)/fy*z+cell_margin))
            ids=project(np.flatnonzero(coarse[index["inverse"]]),T,base)
            mask=np.zeros(n,bool);mask[ids]=True;cache[key]=np.packbits(mask)
            while len(cache)>max(1,(256<<20)//len(packed)):cache.popitem(last=False)
        cache.move_to_end(key);bits=cache[key]
        if header!=base:
            ids=project(np.flatnonzero(np.unpackbits(bits,count=n)),T,header)
            mask=np.zeros(n,bool);mask[ids]=True;bits=np.packbits(mask)
        packed|=bits
    return np.flatnonzero(np.unpackbits(packed,count=n))


def crop_header(header, crop):
    if crop is None:return header
    _,_,fx,fy,cx,cy=map(float,header.split());x,y,w,h=crop
    return f"{w} {h} {fx:.12g} {fy:.12g} {cx-x:.12g} {cy-y:.12g}"


def prepare_batch(folder, header, frames, crop=None, sky_masks=True,background=None):
    folder.mkdir(parents=True)
    lines=[]
    for frame in frames:
        if crop is None:
            lines.append(frame["line"])
            for ext in [".png", ".bin"]:
                (folder/(frame["name"]+ext)).symlink_to((frame["folder"]/(frame["name"]+ext)).resolve())
            for suffix in ["_sky.png","_dynamic.png"]:
                semantic=frame["folder"]/(frame["name"]+suffix)
                if sky_masks and semantic.is_file():(folder/semantic.name).symlink_to(semantic.resolve())
        else:
            w0,h0=map(int,header.split()[:2]);x,y,w,h=crop
            image=cv2.imread(str(frame["folder"]/(frame["name"]+".png")))
            depth=np.fromfile(frame["folder"]/(frame["name"]+".bin"),dtype="<f4",count=w0*h0).reshape(h0,w0)
            if image is None or not cv2.imwrite(str(folder/(frame["name"]+".png")),image[y:y+h,x:x+w]):
                raise ValueError("Cannot prepare training image crop")
            depth[y:y+h,x:x+w].tofile(folder/(frame["name"]+".bin"))
            fields=frame["line"].split();fields[2]="0";lines.append(" ".join(fields))
            for suffix in ["_sky.png","_dynamic.png"]:
                semantic=frame["folder"]/(frame["name"]+suffix)
                if sky_masks and semantic.is_file():
                    mask=cv2.imread(str(semantic),cv2.IMREAD_GRAYSCALE)
                    if not cv2.imwrite(str(folder/semantic.name),mask[y:y+h,x:x+w]):raise ValueError("Cannot crop semantic mask")
    (folder/"frames.txt").write_text(crop_header(header,crop)+"\n"+"\n".join(lines)+"\n")
    if background is not None:(folder/"background.txt").write_text(" ".join(str(x) for x in background)+"\n")


def refine(root, output, binary, voxel=.08, epochs=12, sweeps=2, batch_frames=40,
           max_active=3000000, surface=True, evaluate=True, replay_views=40, static_filter=True,sky_masks=True,initial_state=None):
    started = time.monotonic()
    output = Path(output).resolve(); root = Path(root).resolve(); binary = Path(binary).resolve()
    output.mkdir(parents=True, exist_ok=False)
    _,source_frames=frames_from(root)
    missing_sky=any(not (f["folder"]/(f["name"]+"_sky.png")).is_file() for f in source_frames)
    missing_dynamic=any(not (f["folder"]/(f["name"]+"_dynamic.png")).is_file() for f in source_frames)
    if sky_masks and (missing_sky or missing_dynamic):
        python=os.environ.get("GAUSSIAN_PYTHON","/opt/venvs/my-livo-gaussian/bin/python")
        subprocess.run([python,str(REPO/"tools/livo/gaussian_semantics.py"),str(root)]+(["--dynamic-only"] if not missing_sky else []),check=True)
    semantic_info=root.parent/"semantics.json"
    if not semantic_info.exists():semantic_info=source_frames[0]["folder"].resolve().parent.parent/"semantics.json"
    semantics=json.loads(semantic_info.read_text()) if semantic_info.exists() and sky_masks else {}
    background=semantics.get("background_rgb",[0.,0.,0.])
    if initial_state is None:
        header, frames, model, bounds = seed_map(root, output, voxel, static_filter, sky_masks)
    else:
        # Reuse expensive native geometry/colour preparation during real-data iterations.
        initial_state=Path(initial_state).resolve()
        for name in ["state.npy","bounds.npy","seed_info.json"]:
            shutil.copyfile(initial_state/name,output/name)
        seed_info=json.loads((output/"seed_info.json").read_text())
        seed_info.pop("validation_seeds",None)
        seed_info.update(heldout_rgb_seeds=0,native_geometry_all_scans=bool(seed_info.get("native_surface")))
        (output/"seed_info.json").write_text(json.dumps(seed_info,indent=2))
        header,frames=frames_from(root)
        model=np.load(output/"state.npy",mmap_mode="r+");bounds=np.load(output/"bounds.npy",mmap_mode="r")
        if model.ndim!=2 or model.shape[1]!=59 or bounds.shape!=(len(model),6):
            raise ValueError("Initial state has incompatible Gaussian/scale rows")
        print(f"[shared] reused prepared state: {len(model)} anchors from {initial_state}",flush=True)
    if (root.parent/"camera_poses.json").is_file():
        shutil.copyfile(root.parent/"camera_poses.json",output/"camera_poses.json")
    spatial_index = visibility_index(model[:, :3],header)
    camera_positions = np.array([f["T"][:3,3] for f in frames])
    work = output/"work"; work.mkdir()
    history = []
    for sweep in range(sweeps):
        groups = [(frames[k:k+batch_frames],None) for k in range(0,len(frames),batch_frames)]
        if sweep % 2: groups.reverse()
        group_index = 0
        while groups:
            group,crop = groups.pop(0)
            if not any(not f["validation"] for f in group):
                continue
            # Rehearse all-time nearby observations, including the other side of a
            # temporal batch boundary and later revisits. Never train on heldout RGB.
            ids = {f["name"] for f in group}
            group_positions = np.array([f["T"][:3,3] for f in group])
            center = group_positions.mean(0)
            radius = np.linalg.norm(group_positions-center,axis=1).max()+40
            distance = np.linalg.norm(camera_positions-center,axis=1)
            candidates = [f for k,f in enumerate(frames) if distance[k]<radius and not f["validation"] and f["name"] not in ids]
            current_header=crop_header(header,crop)
            base_active=visible_rows(model[:,:3],group,current_header,spatial_index,far=60.)
            # Distance alone admits cameras looking down a different road. Select
            # replay using common observed surfaces, including opposite-direction
            # revisits when they actually see those surfaces.
            sample=base_active[::max(1,len(base_active)//2048)]
            targets=np.asarray(model[sample,:3],float)
            near=cKDTree(group_positions).query(targets)[0]<80 if len(targets) else np.zeros(0,bool)
            targets=targets[near]
            w,h,fx,fy,cx,cy=map(float,current_header.split())
            overlapping=[]
            for f in candidates:
                T=f["T"];p=(targets-T[:3,3])@T[:3,:3];z=p[:,2]
                seen=(z>.5)&(z<100)&(p[:,0]>-cx/fx*z)&(p[:,0]<(w-cx)/fx*z)&(p[:,1]>-cy/fy*z)&(p[:,1]<(h-cy)/fy*z)
                if seen.sum()>=max(8,.1*len(targets)):overlapping.append(f)
            candidates=overlapping
            if len(candidates)>replay_views:
                candidates = [candidates[k] for k in np.linspace(0,len(candidates)-1,replay_views,dtype=int)]
            elif replay_views == 0:
                candidates = []
            cameras = group+candidates
            active = visible_rows(model[:, :3], cameras, current_header, spatial_index,far=60.) if candidates else base_active
            if len(active)>max_active:
                w0,h0=map(int,header.split()[:2]);x,y,w,h=crop or (0,0,w0,h0)
                if min(w,h)>64:
                    # Keep multi-time observations together; partition pixels,
                    # with a 12 px SSIM/splat halo, instead of dropping geometry.
                    crops=[]
                    for xa,xb in [(x,x+w//2),(x+w//2,x+w)]:
                        for ya,yb in [(y,y+h//2),(y+h//2,y+h)]:
                            lo_x=max(x,xa-12);lo_y=max(y,ya-12)
                            hi_x=min(x+w,xb+12);hi_y=min(y+h,yb+12)
                            crops.append((group,(lo_x,lo_y,hi_x-lo_x,hi_y-lo_y)))
                    groups[0:0]=crops
                elif len(group)>1:
                    middle=len(group)//2;groups[0:0]=[(group[:middle],crop),(group[middle:],crop)]
                else:
                    raise RuntimeError(f"Working set {len(active)} exceeds GPU budget even after image partitioning")
                continue
            if not len(active):continue
            folder = work/f"s{sweep:02d}_b{group_index:04d}"
            prepare_batch(folder, header, cameras, crop,sky_masks,background)
            write_ply(folder/"input.ply", model[active])
            np.asarray(bounds[active],dtype="<f4").tofile(folder/"scale_bounds.bin")
            dest = folder/"optimized"
            print(f"[shared] sweep {sweep+1}/{sweeps} group {group_index}: {len(active)} anchors, {len(group)} views + {len(candidates)} replay, crop={crop}",flush=True)
            with open(output/"training.log","a") as log:
                subprocess.run([str(binary),"--refine-map",str(folder),str(folder/"input.ply"),str(dest),
                                str(epochs),str(int(surface))],stdout=log,stderr=subprocess.STDOUT,check=True)
            trained = read_ply(dest/"point_cloud.ply")
            if len(trained)!=len(active) or not np.array_equal(trained[:,:3],model[active,:3]):
                raise ValueError("Shared-map row identities or fixed centres changed")
            if not np.isfinite(trained).all():
                raise ValueError("Non-finite shared map")
            model[active] = trained
            model.flush()
            del trained
            optimization=json.loads((dest/"training_info.json").read_text()) if (dest/"training_info.json").exists() else {}
            history.append(dict(sweep=sweep,group=group_index,active=len(active),crop=crop,
                                views=[f["name"] for f in cameras],optimization=optimization))
            shutil.rmtree(folder)
            group_index += 1
        write_ply(output/"point_cloud.ply",model)
    save_centres(output/"gaussian_centers.pcd",model)
    info=dict(status="trained_pending_replay",backend="Gaussian-LIC shared fixed-pose surface map",frames=len(frames),
              prepared_input=str(root),initial_state=str(initial_state) if initial_state else None,
              intrinsics=dict(zip(["width","height","fx","fy","cx","cy"],map(float,header.split()))),
              train_frames=sum(not f["validation"] for f in frames),heldout_frames=sum(f["validation"] for f in frames),
              gaussians=len(model),pose_optimization=False,depth_source="accepted native LiDAR surface; measured per-camera depth",
              surface_constraints=surface,voxel_m=voxel,sweeps=sweeps,epochs_per_batch=epochs,
              static_visibility_filter=json.loads((output/"seed_info.json").read_text())["static_visibility_filter"],
              sky_masks=sky_masks,dynamic_masks=sky_masks,
              background_rgb=background,
              batch_frames=batch_frames,max_active=max_active,training_depth_limit_m=60.,elapsed_sec=time.monotonic()-started,
              replay_views=replay_views,metrics=[],working_sets=history)
    if (root.parent/"preparation.json").is_file():
        info["preparation"]=json.loads((root.parent/"preparation.json").read_text())
    (output/"map_info.json").write_text(json.dumps(info,indent=2))
    if evaluate:
        info["metrics"]=replay_saved_map(root,output/"point_cloud.ply",output/"views",binary,background)
    shutil.rmtree(work)
    info.update(status="done",elapsed_sec=time.monotonic()-started)
    (output/"map_info.json").write_text(json.dumps(info,indent=2))
    print(json.dumps({k:info[k] for k in ["status","frames","gaussians","elapsed_sec"]}),flush=True)
    return info


def replay_saved_map(root,model_path,output,binary,background,batch_frames=2):
    """Replay the exported map, retaining completed camera batches after a failure."""
    header,frames=frames_from(Path(root));heldout=[f for f in frames if f["validation"]]
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    table=read_ply(Path(model_path));index=None;metrics=[];work=output.parent/"replay_work"
    for start in range(0,len(heldout),batch_frames):
        group=heldout[start:start+batch_frames];dest=output/f"batch_{start:04d}"
        csv_path=dest/"metrics.csv";rows=list(csv.DictReader(csv_path.open())) if csv_path.is_file() else []
        ready=sorted(r["frame"] for r in rows)==sorted(f["name"] for f in group)
        if not ready:
            if dest.exists():shutil.rmtree(dest)
            if work.exists():shutil.rmtree(work)
            prepare_batch(work,header,group,background=background)
            if index is None:index=visibility_index(table[:,:3],header)
            active=visible_rows(table[:,:3],group,header,index)
            write_ply(work/"input.ply",table[active])
            print(f"[shared] final-map replay {start+1}/{len(heldout)}: {len(active)} anchors",flush=True)
            with open(output.parent/"replay.log","a") as log:
                subprocess.run([str(binary),"--render",str(work),str(work/"input.ply"),str(dest)],stdout=log,stderr=subprocess.STDOUT,check=True)
            rows=list(csv.DictReader(csv_path.open()));shutil.rmtree(work)
        metrics.extend(rows)
    return metrics


def save_centres(path, model):
    with open(path,"wb") as f:
        f.write(("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\n"+
                 f"WIDTH {len(model)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(model)}\nDATA binary\n").encode())
        for start in range(0,len(model),100000):
            block=model[start:start+100000]
            rows=np.empty(len(block),dtype=[("xyz","<f4",3),("rgb","<u4")])
            rows["xyz"]=block[:,:3]
            c=np.uint32(np.clip(block[:,3:6]*C0+.5,0,1)*255)
            rows["rgb"]=(c[:,0]<<16)|(c[:,1]<<8)|c[:,2]
            rows.tofile(f)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("prepared",type=Path);ap.add_argument("output",type=Path)
    ap.add_argument("--binary",type=Path,default=REPO/"build/gaussian/my_livo_gaussian")
    ap.add_argument("--voxel",type=float,default=.08);ap.add_argument("--epochs",type=int,default=12)
    ap.add_argument("--sweeps",type=int,default=2);ap.add_argument("--batch-frames",type=int,default=40)
    ap.add_argument("--max-active",type=int,default=3000000)
    ap.add_argument("--replay-views",type=int,default=40)
    ap.add_argument("--resume-replay",action="store_true",help="Finish heldout replay from an exported training checkpoint")
    ap.add_argument("--initial-state",type=Path,help="Reuse state.npy/bounds.npy/seed_info.json from a prior native preparation or refinement")
    ap.add_argument("--no-surface",action="store_true");ap.add_argument("--no-evaluate",action="store_true")
    ap.add_argument("--no-static-filter",action="store_true")
    ap.add_argument("--no-sky-masks",action="store_true")
    a=ap.parse_args()
    if a.resume_replay:
        output=a.output.resolve();info_path=output/"map_info.json";info=json.loads(info_path.read_text());started=time.monotonic()
        info["metrics"]=replay_saved_map(a.prepared,output/"point_cloud.ply",output/"views",a.binary.resolve(),info["background_rgb"])
        info.update(status="done",elapsed_sec=info.get("elapsed_sec",0)+time.monotonic()-started,
                    replay_adapter=subprocess.check_output([str(a.binary.resolve()),"--version"],text=True).strip())
        temporary=output/"map_info.tmp";temporary.write_text(json.dumps(info,indent=2));temporary.replace(info_path)
        if (output/"work").exists():shutil.rmtree(output/"work")
        print(json.dumps({k:info[k] for k in ["status","frames","gaussians","elapsed_sec"]}),flush=True)
        return
    if not (.01<=a.voxel<=.3 and a.epochs>=1 and a.sweeps>=1 and a.batch_frames>=2 and a.max_active>=1000 and a.replay_views>=0):
        ap.error("Invalid shared mapping budget")
    refine(a.prepared,a.output,a.binary,a.voxel,a.epochs,a.sweeps,a.batch_frames,a.max_active,not a.no_surface,not a.no_evaluate,a.replay_views,not a.no_static_filter,not a.no_sky_masks,a.initial_state)


if __name__=="__main__":main()
