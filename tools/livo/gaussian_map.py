#!/usr/bin/env python3
"""Fixed-pose Gaussian-LIC mapping from a completed MY-LIVO scan archive and its recording.

Uses the existing final per-return SE(3) corrections and calibrated camera time/extrinsics.
No pose is optimized or written. Run with the system ROS Python; CUDA is a separate executable.
"""
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
import cv2
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO / "tools" / "calib"))
from livo.map_export import (resolve_run, read_frames, read_corrections, quat_to_rot,
                             correction_at, apply_correction)
from livo_calib import dump_camera_stream


def decode_rgb(stream, width, height):
    cmd = ["ffmpeg", "-nostdin", "-v", "error", "-threads", "2", "-f", "hevc", "-i", str(stream),
           "-vf", f"scale={width}:{height}", "-pix_fmt", "bgr24", "-f", "rawvideo", "-"]
    proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE)
    try:
        i, size = 0, width * height * 3
        while True:
            buf = proc.stdout.read(size)
            if not buf:
                break
            if len(buf) != size:
                raise RuntimeError("Truncated decoded camera frame")
            yield i, np.frombuffer(buf, np.uint8).reshape(height, width, 3)
            i += 1
        if proc.wait() != 0:
            raise RuntimeError("Camera decoder failed")
    finally:
        proc.stdout.close()
        if proc.poll() is None:
            proc.terminate()
        proc.wait()


def merge_binary(paths, output, kind):
    """Merge same-schema PLY/PCD payloads without loading site maps into memory."""
    headers, offsets, counts = [], [], []
    for path in paths:
        with open(path, "rb") as f:
            lines = []
            while True:
                line = f.readline()
                if not line:
                    raise RuntimeError(f"Invalid {kind}: {path}")
                lines.append(line)
                if (kind == "ply" and line.strip() == b"end_header") or (kind == "pcd" and line.strip() == b"DATA binary"):
                    break
            headers.append(lines); offsets.append(f.tell())
            prefix = b"element vertex " if kind == "ply" else b"POINTS "
            counts.append(next(int(l.split()[-1]) for l in lines if l.startswith(prefix)))
    total = sum(counts)
    schema = lambda h: [l for l in h if l.startswith(b"property ") or l.startswith((b"FIELDS ", b"SIZE ", b"TYPE ", b"COUNT "))]
    if any(schema(h) != schema(headers[0]) for h in headers):
        raise RuntimeError("Submap schemas differ")
    with open(output, "wb") as out:
        for line in headers[0]:
            if line.startswith(b"element vertex "):
                line = f"element vertex {total}\n".encode()
            elif line.startswith((b"WIDTH ", b"POINTS ")):
                line = line.split()[0] + f" {total}\n".encode()
            out.write(line)
        for path, offset in zip(paths, offsets):
            with open(path, "rb") as f:
                f.seek(offset); shutil.copyfileobj(f, out, 4 * 1024 * 1024)
    return total


def run(a):
    if a.backend != "shared" or a.distant_surface:
        raise ValueError("D-stage Gaussian mapping requires the shared native accepted surface without distant supplementation")
    started = time.monotonic()
    fusion = resolve_run(a.run)
    round_dir = fusion.parent.parent.parent
    manifest = json.loads((round_dir / "round.json").read_text()) if (round_dir / "round.json").exists() else {}
    params = dict(line.split("=",1) for line in (fusion / "parameters.txt").read_text().splitlines() if "=" in line)
    bag = Path(a.dataset or manifest.get("dataset", ""))
    calib_path = Path(a.calibration or params.get("calibration.file") or manifest.get("calibration") or "")
    if not bag.is_dir() or not calib_path.is_file():
        raise ValueError("A recording and its calibration are required (--dataset / --calibration)")
    binary = Path(a.binary)
    if not a.prepare_only and not binary.is_file():
        raise ValueError("Build the Gaussian mapper first: bash scripts/build_gaussian.sh")
    if not a.prepare_only:
        version = subprocess.check_output([str(binary.resolve()), "--version"], text=True).strip()
        if version != "MY-LIVO Gaussian-LIC adapter 8":
            raise ValueError("Gaussian executable is outdated; rebuild with bash scripts/build_gaussian.sh")
    output = Path(a.out).resolve() if a.out else fusion / "gaussian"
    output.mkdir(parents=True, exist_ok=True)
    if any(output.iterdir()):
        raise ValueError(f"Select an empty output directory: {output}")
    calib = json.loads(calib_path.read_text())
    camera = calib["camera"]
    scale = a.width / camera["width"]
    width, height = a.width, round(camera["height"] * scale)
    fx, fy, cx, cy = np.array(camera["intrinsics"]) * scale
    cx += (scale-1)/2; cy += (scale-1)/2  # FFmpeg/OpenCV resize pixel-centre convention
    K = np.array([[fx,0,cx],[0,fy,cy],[0,0,1.]])
    mx, my = cv2.initUndistortRectifyMap(K, np.array(camera["distortion"]), None, K, (width,height), cv2.CV_32FC1)
    valid_pixels = cv2.remap(np.ones((height,width), np.uint8), mx, my, cv2.INTER_NEAREST).astype(bool)
    rear_cam = np.array(calib["transforms"]["midrange_camera_to_rear_axle"]["transform_matrix"]).reshape(4,4)
    camera_offset = float(params["effective.img_time_offset_sec"])
    fr, lt, lr = read_frames(fusion)
    if len(fr["stamp"]) < 2 or np.any(np.diff(fr["stamp"]) <= 0):
        raise ValueError("Need a nonempty, strictly increasing native trajectory")
    corr = read_corrections(fusion)
    stamps = fr["stamp"]
    raw_R = quat_to_rot(np.stack([fr[k] for k in ("qx","qy","qz","qw")], 1))
    raw_t = np.stack([fr[k] for k in ("tx","ty","tz")], 1)
    slerp = Slerp(stamps, Rotation.from_matrix(raw_R))
    imu_rear = np.eye(4); imu_rear[:3,:3] = lr; imu_rear[:3,3] = lt
    begin = stamps[0] + a.start
    end = min(stamps[-1], begin + a.duration) if a.duration else stamps[-1]
    if begin >= end:
        raise ValueError("Empty requested mapping interval")
    workspace = output / "input"
    workspace.mkdir()
    archive = np.memmap(fusion / "scans.bin", dtype="<f4", mode="r", offset=8).reshape(-1,5)
    spans = [(begin, end, None)]
    if a.intervals:
        reference = Path(a.reference_poses) if a.reference_poses else fusion / "gaussian" / "camera_poses.json"
        if not reference.is_file():
            raise ValueError("Window comparisons need the complete run's --reference-poses camera_poses.json")
        recorded = json.loads(reference.read_text())
        names = [row["frame"] for row in recorded]
        times = np.array([row["stamp"] for row in recorded], dtype=float)
        if (not len(times) or not np.isfinite(times).all() or np.any(np.diff(times) <= 0) or
                len(names) != len(set(names)) or not all(re.fullmatch(r"frame_[0-9]+", name) for name in names)):
            raise ValueError("Invalid reference frame names or timestamps")
        spans = []
        for item in a.intervals.split(","):
            lo, hi = (float(x) for x in item.split(":"))
            if not (0 <= lo < hi and np.isfinite(hi)):
                raise ValueError("Intervals must be finite nonnegative START:END seconds")
            lo, hi = stamps[0] + lo, min(stamps[-1], stamps[0] + hi)
            if spans and lo < spans[-1][1]:
                raise ValueError("Intervals must be increasing and non-overlapping")
            edge_tolerance = 1e-4
            selected = {row["stamp"]: row for row in recorded if lo-edge_tolerance <= row["stamp"] < hi-edge_tolerance}
            if not selected:
                raise ValueError(f"No reference frames in interval {item}")
            spans.append((lo, hi, selected))

    def selected_images():
        for lo, hi, expected in spans:
            if expected is not None:
                lo, hi = min(expected)-1e-4, max(expected)+1e-4
            image_stamps, stream = dump_camera_stream(bag, workspace, t_from=lo-camera_offset, t_to=hi-camera_offset)
            try:
                chosen, last = {}, -np.inf
                for index, stamp in enumerate(image_stamps + camera_offset):
                    if expected is not None:
                        if stamp in expected:
                            row = expected[stamp]
                            chosen[index] = (stamp, row["validation"], row)
                    elif lo <= stamp <= hi and stamp-last >= a.interval - 1e-4:
                        chosen[index] = (stamp, (len(chosen) % a.submap_frames) % 8 == 4, None)
                        last = stamp
                if expected is not None and len(chosen) != len(expected):
                    raise ValueError("Reference timestamps do not match this midrange recording and time offset")
                for index, image in decode_rgb(stream, width, height):
                    if index in chosen:
                        yield (*chosen[index], image)
            finally:
                stream.unlink(missing_ok=True)

    def project(world,T):
        pc=(world-T[:3,3])@T[:3,:3];z=pc[:,2]
        good=(z>.5)&(z<a.max_depth);world,pc,z=world[good],pc[good],z[good]
        u=np.rint(pc[:,0]*fx/z+cx).astype(int);v=np.rint(pc[:,1]*fy/z+cy).astype(int)
        inside=(u>=0)&(v>=0)&(u<width)&(v<height)
        world,z,u,v=world[inside],z[inside],u[inside],v[inside]
        valid=valid_pixels[v,u];world,z,u,v=world[valid],z[valid],u[valid],v[valid]
        order=np.argsort(z,kind="stable");_,first=np.unique((v*width+u)[order],return_index=True);sel=order[first]
        return world[sel],z[sel],u[sel],v[sel]
    submaps, frame_count, files, poses = [], 0, None, []
    try:
        for t, validation, row, image in selected_images():
            j = int(np.argmin(np.abs(stamps-t)))
            if abs(stamps[j]-t) > 0.12:
                continue  # no pose extrapolation / scanning across a data gap
            R = slerp([t]).as_matrix()[0]
            T = np.eye(4); T[:3,:3] = R
            T[:3,3] = [np.interp(t, stamps, raw_t[:,d]) for d in range(3)]
            T = T @ imu_rear @ rear_cam
            Rc, tc = correction_at(corr, t)
            T[:3,3] = Rc @ T[:3,3] + tc; T[:3,:3] = Rc @ T[:3,:3]
            if row is not None and (row["scan_stamp"] != stamps[j] or
                    not np.allclose(row["T_world_camera"], T, atol=1e-7, rtol=0)):
                raise ValueError(f"Camera pose or scan differs from reference: {row['frame']}")
            # Every return follows the original mapper's deskewed scan-end pose and per-return C(t).
            n = int(fr["count"][j]); offset = (int(fr["offset_bytes"][j])-8)//20
            scan = np.asarray(archive[offset:offset+n], np.float64)
            keep = np.isfinite(scan).all(1) & (np.linalg.norm(scan[:,:3],axis=1) <= a.max_depth)
            scan = scan[keep]
            native = scan[:,:3] @ (raw_R[j] @ lr).T + raw_R[j] @ lt + raw_t[j]
            ptimes = np.clip(fr["begin"][j] + scan[:,4]/1000, fr["begin"][j], stamps[j])
            world = apply_correction(native, ptimes, corr)
            world,z,u,v=project(world,T)
            if len(z) < 100:
                continue
            image = cv2.remap(image, mx, my, cv2.INTER_LINEAR)
            depth = np.zeros((height,width), "<f4"); depth[v,u] = z
            points = np.column_stack([world, image[v,u,::-1]/255., z]).astype("<f4")
            if files is None or frame_count % a.submap_frames == 0:
                if files is not None: files.close()
                sub = workspace / f"submap_{len(submaps):04d}"; sub.mkdir()
                submaps.append(sub)
                files = open(sub / "frames.txt", "w")
                files.write(f"{width} {height} {fx:.12g} {fy:.12g} {cx:.12g} {cy:.12g}\n")
            name = row["frame"] if row is not None else f"frame_{frame_count:06d}"
            if not cv2.imwrite(str(sub / (name+".png")), image): raise RuntimeError("Cannot save image")
            with open(sub / (name+".bin"), "wb") as f:
                depth.tofile(f); points.tofile(f)
            files.write(f"{name} {int(validation)} {len(points)} " + " ".join(f"{x:.16g}" for x in np.r_[T[:3,:3].ravel(),T[:3,3]]) + "\n")
            poses.append({"frame":name, "stamp":t, "scan_stamp":float(stamps[j]), "validation":validation, "T_world_camera":T.tolist()})
            frame_count += 1
            if frame_count % 10 == 0: print(f"[gaussian] prepared {frame_count} frames", flush=True)
    finally:
        if files is not None: files.close()
    if a.intervals and frame_count != sum(len(selected) for _, _, selected in spans):
        raise ValueError("Some reference frames could not be decoded or have insufficient LiDAR coverage")
    if frame_count < 4:
        raise RuntimeError("Too few frames with valid camera/LiDAR coverage")
    (output / "camera_poses.json").write_text(json.dumps(poses, indent=2))
    (output / "preparation.json").write_text(json.dumps(dict(fusion=str(fusion),dataset=str(bag),
        calibration=str(calib_path),camera_offset_sec=camera_offset,frames=frame_count,
        geometry_source="scan proposals; shared mapper associates the global accepted native surface" if a.backend=="shared" else "nearest raw scan",
        camera_start=poses[0]["stamp"],camera_end=poses[-1]["stamp"],options=vars(a)),indent=2))
    if a.prepare_only:
        print(f"[gaussian] prepared {frame_count} frames in {workspace}", flush=True)
        return
    if a.backend == "shared":
        from livo.gaussian_global import refine
        shared = output / "shared"
        info = refine(workspace, shared, binary, voxel=a.voxel, epochs=a.shared_epochs,
                      sweeps=a.shared_sweeps, batch_frames=a.submap_frames,
                      max_active=a.max_active, replay_views=a.replay_views,sky_masks=not a.no_sky_masks,
                      distant_surface=a.distant_surface,training_depth=a.training_depth,replay_reference=a.replay_reference,
                      surface_scale_factor=a.surface_scale_factor,color_weight=a.color_weight)
        for name in ["point_cloud.ply", "gaussian_centers.pcd"]:
            (shared/name).replace(output/name)
        info.update(fusion=str(fusion),dataset=str(bag),calibration=str(calib_path),
                    camera_offset_sec=camera_offset,submaps=1,options=vars(a),
                    elapsed_sec=time.monotonic()-started)
        (output/"map_info.json").write_text(json.dumps(info,indent=2,ensure_ascii=False))
        if not a.keep_inputs: shutil.rmtree(workspace)
        print(json.dumps({k:info[k] for k in ["status","frames","submaps","gaussians","elapsed_sec"]}),flush=True)
        return
    outputs = []
    for sub in submaps:
        dest = output / sub.name
        env = os.environ.copy(); env["OMP_NUM_THREADS"] = "1"
        print(f"[gaussian] optimizing {sub.name}", flush=True)
        subprocess.run([str(binary.resolve()),str(sub),str(dest),str(a.iterations),str(a.refine_passes),str(a.max_gaussians),
                        str(a.depth_weight),str(a.max_scale),str(a.position_lr)], env=env, check=True)
        outputs.append(dest)
    count = merge_binary([p/"point_cloud.ply" for p in outputs], output/"point_cloud.ply", "ply")
    merge_binary([p/"gaussian_centers.pcd" for p in outputs], output/"gaussian_centers.pcd", "pcd")
    import csv
    metrics = []
    for p in outputs:
        metrics.extend(dict(row, submap=p.name) for row in csv.DictReader(open(p/"metrics.csv")))
    info = {"status":"done", "backend":"Gaussian-LIC fixed-pose", "fusion":str(fusion), "dataset":str(bag),
            "calibration":str(calib_path), "camera_offset_sec":camera_offset, "frames":frame_count,
            "submaps":len(outputs), "gaussians":count, "pose_optimization":False,
            "depth_source":"measured LiDAR; no learned depth", "options":vars(a), "metrics":metrics,
            "elapsed_sec":time.monotonic()-started}
    (output / "map_info.json").write_text(json.dumps(info,indent=2,ensure_ascii=False))
    if not a.keep_inputs: shutil.rmtree(workspace)
    print(json.dumps({k:info[k] for k in ("status","frames","submaps","gaussians","elapsed_sec")}),flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("run"); ap.add_argument("--dataset"); ap.add_argument("--calibration"); ap.add_argument("--out")
    ap.add_argument("--binary",default=str(REPO/"build/gaussian/my_livo_gaussian"))
    ap.add_argument("--backend",choices=["shared"],default="shared",
                    help="Shared fixed-pose global map using the D-stage native accepted surface")
    ap.add_argument("--voxel",type=float,default=.08,help="Shared map: measured-anchor association cell in metres")
    ap.add_argument("--shared-epochs",type=int,default=12);ap.add_argument("--shared-sweeps",type=int,default=2)
    ap.add_argument("--replay-views",type=int,default=40)
    ap.add_argument("--replay-reference",help="Shared map: reuse training replay lists from an unpartitioned map_info.json")
    ap.add_argument("--no-sky-masks",action="store_true",help="Disable optional local semantic foreground masks")
    ap.add_argument("--distant-surface",action="store_true",help="Unsupported: D-stage geometry excludes extra distant returns")
    ap.add_argument("--training-depth",type=float,help="Shared map: maximum training camera depth (default: 60 m)")
    ap.add_argument("--surface-scale-factor",type=float,help="Shared map: absolute multiplier of PCA-plane tangent upper bounds (default: 1)")
    ap.add_argument("--color-weight",type=float,default=0.,help="Shared map: soft penalty for DC colors outside [0,1]")
    ap.add_argument("--max-active",type=int,default=3000000,help="Shared map: GPU working-set budget; camera batches split when needed")
    ap.add_argument("--width",type=int,default=640); ap.add_argument("--interval",type=float,default=0.5)
    ap.add_argument("--max-depth",type=float,default=120); ap.add_argument("--submap-frames",type=int,default=40)
    ap.add_argument("--iterations",type=int,default=40); ap.add_argument("--refine-passes",type=int,default=15)
    ap.add_argument("--max-gaussians",type=int,default=600000)
    ap.add_argument("--depth-weight",type=float,default=0.1)
    ap.add_argument("--max-scale",type=float,default=0.5,help="Maximum Gaussian standard deviation in metres")
    ap.add_argument("--position-lr",type=float,default=0,help="Default 0: preserve measured LiDAR centres")
    ap.add_argument("--start",type=float,default=0); ap.add_argument("--duration",type=float)
    ap.add_argument("--intervals",help="Select reference frames in comma-separated half-open START:END seconds after trajectory start")
    ap.add_argument("--reference-poses",help="Complete camera_poses.json for --intervals (default: original fusion/gaussian map)")
    ap.add_argument("--keep-inputs",action="store_true")
    ap.add_argument("--prepare-only",action="store_true",help="Prepare fixed-pose RGB/depth data without CUDA training")
    a = ap.parse_args()
    if not (160<=a.width<=1920 and 0.1<=a.interval<=10 and 4<=a.submap_frames<=200 and a.iterations>0
            and a.refine_passes>=0 and 1000<=a.max_gaussians<=2000000 and a.start>=0 and a.max_depth>1
            and 0<a.depth_weight<=10 and 0<a.max_scale<=1 and 0<=a.position_lr<=0.001 and (a.duration is None or a.duration>0)):
        ap.error("Invalid Gaussian mapping settings")
    if not (.01<=a.voxel<=.3 and a.shared_epochs>0 and a.shared_sweeps>0 and a.replay_views>=0 and a.max_active>=1000
            and (a.training_depth is None or np.isfinite(a.training_depth) and a.training_depth>1)
            and (a.surface_scale_factor is None or 1<=a.surface_scale_factor<=2)
            and 0<=a.color_weight<=1):
        ap.error("Invalid shared mapping settings")
    if a.backend=="shared" and a.position_lr!=0:
        ap.error("Shared surface mapping keeps measured centres fixed (--position-lr 0)")
    if a.distant_surface:
        ap.error("D-stage geometry excludes extra distant returns; --distant-surface is unsupported")
    if a.color_weight and a.backend!="shared":
        ap.error("--color-weight requires the shared backend")
    if (a.intervals and (a.start or a.duration)) or (a.reference_poses and not a.intervals):
        ap.error("Use --intervals with a reference map, separately from --start/--duration")
    run(a)


if __name__ == "__main__":
    main()
