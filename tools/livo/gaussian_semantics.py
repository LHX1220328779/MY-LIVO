#!/usr/bin/env python3
"""Conservative sky masks for Gaussian foreground training; inference stays local."""
import argparse
import json
import os
from pathlib import Path
import time

os.environ.setdefault("HF_HUB_DISABLE_XET","1")
os.environ.setdefault("OPENBLAS_NUM_THREADS","1")
import cv2
import numpy as np
from PIL import Image
import torch
import torch.nn.functional as F
from transformers import SegformerImageProcessor, SegformerForSemanticSegmentation

MODEL="nvidia/segformer-b0-finetuned-ade-512-512"
REVISION="489d5cd81a0b59fab9b7ea758d3548ebe99677da"


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("prepared",type=Path);ap.add_argument("--batch-size",type=int,default=4)
    ap.add_argument("--dynamic-only",action="store_true")
    a=ap.parse_args();started=time.monotonic();torch.set_num_threads(2)
    frames=[]
    for manifest in sorted(a.prepared.glob("submap_*/frames.txt")):
        header,*lines=manifest.read_text().splitlines();w,h=map(int,header.split()[:2])
        for line in lines:
            fields=line.split();frames.append((manifest.parent,fields[0],bool(int(fields[1])),w,h))
    if not frames:raise ValueError("No prepared frames")
    processor=SegformerImageProcessor.from_pretrained(MODEL,revision=REVISION)
    model=SegformerForSemanticSegmentation.from_pretrained(MODEL,revision=REVISION).cuda().eval()
    if model.config.id2label[2]!="sky":raise ValueError("Unexpected semantic label map")
    dynamic_ids=[i for i,label in model.config.id2label.items() if label in {"person","car","bus","truck","van","minibike","bicycle"}]
    if not {12,20,80,83,102}.issubset(dynamic_ids):raise ValueError("Unexpected dynamic semantic labels")
    records=[];sky_colors=[]
    for start in range(0,len(frames),a.batch_size):
        batch=frames[start:start+a.batch_size]
        images=[Image.open(folder/(name+".png")).convert("RGB") for folder,name,_,_,_ in batch]
        inputs=processor(images=images,return_tensors="pt").to("cuda")
        with torch.inference_mode(),torch.autocast("cuda",dtype=torch.float16):
            logits=model(**inputs).logits.float()
            all_probability=logits.softmax(1)
            probability=torch.stack([all_probability[:,2],all_probability[:,dynamic_ids].sum(1)],dim=1)
            probability=F.interpolate(probability,size=(h,w),mode="bilinear",align_corners=False).cpu().numpy()
        for image,prob,(folder,name,validation,w,h) in zip(images,probability,batch):
            rgb=np.asarray(image);gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY)
            dx=cv2.Sobel(gray,cv2.CV_32F,1,0,ksize=3);dy=cv2.Sobel(gray,cv2.CV_32F,0,1,ksize=3)
            edge=(np.hypot(dx,dy)>40).astype(np.uint8)
            edge=cv2.dilate(edge,np.ones((5,5),np.uint8))>0
            dynamic=cv2.erode((prob[1]>.6).astype(np.uint8),np.ones((3,3),np.uint8))
            if not cv2.imwrite(str(folder/(name+"_dynamic.png")),dynamic*255):raise ValueError("Cannot save dynamic mask")
            mask=(prob[0]>.9)&~edge
            mask=cv2.erode(mask.astype(np.uint8),np.ones((3,3),np.uint8))
            if not a.dynamic_only and not cv2.imwrite(str(folder/(name+"_sky.png")),mask*255):raise ValueError("Cannot save sky mask")
            records.append(dict(frame=name,validation=validation,sky_fraction=float(mask.mean()),dynamic_fraction=float(dynamic.mean())))
            if not validation and mask.sum()>100:
                sky_colors.append(np.median(rgb[mask>0],axis=0)/255.)
        if (start//a.batch_size)%20==0:print(f"[sky] {min(start+a.batch_size,len(frames))}/{len(frames)}",flush=True)
    info=dict(model=MODEL,model_revision=getattr(model.config,"_commit_hash",None),confidence=.9,
              protected="image edges; mapper separately protects measured LiDAR pixels",frames=records,
              background_rgb=np.median(sky_colors,axis=0).tolist() if sky_colors else [0.,0.,0.],
              background_fit_training_frames=len(sky_colors),elapsed_sec=time.monotonic()-started)
    info.update(dynamic_classes={i:model.config.id2label[i] for i in dynamic_ids},dynamic_confidence=.6)
    (a.prepared.parent/("dynamics.json" if a.dynamic_only else "semantics.json")).write_text(json.dumps(info,indent=2))
    print(f"[sky] complete: {len(frames)} masks in {info['elapsed_sec']:.1f}s",flush=True)


if __name__=="__main__":main()
