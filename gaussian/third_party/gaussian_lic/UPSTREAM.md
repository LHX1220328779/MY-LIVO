Source: /home/project/Gaussian-LIC
Repository: https://github.com/APRIL-ZJU/Gaussian-LIC
Revision: 793b9bcb5dfdf8d114f3626b010c7fdb9df04740

GaussianModel, incremental alpha-based insertion, sparse Adam, fused SSIM and CUDA rasterizer are retained. ROS ingestion, TensorRT depth completion, LPIPS and the upstream odometry are excluded. The adapter uses calibrated, rectified RGB and measured LiDAR depth with fixed MY-LIVO poses. Changes include batched initialization, finite/empty-mask guards, configurable iterations/budget, corrected SSIM gradient arity, and a standalone runner.

Upstream headers declare GPL-3.0-or-later for Gaussian-LIC and retain separate notices for embedded third-party code (including the Inria research/evaluation license in simple-knn). See source headers.

2026-11-08 shared surface adapter changes: fixed camera/pixel-centre convention; normalized depth backward alpha factor; explicit background compositing; masked RGB/SSIM/relative-depth loss; fixed surface centres/normals with bounded scales; random training backgrounds and persistent SH3 appearance. The upstream odometry remains excluded. Forward-only replay skips the bucket/sample buffers needed solely by backward; training retains them. Saved PLY attributes are staged from CPU separately to avoid a second complete GPU table during loading. Replay checkpoints retain completed camera batches after interruption.

Optional local sky and vehicle/person inference: `nvidia/segformer-b0-finetuned-ade-512-512`, revision `489d5cd81a0b59fab9b7ea758d3548ebe99677da`. Model source: https://huggingface.co/nvidia/segformer-b0-finetuned-ade-512-512 . Weights are downloaded to the standard local model cache, not vendored into this repository. It supplies conservative sky and transient-object masks, not depth or camera poses.

The model card links to NVIDIA’s SegFormer research/evaluation (non-commercial) license: https://github.com/NVlabs/SegFormer/blob/master/LICENSE .
