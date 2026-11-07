Source: /home/project/Gaussian-LIC
Repository: https://github.com/APRIL-ZJU/Gaussian-LIC
Revision: 793b9bcb5dfdf8d114f3626b010c7fdb9df04740

GaussianModel, incremental alpha-based insertion, sparse Adam, fused SSIM and CUDA rasterizer are retained. ROS ingestion, TensorRT depth completion, LPIPS and the upstream odometry are excluded. The adapter uses calibrated, rectified RGB and measured LiDAR depth with fixed MY-LIVO poses. Changes include batched initialization, finite/empty-mask guards, configurable iterations/budget, corrected SSIM gradient arity, and a standalone runner.

Upstream headers declare GPL-3.0-or-later for Gaussian-LIC and retain separate notices for embedded third-party code (including the Inria research/evaluation license in simple-knn). See source headers.
