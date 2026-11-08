#pragma once
// Fixed-pose mapping adapter. No ROS, estimator, learned depth or pose optimization.
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <vector>

struct Params {
    int width = 640, height = 360;
    double fx = 1, fy = 1, cx = 0, cy = 0;
    int select_every_k_frame = 1, patch_size = 10;
    bool depth_completion = false;
    double max_depth = 60;
    int sh_degree = 3;
    bool white_background = false, random_background = false;
    bool convert_SHs_python = false, compute_cov3D_python = false;
    double lambda_erank = 0, scaling_scale = 1;
    double position_lr = 0.00016, feature_lr = 0.005, opacity_lr = 0.05;
    double scaling_lr = 0.005, rotation_lr = 0.001, lambda_dssim = 0.2;
    bool optimize_depth = true, iteration_decay = false, apply_exposure = false;
    double lambda_depth = 0.1, exposure_lr = 0.001;
    int skybox_points_num = 0, skybox_radius = 1000;
};

struct Frame {
    cv::Mat image, depth, sky, dynamic;
    Eigen::Matrix3d R_wc;
    Eigen::Vector3d t_wc;
    std::vector<Eigen::Vector3d> points, colors;
    std::vector<float> depths;
    bool validation = false;
    std::string name;
};
