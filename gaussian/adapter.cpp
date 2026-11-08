#include "gaussian.h"

void Dataset::addFrame(Frame& frame) {
    cv::Mat rgb;
    cv::cvtColor(frame.image, rgb, cv::COLOR_BGR2RGB);
    rgb.convertTo(rgb, CV_32FC3, 1.0 / 255.0);
    auto cam = std::make_shared<Camera>();
    cam->setIntrinsic(rgb.cols, rgb.rows, fx_, fy_, cx_, cy_);
    cam->setPose(frame.R_wc, frame.t_wc);
    cam->image_name_ = frame.name;
    cam->original_image_ = tensor_utils::cvMat2TorchTensor_Float32(rgb, torch::kCPU);
    cam->original_depth_ = tensor_utils::cvMat2TorchTensor_Float32(frame.depth, torch::kCPU);
    if (!frame.sky.empty()) cam->sky_mask_ = tensor_utils::cvMat2TorchTensor_Float32(frame.sky, torch::kCPU);
    if (!frame.dynamic.empty()) cam->dynamic_mask_ = tensor_utils::cvMat2TorchTensor_Float32(frame.dynamic, torch::kCPU);
    is_keyframe_current_ = !frame.validation;
    if (frame.validation) {
        test_cameras_.push_back(cam);
    } else {
        train_cameras_.push_back(cam);
        pointcloud_.assign(frame.points.begin(), frame.points.end());
        pointcolor_.assign(frame.colors.begin(), frame.colors.end());
        pointdepth_ = std::move(frame.depths);
        R_wc_.push_back(frame.R_wc);
        t_wc_.push_back(frame.t_wc);
    }
    ++all_frame_num_;
}
