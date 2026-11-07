#include "gaussian.h"
#include "loss_utils.h"
#include <torch/cuda.h>
#include <iostream>
#include <sstream>
#include <numeric>
namespace fs = std::filesystem;
extern "C" bool gs_read_image(const char*, cv::Mat*);
extern "C" bool gs_write_image(const char*, const cv::Mat*);

namespace {
void readExact(std::ifstream& f, char* dst, size_t n) {
    if (!f.read(dst, n)) throw std::runtime_error("Truncated frame data");
}

void evaluate(const std::shared_ptr<Dataset>& ds, std::shared_ptr<GaussianModel> pc,
              const std::string& output, const std::string& stage, std::ofstream& metrics) {
    torch::NoGradGuard guard;
    auto bg = torch::zeros({3}, torch::TensorOptions().device(torch::kCUDA));
    auto cams = ds->test_cameras_;
    if (cams.empty()) cams.push_back(ds->train_cameras_.back());
    for (const auto& cam : cams) {
        auto pkg = render(cam, pc, bg);
        auto image = std::get<0>(pkg).clamp(0, 1);
        auto depth = std::get<1>(pkg).squeeze();
        auto gt = cam->original_image_.cuda();
        auto gd = cam->original_depth_.cuda();
        auto mask = gd > 0;
        auto color_mask = torch::nn::functional::max_pool2d(mask.to(torch::kFloat).unsqueeze(0).unsqueeze(0),
            torch::nn::functional::MaxPool2dFuncOptions(9).stride(1).padding(4)).squeeze() > 0;
        auto mse = (image - gt).square().masked_select(color_mask.unsqueeze(0).expand_as(gt)).mean();
        double psnr = (-10 * torch::log10(mse.clamp_min(1e-12))).item<double>();
        double full_psnr = loss_utils::psnr_gaussian_splatting(image, gt).item<double>();
        auto im_batch = image.unsqueeze(0), gt_batch = gt.unsqueeze(0);
        double ssim = loss_utils::ssim(im_batch, gt_batch).item<double>();
        auto dm = mask & (depth > 0);
        double mae = dm.any().item<bool>() ? (depth - gd).abs().masked_select(dm).mean().item<double>() : 0;
        auto near_mask = dm & (gd <= 30);
        auto near_error = (depth - gd).abs().masked_select(near_mask);
        double near_mae = near_error.numel() ? near_error.mean().item<double>() : 0;
        double near_median = near_error.numel() ? near_error.median().item<double>() : 0;
        metrics << stage << ',' << (ds->test_cameras_.empty() ? "train" : "heldout") << ',' << cam->image_name_
                << ',' << psnr << ',' << full_psnr << ',' << ssim << ',' << mae << ','
                << dm.sum().item<int>() / double(mask.sum().item<int>()) << ',' << near_mae << ',' << near_median << '\n';
        auto cpu = image.detach().permute({1,2,0}).mul(255).to(torch::kUInt8).cpu().contiguous();
        cv::Mat rgb(cam->image_height_, cam->image_width_, CV_8UC3, cpu.data_ptr<uint8_t>()), bgr;
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
        if (!gs_write_image((output + "/" + stage + "_" + cam->image_name_ + ".png").c_str(), &bgr))
            throw std::runtime_error("Cannot save render");
        auto gt_cpu = gt.detach().permute({1,2,0}).mul(255).to(torch::kUInt8).cpu().contiguous();
        cv::Mat gt_rgb(cam->image_height_, cam->image_width_, CV_8UC3, gt_cpu.data_ptr<uint8_t>());
        cv::cvtColor(gt_rgb, bgr, cv::COLOR_RGB2BGR);
        if (!gs_write_image((output + "/reference_" + cam->image_name_ + ".png").c_str(), &bgr))
            throw std::runtime_error("Cannot save reference");
    }
}
}

int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--version") {
            std::cout << "MY-LIVO Gaussian-LIC adapter 1" << std::endl;
            return 0;
        }
        if (argc != 6 && argc != 8 && argc != 9) throw std::runtime_error("Usage: my_livo_gaussian INPUT OUTPUT ITERATIONS REFINE_PASSES MAX_GAUSSIANS [DEPTH_WEIGHT MAX_SCALE [POSITION_LR]]");
        if (!torch::cuda::is_available()) throw std::runtime_error("CUDA required");
        torch::set_num_threads(1);
        torch::manual_seed(42);
        std::srand(42);
        std::string input = argv[1], output = argv[2];
        if (fs::exists(output + "/point_cloud.ply")) throw std::runtime_error("Output already exists; select a new directory");
        fs::create_directories(output);
        std::ifstream manifest(input + "/frames.txt");
        Params prm;
        // MY-LIVO already supplies accurate measured geometry. Appearance must not move its centres.
        prm.position_lr = argc == 9 ? std::stod(argv[8]) : 0.0;
        if (argc >= 8) prm.lambda_depth = std::stod(argv[6]);
        if (!(manifest >> prm.width >> prm.height >> prm.fx >> prm.fy >> prm.cx >> prm.cy))
            throw std::runtime_error("Invalid camera manifest");
        auto ds = std::make_shared<Dataset>(prm);
        auto pc = std::make_shared<GaussianModel>(prm);
        pc->max_iterations_ = std::stoi(argv[3]);
        pc->max_gaussians_ = std::stoll(argv[5]);
        if (argc >= 8) pc->max_scale_ = std::stod(argv[7]);
        if (pc->max_iterations_ <= 0 || pc->max_gaussians_ < 1000 || std::stoi(argv[4]) < 0)
            throw std::runtime_error("Invalid training budget");
        if (!(prm.lambda_depth > 0 && pc->max_scale_ > 0 && pc->max_scale_ <= 1 && prm.position_lr >= 0))
            throw std::runtime_error("Invalid depth/scale settings");
        auto baseline = std::make_shared<Dataset>(prm);
        std::string name;
        int validation; size_t n;
        while (manifest >> name >> validation >> n) {
            Frame fr;
            fr.name = name; fr.validation = validation;
            for (int r=0; r<3; ++r) for (int c=0; c<3; ++c) manifest >> fr.R_wc(r,c);
            for (int j=0; j<3; ++j) manifest >> fr.t_wc(j);
            if (!manifest || name.find('/') != std::string::npos || name.find("..") != std::string::npos)
                throw std::runtime_error("Invalid frame record");
            if (!gs_read_image((input + "/" + name + ".png").c_str(), &fr.image))
                throw std::runtime_error("Cannot read image");
            if (fr.image.cols != prm.width || fr.image.rows != prm.height) throw std::runtime_error("Invalid image");
            fr.depth = cv::Mat(prm.height, prm.width, CV_32FC1);
            std::ifstream data(input + "/" + name + ".bin", std::ios::binary);
            readExact(data, reinterpret_cast<char*>(fr.depth.data), prm.width * prm.height * sizeof(float));
            for (size_t i=0; i<n; ++i) {
                float row[7]; readExact(data, reinterpret_cast<char*>(row), sizeof(row));
                fr.points.emplace_back(row[0],row[1],row[2]);
                fr.colors.emplace_back(row[3],row[4],row[5]); fr.depths.push_back(row[6]);
            }
            if (!validation && baseline->pointcloud_.size() + n <= size_t(pc->max_gaussians_)) {
                baseline->pointcloud_.insert(baseline->pointcloud_.end(), fr.points.begin(), fr.points.end());
                baseline->pointcolor_.insert(baseline->pointcolor_.end(), fr.colors.begin(), fr.colors.end());
                baseline->pointdepth_.insert(baseline->pointdepth_.end(), fr.depths.begin(), fr.depths.end());
            }
            ds->addFrame(fr);
            if (validation) continue;
            if (!pc->is_init_) {
                if (ds->pointcloud_.empty()) throw std::runtime_error("First training frame has no LiDAR points");
                pc->initialize(ds); pc->trainingSetup(); pc->is_init_ = true;
            } else extend(ds, pc);
            optimize(ds, pc);
            std::cout << " frame=" << name << " gaussians=" << pc->xyz_.size(0) << std::endl;
        }
        if (!manifest.eof() || !pc->is_init_) throw std::runtime_error("Empty or invalid dataset");
        for (int pass=0; pass<std::stoi(argv[4]); ++pass) optimize(ds, pc);
        TORCH_CHECK(torch::isfinite(pc->xyz_).all().item<bool>(), "Non-finite map");
        std::ofstream metrics(output + "/metrics.csv");
        metrics << "stage,split,frame,psnr_lidar_region_db,psnr_full_db,ssim_full,depth_mae_m,depth_coverage,depth_30m_mae_m,depth_30m_median_m\n";
        evaluate(ds, pc, output, "trained", metrics);
        auto untrained = std::make_shared<GaussianModel>(prm);
        if (!baseline->pointcloud_.empty()) {
            untrained->initialize(baseline);
            evaluate(ds, untrained, output, "initial", metrics);
        }
        torch::NoGradGuard guard;
        pc->saveMap(output);
        auto xyz = pc->xyz_.detach().cpu().contiguous();
        auto rgb = (pc->features_dc_.squeeze(1).detach() * C0 + 0.5).clamp(0,1).mul(255).to(torch::kUInt8).cpu().contiguous();
        std::ofstream pcd(output + "/gaussian_centers.pcd", std::ios::binary);
        pcd << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\nWIDTH " << xyz.size(0)
            << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << xyz.size(0) << "\nDATA binary\n";
        auto x = xyz.accessor<float,2>(); auto c = rgb.accessor<uint8_t,2>();
        for (int64_t i=0; i<xyz.size(0); ++i) {
            float p[3] = {x[i][0], x[i][1], x[i][2]};
            uint32_t color = (uint32_t(c[i][0])<<16) | (uint32_t(c[i][1])<<8) | c[i][2];
            pcd.write(reinterpret_cast<char*>(p),12); pcd.write(reinterpret_cast<char*>(&color),4);
        }
        if (!pcd || !metrics) throw std::runtime_error("Failed to save products");
        std::cout << "Gaussian map saved: " << output << " count=" << xyz.size(0) << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Gaussian mapping failed: " << e.what() << std::endl;
        return 1;
    }
}
