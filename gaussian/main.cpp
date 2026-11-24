#include "gaussian.h"
#include "loss_utils.h"
#include <torch/cuda.h>
#include <iostream>
#include <sstream>
#include <numeric>
#include <limits>
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
    auto bg = pc->bg_.defined()?pc->bg_:torch::zeros({3}, torch::TensorOptions().device(torch::kCUDA));
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
        auto alpha_cpu = (1.0f-std::get<2>(pkg)).squeeze().clamp(0,1).mul(65535).to(torch::kInt32).cpu().contiguous();
        cv::Mat alpha32(cam->image_height_,cam->image_width_,CV_32SC1,alpha_cpu.data_ptr<int32_t>()), alpha16;
        alpha32.convertTo(alpha16,CV_16U);
        if (!gs_write_image((output + "/" + stage + "_" + cam->image_name_ + "_alpha.png").c_str(), &alpha16))
            throw std::runtime_error("Cannot save render alpha");
        auto depth_cpu=depth.to(torch::kFloat32).cpu().contiguous();
        std::ofstream depth_file(output + "/" + stage + "_" + cam->image_name_ + "_depth.bin",std::ios::binary);
        depth_file.write(reinterpret_cast<const char*>(depth_cpu.data_ptr<float>()),depth_cpu.numel()*sizeof(float));
        if (!depth_file) throw std::runtime_error("Cannot save render depth");
        auto gt_cpu = gt.detach().permute({1,2,0}).mul(255).to(torch::kUInt8).cpu().contiguous();
        cv::Mat gt_rgb(cam->image_height_, cam->image_width_, CV_8UC3, gt_cpu.data_ptr<uint8_t>());
        cv::cvtColor(gt_rgb, bgr, cv::COLOR_RGB2BGR);
        if (!gs_write_image((output + "/reference_" + cam->image_name_ + ".png").c_str(), &bgr))
            throw std::runtime_error("Cannot save reference");
    }
}

// Replay actual saved Gaussians for full-map/overlap inspection, with fixed recorded cameras.
// The caller may select a conservative camera frustum on CPU to fit a long map in GPU memory.
std::shared_ptr<GaussianModel> loadMap(const std::string& model, const Params& prm) {
    torch::NoGradGuard guard;
    std::ifstream ply(model, std::ios::binary);
    std::string line; int64_t count = 0; std::vector<std::string> props;
    while (std::getline(ply, line) && line != "end_header") {
        if (line.rfind("element vertex ", 0) == 0) count = std::stoll(line.substr(15));
        if (line.rfind("property float ", 0) == 0) props.push_back(line.substr(15));
    }
    std::vector<std::string> expected = {"x", "y", "z", "f_dc_0", "f_dc_1", "f_dc_2"};
    for (int i=0; i<45; ++i) expected.push_back("f_rest_"+std::to_string(i));
    expected.push_back("opacity");
    for (int i=0; i<3; ++i) expected.push_back("scale_"+std::to_string(i));
    for (int i=0; i<4; ++i) expected.push_back("rot_"+std::to_string(i));
    if (!ply || count <= 0 || props != expected) throw std::runtime_error("Replay requires this mapper's binary SH3 PLY schema");
    std::vector<float> values(count*59);
    readExact(ply, reinterpret_cast<char*>(values.data()), values.size()*sizeof(float));
    auto table = torch::from_blob(values.data(), {count,59}, torch::kFloat32);
    auto pc = std::make_shared<GaussianModel>(prm);
    pc->xyz_ = table.slice(1,0,3).contiguous().cuda();
    pc->features_dc_ = table.slice(1,3,6).reshape({count,1,3}).contiguous().cuda();
    pc->features_rest_ = table.slice(1,6,51).reshape({count,3,15}).transpose(1,2).contiguous().cuda();
    pc->opacity_ = table.slice(1,51,52).contiguous().cuda();
    pc->scaling_ = table.slice(1,52,55).contiguous().cuda();
    pc->rotation_ = table.slice(1,55,59).contiguous().cuda();
    std::vector<float>().swap(values);
    return pc;
}

void refineSavedMap(const std::string& input, const std::string& model, const std::string& output,
                    int epochs, bool surface) {
    if (fs::exists(output)) throw std::runtime_error("Choose a new map output directory");
    Params prm; std::ifstream manifest(input+"/frames.txt");
    if (!(manifest >> prm.width >> prm.height >> prm.fx >> prm.fy >> prm.cx >> prm.cy)) throw std::runtime_error("Invalid camera manifest");
    auto pc = loadMap(model, prm);
    std::ifstream background(input+"/background.txt");
    if (background) {
        std::vector<float> rgb(3);
        for (auto& c:rgb) if (!(background>>c) || !std::isfinite(c) || c<0 || c>1) throw std::runtime_error("Invalid background RGB");
        pc->bg_=torch::tensor(rgb,torch::kFloat32).cuda();
    }
    auto ds = std::make_shared<Dataset>(prm);
    std::string name; int validation; size_t n;
    while (manifest >> name >> validation >> n) {
        Frame fr; fr.name=name; fr.validation=epochs == 0 || validation;
        for (int r=0;r<3;++r) for (int c=0;c<3;++c) manifest >> fr.R_wc(r,c);
        for (int j=0;j<3;++j) manifest >> fr.t_wc(j);
        if (!manifest || name.find('/') != std::string::npos || name.find("..") != std::string::npos) throw std::runtime_error("Invalid camera record");
        if (epochs>0 && validation) continue;  // heldout RGB/depth is read only during final replay
        if (!gs_read_image((input+"/"+name+".png").c_str(), &fr.image) || fr.image.cols!=prm.width || fr.image.rows!=prm.height) throw std::runtime_error("Invalid replay RGB");
        fr.depth=cv::Mat(prm.height,prm.width,CV_32FC1);
        std::ifstream data(input+"/"+name+".bin", std::ios::binary);
        readExact(data,reinterpret_cast<char*>(fr.depth.data),prm.width*prm.height*sizeof(float));
        if (epochs>0 && fs::exists(input+"/"+name+"_sky.png")) {
            cv::Mat sky;
            if (!gs_read_image((input+"/"+name+"_sky.png").c_str(),&sky)) throw std::runtime_error("Cannot read sky mask");
            cv::cvtColor(sky,fr.sky,cv::COLOR_BGR2GRAY);
            fr.sky.convertTo(fr.sky,CV_32F,1.0/255.0);
        }
        if (epochs>0 && fs::exists(input+"/"+name+"_dynamic.png")) {
            cv::Mat dynamic;
            if (!gs_read_image((input+"/"+name+"_dynamic.png").c_str(),&dynamic)) throw std::runtime_error("Cannot read dynamic mask");
            cv::cvtColor(dynamic,fr.dynamic,cv::COLOR_BGR2GRAY);
            fr.dynamic.convertTo(fr.dynamic,CV_32F,1.0/255.0);
        }
        ds->addFrame(fr);
    }
    if (!manifest.eof() || (ds->test_cameras_.empty() && ds->train_cameras_.empty())) throw std::runtime_error("No valid map cameras");
    fs::create_directories(output);
    if (epochs > 0) {
        if (ds->train_cameras_.empty()) throw std::runtime_error("Refinement needs training cameras");
        pc->position_lr_ = 0;
        pc->xyz_.set_requires_grad(false);
        pc->features_dc_.set_requires_grad(true);
        pc->features_rest_.set_requires_grad(true);
        pc->opacity_.set_requires_grad(true);
        pc->scaling_.set_requires_grad(true);
        pc->rotation_.set_requires_grad(!surface);
        pc->surface_mode_ = surface;
        pc->random_background_ = surface;
        pc->max_scale_ = 0.3;
        pc->sh_degree_ = 3;
        pc->lambda_depth_ = surface ? 0.2 : 0.05;
        pc->feature_lr_ = 0.0025;
        pc->scaling_lr_ = 0.002;
        pc->opacity_lr_ = 0.01;
        if (surface) {
            std::vector<float> bounds(pc->xyz_.size(0)*6);
            std::ifstream bf(input+"/scale_bounds.bin", std::ios::binary);
            readExact(bf,reinterpret_cast<char*>(bounds.data()),bounds.size()*sizeof(float));
            auto bt=torch::from_blob(bounds.data(),{pc->xyz_.size(0),6},torch::kFloat32).cuda();
            pc->scale_lower_=bt.slice(1,0,3).contiguous();
            pc->scale_upper_=bt.slice(1,3,6).contiguous();
            TORCH_CHECK(torch::isfinite(bt).all().item<bool>() &&
                        (pc->scale_lower_<=pc->scale_upper_).all().item<bool>(),
                        "Invalid per-surface scale bounds");
            torch::NoGradGuard guard;
            pc->scaling_.copy_(torch::minimum(torch::maximum(pc->scaling_,pc->scale_lower_),pc->scale_upper_));
        }
        double color_weight=0;
        std::ifstream regularization(input+"/appearance_regularization.txt");
        if (regularization) {
            if (!(regularization>>color_weight) || !std::isfinite(color_weight) ||
                    color_weight<0 || color_weight>1 || !surface)
                throw std::runtime_error("Invalid surface appearance regularization");
        }
        pc->trainingSetup();
        pc->max_iterations_ = ds->train_cameras_.size();
        const bool fresh_appearance=pc->features_rest_.detach().abs().max().item<float>()<1e-10f;
        double best_loss=std::numeric_limits<double>::infinity();
        int stale=0,epochs_run=0;
        for(int pass=0;pass<epochs;++pass) {
            // Warm up a fresh map without repeatedly disabling coefficients
            // already shared with neighbouring/revisited working sets.
            pc->sh_degree_=fresh_appearance?std::min(3,1+pass/std::max(1,epochs/3)):3;
            optimize(ds,pc,color_weight);
            epochs_run=pass+1;
            if (pc->last_loss_<best_loss*.995) { best_loss=pc->last_loss_;stale=0; } else ++stale;
            std::cout << " shared refinement epoch=" << epochs_run << '/' << epochs << " gaussians=" << pc->xyz_.size(0) << " loss=" << pc->last_loss_ << std::endl;
            const int warmup=fresh_appearance?std::max(8,2*(epochs/3)+3):8;
            if (surface && epochs_run>=warmup && stale>=3) break;
        }
        std::ofstream(output+"/training_info.json") << "{\"epochs\":" << epochs_run << ",\"loss\":" << pc->last_loss_
                << ",\"color_weight\":" << color_weight << "}\n";
        torch::NoGradGuard guard;
        pc->saveMap(output);
    }
    std::ofstream metrics(output+"/metrics.csv");
    metrics << "stage,split,frame,psnr_lidar_region_db,psnr_full_db,ssim_full,depth_mae_m,depth_coverage,depth_30m_mae_m,depth_30m_median_m\n";
    if (epochs == 0 || !ds->test_cameras_.empty()) evaluate(ds,pc,output,"merged",metrics);
    std::cout << "Saved map: gaussians=" << pc->xyz_.size(0) << " train=" << ds->train_cameras_.size() << " heldout=" << ds->test_cameras_.size() << std::endl;
}
}

int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--version") {
            std::cout << "MY-LIVO Gaussian-LIC adapter 8" << std::endl;
            return 0;
        }
        const bool replay = argc == 5 && std::string(argv[1]) == "--render";
        const bool refine = argc == 7 && std::string(argv[1]) == "--refine-map";
        if (!replay && !refine && argc != 6 && argc != 8 && argc != 9) throw std::runtime_error("Usage: my_livo_gaussian INPUT OUTPUT ITERATIONS REFINE_PASSES MAX_GAUSSIANS [DEPTH_WEIGHT MAX_SCALE [POSITION_LR]]; or --render INPUT MODEL_PLY OUTPUT; or --refine-map INPUT MODEL_PLY OUTPUT EPOCHS SURFACE");
        if (!torch::cuda::is_available()) throw std::runtime_error("CUDA required");
        torch::set_num_threads(1);
        torch::manual_seed(42);
        std::srand(42);
        if (replay || refine) {
            const int epochs=refine?std::stoi(argv[5]):0;
            if (refine && epochs<1) throw std::runtime_error("Positive refinement epoch count required");
            refineSavedMap(argv[2],argv[3],argv[4],epochs,refine && std::stoi(argv[6]));
            return 0;
        }
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
