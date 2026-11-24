// Regenerate intensity chunks/row masks, or independent paired-frame RGB chunks from scan archives.
// Does not rerun or alter the estimator; rejects an existing output directory.
#include "rtk_fusion.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <filesystem>
#include <json/json.h>
#include <map>

int main(int argc, char **argv)
{
  try {
    const bool rgb_only = argc == 4 && std::string(argv[3]) == "--rgb-only";
    if (argc != 3 && !rgb_only)
      throw std::runtime_error("usage: reexport_intensity SOURCE_FUSION NEW_OUTPUT [--rgb-only]");
    const std::filesystem::path source = std::filesystem::absolute(argv[1]);
    const std::filesystem::path output = std::filesystem::absolute(argv[2]);
    if (std::filesystem::exists(output)) throw std::runtime_error("output must be a new directory");
    std::filesystem::create_directories(output);
    std::map<std::string, std::string> params;
    std::ifstream settings(source / "parameters.txt");
    if (!settings) throw std::runtime_error("missing archived parameters.txt");
    std::string setting;
    while (std::getline(settings, setting)) {
      const auto eq = setting.find('=');
      if (eq != std::string::npos) params.emplace(setting.substr(0, eq), setting.substr(eq+1));
    }
    std::ifstream csv(source / "frames.csv");
    std::vector<RtkFusion::Frame> frames;
    M3D rotation = M3D::Identity(); V3D translation = V3D::Zero();
    std::string line;
    while (std::getline(csv,line)) {
      if (line.rfind("# lidar_to_imu_translation ",0)==0) {
        std::istringstream row(line.substr(std::string("# lidar_to_imu_translation ").size()));
        row >> translation.x() >> translation.y() >> translation.z(); continue;
      }
      if (line.rfind("# lidar_to_imu_rotation_row_major ",0)==0) {
        std::istringstream row(line.substr(std::string("# lidar_to_imu_rotation_row_major ").size()));
        for (int r=0;r<3;++r) for(int c=0;c<3;++c) row >> rotation(r,c); continue;
      }
      if (line.empty() || line[0]=='#' || line.rfind("stamp,",0)==0) continue;
      std::replace(line.begin(),line.end(),',',' '); std::istringstream row(line);
      RtkFusion::Frame f; double x,y,z,qx,qy,qz,qw;
      if (!(row >> f.stamp >> f.begin >> f.offset >> f.count >> x >> y >> z >> qx >> qy >> qz >> qw >> f.color_offset >> f.color_count))
        throw std::runtime_error("invalid frame row");
      f.raw=gtsam::Pose3(gtsam::Rot3(Eigen::Quaterniond(qw,qx,qy,qz)),V3D(x,y,z));frames.push_back(f);
    }
    if (frames.empty()) throw std::runtime_error("no archived frames");
    int ros_argc=1; char* ros_argv[]={argv[0],nullptr};rclcpp::init(ros_argc,ros_argv);
    auto node=std::make_shared<rclcpp::Node>("intensity_reexport");
    {
      const auto scratch=output/"context";
      RtkFusion fusion(node,rotation,translation,V3D::Zero(),scratch.string());
      fusion.finish(); // Stop the empty worker before loading the saved final state.
      fusion.archive.close();fusion.source_archive.close();fusion.color_archive.close();fusion.frame_index.close();
      fusion.directory=output;
      fusion.frames=std::move(frames);fusion.observations_enabled=true;
      const auto number = [&](const char *key, double &value) {
        const auto found=params.find(key); if(found!=params.end()) value=std::stod(found->second);
      };
      const auto flag = [&](const char *key, bool &value) {
        const auto found=params.find(key); if(found!=params.end()) value=found->second=="true";
      };
      number("rtk.save_map_max_range", fusion.save_map_max_range);
      number("rtk.save_map_filter", fusion.save_map_filter);
      number("rtk.save_map_priority_ratio", fusion.save_map_priority_ratio);
      number("rtk.save_map_priority_margin_m", fusion.save_map_priority_margin);
      number("rtk.save_map_priority_voxel_m", fusion.save_map_priority_voxel);
      number("rtk.save_map_priority_free_range_m", fusion.save_map_priority_free_range);
      flag("rtk.save_map_priority_enabled", fusion.save_map_priority_enabled);
      flag("rtk.save_map_priority_fill", fusion.save_map_priority_fill);
      if (rgb_only) {
        fusion.color_enabled = true;
        number("rtk.save_rgb_max_range", fusion.save_rgb_max_range);
        number("rtk.save_rgb_voxel", fusion.save_rgb_voxel);
        if (fusion.save_rgb_max_range < 0) fusion.save_rgb_max_range = fusion.save_map_max_range;
        if (!std::filesystem::is_regular_file(source / "colors.bin"))
          throw std::runtime_error("RGB re-export requires paired colours");
      }
      // Restore physical mounts in the archived source order, as LIVMapper does.
      if (std::filesystem::exists(source/"sources.bin")) {
        Json::Value calibration; std::ifstream calib(params.at("multi_lidar.calibration_file"));
        if (!(calib >> calibration)) throw std::runtime_error("cannot read original calibration");
        std::string keys=params.at("multi_lidar.transform_keys");
        for(char &c:keys) if(c=='[' || c==']' || c==',') c=' ';
        std::istringstream names(keys);std::string key;std::vector<V3D> origins;
        while(names>>key) {
          bool found=false;
          for(const auto &entry:calibration["lidar"]) if(entry.isMember(key)) {
            const auto &m=entry[key];origins.emplace_back(m[3].asDouble(),m[7].asDouble(),m[11].asDouble());found=true;break;
          }
          if(!found) throw std::runtime_error("missing physical mount "+key);
        }
        fusion.setLidarOrigins(origins);
      }
      std::ifstream knots(source/"corrections.txt");
      while (std::getline(knots,line)) {
        if(line.empty() || line[0]=='#') continue;
        std::istringstream row(line);RtkFusion::Keyframe k;double x,y,z,qx,qy,qz,qw;
        if(!(row>>k.stamp>>x>>y>>z>>qx>>qy>>qz>>qw)) throw std::runtime_error("invalid correction knot");
        k.raw=gtsam::Pose3();k.optimized=gtsam::Pose3(gtsam::Rot3(Eigen::Quaterniond(qw,qx,qy,qz)),V3D(x,y,z));
        fusion.keyframes.push_back(k);
      }
      if(fusion.keyframes.empty())throw std::runtime_error("no correction knots");
      for(const char* file:{"scans.bin","sources.bin","frames.csv","corrections.txt","parameters.txt","keyframes.csv","colors.bin"}) {
        if (!std::filesystem::exists(source/file)) continue;
        const auto target=output/file;
        if(std::filesystem::exists(target)) throw std::runtime_error("output archive link already exists");
        std::filesystem::create_symlink(source/file,target);
      }
      if (rgb_only) {
        std::cout<<"Re-exporting "<<fusion.frames.size()<<" saved frames using the independent RtkFusion::exportRgbMap"<<std::endl;
        fusion.exportRgbMap();
      } else {
        std::cout<<"Re-exporting "<<fusion.frames.size()<<" saved frames using RtkFusion::exportIntensityMap"<<std::endl;
        fusion.exportIntensityMap();
      }
    }
    rclcpp::shutdown();return 0;
  } catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}
}
