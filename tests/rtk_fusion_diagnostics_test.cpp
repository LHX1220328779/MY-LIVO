#include "backend/rtk_fusion_diagnostics.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using namespace my_livo::backend;

void Require(bool condition, const char *message)
{
  if (!condition) throw std::runtime_error(message);
}

std::vector<std::string> Split(const std::string &line)
{
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) fields.push_back(field);
  if (!line.empty() && line.back() == ',') fields.emplace_back();
  return fields;
}

void TestDiagnosticRows()
{
  const std::filesystem::path directory =
      std::filesystem::temp_directory_path() /
      "my_livo_rtk_fusion_diagnostics_test";
  std::filesystem::remove_all(directory);
  std::filesystem::create_directories(directory);
  const auto keyframe_path = directory / "keyframes.csv";
  const auto alignment_path = directory / "alignment.csv";
  const auto source_keyframe_path = directory / "source_keyframes.csv";

  {
    RtkFusionDiagnostics::Options options;
    options.keyframe_csv_path = keyframe_path.string();
    options.initial_alignment_csv_path = alignment_path.string();
    RtkFusionDiagnostics diagnostics(options);

    RtkFusionDiagnostics::InitialAlignmentRecord alignment;
    alignment.initialization_timestamp = 10.0;
    alignment.first_sample_timestamp = 9.0;
    alignment.last_sample_timestamp = 10.0;
    alignment.sample_count = 101;
    alignment.T_mine_lio_initial = Pose3d(
        Eigen::Quaterniond::Identity(), Eigen::Vector3d(1.0, 2.0, 3.0));
    alignment.position_rms_m = 0.01;
    alignment.position_max_m = 0.02;
    alignment.orientation_max_deg = 0.03;
    diagnostics.RecordInitialAlignment(alignment);

    KeyframeCloud::Ptr cloud(new KeyframeCloud());
    cloud->push_back(KeyframePoint());
    const Pose3d T_lio(Eigen::Quaterniond::Identity(),
                       Eigen::Vector3d(2.0, 3.0, 4.0));
    Keyframe keyframe(0, 12.0, T_lio, cloud, Matrix6d::Identity(), 1U);
    RtkObservation observation;
    observation.timestamp = 12.0;
    observation.lower_timestamp = 11.99;
    observation.upper_timestamp = 12.01;
    observation.interpolation_alpha = 0.5;
    observation.lower_ins_pos_mode = 4;
    observation.upper_ins_pos_mode = 4;
    observation.health = 1.0;
    observation.raw_position = Eigen::Vector3d(0.5, 1.0, 1.5);
    observation.lever_arm_correction = Eigen::Vector3d(0.5, 1.0, 1.5);
    observation.position = Eigen::Vector3d(1.0, 2.0, 3.0);
    observation.orientation = Eigen::Quaterniond::Identity();
    observation.position_covariance.diagonal() << 0.09, 0.09, 0.25;
    RtkObservationQuery query;
    query.observation = observation;
    query.reason = RtkObservationRejectReason::kAccepted;
    const Pose3d before(Eigen::Quaterniond::Identity(),
                        Eigen::Vector3d(2.0, 3.0, 4.0));
    const Pose3d after(Eigen::Quaterniond::Identity(),
                       Eigen::Vector3d(1.5, 2.5, 3.5));
    diagnostics.RecordKeyframe(
        keyframe, query, "accepted", true, before, after);
  }

  std::ifstream keyframe_stream(keyframe_path);
  std::string header;
  std::string row;
  std::getline(keyframe_stream, header);
  std::getline(keyframe_stream, row);
  const auto header_fields = Split(header);
  const auto fields = Split(row);
  Require(header_fields.size() == 65, "diagnostic header width changed");
  Require(fields.size() == header_fields.size(),
          "diagnostic row width differs from header");
  Require(fields[5] == "accepted" && fields[6] == "1",
          "factor decision was not recorded");
  Require(std::abs(std::stod(fields[51]) - std::sqrt(3.0)) < 1.0e-12,
          "LIO/RTK corrected residual norm is wrong");
  Require(std::abs(std::stod(fields[63]) - std::sqrt(0.75)) < 1.0e-12,
          "graph update magnitude is wrong");

  std::ifstream alignment_stream(alignment_path);
  std::getline(alignment_stream, header);
  std::getline(alignment_stream, row);
  Require(Split(header).size() == Split(row).size(),
          "initial alignment row width differs from header");
  std::ofstream source_keyframe_stream(source_keyframe_path);
  source_keyframe_stream << "id,timestamp\n0,12\n";
}
}  // namespace

int main()
{
  try
  {
    TestDiagnosticRows();
    std::cout << "rtk_fusion_diagnostics_test passed\n";
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << "rtk_fusion_diagnostics_test failed: " << error.what()
              << '\n';
    return 1;
  }
}
