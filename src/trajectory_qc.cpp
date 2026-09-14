/// @brief Headless trajectory quality check for GLIM dump directories.
///
/// Screens a map (or a batch of maps) for SLAM failures before any downstream use.
/// A pose jump teleports the trajectory by more than the platform can physically
/// move - typical of feature-poor geometry (long straight culverts, tunnels,
/// corridors) where LiDAR odometry slides along the degenerate axis.
///
/// Exit code: 0 = all passed, 1 = at least one map failed, 2 = usage/IO error.

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <glim/mapping/sub_map.hpp>
#include <glim/viewer/editor/trajectory_qc.hpp>

namespace {

/// @brief Load all submaps of a dump directory (000000, 000001, ...).
std::vector<glim::SubMap::ConstPtr> load_submaps(const std::string& path) {
  namespace fs = boost::filesystem;
  std::vector<glim::SubMap::ConstPtr> submaps;

  for (int i = 0;; i++) {
    std::ostringstream ss;
    ss << path << "/" << std::setw(6) << std::setfill('0') << i;
    if (!fs::exists(ss.str() + "/data.txt")) {
      break;
    }
    auto submap = glim::SubMap::load(ss.str());
    if (!submap) {
      spdlog::warn("failed to load submap {}", ss.str());
      continue;
    }
    submaps.push_back(submap);
  }
  return submaps;
}

std::string basename_of(const std::string& path) {
  return boost::filesystem::path(path).filename().string();
}

}  // namespace

int main(int argc, char** argv) {
  using namespace boost::program_options;

  options_description desc("GLIM trajectory quality check (headless)");
  desc.add_options()                                                                                             //
    ("help,h", "produce help message")                                                                           //
    ("map_path", value<std::vector<std::string>>()->multitoken(), "Map dump directory (repeatable for batch)")    //
    ("report,r", value<std::string>()->default_value(""), "Write report to this path (single map only)")          //
    ("report_dir", value<std::string>()->default_value(""), "Write <name>_qc.txt into this directory (batch)")    //
    ("max_speed", value<double>()->default_value(1.5), "[m/s] speed above this is suspicious")                    //
    ("max_accel", value<double>()->default_value(6.0), "[m/s^2] acceleration above the platform limit")           //
    ("max_imu_mismatch", value<double>()->default_value(0.8), "[m/s] |v_pose - v_imu|, secondary signal only")     //
    ("max_angular_rate", value<double>()->default_value(90.0), "[deg/s] angular rate above this is suspicious")   //
    ("max_gap", value<double>()->default_value(0.5), "[s] frame interval above this is a data gap")               //
    ("merge_gap", value<double>()->default_value(2.0), "[s] anomalies closer than this merge into one event")     //
    ("quiet,q", "only print the one-line verdict per map")                                                        //
    ;

  positional_options_description po;
  po.add("map_path", -1);

  variables_map vm;
  try {
    store(command_line_parser(argc, argv).options(desc).positional(po).run(), vm);
    notify(vm);
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n\n" << desc << std::endl;
    return 2;
  }

  if (vm.count("help") || !vm.count("map_path")) {
    std::cout << desc << std::endl;
    std::cout << "\nExamples:\n"
              << "  trajectory_qc /path/to/map\n"
              << "  trajectory_qc /path/to/map -r /tmp/qc.txt\n"
              << "  trajectory_qc /maps/*/ --report_dir /tmp/qc --quiet\n"
              << std::endl;
    return vm.count("help") ? 0 : 2;
  }

  auto logger = spdlog::stdout_color_mt("qc");
  logger->set_pattern("%v");
  spdlog::set_default_logger(logger);

  const bool quiet = vm.count("quiet");
  glim::TrajectoryQCConfig config;
  config.max_speed = vm["max_speed"].as<double>();
  config.max_accel = vm["max_accel"].as<double>();
  config.max_imu_mismatch = vm["max_imu_mismatch"].as<double>();
  config.max_angular_rate = vm["max_angular_rate"].as<double>();
  config.max_gap = vm["max_gap"].as<double>();
  config.merge_gap = vm["merge_gap"].as<double>();

  const auto paths = vm["map_path"].as<std::vector<std::string>>();
  const std::string report_path = vm["report"].as<std::string>();
  const std::string report_dir = vm["report_dir"].as<std::string>();
  if (!report_dir.empty()) {
    boost::filesystem::create_directories(report_dir);
  }

  struct Row {
    std::string name;
    glim::QCReport report;
  };
  std::vector<Row> rows;
  int n_failed = 0;

  for (const auto& path : paths) {
    if (!boost::filesystem::exists(path)) {
      logger->error("{}: does not exist", path);
      n_failed++;
      continue;
    }

    const auto submaps = load_submaps(path);
    if (submaps.empty()) {
      logger->error("{}: no submaps found (is it a GLIM dump directory?)", path);
      n_failed++;
      continue;
    }

    auto report = glim::TrajectoryQC::analyze(submaps, config);
    report.map_path = path;

    if (quiet) {
      logger->info("{:<28} {}", basename_of(path), report.verdict());
    } else {
      logger->info("");
      logger->info("===== {} =====", path);
      glim::TrajectoryQC::log_report(report, logger);
    }

    if (!report.passed()) {
      n_failed++;
    }

    if (paths.size() == 1 && !report_path.empty()) {
      glim::TrajectoryQC::write_report(report, report_path);
      logger->info("report written to {}", report_path);
    }
    if (!report_dir.empty()) {
      const std::string out = report_dir + "/" + basename_of(path) + "_qc.txt";
      glim::TrajectoryQC::write_report(report, out);
    }

    rows.push_back({basename_of(path), report});
  }

  // Batch summary
  if (rows.size() > 1) {
    logger->info("");
    logger->info("{:<28}{:>10}{:>10}{:>11}{:>9}  {}", "map", "path[m]", "max[m/s]", "step[m]", "events", "verdict");
    for (const auto& r : rows) {
      logger->info(
        "{:<28}{:>10.1f}{:>10.2f}{:>11.3f}{:>9}  {}",
        r.name,
        r.report.path_length,
        r.report.max_v_pose,
        r.report.max_frame_step,
        r.report.events.size(),
        r.report.verdict());
    }
    logger->info("");
    logger->info("{} / {} maps passed", rows.size() - n_failed, rows.size());
  }

  return n_failed ? 1 : 0;
}
