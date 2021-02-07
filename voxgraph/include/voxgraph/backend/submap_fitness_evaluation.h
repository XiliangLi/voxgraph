#ifndef VOXGRAPH_BACKEND_SUBMAP_FITNESS_EVALUATION_H_
#define VOXGRAPH_BACKEND_SUBMAP_FITNESS_EVALUATION_H_

#include "voxgraph/common.h"
#include "voxgraph/frontend/submap_collection/voxgraph_submap.h"

namespace voxgraph {
class SubmapFitnessEvalution {
 public:
  struct Config {
    float max_valid_distance = 0;
    bool only_isopoints = 0;
    float k_overlap = 0;

    friend inline std::ostream& operator<<(std::ostream& s, const Config& v) {
      s << std::endl
        << "SubmapFitnessEvalution using Config:" << std::endl
        << "  max_valid_distance: " << v.max_valid_distance << std::endl
        << "  only_isopoints: " << v.only_isopoints << std::endl
        << "  k_overlap: " << v.k_overlap << std::endl
        << "-------------------------------------------" << std::endl;
      return (s);
    }
  };
  static Config getConfigFromRosParam(const ros::NodeHandle& nh_private) {
    Config config;
    nh_private.param("max_valid_distance", config.max_valid_distance,
                     config.max_valid_distance);
    nh_private.param("only_isopoints", config.only_isopoints,
                     config.only_isopoints);
    nh_private.param("k_overlap", config.k_overlap, config.k_overlap);
    return config;
  }

  explicit SubmapFitnessEvalution(const ros::NodeHandle& nh_private)
      : config_(getConfigFromRosParam(nh_private)) {}
  virtual ~SubmapFitnessEvalution() = default;

  bool evaluateFitness(const VoxgraphSubmap& submap_a,
                       const VoxgraphSubmap& submap_b, Transformation T_A_B) {
    auto const& registration_points_a = submap_a.getRegistrationPoints(
        VoxgraphSubmap::RegistrationPointType::kIsosurfacePoints);
    auto const& registration_points_b = submap_b.getRegistrationPoints(
        VoxgraphSubmap::RegistrationPointType::kIsosurfacePoints);

    int num_valid = 0;
    int total_points = 0;
    for (auto const& pt : registration_points_a.getItem()) {
      if (config_.only_isopoints && pt.distance != 0) continue;
      total_points++;
      voxblox::Pointcloud T_B_APt;
      voxblox::transformPointcloud(T_A_B.inverse(), {pt.position}, &T_B_APt);
      if (checkDistanceValid(submap_b.getTsdfMap().getTsdfLayer(), T_B_APt[0]))
        num_valid++;
    }
    if ((static_cast<float>(num_valid) / total_points) < config_.k_overlap)
      return false;

    num_valid = 0;
    total_points = 0;
    for (auto const& pt : registration_points_b.getItem()) {
      if (config_.only_isopoints && pt.distance != 0) continue;
      voxblox::Pointcloud T_B_APt;
      voxblox::transformPointcloud(T_A_B, {pt.position}, &T_B_APt);
      if (checkDistanceValid(submap_a.getTsdfMap().getTsdfLayer(), T_B_APt[0]))
        num_valid++;
    }
    if ((static_cast<float>(num_valid) / total_points) < config_.k_overlap)
      return false;

    return true;
  }

  bool checkDistanceValid(const voxblox::Layer<voxblox::TsdfVoxel>& tsdf_layer,
                          voxblox::Point point) {
    auto tsdf_voxel = tsdf_layer.getVoxelPtrByCoordinates(point);
    if (!tsdf_voxel) return false;
    if (tsdf_voxel->distance < config_.max_valid_distance) return true;
    return false;
  }

 private:
  Config config_;
};
}  // namespace voxgraph

#endif  // VOXGRAPH_BACKEND_SUBMAP_FITNESS_EVALUATION_H_
