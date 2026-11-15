// Prior maps derived from the static world: a point-cloud map for scan matching (like Autoware's
// pointcloud_map) and a site costmap (surveyed terrain + mapped structures) for free-space planning.
#pragma once
#include "iad_gz_sim/common/grid.hpp"
#include "iad_gz_sim/common/point_types.hpp"
#include "iad_gz_sim/world.hpp"

namespace iad::sim {

Cloud buildPointcloudMap(const World& w, double terrain_spacing, double object_spacing, double voxel);

// Cost semantics of nav_msgs/OccupancyGrid. Overhead structure higher than `clearance` above the
// ground (crane booms, conveyors) is not an obstacle.
CostGrid buildCostmap(const World& w, double res, double clearance);

}  // namespace iad::sim
