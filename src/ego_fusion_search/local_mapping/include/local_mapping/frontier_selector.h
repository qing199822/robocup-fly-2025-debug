#ifndef LOCAL_MAPPING_FRONTIER_SELECTOR_H_
#define LOCAL_MAPPING_FRONTIER_SELECTOR_H_

#include <nav_msgs/OccupancyGrid.h>

#include "local_mapping/voxel_map.h"

namespace local_mapping {

struct FrontierGoal {
  bool valid;
  int cell_x;
  int cell_y;
  double x;
  double y;
  double z;
  double yaw;
  double distance_from_robot;
};

class FrontierSelector {
 public:
  FrontierSelector(int min_cluster_cells, double max_distance);
  FrontierGoal select(const nav_msgs::OccupancyGrid& grid,
                      const Vec3& robot) const;

 private:
  int min_cluster_cells_;
  double max_distance_;
};

}  // namespace local_mapping

#endif  // LOCAL_MAPPING_FRONTIER_SELECTOR_H_
