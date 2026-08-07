#ifndef LOCAL_MAPPING_VOXEL_MAP_H_
#define LOCAL_MAPPING_VOXEL_MAP_H_

#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

namespace local_mapping {

enum class CellState { UNKNOWN, FREE, OCCUPIED };

struct Vec3 {
  double x;
  double y;
  double z;
};

struct Clearance {
  bool known;
  double metres;
};

class VoxelMap {
 public:
  VoxelMap(double resolution, int occupied_hits, int free_hits,
           double dynamic_ttl);

  void integrateStaticRay(const Vec3& origin, const Vec3& endpoint);
  void integrateStaticRays(const Vec3& origin,
                           const std::vector<Vec3>& endpoints);
  void integrateDynamicPoint(const Vec3& point, double stamp);
  CellState stateAt(const Vec3& point, double now) const;
  Clearance axisClearance(const Vec3& origin, const Vec3& unit_axis,
                          double max_distance, double now) const;
  std::vector<Vec3> staticOccupiedPoints(double now) const;
  std::vector<Vec3> dynamicOccupiedPoints(double now) const;

 private:
  struct Key {
    std::int64_t x;
    std::int64_t y;
    std::int64_t z;

    bool operator<(const Key& other) const;
    bool operator==(const Key& other) const;
  };

  struct KeyHash {
    std::size_t operator()(const Key& key) const;
  };

  struct Evidence {
    std::uint32_t occupied = 0;
    std::uint32_t free = 0;
  };

  Key keyFor(const Vec3& point) const;
  Vec3 centreFor(const Key& key) const;
  std::size_t sampleCount(long double distance) const;
  CellState staticState(const Key& key) const;
  CellState stateForKey(const Key& key, double now) const;
  void pruneExpiredDynamic(double now) const;
  double entryDistance(const Vec3& origin, const Vec3& unit_axis,
                       const Key& key) const;

  double resolution_;
  std::uint32_t occupied_hits_;
  std::uint32_t free_hits_;
  double dynamic_ttl_;
  std::map<Key, Evidence> static_cells_;
  mutable std::map<Key, double> dynamic_cells_;
};

}  // namespace local_mapping

#endif  // LOCAL_MAPPING_VOXEL_MAP_H_
