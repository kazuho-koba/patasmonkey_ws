#ifndef PM_PERCEPTION__ROLLING_ELEVATION_GRID_HPP_
#define PM_PERCEPTION__ROLLING_ELEVATION_GRID_HPP_

#include <cstdint>
#include <limits>
#include <vector>

namespace pm_perception
{

struct ElevationCell
{
  int64_t world_x{std::numeric_limits<int64_t>::min()};
  int64_t world_y{std::numeric_limits<int64_t>::min()};
  float elevation{0.0F};
  float elevation_m2{0.0F};
  float obstacle_height{0.0F};
  float obstacle_confidence{0.0F};
  uint32_t observation_count{0U};
  int64_t last_observed_ns{0};
  int64_t last_obstacle_observed_ns{0};
};

class RollingElevationGrid
{
public:
  RollingElevationGrid(double size_x, double size_y, double resolution);

  void recenter(double center_x, double center_y);
  bool worldToCell(double x, double y, int64_t & world_x, int64_t & world_y) const;
  ElevationCell & touch(int64_t world_x, int64_t world_y);
  const ElevationCell * get(int64_t world_x, int64_t world_y) const;
  void fuseGround(
    int64_t world_x, int64_t world_y, float elevation, int64_t stamp_ns,
    float merge_threshold);
  void observeObstacle(
    int64_t world_x, int64_t world_y, float elevation, int64_t stamp_ns,
    float minimum_height);

  int width() const {return width_;}
  int height() const {return height_;}
  double resolution() const {return resolution_;}
  int64_t originCellX() const {return origin_cell_x_;}
  int64_t originCellY() const {return origin_cell_y_;}
  double originX() const {return static_cast<double>(origin_cell_x_) * resolution_;}
  double originY() const {return static_cast<double>(origin_cell_y_) * resolution_;}
  float variance(const ElevationCell & cell, float measurement_variance) const;

private:
  size_t storageIndex(int64_t world_x, int64_t world_y) const;
  bool contains(int64_t world_x, int64_t world_y) const;

  int width_;
  int height_;
  double resolution_;
  int64_t origin_cell_x_{0};
  int64_t origin_cell_y_{0};
  std::vector<ElevationCell> cells_;
};

}  // namespace pm_perception

#endif  // PM_PERCEPTION__ROLLING_ELEVATION_GRID_HPP_
