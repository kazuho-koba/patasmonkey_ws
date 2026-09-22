#include "pm_perception/rolling_elevation_grid.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pm_perception
{

namespace
{
int positiveModulo(int64_t value, int modulus)
{
  const int result = static_cast<int>(value % modulus);
  return result < 0 ? result + modulus : result;
}
}  // namespace

RollingElevationGrid::RollingElevationGrid(
  double size_x, double size_y, double resolution)
: width_(static_cast<int>(std::round(size_x / resolution))),
  height_(static_cast<int>(std::round(size_y / resolution))),
  resolution_(resolution),
  cells_(static_cast<size_t>(width_ * height_))
{
  if (size_x <= 0.0 || size_y <= 0.0 || resolution <= 0.0 || width_ < 1 || height_ < 1) {
    throw std::invalid_argument("map dimensions and resolution must be positive");
  }
}

void RollingElevationGrid::recenter(double center_x, double center_y)
{
  const int64_t center_cell_x = static_cast<int64_t>(std::floor(center_x / resolution_));
  const int64_t center_cell_y = static_cast<int64_t>(std::floor(center_y / resolution_));
  origin_cell_x_ = center_cell_x - width_ / 2;
  origin_cell_y_ = center_cell_y - height_ / 2;
}

bool RollingElevationGrid::contains(int64_t world_x, int64_t world_y) const
{
  return world_x >= origin_cell_x_ && world_x < origin_cell_x_ + width_ &&
         world_y >= origin_cell_y_ && world_y < origin_cell_y_ + height_;
}

bool RollingElevationGrid::worldToCell(
  double x, double y, int64_t & world_x, int64_t & world_y) const
{
  world_x = static_cast<int64_t>(std::floor(x / resolution_));
  world_y = static_cast<int64_t>(std::floor(y / resolution_));
  return contains(world_x, world_y);
}

size_t RollingElevationGrid::storageIndex(int64_t world_x, int64_t world_y) const
{
  return static_cast<size_t>(
    positiveModulo(world_y, height_) * width_ + positiveModulo(world_x, width_));
}

ElevationCell & RollingElevationGrid::touch(int64_t world_x, int64_t world_y)
{
  ElevationCell & cell = cells_[storageIndex(world_x, world_y)];
  if (cell.world_x != world_x || cell.world_y != world_y) {
    cell = ElevationCell();
    cell.world_x = world_x;
    cell.world_y = world_y;
  }
  return cell;
}

const ElevationCell * RollingElevationGrid::get(int64_t world_x, int64_t world_y) const
{
  if (!contains(world_x, world_y)) {
    return nullptr;
  }
  const ElevationCell & cell = cells_[storageIndex(world_x, world_y)];
  return cell.world_x == world_x && cell.world_y == world_y ? &cell : nullptr;
}

void RollingElevationGrid::fuseGround(
  int64_t world_x, int64_t world_y, float elevation, int64_t stamp_ns,
  float merge_threshold)
{
  ElevationCell & cell = touch(world_x, world_y);
  if (cell.observation_count == 0U || elevation < cell.elevation - merge_threshold) {
    if (cell.observation_count > 0U) {
      cell.obstacle_height = std::max(cell.obstacle_height, cell.elevation - elevation);
      cell.obstacle_confidence = std::min(1.0F, cell.obstacle_confidence + 0.2F);
      cell.last_obstacle_observed_ns = stamp_ns;
    }
    cell.elevation = elevation;
    cell.elevation_m2 = 0.0F;
    cell.observation_count = 1U;
  } else if (std::abs(elevation - cell.elevation) <= merge_threshold) {
    const uint32_t next_count = cell.observation_count + 1U;
    const float delta = elevation - cell.elevation;
    cell.elevation += delta / static_cast<float>(next_count);
    cell.elevation_m2 += delta * (elevation - cell.elevation);
    cell.observation_count = next_count;
  } else {
    observeObstacle(world_x, world_y, elevation, stamp_ns, merge_threshold);
  }
  cell.last_observed_ns = stamp_ns;
}

void RollingElevationGrid::observeObstacle(
  int64_t world_x, int64_t world_y, float elevation, int64_t stamp_ns,
  float minimum_height)
{
  ElevationCell & cell = touch(world_x, world_y);
  if (cell.observation_count == 0U) {
    return;
  }
  const float height = elevation - cell.elevation;
  if (height >= minimum_height) {
    cell.obstacle_height = std::max(cell.obstacle_height, height);
    cell.obstacle_confidence = std::min(1.0F, cell.obstacle_confidence + 0.1F);
    cell.last_obstacle_observed_ns = stamp_ns;
  }
}

float RollingElevationGrid::variance(
  const ElevationCell & cell, float measurement_variance) const
{
  if (cell.observation_count < 2U) {
    return measurement_variance;
  }
  return measurement_variance +
         cell.elevation_m2 / static_cast<float>(cell.observation_count - 1U);
}

}  // namespace pm_perception
