#include "terrain_mapping/terrain_processing.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>


namespace terrain_mapping
{

namespace
{
bool inRange(const grid_map::Matrix & m, int i, int j)
{
  return i >= 0 && j >= 0 && i < m.rows() && j < m.cols();
}

// Fill holes with the mean of valid neighbors; single pass, so filled values never propagate.
grid_map::Matrix inpaint(const grid_map::Matrix & in, int radius, int minNeighbors)
{
  grid_map::Matrix out = in;
  for (int i = 0; i < in.rows(); ++i) {
    for (int j = 0; j < in.cols(); ++j) {
      if (std::isfinite(in(i, j))) {continue;}
      double sum = 0.0;
      int n = 0;
      for (int di = -radius; di <= radius; ++di) {
        for (int dj = -radius; dj <= radius; ++dj) {
          if (!inRange(in, i + di, j + dj)) {continue;}
          const float v = in(i + di, j + dj);
          if (std::isfinite(v)) {sum += v; ++n;}
        }
      }
      if (n >= minNeighbors) {out(i, j) = static_cast<float>(sum / n);}
    }
  }
  return out;
}

grid_map::Matrix median3x3(const grid_map::Matrix & in)
{
  grid_map::Matrix out = in;
  std::vector<float> vals;
  vals.reserve(9);
  for (int i = 0; i < in.rows(); ++i) {
    for (int j = 0; j < in.cols(); ++j) {
      if (!std::isfinite(in(i, j))) {continue;}
      vals.clear();
      for (int di = -1; di <= 1; ++di) {
        for (int dj = -1; dj <= 1; ++dj) {
          if (!inRange(in, i + di, j + dj)) {continue;}
          const float v = in(i + di, j + dj);
          if (std::isfinite(v)) {vals.push_back(v);}
        }
      }
      std::nth_element(vals.begin(), vals.begin() + vals.size() / 2, vals.end());
      out(i, j) = vals[vals.size() / 2];
    }
  }
  return out;
}
}  // namespace

void processTerrain(grid_map::GridMap & map, const TerrainProcessingParams & p)
{
  const grid_map::Matrix & raw = map.get(kLayerElevation);
  map.add(kLayerFiltered, median3x3(inpaint(raw, p.inpaintRadiusCells, p.inpaintMinNeighbors)));
}

bool isRobotPoint(const Eigen::Vector3f & p, const RobotShapes & shapes)
{
  for (const auto & c : shapes.limbs) {
    const Eigen::Vector3f ab = c.b - c.a;
    const float len2 = ab.squaredNorm();
    const float t = len2 > 0.f ? std::clamp((p - c.a).dot(ab) / len2, 0.f, 1.f) : 0.f;
    if ((p - (c.a + t * ab)).squaredNorm() <= c.radius * c.radius) {return true;}
  }
  const Eigen::Vector3f q = shapes.mapToTrunk * p - shapes.trunkBoxCenter;
  return (q.cwiseAbs() - shapes.trunkBoxHalf).maxCoeff() <= 0.f;
}

}  // namespace terrain_mapping
