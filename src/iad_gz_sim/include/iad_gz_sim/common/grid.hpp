// Row-major 2D grid. (ox, oy) is the world position of cell (0,0)'s lower-left corner.
// Cost semantics follow nav_msgs/OccupancyGrid: -1 unknown, 0 free, 1..99 cost, 100 lethal.
#pragma once
#include <cmath>
#include <cstdint>
#include <vector>

namespace iad {

template <typename T>
struct Grid {
  int w = 0, h = 0;
  double res = 0.5, ox = 0, oy = 0;
  std::vector<T> data;

  Grid() = default;
  Grid(int w_, int h_, double r, double ox_, double oy_, T init = T{})
      : w(w_), h(h_), res(r), ox(ox_), oy(oy_), data(size_t(w_) * size_t(h_), init) {}

  bool inside(int i, int j) const { return i >= 0 && j >= 0 && i < w && j < h; }
  bool toCell(double x, double y, int& i, int& j) const {
    i = int(std::floor((x - ox) / res));
    j = int(std::floor((y - oy) / res));
    return inside(i, j);
  }
  double cx(int i) const { return ox + (i + 0.5) * res; }
  double cy(int j) const { return oy + (j + 0.5) * res; }
  T& at(int i, int j) { return data[size_t(j) * size_t(w) + size_t(i)]; }
  const T& at(int i, int j) const { return data[size_t(j) * size_t(w) + size_t(i)]; }
  T valueAt(double x, double y, T outside) const {
    int i, j;
    return toCell(x, y, i, j) ? at(i, j) : outside;
  }
  bool empty() const { return data.empty(); }
};

using CostGrid = Grid<int8_t>;
constexpr int8_t kLethal = 100;
constexpr int8_t kUnknown = -1;

}  // namespace iad
