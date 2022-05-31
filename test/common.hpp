#include <array>
#include <chrono> // for high_resolution_clock
#include "easyprint.hpp"
#include <iostream>
#include <fstream>
#include <aod/rtree.hpp>
#include <vector>
#include <random>

using std::cerr;
using std::cout;
using std::endl;

using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

auto now = high_resolution_clock::now;

#define duration_ms(a)                                                  \
  std::chrono::duration_cast<std::chrono::milliseconds>(a).count()
#define duration_ns(a)                                                  \
  std::chrono::duration_cast<std::chrono::nanoseconds>(a).count()

template <typename T> std::string pp(const T &x) {
  return easyprint::stringify(x);
}

using Point = std::vector<double>;
struct Grid {
  int dims;
  std::vector<Point> points;
};

void shuffle_deterministic(Grid& grid) {
  std::mt19937 gen(0);
  std::shuffle(std::begin(grid.points), std::end(grid.points), gen);
}

// for debugging, creating the "expected" vectors
void sort_points(std::vector<Point> &points) {
  std::sort(points.begin(), points.end(), [](const Point& a, const Point& b) {
    if( a[0] == b[0] ) {
      return a[1] < b[1];
    } else {
      return a[0] < b[0];
    }
  });
}

/**
 * see https://stackoverflow.com/a/49514906/8720686
 * Given length=3,dims=2 it gives
 * {{0, 0}, {0, 1}, {0, 2},
 *  {1, 0}, {1, 1}, {1, 2},
 *  {2, 0}, {2, 1}, {2, 2}}
 */
Grid make_grid(int length, int dims = 2, bool shuffle = true) {
  Grid grid;
  grid.dims = dims;
  int size = pow(length, dims);
  grid.points.resize(size);
  for (int i = 0; i < size; i++) {
    auto &el = grid.points[i];
    // not optimal, but makes things simple (no template, runtime only)
    el.resize(dims);
    for (int dim = 0; dim < dims; dim++) {
      int denum = pow(length, dim);
      int x = (int)(i / denum) % length;
      int mod = x % length;
      el[dims - dim - 1] = mod;
    }
  }
  if(shuffle) {
    shuffle_deterministic(grid);
  }

  return grid;
}

aod::Rtree<Point> grid_to_aod_rtree(const Grid &grid) {
  aod::Rtree<Point> tree(grid.dims);
  for (auto &el : grid.points) {
    // const double *pos = &el[0];
    tree.insert(el, el, el);
  }
  return tree;
}

aod::Rtree<Point> grid_to_aod_rtree_with_transaction(const Grid &grid) {
  aod::Rtree<Point> tree(grid.dims);
  tree.with_transaction([&grid](aod::Rtree<Point> &tree) {
    for (auto &el : grid.points) {
      tree.insert(el, el, el);
    }
  });

  return tree;
}

const int benchmark_size = 200;
const int searches = 10000;
const char* search_str = "search x10000 ";
