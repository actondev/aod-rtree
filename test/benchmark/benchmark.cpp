#include "../common.hpp"
#include <RTree.h> // nushoing/superliminal
#include <benchmark/benchmark.h>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <functional>
#include <mdds/rtree.hpp>

template <int DIMS>
RTree<Point, double, DIMS> grid_to_superliminial_rtree(const Grid &grid) {
  RTree<Point, double, DIMS> tree;
  for (const Point &el : grid.points) {
    const double *pos = &el[0];
    tree.Insert(pos, pos, el);
  }
  return tree;
}

using rt_type = mdds::rtree<double, std::string>;

// default mdds options make it really slow.
// following superliminal defaults for min & max node size
struct mdds_options {
  constexpr static size_t dimensions = 2;
  // the commented value is the default one
  constexpr static size_t min_node_size = 4;               // 40
  constexpr static size_t max_node_size = 8;               // 100
  constexpr static size_t max_tree_depth = 100;            //
  constexpr static bool enable_forced_reinsertion = false; // true
  constexpr static size_t reinsertion_size = 2;            // 30
};
mdds::rtree<double, Point, mdds_options> grid_to_mdds_rtree(const Grid &grid) {

  mdds::rtree<double, Point, mdds_options> tree;
  for (const Point &el : grid.points) {
    const double *pos = &el[0];
    // rt_type::extent_type bounds({-2.0, -1.0}, {1.0, 2.0});
    tree.insert({{pos[0], pos[1]}, {pos[0], pos[1]}}, el);
  }
  return tree;
}

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
typedef bg::model::point<double, 2, bg::cs::cartesian> point_t;
typedef bg::model::box<point_t> box_t;
typedef std::pair<box_t, Point> value_t;
typedef bgi::rtree<value_t, bgi::quadratic<8, 4>> boost_rtree_t;

boost_rtree_t grid_to_boost_rtree(const Grid &grid) {
  boost_rtree_t tree;
  for (const Point &el : grid.points) {
    box_t b(point_t(el[0], el[1]), point_t(el[0], el[1]));
    tree.insert(value_t(b, el));
  }
  return tree;
}

static void aod_rtree_init(benchmark::State &state) {
  int size = state.range(0);
  auto grid = make_grid(size, 2);
  for (auto _ : state) {
    aod::Rtree<Point> tree = grid_to_aod_rtree(grid);
    benchmark::DoNotOptimize(tree);
  }
}

static void aod_rtree_copy(benchmark::State &state) {
  int size = state.range(0);
  auto grid = make_grid(size, 2);
  aod::Rtree<Point> tree = grid_to_aod_rtree(grid);
  for (auto _ : state) {
    aod::Rtree<Point> tree2 = tree;
    benchmark::DoNotOptimize(tree2);
  }
}

static void aod_rtree_search(benchmark::State &state) {
  int size = state.range(0);
  auto grid = make_grid(size, 2);
  aod::Rtree<Point> tree = grid_to_aod_rtree(grid);
  std::vector<Point> results;
  for (auto _ : state) {
    results.clear();
    tree.search({5, 5}, {9, 9}, results);
    if (results.size() != 25) {
      // cout << "results size " << results.size() << endl;
    }
    benchmark::DoNotOptimize(results);
  }
}

static void superliminal_rtree_init(benchmark::State &state) {
  int size = state.range(0);
  auto grid = make_grid(size, 2);
  for (auto _ : state) {
    RTree<Point, double, 2> tree = grid_to_superliminial_rtree<2>(grid);
    benchmark::DoNotOptimize(tree);
  }
}

static void superliminal_rtree_copy(benchmark::State &state) {
  int size = state.range(0);
  auto grid = make_grid(size, 2);
  RTree<Point, double, 2> tree = grid_to_superliminial_rtree<2>(grid);
  for (auto _ : state) {
    RTree<Point, double, 2> tree2 = tree;
    benchmark::DoNotOptimize(tree2);
  }
}

static void superliminal_rtree_search(benchmark::State &state) {
  int size = state.range(0);
  auto grid = make_grid(size, 2);
  std::vector<Point> results;
  std::function<bool(Point)> callback = [&results](Point x) {
    results.push_back(x);
    return true;
  };
  RTree<Point, double, 2> tree = grid_to_superliminial_rtree<2>(grid);
  const double low[2] = {5, 5};
  const double high[2] = {9, 9};
  for (auto _ : state) {
    results.clear();
    tree.Search(low, high, callback);
    // assert(results.size() == 25);
    benchmark::DoNotOptimize(results);
  }
}

static void mdds_rtree_init(benchmark::State &state) {
  int size = state.range(0);
  auto grid = make_grid(size, 2);
  for (auto _ : state) {
    mdds::rtree<double, Point, mdds_options> tree = grid_to_mdds_rtree(grid);
    benchmark::DoNotOptimize(tree);
  }
}

static void mdds_rtree_copy(benchmark::State &state) {
  int size = state.range(0);
  auto grid = make_grid(size, 2);
  mdds::rtree<double, Point, mdds_options> tree = grid_to_mdds_rtree(grid);
  for (auto _ : state) {
    mdds::rtree<double, Point, mdds_options> tree2 = tree;
    benchmark::DoNotOptimize(tree2);
  }
}

static void mdds_rtree_search(benchmark::State &state) {
  int size = state.range(0);
  auto grid = make_grid(size, 2);
  mdds::rtree<double, Point, mdds_options> tree = grid_to_mdds_rtree(grid);
  for (auto _ : state) {
    auto results =
        tree.search({{5.0, 5.0}, {9.0, 9.0}}, rt_type::search_type::overlap);
    benchmark::DoNotOptimize(results);
    // state.PauseTiming();
    // int count = 0;
    // for(auto r_ : results) {
    //   ++count;
    // }
    // assert(count == 25);
    // state.ResumeTiming();
  }
}

static void boost_rtree_init(benchmark::State &state) {
  int size = state.range(0);
  auto grid = make_grid(size, 2);
  for (auto _ : state) {
    boost_rtree_t tree = grid_to_boost_rtree(grid);
    benchmark::DoNotOptimize(tree);
  }
}

static void boost_rtree_copy(benchmark::State &state) {
  int size = state.range(0);
  auto grid = make_grid(size, 2);
  for (auto _ : state) {
    boost_rtree_t tree = grid_to_boost_rtree(grid);
    benchmark::DoNotOptimize(tree);
  }
}

static void boost_rtree_search(benchmark::State &state) {
  int size = state.range(0);
  auto grid = make_grid(size, 2);
  boost_rtree_t tree = grid_to_boost_rtree(grid);
  std::vector<value_t> res;
  box_t search_box(point_t(5, 5), point_t(9, 9));
  for (auto _ : state) {
    res.clear();
    tree.query(bgi::intersects(search_box), std::back_inserter(res));
    // assert(res.size() == 25);
  }
}

const int max_N = 512; // 512

BENCHMARK(aod_rtree_init)
    ->RangeMultiplier(2)
    ->Range(16, max_N)
    ->Unit(benchmark::kMillisecond);

BENCHMARK(superliminal_rtree_init)
    ->RangeMultiplier(2)
    ->Range(16, max_N)
    ->Unit(benchmark::kMillisecond);

BENCHMARK(mdds_rtree_init)
    ->RangeMultiplier(2)
    ->Range(16, max_N)
    ->Unit(benchmark::kMillisecond);

BENCHMARK(boost_rtree_init)
    ->RangeMultiplier(2)
    ->Range(16, max_N)
    ->Unit(benchmark::kMillisecond);

// search
BENCHMARK(aod_rtree_search)
    ->RangeMultiplier(2)
    ->Range(16, max_N)
    ->Unit(benchmark::kMillisecond);

BENCHMARK(superliminal_rtree_search)
    ->RangeMultiplier(2)
    ->Range(16, max_N)
    ->Unit(benchmark::kMillisecond);

BENCHMARK(mdds_rtree_search)
    ->RangeMultiplier(2)
    ->Range(16, 64) // slow
    ->Unit(benchmark::kMillisecond);

BENCHMARK(boost_rtree_search)
    ->RangeMultiplier(2)
    ->Range(16, max_N)
    ->Unit(benchmark::kMillisecond);

// copy
BENCHMARK(aod_rtree_copy)
    ->RangeMultiplier(2)
    ->Range(16, max_N)
    ->Unit(benchmark::kMillisecond);

BENCHMARK(superliminal_rtree_copy)
    ->RangeMultiplier(2)
    ->Range(16, max_N)
    ->Unit(benchmark::kMillisecond);

BENCHMARK(mdds_rtree_copy)
    ->RangeMultiplier(2)
    ->Range(16, max_N)
    ->Unit(benchmark::kMillisecond);

BENCHMARK(boost_rtree_copy)
    ->RangeMultiplier(2)
    ->Range(16, 256)
    ->Unit(benchmark::kMillisecond);
