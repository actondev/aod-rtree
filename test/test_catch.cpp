#include "./common.hpp"
#include <RTree.h>
#include <catch2/catch.hpp>
#include <sstream>
#include <string>
RTree<Point, double, 2> grid_to_superliminal_rtree(const Grid &grid) {
  RTree<Point, double, 2> tree;
  for (const Point &el : grid.points) {
    const double *pos = &el[0];
    tree.Insert(pos, pos, el);
  }
  return tree;
}

TEST_CASE("superliminal RTree 200x200", "[aod::Rtree][benchmark]") {
  int size = benchmark_size;
  auto grid = make_grid(size, 2);
  auto t1 = high_resolution_clock::now();
  RTree<Point, double, 2> tree = grid_to_superliminal_rtree(grid);
  auto t2 = high_resolution_clock::now();
  WARN("init size " << size << " took " << duration_ms(t2 - t1) << "ms");
  REQUIRE(tree.Count() == size * size);

  std::vector<Point> expected = {
    { 5.0, 2.0 },
    { 5.0, 3.0 },
    { 5.0, 4.0 },
    { 6.0, 2.0 },
    { 6.0, 3.0 },
    { 6.0, 4.0 },
  };
  double low[2] = {5, 2};
  double high[2] = {6, 4};
  std::vector<Point> found;
  using Callback = std::function<bool(const Point &)>;
  Callback cb = [&](const Point &point) {
    found.push_back(point);
    return true;
  };
  tree.Search(low, high, cb);
  REQUIRE_THAT(found, Catch::Matchers::UnorderedEquals(expected));

  t1 = now();
  for (int i = 0; i < searches; i++) {
    found.clear();
    tree.Search(low, high, cb);
  }
  t2 = now();
  WARN(search_str << duration_ms(t2 - t1) << " ms ");

  t1 = now();
  RTree<Point, double, 2> tree2 = tree;
  t2 = now();
  WARN("copy took " << duration_ms(t2 - t1) << " ms ");
}

TEST_CASE("aod::Rtree 200x200", "[aod::Rtree][benchmark]") {
  int size = benchmark_size;
  auto grid = make_grid(size, 2);
  auto t1 = high_resolution_clock::now();
  aod::Rtree<Point> tree = grid_to_aod_rtree(grid);
  auto t2 = high_resolution_clock::now();
  WARN("init size " << size << " took " << duration_ms(t2 - t1) << "ms");
  REQUIRE(tree.size() == size * size);

  if (false) {
    std::ofstream ofs("aod-rtree-benchmark-init.xml", std::ofstream::out);
    ofs << tree.to_xml();
    ofs.close();
  }

  std::vector<Point> expected = {
      {5.0, 2.0}, {5.0, 3.0}, {5.0, 4.0}, {6.0, 2.0}, {6.0, 3.0}, {6.0, 4.0},
  };
  std::vector<double> low = {5, 2};
  std::vector<double> high = {6, 4};
  auto found = tree.search(low, high);
  REQUIRE_THAT(found, Catch::Matchers::UnorderedEquals(expected));

  t1 = now();
  for (int i = 0; i < searches; i++) {
    tree.search(low, high, found);
  }
  t2 = now();
  WARN(search_str << duration_ms(t2 - t1) << " ms ");

  t1 = now();
  aod::Rtree<Point> tree2 = tree;
  t2 = now();
  WARN("copy took " << duration_ms(t2 - t1) << " ms ");

  REQUIRE(tree2.size() == tree.size());
  auto found2 = tree2.search(low, high);
  REQUIRE_THAT(found2, Catch::Matchers::UnorderedEquals(expected));

  // removing fat cross
  t1 = now();
  // removing a fat cross in the center, leaving squares [side x side]
  int retain_side = 2;
  int expected_removed1 = size * (size - 2 * retain_side);
  int removed1 =
      tree.remove({0, retain_side}, {size - 1, size - 1 - retain_side});
  REQUIRE(removed1 == expected_removed1);
  int expected_removed2 = 2 * (size - 2 * retain_side) * retain_side;
  int removed2 =
      tree.remove({retain_side, 0}, {size - 1 - retain_side, size - 1});
  REQUIRE(removed2 == expected_removed2);
  REQUIRE(tree.size() ==
          size * size - removed1 - removed2); // 4 left in each corner (2x2)
  REQUIRE(tree.size() == (4 * retain_side * retain_side));
  found = tree.search({0, 0}, {size, size});
  REQUIRE(found.size() == (4 * retain_side * retain_side));
  t2 = now();
  WARN("removing fat cross took " << duration_ms(t2 - t1) << " ms ");
  {
    std::ofstream ofs("aod-rtree-benchmark-removed-fat-cross.xml",
                      std::ofstream::out);
    ofs << tree.to_xml();
    ofs.close();
  }
}

TEST_CASE("aod::Rtree 8x8", "[aod::Rtree][fixmess]") {
  const int size = 8;
  auto grid = make_grid(size, 2, false );
  shuffle_deterministic(grid);

  aod::Rtree<Point> tree = grid_to_aod_rtree(grid);
  REQUIRE(tree.size() == size * size);

  if (true) {
    std::ofstream ofs("aod-rtree-8x8-init.xml", std::ofstream::out);
    ofs << tree.to_xml();
    ofs.close();
  }

  std::vector<Point> expected;
  std::vector<Point> found;

  // removing a fat cross in the center, leaving corners with side=2
  int retain_side = 2;

  found = tree.search({0, retain_side}, {size - 1, size - 1 - retain_side});
  REQUIRE(found.size() == 32);
  // horizontal: 8 * (8-2*retain_side) = 32
  // {0, 2}, {7, 5}
  int removed1 =
      tree.remove({0, retain_side}, {size - 1, size - 1 - retain_side});
  REQUIRE(removed1 == 32);
  REQUIRE(tree.size() == size * size - removed1);
  found = tree.search({0, 0}, {10, 10});
  REQUIRE(found.size() == size * size - removed1);

  if (false) {
    std::ofstream ofs("aod-rtree-8x8-removed-1.xml", std::ofstream::out);
    ofs << tree.to_xml();
    ofs.close();
  }

  int removed2 = tree.remove({retain_side, 0},
                             {size - 1 - retain_side, size - 1}); // vertical
  // 2 horizontal stripes removed 2 rowsof 3 in bottom row & same in top
  // 2 * (size-2*retain_side)*retain_side = 2*4*2 = 16
  REQUIRE(removed2 == 16);
  REQUIRE(tree.size() ==
          size * size - removed1 - removed2); // 4 left in each corner (2x2)
  REQUIRE(tree.size() == 16);                 // 4 left in each corner (2x2)

  if (false) {
    std::ofstream ofs("aod-rtree-8x8-removed.xml", std::ofstream::out);
    ofs << tree.to_xml();
    ofs.close();
  }

  found = tree.search({0, 0}, {10, 10});
  sort_points(found);
  expected = {{0.0, 0.0}, {0.0, 1.0}, {1.0, 0.0}, {1.0, 1.0},
              {0.0, 6.0}, {0.0, 7.0}, {1.0, 6.0}, {1.0, 7.0},
              {6.0, 0.0}, {6.0, 1.0}, {7.0, 0.0}, {7.0, 1.0},
              {6.0, 6.0}, {6.0, 7.0}, {7.0, 6.0}, {7.0, 7.0}};
  REQUIRE_THAT(found, Catch::Matchers::UnorderedEquals(expected));
}

TEST_CASE("aod::Rtree 10x10", "[aod::Rtree]") {
  const int size = 10;
  auto grid = make_grid(size);
  shuffle_deterministic(grid);

  aod::Rtree<Point> tree = grid_to_aod_rtree(grid);
  REQUIRE(tree.size() == size * size);

  if (false) {
    std::ofstream ofs("aod-rtree-10x10-init.xml", std::ofstream::out);
    ofs << tree.to_xml();
    ofs.close();
  }

  std::vector<Point> expected;
  std::vector<Point> found;

  // removing a fat cross in the center, leaving corners with side=2
  int retain_side = 2;

  found = tree.search({0, retain_side}, {size - 1, size - 1 - retain_side});
  REQUIRE(found.size() == 60);
  // horizontal: 10 * (10-2*retain_side) = 10*(10-2*2) = 60
  int removed1 =
      tree.remove({0, retain_side}, {size - 1, size - 1 - retain_side});
  REQUIRE(removed1 == 60);
  if (false) {
    std::ofstream ofs("aod-rtree-10x10-removed-1.xml", std::ofstream::out);
    ofs << tree.to_xml();
    ofs.close();
  }
  REQUIRE(tree.size() == size * size - removed1);
  found = tree.search({0, 0}, {10, 10});
  REQUIRE(found.size() == size * size - removed1);

  // return;

  int removed2 = tree.remove({retain_side, 0},
                             {size - 1 - retain_side, size - 1}); // vertical
  if (false) {
    std::ofstream ofs("aod-rtree-10x10-removed-2.xml", std::ofstream::out);
    ofs << tree.to_xml();
    ofs.close();
  }
  // 2 horizontal stripes removed 2 rowsof n in bottom row & same in top
  // 2 * (size-2*retain_side)*retain_side = 2*(10-2*2)*2 = 2*6*2
  REQUIRE(removed2 == 24);
  REQUIRE(tree.size() ==
          size * size - removed1 - removed2); // 4 left in each corner (2x2)
  REQUIRE(tree.size() == 16);                 // 4 left in each corner (2x2)

  found = tree.search({0, 0}, {10, 10});
  sort_points(found);
  expected = {
    { 0.0, 0.0 }, { 0.0, 1.0 }, { 1.0, 0.0 }, { 1.0, 1.0 },
    { 0.0, 8.0 }, { 0.0, 9.0 }, { 1.0, 8.0 }, { 1.0, 9.0 },
    { 8.0, 0.0 }, { 8.0, 1.0 }, { 9.0, 0.0 }, { 9.0, 1.0 },
    { 8.0, 8.0 }, { 8.0, 9.0 }, { 9.0, 8.0 }, { 9.0, 9.0 } };
  REQUIRE_THAT(found, Catch::Matchers::UnorderedEquals(expected));
}

TEST_CASE("aod::Rtree 5x5 scan", "[aod::Rtree]") {
  std::vector<int> sizes = {16, 32, 64, 128, 256, 512};
  std::vector<Point> found;
  for (int size : sizes) {
    auto grid = make_grid(size, 2, true);
    aod::Rtree<Point> tree = grid_to_aod_rtree(grid);
    const int n_5x5 = size / 5;
    for (int i = 0; i < n_5x5; ++i) {
      for (int j = 0; i < n_5x5; ++i) {
        std::vector<double> low = {i * 5, j * 5};
        std::vector<double> high = {i * 5 + 5 - 1, j * 5 + 5 - 1};
        found = tree.search(low, high);
        if (found.size() != 25) {
          sort_points(found);
          cout << " Error: size " << size << " : " << pp(low) << pp(high)
               << " found " << found.size() << pp(found) << endl;
          std::ofstream ofs("aod-rtree-scan-bug.xml", std::ofstream::out);
          ofs << tree.to_xml();
          ofs.close();
        }
        REQUIRE(found.size() == 25);
      }
    }
  }
}

void validate_tree(aod::Rtree<Point> &tree) {
  REQUIRE(tree.validate_mbrs() == true);
  REQUIRE(tree.has_duplicate_nodes() == false);
}

TEST_CASE("tree options", "[aod::Rtree]") {
  const int size = 200;
  auto t1 = high_resolution_clock::now();
  auto t2 = high_resolution_clock::now();
  auto grid = make_grid(size, 2, true);

  aod::Rtree<Point>::Options options1;
  options1.m = 1;
  options1.M = 2;
  aod::Rtree<Point> tree1(2, options1);
  t1 = now();
  for (auto &el : grid.points) {
    tree1.insert(el, el, el);
  }
  t2 = now();
  WARN("init tree1 (m=1, M=2) took " << duration_ms(t2 - t1) << "ms");
  
  aod::Rtree<Point>::Options options2;
  options2.m = 4;
  options2.M = 8;
  aod::Rtree<Point> tree2(2, options2);
  t1 = now();
  for (auto &el : grid.points) {
    tree2.insert(el, el, el);
  }
  t2 = now();
  WARN("init tree2 (m=4, M=8) took " << duration_ms(t2 - t1) << "ms");

  aod::Rtree<Point>::Options options3;
  options3.m = 16;
  options3.M = 32;
  aod::Rtree<Point> tree3(2, options3);
  t1 = now();
  for (auto &el : grid.points) {
    tree3.insert(el, el, el);
  }
  t2 = now();
  WARN("init tree3 (m=16, M=32) took " << duration_ms(t2 - t1) << "ms");
}

TEST_CASE("aod::Rtree tree validations", "[aod::Rtree][fixme]") {
  std::vector<Point> found;
  std::vector<int> sizes = {16, 32, 64, 128, 256, 512};
  for (int size = 4; size<=10; size++) {
    cout << " size " << size << endl;
    auto grid = make_grid(size, 2, true);
    aod::Rtree<Point> tree = grid_to_aod_rtree(grid);
    // if (!tree.validate_mbrs()) {
      // cout << " size " << size << " invalid mbrs " << endl;
      std::ostringstream name;
      name << "aod-rtree-" << size <<"-init.xml";
      std::ofstream ofs(name.str(), std::ofstream::out);
      ofs << tree.to_xml();
      ofs.close();
      // ASSERT(0);
    // }
    if (tree.has_duplicate_nodes()) {
      cout << " size " << size << " duplicate nodes" << endl;
      std::ofstream ofs("aod-rtree-duplicate-nodes-bug.xml",
                        std::ofstream::out);
      ofs << tree.to_xml();
      ofs.close();
      ASSERT(0);
    }
    if (tree.has_duplicate_entries()) {
      cout << " size " << size << " duplicate entries" << endl;
      cout << tree.to_xml() << endl;
      std::ofstream ofs("aod-rtree-duplicate-entries-bug.xml",
                        std::ofstream::out);
      ofs << tree.to_xml();
      ofs.close();
      ASSERT(0);
    }
    continue;

    // removing fat cross: leaving only areas of 2x2 in each corner
    int retain_side = 2;
    int expected_removed1 = size * (size - 2 * retain_side);
    int removed1 =
        tree.remove({0, retain_side}, {size - 1, size - 1 - retain_side});
    REQUIRE(removed1 == expected_removed1);
    int expected_removed2 = 2 * (size - 2 * retain_side) * retain_side;
    int removed2 =
        tree.remove({retain_side, 0}, {size - 1 - retain_side, size - 1});
    REQUIRE(removed2 == expected_removed2);
    REQUIRE(tree.size() ==
            size * size - removed1 - removed2); // 4 left in each corner (2x2)
    REQUIRE(tree.size() == (4 * retain_side * retain_side));
    found = tree.search({0, 0}, {size, size});
    REQUIRE(found.size() == (4 * retain_side * retain_side));

    // checking again mbrs etc
    if (!tree.validate_mbrs()) {
      cout << " size " << size << " invalid mbrs " << endl;
      std::ofstream ofs("aod-rtree-mbr-bug-after-cross-removal.xml", std::ofstream::out);
      ofs << tree.to_xml();
      ofs.close();
      ASSERT(0);
    }
    if (tree.has_duplicate_nodes()) {
      cout << " size " << size << " duplicate nodes" << endl;
      std::ofstream ofs("aod-rtree-duplicate-nodes-bug-after-cross-removal.xml",
                        std::ofstream::out);
      ofs << tree.to_xml();
      ofs.close();
      ASSERT(0);
    }
  }
}

TEST_CASE("clear", "[aod::Rtree]") {
  const int size = 100;
  auto grid = make_grid(size, 2, true);

  aod::Rtree<Point> tree(2);
  
  for (auto &el : grid.points) {
    tree.insert(el, el, el);
  }
  REQUIRE(tree.size() == size*size);
  auto found = tree.search({10, 10}, {14, 14});
  REQUIRE(found.size() == 25);
  tree.clear();
  REQUIRE(tree.size() == 0);
  
  found = tree.search({10, 10}, {14, 14});
  REQUIRE(found.size() == 0);
  
  for (auto &el : grid.points) {
    tree.insert(el, el, el);
  }
  REQUIRE(tree.size() == size*size);

  found = tree.search({10, 10}, {14, 14});
  REQUIRE(found.size() == 25);
}

TEST_CASE("bounds & offset", "[aod::Rtree]") {
  aod::Rtree<std::string> tree(2);
  std::vector<std::string> expected;
  std::vector<std::string> found;
  aod::Rtree<std::string>::Rect bounds;
  aod::Rtree<std::string>::Rect expected_bounds;
  tree.insert({0, 0}, {1, 1}, "0,0->1,1");
  tree.insert({1, 1}, {2, 2}, "1,1->2,2");
  tree.insert({3, 3}, {4, 4}, "3,3->4,4");
  
  found = tree.search({0, 0}, {2, 2});
  expected = {"0,0->1,1", "1,1->2,2"};
  REQUIRE_THAT(found, Catch::Matchers::UnorderedEquals(expected));  

  bounds = tree.bounds();
  expected_bounds.low = {0, 0};
  expected_bounds.high = {4, 4};
  REQUIRE(bounds.low == expected_bounds.low);
  REQUIRE(bounds.high == expected_bounds.high);

  int removed = tree.remove({0, 0}, {0.5, 0.5});
  REQUIRE(removed == 1);

  bounds = tree.bounds();
  expected_bounds.low = {1, 1};
  REQUIRE(bounds.low == expected_bounds.low);
  REQUIRE(bounds.high == expected_bounds.high);

  found = tree.search({1, 1}, {1,1});
  expected = {"1,1->2,2"};
  REQUIRE(found == expected);

  tree.offset({10, 10});
  
  found = tree.search({1, 1}, {1, 1});
  expected = {};
  REQUIRE(found == expected);

  found = tree.search({11, 11}, {11,11});
  expected = {"1,1->2,2"};
  REQUIRE(found == expected);

  bounds = tree.bounds();
  expected_bounds.low = {11, 11};
  expected_bounds.high = {14, 14};
  REQUIRE(bounds.low == expected_bounds.low);
  REQUIRE(bounds.high == expected_bounds.high);
}
