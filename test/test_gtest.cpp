#include <gtest/gtest.h>
#include "./common.hpp"

TEST(benchmark, aod_rtree_200x200) {
  int size = benchmark_size;
  auto grid = make_grid(size, 2);
  auto t1 = high_resolution_clock::now();
  aod::Rtree<Point> tree = grid_to_aod_rtree(grid);
  auto t2 = high_resolution_clock::now();
  RecordProperty("init", duration_ms(t2 - t1));
  cout <<"init size " << size << " took " << duration_ms(t2 - t1) << "ms" << endl;
  EXPECT_EQ(tree.size(), size*size);

  if (false) {
    std::ofstream ofs("aod-rtree-benchmark-init.xml", std::ofstream::out);
    ofs << tree.to_xml();
    ofs.close();
  }

  std::vector<Point> expected = {
    { 5.0, 2.0 },
    { 5.0, 3.0 },
    { 5.0, 4.0 },
    { 6.0, 2.0 },
    { 6.0, 3.0 },
    { 6.0, 4.0 },
  };
  std::vector<double> low = {5 , 2};
  std::vector<double> high = {6 , 4};
  auto found = tree.search(low, high);
  sort_points(found);
  EXPECT_EQ(found, expected);

  t1 = now();
  for (int i = 0; i < searches; i++) {
    tree.search(low, high, found);
  }
  t2 = now();
  cout <<search_str << duration_ms(t2 - t1) << " ms " << endl;

  t1 = now();
  aod::Rtree<Point>tree2 = tree;
  t2 = now();
  cout <<"copy took " << duration_ms(t2 - t1) << " ms " << endl;

  EXPECT_EQ(tree2.size(), tree.size());
  auto found2 = tree2.search(low, high);
  sort_points(found2);
  sort_points(expected);
  EXPECT_EQ(found2, expected);

  // removing fat cross
  t1 = now();
  // removing a fat cross in the center, leaving squares [side x side]
  int retain_side = 2;
  int expected_removed1 = size * (size-2*retain_side);
  int removed1 = tree.remove({0,retain_side},{size-1,size-1-retain_side});
  EXPECT_EQ(removed1, expected_removed1);
  int expected_removed2 = 2 * (size-2*retain_side)*retain_side;
  int removed2 = tree.remove({retain_side, 0}, {size-1-retain_side, size-1});
  EXPECT_EQ(removed2, expected_removed2);
  EXPECT_EQ(tree.size(),size*size - removed1 - removed2); // 4 left in each corner (2x2)
  EXPECT_EQ(tree.size(), (4*retain_side*retain_side));
  found = tree.search({0, 0}, {size, size});
  EXPECT_EQ(found.size(), (4*retain_side*retain_side));
  t2 = now();
  cout <<"removing fat cross took " << duration_ms(t2 - t1) << " ms " << endl;
  if(false){
    std::ofstream ofs("aod-rtree-benchmark-removed-fat-cross.xml", std::ofstream::out);
    ofs << tree.to_xml();
    ofs.close();
  }
}

