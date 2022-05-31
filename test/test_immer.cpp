#include <catch2/catch.hpp>
#include <immer/vector.hpp>
#include <immer/vector_transient.hpp>
#include <iostream>
using std::cout;
using std::endl;

TEST_CASE("immer basics", "[immer]") {
  immer::vector<int> vec;

  auto vec_tr = vec.transient();
  auto vec_tr2 = vec.transient();
  vec_tr.push_back(1);
  vec_tr2.push_back(2);
  // vec_tr[0] = 2; // nop!
  // vec_tr.set(0, 2);

  auto res = vec_tr2.persistent();
  cout << "final size " << res.size() << endl;
  for (auto x : res) {
    cout << "x " << x << endl;
  }

  // vec_tr.resize(10);
}


