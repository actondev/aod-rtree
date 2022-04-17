#pragma once

#include "./rtree_base.hpp"
namespace aod {

#define PRE template <class DATATYPE>
#define QUAL Rtree<DATATYPE>

// inheriting from the base class: minimizing the header-only template
PRE class Rtree : public RtreeBase {
 private:
  std::vector<DATATYPE> m_data;
  void set_data(Did did, const DATATYPE &data);

  const DATATYPE &get_data(Did did);

 public:
  using RtreeBase::RtreeBase;
  using RtreeBase::remove; // base version of remove, without predicate

  using Predicate = std::function<bool(const DATATYPE &)>;
  using SearchCb = std::function<bool(const DATATYPE &)>;
  void insert(const Vec &low, const Vec &high, const DATATYPE &data);

  std::vector<DATATYPE> search(const Vec &low, const Vec &high);

  void search(const Vec &low, const Vec &high, std::vector<DATATYPE> &results);
  /// Return true from the search callback to continue searching,
  /// false to stop.
  void search(const Vec &low, const Vec &high, SearchCb);

  int remove(const Vec &low, const Vec &high, Predicate);
};

// implementation

PRE void QUAL::set_data(Did did, const DATATYPE &data) {
  ASSERT(did);
  m_data[did.id] = data;
}

PRE const DATATYPE &QUAL::get_data(Did did) {
  ASSERT(did);
  return m_data[did.id];
};

PRE void QUAL::insert(const Vec &low, const Vec &high, const DATATYPE &data) {
  const Did did = make_data_id();
  m_data.resize(m_data_count);
  set_data(did, data);
  RtreeBase::insert(low, high, did);
}

PRE std::vector<DATATYPE> QUAL::search(const Vec &low, const Vec &high) {
  std::vector<DATATYPE> results;
  search(low, high, results);
  return results;
}

PRE void QUAL::search(const Vec &low, const Vec &high,
                      std::vector<DATATYPE> &results) {
  results.clear();
  RtreeBase::SearchCb cb = [&](Did did) {
    results.push_back(get_data(did));
    return true;
  };
  RtreeBase::search(low, high, cb);
}

PRE void QUAL::search(const Vec &low, const Vec &high, SearchCb cb) {
  RtreeBase::SearchCb base_cb = [&](Did did) {
    const DATATYPE &data = get_data(did);
    return cb(data);
  };
  RtreeBase::search(low, high, cb);
}

PRE int QUAL::remove(const Vec &low, const Vec &high, Predicate pred) {
  RtreeBase::Predicate base_pred = [&](Did did) {
    const DATATYPE &data = get_data(did);
    return pred(data);
  };

  return remove(low, high, base_pred);
}

#undef PRE
#undef QUAL

}
