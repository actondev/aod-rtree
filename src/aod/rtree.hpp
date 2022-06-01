#pragma once

#include "./rtree_base.hpp"
#include <immer/vector.hpp>
namespace aod {

#define PRE template <class DATATYPE>
#define QUAL Rtree<DATATYPE>

// inheriting from the base class: minimizing the header-only template
PRE class Rtree : public RtreeBase {
 private:
  // std::vector<DATATYPE> m_data;
  immer::vector<DATATYPE> m_data;
  void set_data(Did did, const DATATYPE &data);

  const DATATYPE &get_data(Did did) const;

 public:
  using RtreeBase::RtreeBase;
  using RtreeBase::remove; // base version of remove, without predicate

  using Predicate = std::function<bool(const DATATYPE &)>;
  using PredicateExt = std::function<bool(const DATATYPE &, const Rect&)>;
  
  using SearchCb = std::function<bool(const DATATYPE &)>;
  using SearchCbExt = std::function<bool(const DATATYPE &, const Rect&)>;
  
  void insert(const Vec &low, const Vec &high, const DATATYPE &data);

  std::vector<DATATYPE> search(const Vec &low, const Vec &high) const;

  int search(const Vec &low, const Vec &high, std::vector<DATATYPE> &results) const;
  int search(const Vec &low, const Vec &high, SearchCb) const;
  int search(const Vec &low, const Vec &high, SearchCbExt) const;

  int remove(const Vec &low, const Vec &high, Predicate);
  int remove(const Vec &low, const Vec &high, PredicateExt);

  template <typename Fn>
  void with_transaction(Fn fn) {
    Transaction transaction(this);
    fn(*this);
  }
};

// implementation

PRE void QUAL::set_data(Did did, const DATATYPE &data) {
  ASSERT(did);
  // auto trans = m_data.transient();
  m_data = m_data.push_back(data);
  // m_data[did.id] = data;
}

PRE const DATATYPE &QUAL::get_data(Did did) const {
  ASSERT(did);
  return m_data[did.id];
};

PRE void QUAL::insert(const Vec &low, const Vec &high, const DATATYPE &data) {
  const Did did = make_data_id();
  // m_data.resize(m_data_count);
  set_data(did, data);
  RtreeBase::insert(low, high, did);
}

PRE std::vector<DATATYPE> QUAL::search(const Vec &low, const Vec &high) const {
  std::vector<DATATYPE> results;
  search(low, high, results);
  return results;
}

PRE int QUAL::search(const Vec &low, const Vec &high,
                      std::vector<DATATYPE> &results) const {
  results.clear();
  RtreeBase::SearchCb cb = [&results, this](Eid e) {
    const Entry &entry = get_entry(e);
    results.push_back(get_data(entry.data_id));
    return true;
  };
  return RtreeBase::search(low, high, cb);
}

PRE int QUAL::search(const Vec &low, const Vec &high, SearchCb cb) const {
  RtreeBase::SearchCb base_cb = [&](Eid e) {
    const Entry& entry = get_entry(e);
    const DATATYPE &data = get_data(entry.data_id);
    return cb(data);
  };
  return RtreeBase::search(low, high, base_cb);
}

PRE int QUAL::search(const Vec &low, const Vec &high, SearchCbExt cb) const {
  Rect callback_rect;
  callback_rect.low.resize(m_dims);
  callback_rect.high.resize(m_dims);
  RtreeBase::SearchCb base_cb = [&](Eid e) {
    const Entry& entry = get_entry(e);
    const DATATYPE &data = get_data(entry.data_id);
    copy_rect(entry.rect_id, callback_rect);
    return cb(data, callback_rect);
  };
  return RtreeBase::search(low, high, base_cb);
}


PRE int QUAL::remove(const Vec &low, const Vec &high, Predicate pred) {
  RtreeBase::Predicate base_pred = [&](Eid e) {
    const Entry& entry = get_entry(e);
    const DATATYPE &data = get_data(entry.data_id);
    return pred(data);
  };

  return remove(low, high, base_pred);
}

PRE int QUAL::remove(const Vec &low, const Vec &high, PredicateExt pred) {
  Rect rect;
  rect.low.resize(m_dims);
  rect.high.resize(m_dims);

  RtreeBase::Predicate base_pred = [&](Eid e) {
    const Entry& entry = get_entry(e);
    const DATATYPE &data = get_data(entry.data_id);
    copy_rect(entry.rect_id, rect);
    return pred(data, rect);
  };

  return remove(low, high, base_pred);
}

#undef PRE
#undef QUAL

}
