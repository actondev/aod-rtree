#include "./rtree_base.hpp"

#include <immer/vector_transient.hpp>

#include <iostream>
#include <set>
#include <sstream>

#include <assert.h>
#define ASSERT assert

#ifndef Min
#define Min std::min
#endif // Min

#ifndef Max
#define Max std::max
#endif // Max

namespace aod {

#ifdef DEBUG
using std::cout;
#endif
using std::cout;

using std::endl;

int combine_rects_count = 0;

template <typename T>
std::vector<T> &resize(std::vector<T> &vec, size_t new_size) {
  vec.resize(new_size);
  return vec;
}

template <typename T> void assign(T &where, const T &what) {
  if (&where != &what) {
    where = what;
  }
}

template <typename T>
void resize(immer::vector_transient<T> &trans, size_t new_size, T default_value = T{}) {
  if (new_size < trans.size()) {
    trans.take(new_size);
  } else {
    while (trans.size() < new_size) {
      trans.push_back(default_value);
    }
  }
}

template <typename T> inline std::vector<T> &transient(std::vector<T> &x) {
  return x;
}
template <typename T> inline std::vector<T> &persistent(std::vector<T> &x) {
  return x;
}

template <typename T> inline auto transient(immer::vector<T> &x) {
  return x.transient();
}
template <typename T>
inline immer::vector<T> persistent(immer::vector_transient<T> &x) {
  return x.persistent();
}

RtreeBase::Transaction::Transaction(RtreeBase *tree) : tree(tree) {
  if (tree->m_is_in_transaction)
    return;

  is_active = true;
  tree->m_is_in_transaction = true;
  assert(!tree->m_transients);
  tree->m_transients = Transients{};
  Transients &transients = tree->m_transients.value();
  transients.low = transient(tree->m_rects_low);
  transients.high = transient(tree->m_rects_high);
  transients.entries = transient(tree->m_entries);
#if !MUTABLE_NODE_ENTRIES
  transients.node_entries = transient(tree->m_node_entries);
#endif
}

RtreeBase::Transaction::~Transaction() {
  if (!is_active)
    return;

  assign(tree->m_rects_low, persistent(tree->m_transients.value().low));
  assign(tree->m_rects_high, persistent(tree->m_transients.value().high));
  assign(tree->m_entries, persistent(tree->m_transients.value().entries));
#if !MUTABLE_NODE_ENTRIES
  assign(tree->m_node_entries, persistent(tree->m_transients.value().node_entries));
#endif

  tree->m_is_in_transaction = false;
  tree->m_transients = std::nullopt;
}

template <typename T> void container_set(std::vector<T> &vec, size_t idx, const T& value) {vec[idx] = value;}

template <typename T>
inline void container_set(immer::vector_transient<T> &vec, size_t idx,
                          const T value) {
  vec.set(idx, value);
}

std::ostream &operator<<(std::ostream &os, const RtreeBase::Eid &id) {
  os << "Eid{" << id.id << "}";
  return os;
}

std::ostream &operator<<(std::ostream &os, const RtreeBase::Nid &id) {
  os << "Nid{" << id.id << "}";
  return os;
}

std::ostream &operator<<(std::ostream &os, const RtreeBase::Rid &id) {
  os << "Rid{" << id.id << "}";
  return os;
}

std::ostream &operator<<(std::ostream &os, const RtreeBase::Did &id) {
  os << "Did{" << id.id << "}";
  return os;
}

std::ostream &operator<<(std::ostream &os, const RtreeBase::TraversalEntry &p) {
  os << "Parent{" << p.entry << "->" << p.node << "}";
  return os;
}

std::ostream &operator<<(std::ostream &os, const RtreeBase::Traversal &tr) {
  os << "Traversal{ " << endl;
  for (auto const &parent : tr) {
    os << parent << ", ";
  }
  os << " }";
  return os;
}

RtreeBase::Rid RtreeBase::make_rect_id() {
  Rid res{m_rects_count++};
  resize(m_transients.value().low, m_rects_count * m_dims, 0.0);
  resize(m_transients.value().high, m_rects_count * m_dims, 0.0);

  return res;
}
RtreeBase::Nid RtreeBase::make_node_id() {
  Nid nid{m_nodes_count++};
  m_nodes.resize(m_nodes_count);
  size_t sz_node_entries = m_nodes_count * M;
  resize(
#if MUTABLE_NODE_ENTRIES
      m_node_entries,
      #else
      m_transients.value().node_entries,
      #endif
      sz_node_entries);

  return nid;
}
RtreeBase::Eid RtreeBase::make_entry_id() {
  Eid e{m_entries_count++};
  resize(m_transients.value().entries, m_entries_count);

  Rid r = make_rect_id();
  set_entry_rect(e, r);
  return e;
}

void RtreeBase::set_entry_rect(Eid e, Rid r) {
  #if MUTABLE
  m_entries[e.id].rect_id = r;
  #else
  Entry entry = get_entry(e);
  entry.rect_id = r;
  container_set(m_transients.value().entries, e.id, entry);
  #endif
}
void RtreeBase::set_entry_data(Eid e, Did d) {
#if MUTABLE
  m_entries[e.id].data_id = d;
#else
  Entry entry = get_entry(e);
  entry.data_id = d;
  container_set(m_transients.value().entries, e.id, entry);
#endif
}

void RtreeBase::set_entry_child(Eid e, Nid n) {
  #if MUTABLE
  m_entries[e.id].child_id = n;
#else
  Entry entry = get_entry(e);
  entry.child_id = n;
  container_set(m_transients.value().entries, e.id, entry);
#endif
}

RtreeBase::Did RtreeBase::make_data_id() {
  Did d{m_data_count++};
  return d;
}

inline RtreeBase::Node &RtreeBase::get_node(Nid n) { return m_nodes[n.id]; }
inline const RtreeBase::Node &RtreeBase::get_node(Nid n) const {
  return m_nodes[n.id];
}

inline RtreeBase::Eid RtreeBase::get_node_entry(Nid n, int idx) const {
  const size_t loc = n.id * M + idx;
  #if MUTABLE_NODE_ENTRIES
  return m_node_entries[loc];
  #else
  return m_is_in_transaction ? m_transients.value().node_entries[loc] : m_node_entries[loc];
  #endif
}
inline void RtreeBase::set_node_entry(Nid n, int idx, Eid e) {
  const size_t loc = n.id * M + idx;

  container_set(
#if MUTABLE_NODE_ENTRIES
      m_node_entries,
#else
      m_transients.value().node_entries,
#endif
      loc, e);
}

inline RtreeBase::ELEMTYPE RtreeBase::rect_volume(Rid r) const {
  ELEMTYPE volume = (ELEMTYPE)1;

  for (int i = 0; i < m_dims; ++i) {
    volume *= rect_high_ro(r, i) - rect_low_ro(r, i);
  }

  ASSERT(volume >= (ELEMTYPE)0);

  return volume;
}
inline bool RtreeBase::rect_contains(Rid bigger, Rid smaller) const {
  for (int index = 0; index < m_dims; ++index) {
    if (rect_low_ro(bigger, index) > rect_low_ro(smaller, index) ||
        rect_high_ro(bigger, index) < rect_high_ro(smaller, index)) {
      return false;
    }
  }
  return true;
}

inline bool RtreeBase::rects_overlap(const RectRo &a, Rid b) const {
  for (int index = 0; index < m_dims; ++index) {
    if (rect_low_ro(a, index) > rect_high_ro(b, index) ||
        rect_low_ro(b, index) > rect_high_ro(a, index)) {
      return false;
    }
  }
  return true;
}

inline bool RtreeBase::rects_overlap(Rid a, Rid b) const {
  for (int index = 0; index < m_dims; ++index) {
    if (rect_low_ro(a, index) > rect_high_ro(b, index) ||
        rect_low_ro(b, index) > rect_high_ro(a, index)) {
      return false;
    }
  }
  return true;
}

inline void RtreeBase::copy_rect(Rid src, Rid dst) {
  Transients &transients = m_transients.value();
  for (int i = 0; i < m_dims; ++i) {
    container_set(transients.low, rect_index(dst, i), rect_low_ro(src, i));
    container_set(transients.high, rect_index(dst, i), rect_high_ro(src, i));
  }
}
void RtreeBase::copy_rect(Rid src, Rect &dst) const {
  for (int i = 0; i < m_dims; ++i) {
    dst.low[i] = rect_low_ro(src, i);
    dst.high[i] = rect_high_ro(src, i);
  }
}
inline void RtreeBase::combine_rects(Rid a, Rid b, Rid dst) {
  combine_rects_count++;
  Transients &transients = m_transients.value();
  for (int i = 0; i < m_dims; i++) {
    container_set(transients.low, rect_index(dst, i),
                  Min(rect_low_ro(a, i), rect_low_ro(b, i)));
    container_set(transients.high, rect_index(dst, i),
                  Max(rect_high_ro(a, i), rect_high_ro(b, i)));
  }
}

void RtreeBase::absorb_rect(Rid src, Rect &dst) const {
  for (int i = 0; i < m_dims; i++) {
    dst.low[i] = Min(dst.low[i], rect_low_ro(src, i));
    dst.high[i] = Max(dst.high[i], rect_high_ro(src, i));
  }
}

RtreeBase::RtreeBase(int dimensions, const Options &options)
    : m{options.m}, M{options.M}, m_dims(dimensions) {
  ASSERT(m_dims > 0);
  ASSERT(m < M);
  init();
}

void RtreeBase::init() {
  // needed for initializations (writing into transients)
  Transaction transaction(this);

  m_rects_count = 0;
  m_nodes_count = 0;
  m_internal_rects_count = 0;
  m_entries_count = 0;
  m_data_count = 0;
  m_size = 0;

  // m_rects_low.resize(0);
  // m_rects_high.resize(0);
  m_nodes.resize(0);
  // m_entries.resize(0);

  m_root_id = make_node_id();

  m_temp_rect = make_rect_id();
  m_partition.entries.resize(M + 1);

  m_partition.groups_rects[0] = make_rect_id();
  m_partition.groups_rects[1] = make_rect_id();

  m_partition.temp_rects[0] = make_rect_id();
  m_partition.temp_rects[1] = make_rect_id();
  m_partition.cover_rect = make_rect_id();
  m_partition.entries_areas.resize(M + 1);

  m_internal_rects_count = m_rects_count;
}

RtreeBase::Nid RtreeBase::choose_leaf(Rid r, Traversal &traversal) {
  TraversalEntry p;
  p.node = m_root_id;
  traversal.push_back(p);
  return choose_node(m_root_id, r, 0, traversal);
}

RtreeBase::Nid RtreeBase::choose_node(Nid n, Rid r, int height,
                                      Traversal &traversal) {
  ASSERT(n);
  ASSERT(r);
  Node &node = get_node(n);
  if (node.height == height) {
    return n;
  }
  Eid subtree = choose_subtree(n, r);
  Nid child = get_entry(subtree).child_id;
  TraversalEntry p;
  p.entry = subtree;
  p.node = child;
  traversal.push_back(p);

  ASSERT(subtree);
  return choose_node(child, r, height, traversal);
}

RtreeBase::Eid RtreeBase::choose_subtree(Nid n, Rid r) {
  // CL3. If N is not a leaf, let F be the entry in N whose rectangle
  // FI needs least enlargment to include EI. Resolved ties by
  // choosing the entry with the rectangle of smallest area.

  bool first_run = true;
  ELEMTYPE increase;
  ELEMTYPE best_incr = (ELEMTYPE)-1;
  ELEMTYPE area;
  ELEMTYPE best_area;
  Eid best;

  Node &node = get_node(n);
  ASSERT(node.count);

  for (int i = 0; i < node.count; i++) {
    Eid e = get_node_entry(n, i);
    const Entry &entry = get_entry(e);
    area = rect_volume(entry.rect_id);
    combine_rects(r, entry.rect_id, m_temp_rect);
    increase = rect_volume(m_temp_rect) - area;
    if ((increase < best_incr) || first_run) {
      // clearly better (or init)
      best = e;
      best_area = area;
      best_incr = increase;
      first_run = false;
    } else if ((increase == best_incr) && (area < best_area)) {
      // Resolved ties by choosing the entry with the rectangle of
      // smallest area
      best = e;
      best_area = area;
      best_incr = increase;
    }
  }
  return best;
}

void RtreeBase::update_entry_rect(Eid e) {
  const Entry &the_entry = get_entry(e);
  Rid r = the_entry.rect_id;
  Nid n = the_entry.child_id;
  if (!n)
    return;
  const Node &node = get_node(n);
  if (!node.count)
    return;
  // going through the child node entries
  Eid child0_id = get_node_entry(n, 0);
  const Entry &child0 = get_entry(child0_id);
  copy_rect(child0.rect_id, r);
  for (int i = 1; i < node.count; ++i) {
    Eid child = get_node_entry(n, i);
    combine_rects(get_entry(child).rect_id, r, r);
  }
}

void RtreeBase::insert(const Vec &low, const Vec &high, Did did) {
  Transaction transaction(this);

  ++m_size;
  // 1
  const Eid e = make_entry_id(); // also sets the rect
  set_entry_data(e, did);

  const Entry &entry = get_entry(e);
  const Rid r = entry.rect_id;

  Transients &transients = m_transients.value();

  for (int i = 0; i < m_dims; ++i) {
    container_set(transients.low, rect_index(r, i), low[i]);
    container_set(transients.high, rect_index(r, i), high[i]);
  }

  m_traversal.clear();
  const Nid n = choose_leaf(entry.rect_id, m_traversal);
  const Nid nn = insert(n, e);

  adjust_tree(m_traversal, e, nn);

  // cout << "inserted, now comb rects call count " << combine_rects_count << endl;
  combine_rects_count = 0;

}

void RtreeBase::reinsert_entry(Eid e) {
  const Entry &entry = get_entry(e);
  if (entry.child_id) {
    const Node &entry_node = get_node(entry.child_id);
    const Node &root = get_node(m_root_id);
    const bool should_split_reinsert = root.height <= entry_node.height;

    if (should_split_reinsert) {
      const int children_count = get_node(entry.child_id).count;
      for (int i = 0; i < children_count; ++i) {
        reinsert_entry(get_node_entry(entry.child_id, i));
      }
      // TODO entry id & child node id could be freed
      return;
    }
  }

  m_traversal.clear();
  TraversalEntry p_root;
  p_root.node = m_root_id;
  m_traversal.push_back(p_root);
  const int height = entry.child_id ? get_node(entry.child_id).height + 1 : 0;
  const Nid n = choose_node(m_root_id, entry.rect_id, height, m_traversal);
  const Nid nn = insert(n, e);

  adjust_tree(m_traversal, e, nn);
}

RtreeBase::Nid RtreeBase::insert(Nid n, Eid e) {
  Nid nn; // new node (falsy)
  Node &node = get_node(n);
  if (node.count < M) {
    plain_insert(n, e);
  } else {
    nn = split_and_insert(n, e);
    ASSERT(nn);
  }
  // NB: nn (new node - after split_and_insert) isn't yet a part of
  // the tree!
  return nn;
}

void RtreeBase::plain_insert(Nid n, Eid e) {
  Node &node = get_node(n);
  ASSERT(node.count < M);
  set_node_entry(n, node.count, e);
  ++node.count;
}

RtreeBase::Nid RtreeBase::split_and_insert(Nid n, Eid e) {
  Nid nn = make_node_id();
  Node &node = get_node(n);
  Node &new_node = get_node(nn);
  new_node.height = node.height; // important!
  ASSERT(node.count == M); // if node is not full, makes no sense being here

  std::vector<Eid> &entries = m_partition.entries;
  entries.clear();
  entries.resize(M + 1);

  for (int i = 0; i < node.count; ++i) {
    entries[i] = get_node_entry(n, i);
  }
  entries[M] = e; // the new entry

  Seeds seeds = pick_seeds(entries);

  node.count = 0;
  new_node.count = 0;

  distribute_entries(n, nn, entries, seeds);
  // debugging: just placing first half entries to n, rest to nn
  // distribute_entries_naive(n, nn, entries);

  ASSERT(node.count + new_node.count == M + 1);
  ASSERT(node.count >= m);
  ASSERT(new_node.count >= m);

  return nn;
}

void RtreeBase::distribute_entries(Nid n, Nid nn, std::vector<Eid> entries,
                                   const Seeds &seeds) {
  const Node &node = get_node(n);
  const Node &new_node = get_node(nn);

  plain_insert(n, entries[seeds[0]]);
  plain_insert(nn, entries[seeds[1]]);

  // initializing helper rects
  copy_rect(get_entry(entries[seeds[0]]).rect_id, m_partition.groups_rects[0]);
  copy_rect(get_entry(entries[seeds[1]]).rect_id, m_partition.groups_rects[1]);
  m_partition.groups_areas[0] = rect_volume(m_partition.groups_rects[0]);
  m_partition.groups_areas[1] = rect_volume(m_partition.groups_rects[1]);

  // quick remove seeds from the entries. First removing the 2nd seed, as it
  // will be bigger, cause it might be the last one
  entries[seeds[1]] = entries.back();
  entries.pop_back();
  entries[seeds[0]] = entries.back();
  entries.pop_back();

  ASSERT(entries.size() == static_cast<uint>(M - 1)); // (M+1) -2

  ELEMTYPE biggestDiff;
  uint group, entry_index = 0, betterGroup = 0;
  // existing node: group[0], new node: group[1]
  const Nid groups[2] = {n, nn};
  const Node *nodes[2] = {&node, &new_node};
  const int max_fill = M + 1 - m;
  while (!entries.empty() && node.count < max_fill &&
         new_node.count < max_fill) {
    biggestDiff = (ELEMTYPE)-1;
    for (uint i = 0; i < entries.size(); ++i) {
      const Entry &entry = get_entry(entries[i]);
      const Rid r = entry.rect_id;
      combine_rects(r, m_partition.groups_rects[0], m_partition.temp_rects[0]);
      combine_rects(r, m_partition.groups_rects[1], m_partition.temp_rects[1]);
      const ELEMTYPE growth0 =
          rect_volume(m_partition.temp_rects[0]) - m_partition.groups_areas[0];
      const ELEMTYPE growth1 =
          rect_volume(m_partition.temp_rects[1]) - m_partition.groups_areas[1];
      ELEMTYPE diff = growth1 - growth0;
      if (diff > 0) {
        group = 0;
      } else {
        group = 1;
        diff = -diff;
      }
      if (diff > biggestDiff) {
        biggestDiff = diff;
        entry_index = i;
        betterGroup = group;
      } else if ((diff == biggestDiff) &&
                 (nodes[group]->count < nodes[betterGroup]->count)) {
        entry_index = i;
        betterGroup = group;
      }
    }
    combine_rects(get_entry(entries[entry_index]).rect_id,
                  m_partition.groups_rects[betterGroup],
                  m_partition.groups_rects[betterGroup]);
    m_partition.groups_areas[betterGroup] =
        rect_volume(m_partition.groups_rects[betterGroup]);

    plain_insert(groups[betterGroup], entries[entry_index]);

    // quick remove:
    entries[entry_index] = entries.back();
    entries.pop_back();
  }
  // exited early cause one node filled too much
  if (node.count + new_node.count < M + 1) {
    Nid to_fill;
    if (node.count >= max_fill) {
      to_fill = nn;
    } else {
      to_fill = n;
    }
    for (const Eid &e : entries) {
      plain_insert(to_fill, e);
    }
  }
}

void RtreeBase::distribute_entries_naive(Nid n, Nid nn,
                                         std::vector<Eid> entries) {
  uint half = entries.size() / 2 + 1;
  for (uint i = 0; i < half; ++i) {
    plain_insert(n, entries[i]);
  }
  for (uint i = half; i < entries.size(); ++i) {
    plain_insert(nn, entries[i]);
  }
}

void RtreeBase::adjust_rects(const Traversal &traversal) {
  // important! no uint here (decreasing, want to reach 0)
  for (int i = static_cast<int>(traversal.size()) - 1; i >= 0; --i) {
    const TraversalEntry &parent = traversal[i];
    if (parent.entry) {
      update_entry_rect(parent.entry);
    }
  }
}

/// The traversal is a vector of Entry->Node pairs. First one will
/// have null entry -> root node. The last one will have enytry ->
/// leaf node, leaf node being the node returned by choose_leaf. Note:
/// newly inserted Entry e is not necessarily places into that leaf
/// node if a split took place (a split took place if nn isn't
/// nullish).
void RtreeBase::adjust_tree(const Traversal &traversal, Eid e, Nid nn) {
  Nid n;
  for (int level = traversal.size() - 1; level >= 0; --level) {
    const TraversalEntry &traversal_entry = traversal[level];
    n = traversal_entry.node; // in the first iteration, this is the leaf node
    // const Node &node = get_node(traversal_entry.node);
    // a parent is: entry -> node
    if (traversal_entry.entry) {
      // aka not root node. root node doesn't belong to an entry
      if (nn) {
        // if split (new node), need to readjust the parent entry
        // rect. Some leaf nodes now probably not part of the traversal, entries
        // have been redistributed!
        update_entry_rect(traversal_entry.entry);
      } else {
        // making room for newly inserted entry's rect
        const Entry &parent_entry = get_entry(traversal_entry.entry);
        combine_rects(parent_entry.rect_id, get_entry(e).rect_id,
                      parent_entry.rect_id);
      }
    }

    // bool is_above_new_node = level < (int)traversal.size() - 1;
    // propagating new node (split) upwards.
    if (nn && level >= 1) {
      const TraversalEntry &parent = traversal[level - 1];
#ifndef NDEBUG
      // new_node used only for assert, ifdef NDBEG : warning for unused
      // variable
      const Node &new_node = get_node(nn);
      ASSERT(get_node(parent.node).height == new_node.height + 1);
#endif

      const Eid ne = make_entry_id();
      set_entry_child(ne, nn);
      update_entry_rect(ne);

      nn = insert(parent.node, ne);
    }
  }
  ASSERT(n == m_root_id);
  if (nn) {
    // Root split!

    // Important: make_node_id before getting nodes! Otherwise,
    // references might be invalid.
    const Nid new_root_id = make_node_id();

    Node &new_root = get_node(new_root_id);
    const Node &existing_root = get_node(m_root_id);
    new_root.height = existing_root.height + 1;

#ifndef NDEBUG
    // new_node used only for assert, ifdef NDBEG : warning for unused variable
    const Node &new_node = get_node(nn);
    ASSERT(existing_root.height == new_node.height);
#endif

    const Eid old_root_e = make_entry_id();
    const Eid new_node_e = make_entry_id();

    set_entry_child(old_root_e, m_root_id);
    update_entry_rect(old_root_e);

    set_entry_child(new_node_e, nn);
    update_entry_rect(new_node_e);

    plain_insert(new_root_id, old_root_e);
    plain_insert(new_root_id, new_node_e);

    m_root_id = new_root_id;
  }
  // TODO prune tree? if root has only one child
}

/// Pick first entry for each group. We get 2 groups after splitting a
/// node. Returns the entries indexes of those seed entries.  Code
/// from Superliminal rtree
RtreeBase::Seeds RtreeBase::pick_seeds(const std::vector<Eid> &entries) {
  Seeds res;
  ASSERT(entries.size() == static_cast<uint>(M + 1));

  copy_rect(get_entry(entries[0]).rect_id, m_partition.cover_rect);
  for (size_t i = 0; i < entries.size(); ++i) {
    combine_rects(m_partition.cover_rect, get_entry(entries[i]).rect_id,
                  m_partition.cover_rect);
  }
  m_partition.cover_area = rect_volume(m_partition.cover_rect);

  int seed0 = 0, seed1 = 0;
  ELEMTYPE worst, waste;
  worst = -m_partition.cover_area - 1;
  for (uint i = 0; i < entries.size(); ++i) {
    m_partition.entries_areas[i] = rect_volume(get_entry(entries[i]).rect_id);
  }

  for (uint i = 0; i < entries.size() - 1; ++i) {
    for (uint j = i + 1; j < entries.size(); ++j) {
      combine_rects(get_entry(entries[i]).rect_id,
                    get_entry(entries[j]).rect_id, m_temp_rect);
      waste = rect_volume(m_temp_rect) - m_partition.entries_areas[i] -
              m_partition.entries_areas[j];
      if (waste > worst) {
        worst = waste;
        seed0 = i;
        seed1 = j;
      }
    }
  }
  ASSERT(seed0 != seed1);
  res[0] = seed0;
  res[1] = seed1;

  return res;
}

size_t RtreeBase::size() { return m_size; }

std::vector<RtreeBase::Eid> RtreeBase::search(const Vec &low, const Vec &high) const {
  std::vector<Eid> res;

  search(low, high, res);
  return res;
}

int RtreeBase::search(const Vec &low, const Vec &high,
                      std::vector<Eid> &results) const {
  ASSERT(low.size() == static_cast<uint>(m_dims));
  ASSERT(high.size() == static_cast<uint>(m_dims));

  SearchCb cb = [&results](const Eid &e) {
    results.push_back(e);
    return true;
  };

  int found_count = 0;
  RectRo r{low, high};
  search(m_root_id, r, found_count, cb);
  return found_count;
}

int RtreeBase::search(const Vec &low, const Vec &high, SearchCb cb) const {
  ASSERT(low.size() == static_cast<uint>(m_dims));
  ASSERT(high.size() == static_cast<uint>(m_dims));

  int found_count = 0;
  RectRo r{low, high};
  search(m_root_id, r, found_count, cb);
  return found_count;
}

bool RtreeBase::search(Nid n, const RectRo &r, int &found_count, SearchCb cb) const {
  const Node &node = get_node(n);
  if (node.is_internal()) {
    for (int i = 0; i < node.count; ++i) {
      const Eid e = get_node_entry(n, i);
      const Entry &entry = get_entry(e);
      if (rects_overlap(r, entry.rect_id)) {
        if (!search(entry.child_id, r, found_count, cb)) {
          // stop searching
          return false;
        }
      }
    }
  } else {
    // leaf
    for (int i = 0; i < node.count; ++i) {
      const Eid e = get_node_entry(n, i);
      const Entry &entry = get_entry(e);
      // TODO define some algorithm for search: overlap vs contain, etc..
      if (rects_overlap(r, entry.rect_id)) {
        if (!cb(e)) {
          // stop searching
          return false;
        } else {
          ++found_count;
        }
      }
    }
  }
  return true; // continue searching
}

// TODO could be const but I'm using copy_rect & combine_rects
RtreeBase::Rect RtreeBase::bounds() const {
  Rect res;
  res.low.resize(m_dims, 0);
  res.high.resize(m_dims, 0);
  const Node &root = get_node(m_root_id);
  if (!root.count)
    return res;

  const Entry &entry0 = get_entry(get_node_entry(m_root_id, 0));
  copy_rect(entry0.rect_id, res);
  for (int i = 0; i < root.count; ++i) {
    const Entry &entry = get_entry(get_node_entry(m_root_id, i));
    absorb_rect(entry.rect_id, res);
  }

  return res;
}

// TODO: could just store the offset & apply it in every public
// function's (insert, search, remove, bounds) input/output
void RtreeBase::offset(const Vec &offset) {
  Transaction transaction(this);
  ASSERT(offset.size() == m_dims);
  ASSERT(m_rects_low.size() == m_rects_high.size());

  Transients &transients = m_transients.value();

  for (int i = 0; i < m_rects_low.size(); i += m_dims) {
    for (int d = 0; d < m_dims; ++d) {
      container_set(transients.low, i + d, transients.low[i + d] + offset[d]);
      container_set(transients.high, i + d, transients.high[i + d] + offset[d]);
    }
  }
}

void RtreeBase::clear() { init(); }

int RtreeBase::dimensions() const { return m_dims; }

int RtreeBase::remove(const Vec &low, const Vec &high) {
  Predicate pred = [](Eid) { return true; };
  return remove(low, high, pred);
}

int RtreeBase::remove(const Vec &low, const Vec &high, Predicate pred) {
  Transaction transaction(this);

  ASSERT(low.size() == static_cast<uint>(m_dims));
  ASSERT(high.size() == static_cast<uint>(m_dims));

  RectRo r{low, high};
  int removed = 0;
  Traversals remove_traverals;
  Traversal cur_traversal;
  TraversalEntry p;
  p.node = m_root_id;
  cur_traversal.push_back(p);
  remove(m_root_id, r, removed, remove_traverals, cur_traversal, pred);

  // sorting remove_traverals: longer traverals first, thus leaf node
  // entries (data) removed first
  // TODO is this necessary?
  std::sort(remove_traverals.begin(), remove_traverals.end(),
            [](const Traversal &a, const Traversal &b) {
              return a.size() > b.size();
            });
  condense_tree(remove_traverals);

  m_size -= removed;
  return removed;
}

void RtreeBase::remove(Nid n, const RectRo &r, int &counter,
                       Traversals &traversals, const Traversal &cur_traversal,
                       Predicate cb) {
  Node &node = get_node(n);
  if (node.is_internal()) {
    for (int i = 0; i < node.count; ++i) {
      Eid e = get_node_entry(n, i);
      const Entry &entry = get_entry(e);
      if (rects_overlap(r, entry.rect_id)) {
        Traversal sub_traversal = cur_traversal;
        TraversalEntry p;
        p.entry = e;
        p.node = entry.child_id;
        sub_traversal.push_back(p);
        remove(entry.child_id, r, counter, traversals, sub_traversal, cb);
      }
    }
  } else {
    // leaf
    bool removed = false;
    for (int i = 0; i < node.count; ++i) {
      Eid e = get_node_entry(n, i);
      const Entry &entry = get_entry(e);
      if (rects_overlap(r, entry.rect_id)) {
        if (cb(e)) {
          removed = true;
          ++counter;
          remove_node_entry(n, i);
          --i; // need to revisit i again (remove_node_entry shuffles entries)
        }
      }
    }
    if (removed) {
      traversals.push_back(cur_traversal);
    }
  }
}

bool RtreeBase::remove_node_entry(Nid n, Eid e) {
  Node &node = get_node(n);
  int idx = -1;
  for (int i = 0; i < node.count; ++i) {
    if (get_node_entry(n, i) == e) {
      idx = i;
      break;
    }
  }
  if (idx == -1)
    return false;

  set_node_entry(n, idx, get_node_entry(n, node.count - 1));
  --node.count;
  return true;
}

void RtreeBase::remove_node_entry(Nid n, int idx) {
  Node &node = get_node(n);
  ASSERT(idx < node.count);
  set_node_entry(n, idx, get_node_entry(n, node.count - 1));
  --node.count;
}

void RtreeBase::condense_tree(const Traversals &traversals) {
  std::set<Eid> entries_to_reinsert;
  for (const Traversal &traversal : traversals) {
    for (int i = traversal.size() - 1; i >= 1; --i) {
      const TraversalEntry &x = traversal[i];
      Node &node = get_node(x.node);
      if (node.count < m) {
        const TraversalEntry &p = traversal[i - 1];
        if (node.count == 0) {
          // TODO should I just go on? seems like this was already processed
        }
        // Remove X from its parent p
        for (int j = 0; j < node.count; ++j) {
          // Add all children of X to S
          entries_to_reinsert.insert(get_node_entry(x.node, j));
        }
        node.count = 0;
        // this might return false, but it's fine (in case that entry
        // already removed)
        remove_node_entry(p.node, x.entry);
      }
    }
    adjust_rects(traversal);
  }

  // if root has no children, make it a leaf node
  Node &root = get_node(m_root_id);
  if (root.count == 0) {
    root.height = 0;
  }
  // skipping reinserting entries that point to empty node
  std::set<Eid>::iterator it = entries_to_reinsert.begin();
  while (it != entries_to_reinsert.end()) {
    auto current = it++;
    const Entry &entry = get_entry(*current);
    const Nid n = entry.child_id;
    if (n && get_node(n).count == 0) {
      entries_to_reinsert.erase(current);
    }
  }

  for (Eid e : entries_to_reinsert) {
    reinsert_entry(e);
  }
}

inline size_t RtreeBase::rect_index(Rid r, const int dim) const {
  return r.id * m_dims + dim;
}

inline RtreeBase::ELEMTYPE RtreeBase::rect_low_ro(const Rid r, const int dim) const {
  const auto idx = r.id * m_dims + dim;
  return m_transients ? m_transients.value().low[idx] : m_rects_low[idx];
}

inline RtreeBase::ELEMTYPE RtreeBase::rect_high_ro(const Rid r, const int dim) const {
  const auto idx = r.id * m_dims + dim;
  return m_transients ? m_transients.value().high[idx] : m_rects_high[idx];
}

inline RtreeBase::ELEMTYPE RtreeBase::rect_low_ro(const RectRo &r, const int dim) const {
  return r.low[dim];
}
inline RtreeBase::ELEMTYPE RtreeBase::rect_high_ro(const RectRo &r, const int dim) const {
  return r.high[dim];
}

// debug/print related functions from now on (probably)

int RtreeBase::count(Nid n) {
  int counter = 0;
  count(n, counter);
  return counter;
}

void RtreeBase::count(Nid n, int &counter) {
  const Node &node = get_node(n);
  if (node.is_internal()) {
    for (int i = 0; i < node.count; ++i) {
      const Entry &entry = get_entry(get_node_entry(n, i));
      ASSERT(entry.child_id);
      count(entry.child_id, counter);
    }
  } else {
    // leaf
    counter += node.count;
  }
}

void RtreeBase::entry_to_string(Eid e, int level, int spaces,
                                std::ostream &os) {
  if (!e)
    return;
  auto indent = [&]() {
    for (int i = 0; i < level * spaces; ++i) {
      os << " ";
    }
  };
  const Entry &entry = get_entry(e);
  Rid r = entry.rect_id;
  indent();
  os << " <Entry id=\"" << e.id << "\"";
  os << " rect-id=\"" << r.id << "\"";
  os << " mbr=\"";
  rect_to_string(r, os);
  os << "\"";
  if (entry.data_id) {
    // self closing entry, adding data-id
    os << " data-id=\"" << entry.data_id.id << "\"";
    // os << " data=\"" << pp(get_data(entry.data_id)) << "\"";
    os << " />" << endl;
  } else if (entry.child_id) {
    // child node
    os << " >" << endl;
    node_to_string(entry.child_id, level + 1, spaces, os);
    indent();
    os << " </Entry>" << endl;
  } else {
    // should not reach this
    os << " />" << endl;
  }
}

std::string RtreeBase::entry_to_string(Eid e, int level, int spaces) {
  std::ostringstream os;
  entry_to_string(e, level, spaces, os);
  return os.str();
};

void RtreeBase::node_to_string(Nid n, int level, int spaces, std::ostream &os) {
  if (!n)
    return;
  Node node = get_node(n);
  auto indent = [&]() {
    for (int i = 0; i < level * spaces; ++i) {
      os << " ";
    }
  };
  indent();
  os << "<Node id=\"" << n.id << "\" height=\"" << node.height
     << "\" children=\"" << node.count << "\" >" << endl;
  for (int i = 0; i < node.count; i++) {
    Eid e = get_node_entry(n, i);
    entry_to_string(e, level, spaces, os);
  }
  indent();
  os << "</Node>" << endl;
};

std::string RtreeBase::node_to_string(Nid nid, int level, int spaces) {
  std::ostringstream os;
  node_to_string(nid, level, spaces, os);
  return os.str();
};

std::string RtreeBase::to_string() {
  std::ostringstream os;
  to_string(4, os);
  return os.str();
}

void RtreeBase::to_string(int spaces, std::ostream &os) {
  node_to_string(m_root_id, 0, spaces, os);
}

void RtreeBase::rect_to_string(Rid rid, std::ostream &os) {
  ASSERT(rid);
  for (int i = 0; i < m_dims; i++) {
    if (i == 0) {
      os << "{";
    }
    os << rect_low_ro(rid, i);
    if (i != m_dims - 1) {
      os << ", ";
    } else {
      os << "}";
    }
  }

  os << "...";

  for (int i = 0; i < m_dims; i++) {
    if (i == 0) {
      os << "{";
    }
    os << rect_high_ro(rid, i);
    if (i != m_dims - 1) {
      os << ", ";
    } else {
      os << "}";
    }
  }
}

std::string RtreeBase::rect_to_string(Rid rid) {
  std::ostringstream os;
  rect_to_string(rid, os);
  return os.str();
}

std::ostream &operator<<(std::ostream &os, const RtreeBase::Xml &xml) {
  os << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\" ?>"
     << std::endl;
  xml.tree->to_string(xml.spaces, os);
  return os;
}

RtreeBase::Xml RtreeBase::to_xml() { return Xml(this); }

RtreeBase::Options RtreeBase::default_options;

#ifdef DEBUG
bool RtreeBase::validate_mbrs() {
  Transaction transaction(this);

  Nid n = m_root_id;
  const Node &node = get_node(n);
  for (int i = 0; i < node.count; ++i) {
    if (!validate_mbrs(get_node_entry(n, i))) {
      return false;
    }
  }

  return true;
}

bool RtreeBase::validate_mbrs(Eid e) {
  const Entry &entry = get_entry(e);
  Nid n = entry.child_id;
  Rid r = entry.rect_id;
  if (!n)
    return true;
  const Eid child0 = get_node_entry(n, 0);
  const Rid &child0_r = get_entry(child0).rect_id;
  const Node &node = get_node(n);

  copy_rect(child0_r, m_temp_rect);
  for (int i = 1; i < node.count; ++i) {
    const Rid child_r = get_entry(get_node_entry(n, i)).rect_id;
    combine_rects(child_r, m_temp_rect, m_temp_rect);
  }

  // entry rect r & temp_rect must be the same
  for (int i = 0; i < m_dims; ++i) {
    if (rect_low_ro(m_temp_rect, i) != rect_low_ro(r, i) ||
        rect_high_ro(m_temp_rect, i) != rect_high_ro(r, i)) {
      cout << "invalid mbr " << r << " " << rect_to_string(r) << " vs "
           << rect_to_string(m_temp_rect) << endl;
      return false;
    }
  }

  for (int i = 0; i < node.count; ++i) {
    if (!validate_mbrs(get_node_entry(n, i))) {
      return false;
    }
  }
  return true;
}

bool RtreeBase::has_duplicate_nodes() {
  std::set<id_t> nodes;

  std::function<bool(Nid)> traverse = [&](Nid n) {
    const Node &node = get_node(n);
    if (nodes.count(n.id) > 0) {
      cout << "Node " << n << " already in the container" << endl;
      return true;
    }
    nodes.insert(n.id);
    if (!node.is_internal()) {
      // leaf
      return false;
    }
    for (int i = 0; i < node.count; ++i) {
      Eid e = get_node_entry(n, i);
      const Entry &entry = get_entry(e);
      if (traverse(entry.child_id)) {
        return true;
      }
    }
    return false;
  };

  return traverse(m_root_id);
}

bool RtreeBase::has_duplicate_entries() {
  std::set<id_t> entries;

  std::function<bool(Nid)> traverse = [&](Nid n) {
    Node &node = get_node(n);

    for (int i = 0; i < node.count; ++i) {
      Eid e = get_node_entry(n, i);

      if (entries.count(e.id) > 0) {
        cout << "Entry " << e << " already in the container" << endl;
        return true;
      }
      entries.insert(e.id);

      const Entry &entry = get_entry(e);
      if (node.is_internal() && traverse(entry.child_id)) {
        return true;
      }
    }
    return false;
  };

  return traverse(m_root_id);
}

#endif // debug

} // namespace aod
