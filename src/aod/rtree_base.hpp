#pragma once

#include <cstdint>
#include <functional>
#include <limits>
#include <ostream>
#include <vector>
#include <array>
#include <assert.h>
#define ASSERT assert

namespace aod {

class RtreeBase {
 public:
  using ELEMTYPE = double;
  using Vec = std::vector<ELEMTYPE>;
  struct Rect {
    Vec low;
    Vec high;
  };

  /// Rect read only
  struct RectRo {
    const Vec& low;
    const Vec& high;
  };
  using id_t = uint32_t;

  struct Rid;
  struct Nid;
  struct Eid;
  struct Did;

  struct Id {
    static const id_t nullid = std::numeric_limits<id_t>::max();
    id_t id = nullid;
    inline operator bool() const { return id != nullid; }
    inline friend bool operator==(const Id &lhs, const Id &rhs) {
      return lhs.id == rhs.id;
    }
    // for ordered set
    inline friend bool operator<(const Id &lhs, const Id &rhs) {
      return lhs.id < rhs.id;
    }
  };

  // Rect Id
  struct Rid : Id {};
  /// Node Id
  struct Nid : Id {};
  /// Entry Id. Each node has `m <= count <= M` children
  struct Eid : Id {};
  /// Data id
  struct Did : Id {};
  
  struct Entry {
  Rid rect_id;
  Nid child_id;
  Did data_id;
};
  struct TraversalEntry {
  Eid entry;
  Nid node;
};

  struct Options {
    int m = 4;
    int M = 8;
  };

  struct Node {
    int count = 0;  ///< children entries count. Between m and M
    int height = 0; ///< zero for leaf, positive otherwise

    bool is_internal() const { return (height > 0); }
    bool is_leaf() const { return (height == 0); } // A leaf contains data
  };

  friend struct Xml;

  struct Xml {
    RtreeBase *tree;
    int spaces = 4;
    Xml() = delete;
    Xml(RtreeBase *tree) : tree{tree} {}
    friend std::ostream &operator<<(std::ostream &os, const Xml& xml);
  };
  using Traversal = std::vector<TraversalEntry>;

 protected:
  static Options default_options;
  using Traversals = std::vector<Traversal>;

  using Predicate = std::function<bool(Eid)>;
  using SearchCb = std::function<bool(Eid)>;

  /// Helper struct used in partitioning M+1 entries into two groups
  struct Partition {
    std::vector<Eid> entries; // up to M+1
    Rid groups_rects[2];      // each group's MBR
    ELEMTYPE groups_areas[2]; // area covered by each group's MBR
    Rid temp_rects[2];        // temporary rect for each group, comparing growth
    std::vector<ELEMTYPE> entries_areas;
    Rid cover_rect;
    ELEMTYPE cover_area = 0;
  };

  /// min entries per node. Could be hardcoded 2 as well, the paper
  /// shows good performance with hardcoded 2 as well
  const int m = 4;

  // max entries per node
  const int M = 8;

  int m_dims;

  size_t m_size = 0;
  id_t m_rects_count = 0;
  id_t m_internal_rects_count = 0;
  id_t m_entries_count = 0;
  id_t m_nodes_count = 0;
  id_t m_data_count = 0;

  Nid m_root_id;

  mutable Rid m_temp_rect;
  mutable Partition m_partition;
  mutable Traversal m_traversal;

  std::vector<ELEMTYPE> m_rects_low;
  std::vector<ELEMTYPE> m_rects_high;
  std::vector<Node> m_nodes;
  std::vector<Eid> m_node_entries;
  std::vector<Entry> m_entries;

  Rid make_rect_id();
  Nid make_node_id();
  Eid make_entry_id();
  Did make_data_id();

  Eid get_node_entry(Nid n, int idx) const;
  void set_node_entry(Nid n, int idx, Eid e);

  Node &get_node(Nid n);
  const Node &get_node(Nid n) const;

  inline Entry &get_entry(Eid e) { return m_entries[e.id]; }
  inline const Entry &get_entry(Eid e) const { return m_entries[e.id]; }

  ELEMTYPE rect_volume(Rid) const;
  inline ELEMTYPE rect_low_ro(const Rid r, const int dim) const {
    return m_rects_low[r.id * m_dims + dim];
  }
  inline const ELEMTYPE rect_high_ro(const Rid r, const int dim) const {
    return m_rects_high[r.id * m_dims + dim];
  }
  inline ELEMTYPE rect_low_ro(const RectRo& r, const int dim) const {
    return r.low[dim];
  }
  inline const ELEMTYPE rect_high_ro(const RectRo& r, const int dim) const {
    return r.high[dim];
  }
  // NB: these are defined as inline in the cpp, so they cannot be
  // used from the outside. Not sure how to deal with this
  ELEMTYPE &rect_low_rw(const Rid r, const int dim);
  ELEMTYPE &rect_high_rw(const Rid r, const int dim);
  bool rect_contains(Rid bigger, Rid smaller) const;

  bool rects_overlap(const RectRo&, Rid) const;
  bool rects_overlap(Rid, Rid) const;

  bool rect_contains_low(const RectRo& rect, Rid other) const;

  std::string rect_to_string(Rid);
  void rect_to_string(Rid, std::ostream &os);
  std::string traversal_to_string(const Traversal &traversal);
  std::string node_to_string(Nid nid, int level = 0, int spaces = 4);
  void node_to_string(Nid nid, int level, int spaces, std::ostream &);
  std::string entry_to_string(Eid, int level = 0, int spaces = 4);
  void entry_to_string(Eid, int level, int spaces, std::ostream &);

  void copy_rect(Rid src, Rid dst);
  void copy_rect(Rid src, Rect& dst) const;

  void insert(const Vec &low, const Vec &high, Did);

  int search(const Vec &low, const Vec &high, SearchCb cb) const;
  int search_low(const Vec &low, const Vec &high, SearchCb cb) const;

  int iterate(SearchCb cb) const;

  std::string to_string();

  void reinsert_entry(Eid e);
  /// Splits node, places the new M+1 entries (old & new one "e"), & returns the
  /// new node id
  Nid split_and_insert(Nid n, Eid e);

  /// Ascend from leaf node L to the root, adjusting rectangles and
  /// propagating node splits as necessary
  void adjust_tree(const Traversal &traversal, Eid e, Nid nn);
  void adjust_rects(const Traversal &traversal);

  using Seeds = std::array<size_t, 2>;
  Seeds pick_seeds(const std::vector<Eid> &entries);
  void distribute_entries(Nid n, Nid nn, std::vector<Eid> entries,
                          const Seeds &seeds);
  void distribute_entries_naive(Nid n, Nid nn, std::vector<Eid> entries);

  bool search(Nid, const RectRo&, int &found_count, SearchCb) const;
  bool search_low(Nid, const RectRo&, int &found_count, SearchCb) const;

  /// insert entry into leaf node: if a split occured, returns a valid new node
  Nid insert(Nid, Eid);
  void plain_insert(Nid, Eid);

  /// Given an R-tree whose root node is T, find the leaf node
  /// containing the index entry E
  Nid choose_leaf(Rid r, Traversal &traversal);
  Nid choose_node(Nid n, Rid r, int height, Traversal &traversal);
  Eid choose_subtree(Nid n, Rid r);

  /// Given a leaf node L, from which an entry has been deleted,
  /// eliminate the node if it has too few entries and relocate its
  /// entries. Propagate node elimination upward as necessary. Adjust
  /// all covering rectagles on the path to the root, making them
  /// smaller if possible
  void remove(Nid n, const RectRo& r, int &removed, Traversals &,
              const Traversal &cur_traversal, Predicate cb);

  bool remove_node_entry(Nid n, Eid e);
  void remove_node_entry(Nid n, int idx);

  /// Called after removal of Node(s), by passing the traversal(s) of
  /// the tree that took place for the removed entries
  void condense_tree(const Traversals &);

  int count(Nid n);
  void count(Nid n, int &); // recursive

  void combine_rects(Rid a, Rid b, Rid dst);
  void absorb_rect(Rid a, Rect& dst) const;
  void update_entry_rect(Eid e);

  void init();
public:
  Xml to_xml();
  size_t size();
  RtreeBase() = delete;
  RtreeBase(int dimensions, const Options& options = default_options);
  void to_string(int spaces, std::ostream &);
  int remove(const Vec &low, const Vec &high);
  int remove(const Vec &low, const Vec &high, Predicate);
  void clear();
  int dimensions() const;
  Rect bounds() const;
  void offset(const Vec&);

  #ifdef DEBUG
  bool has_duplicate_nodes();
  bool has_duplicate_entries();
  bool validate_mbrs();
  bool validate_mbrs(Eid e); // could be const but ..
  #endif

 private:
  // private declarations might be inlined in the cpp file!
  bool iterate(Nid, int &iterate_count, SearchCb cb) const;
};

}
