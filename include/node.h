#ifndef NODE_H_
#define NODE_H_

#include <iostream>
#include <iomanip>

#include "global_common.h"
#include "visibility_set.h"

class Node;
using NodePtr = std::shared_ptr<Node>;

class Node {
  public:
    Node() = default;
    Node(const Idx index);
    Node(const NodePtr other);

    void DeepCopy(const NodePtr other);
    void CopyAsChild(const NodePtr other);
    void Print(std::ostream& out, const SizeType time) const;

    void SetIndex(const Idx index);
    Idx Index() const;

    void SetCostToCome(const RealNum cost);
    void IncreaseCostToComeBy(const RealNum addon_cost);
    RealNum CostToCome() const;


    void SetParent(NodePtr parent);
    void RemoveParent();
    NodePtr Parent() const;

    void SetVisSet(const VisibilitySet& set);
    void ExtendVisSet(const VisibilitySet& set);
    const VisibilitySet& VisSet() const;
    Idx CoverageSize() const;

    void SetSubsumed(const bool subsmued);
    bool IsSubsumed() const;

    void SetChecked(const bool checked);
    bool IsChecked() const;

    void SetValid(const bool valid);
    bool IsValid() const;

    bool BetterThan(const NodePtr other) const;
    void Extend(const Idx index, const RealNum cost, const VisibilitySet& set);
    void Update(const RealNum edge_cost, const VisibilitySet& set);

    void SetLocalPath(const std::vector<Idx>& path);
    std::vector<Idx> LocalPath() const;
    void AppendLocalPath(std::vector<Idx>* path) const;

    void Subsume(const NodePtr other, bool skip_update=false);
    SizeType NumberOfSubsumed() const;
    std::vector<NodePtr> SubsumedNodes() const;
    void ClearSubsumeHistory();

    void ResetSuccessorStatusID();
    void IncreaseSuccessorStatusID();
    void SetSuccessorStaturID(const Idx id);
    Idx SuccessorStatusID() const;

    bool Latest() const;
    void SetLatest(const bool if_latest);

    bool ReusedFromClosedSet() const;
    void SetReuseFromClosedSet(const bool reuse);

    bool IsBounded(const RealNum p, const RealNum eps) const;
    RealNum LocalPathCost() const;

    void SetSearchID(const Idx search_id);
    Idx SearchID() const;

    void SetExtendedGraphSize(const SizeType& size);
    SizeType ExtendedGraphSize() const;

#if USE_GHOST_DATA
    void SetGhostVisSet(const VisibilitySet& set);
    void ExtendGhostVisSet(const VisibilitySet& set);
    const VisibilitySet& GhostVisSet() const;
    VisibilitySet& GhostVisSet();
    Idx GhostCoverageSize() const;

    void SetGhostCost(const RealNum cost);
    void IncreaseGhostCostBy(const RealNum addon_cost);
    void UpdateGhostCost(const RealNum cost);
    RealNum GhostCost() const;
#endif

#if USE_HEURISTIC
    void SetHeuristic(const RealNum h);
    RealNum Heuristic() const;
#endif

  private:
    bool is_subsumed_{false};
    bool checked_collision_{false};
    bool valid_{true};
    bool if_latest_{true};
    bool reused_from_closed_set_{false};

    Idx index_{0};
    Idx successor_status_id_{0};
    Idx search_id_{0};
    SizeType num_subsumed_{0};
    RealNum cost_to_come_{0};
    RealNum local_path_cost_{0};
    VisibilitySet vis_set_;
    NodePtr parent_{nullptr};
    std::vector<Idx> local_path_;
    std::vector<NodePtr> subsumed_nodes_;

    SizeType extended_graph_size_{0};

#if USE_GHOST_DATA
    RealNum ghost_cost_ {0};
    VisibilitySet ghost_vis_set_;
#endif

#if USE_HEURISTIC
    RealNum h_ {0};
#endif

}; // Node

#endif // NODE_H_
