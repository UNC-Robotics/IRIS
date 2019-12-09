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
	Node(const NodePtr& other);

	void DeepCopy(const NodePtr& other);
	void CopyAsChild(const NodePtr& other);
	void Print(std::ostream &out, const SizeType time) const;

	void SetIndex(const Idx index);
	Idx Index() const;

	void SetCostToCome(const RealNum cost);
	void IncreaseCostToComeBy(const RealNum addon_cost);
	RealNum CostToCome() const;


	void SetParent(NodePtr& parent);
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

	bool BetterThan(const NodePtr& other) const;
	void Extend(const Idx index, const RealNum cost, const VisibilitySet& set);
	void Update(const RealNum edge_cost, const VisibilitySet& set);

	void SetLocalPath(const std::vector<Idx>& path);
	std::vector<Idx> LocalPath() const;
	void AppendLocalPath(std::vector<Idx> *path) const;

	void Subsume(const NodePtr& other);
	SizeType NumberOfSubsumed() const;

	void ResetSuccessorStatusID();
	void IncreaseSuccessorStatusID();
	void SetSuccessorStaturID(const Idx id);
	Idx SuccessorStatusID() const;

	bool Latest() const;
	void SetLatest(const bool if_latest);

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
	SizeType num_subsumed_{0};
	bool checked_collision_{false};
	bool valid_{true};
	Idx index_;
	RealNum cost_to_come_{0};
	VisibilitySet vis_set_;
	NodePtr parent_{nullptr};

	bool if_latest_{true};

	std::vector<Idx> local_path_;
	Idx successor_status_id_{0};

#if USE_GHOST_DATA
	RealNum ghost_cost_{0};
	VisibilitySet ghost_vis_set_;
#endif

#if USE_HEURISTIC
	RealNum h_{0};
#endif

}; // Node

#endif // NODE_H_
