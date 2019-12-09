#include "node.h"

Node::Node(const Idx index) : index_(index) {
	// vis sets are default to be empty
}
	
Node::Node(const NodePtr& other) {
	this->DeepCopy(other);
}

void Node::DeepCopy(const NodePtr& other) {
	valid_ = other->IsValid();
	index_ = other->Index();
	cost_to_come_ = other->CostToCome();
	vis_set_ = other->VisSet();
	parent_ = other->Parent();
	local_path_ = other->LocalPath();

	if_latest_  = other->Latest();

#if USE_GHOST_DATA
	ghost_cost_ = other->GhostCost();
	ghost_vis_set_ = other->GhostVisSet();
#endif

#if USE_HEURISTIC
	h_ = other->Heuristic();
#endif

}

void Node::CopyAsChild(const NodePtr& other) {
	valid_ = other->IsValid();
	index_ = other->Index();
	cost_to_come_ = other->CostToCome();
	vis_set_ = other->VisSet();
	parent_ = other;

#if USE_GHOST_DATA
	ghost_cost_ = other->GhostCost();
	ghost_vis_set_ = other->GhostVisSet();
#endif

#if USE_HEURISTIC
	h_ = other->Heuristic();
#endif

}

void Node::Print(std::ostream &out, const SizeType time) const {
	out << time << "\t\t"
	<< std::setiosflags(std::ios::fixed) 
	<< std::setprecision(4) 
	<< cost_to_come_ << "   \t"
	<< CoverageSize()*100.0 / RealNum(MAX_COVERAGE_SIZE) << "  \t";

#if USE_GHOST_DATA
	out << ghost_cost_ << "  \t"
	<< GhostCoverageSize()*100.0 / RealNum(MAX_COVERAGE_SIZE);
#endif

	out << std::endl;
}

void Node::SetIndex(const Idx index) {
	index_ = index;
}

Idx Node::Index() const {
	return index_;
}

void Node::SetCostToCome(const RealNum cost) {
	cost_to_come_ = cost;
}

void Node::IncreaseCostToComeBy(const RealNum addon_cost) {
	cost_to_come_ += addon_cost;
}

RealNum Node::CostToCome() const {
	return cost_to_come_;
}

void Node::SetParent(NodePtr& parent) {
	parent_ = parent;
}

NodePtr Node::Parent() const {
	return parent_;
}

void Node::SetVisSet(const VisibilitySet& set) {
	vis_set_ = set;
}

void Node::ExtendVisSet(const VisibilitySet& set) {
	vis_set_.Insert(set);
}

const VisibilitySet& Node::VisSet() const {
	return vis_set_;
}

Idx Node::CoverageSize() const {
	return vis_set_.Size();
}

void Node::SetSubsumed(const bool subsumed) {
	is_subsumed_ = subsumed;
}

bool Node::IsSubsumed() const {
	return is_subsumed_;
}

void Node::SetChecked(const bool checked) {
	checked_collision_ = checked;
}

bool Node::IsChecked() const {
	return checked_collision_;
}

void Node::SetValid(const bool valid) {
	valid_ = valid;
}

bool Node::IsValid() const {
	return valid_;
}

bool Node::BetterThan(const NodePtr& other) const {
	if (this->CoverageSize() < other->CoverageSize()) {
		return false;
	}
	
	if (this->CoverageSize() > other->CoverageSize()) {
		return true;
	}

	return (this->CostToCome() < other->CostToCome());
}

void Node::Extend(const Idx index, const RealNum cost, const VisibilitySet& set) {
	this->SetIndex(index);
	this->IncreaseCostToComeBy(cost);
	this->ExtendVisSet(set);

#if USE_GHOST_DATA
	this->IncreaseGhostCostBy(cost);
	this->ExtendGhostVisSet(set);
#endif
}

void Node::Update(const RealNum edge_cost, const VisibilitySet& set) {
	this->cost_to_come_ = this->parent_->CostToCome() + edge_cost;
	this->vis_set_.Insert(set);

#if USE_GHOST_DATA
	this->ghost_cost_ = std::min(this->ghost_cost_, this->parent_->GhostCost() + edge_cost);
	this->vis_set_.Insert(set);
#endif
}

void Node::SetLocalPath(const std::vector<Idx>& path) {
	local_path_ = path;
}

std::vector<Idx> Node::LocalPath() const {
	return local_path_;
}

void Node::AppendLocalPath(std::vector<Idx> *path) const {
	// local_path_ has no current index
	SizeType len = local_path_.size();
	for (Idx i = 1; i < len-1; ++i) {
		path->push_back(local_path_[i]);
	}
}

void Node::Subsume(const NodePtr& other) {
#if USE_GHOST_DATA
	this->UpdateGhostCost(other->GhostCost());
	this->ExtendGhostVisSet(other->GhostVisSet());
#endif

	other->SetSubsumed(true);
	num_subsumed_++;
}

SizeType Node::NumberOfSubsumed() const {
	return num_subsumed_;
}

void Node::ResetSuccessorStatusID() {
	successor_status_id_ = 0;
}

void Node::IncreaseSuccessorStatusID() {
	successor_status_id_++;
}

void Node::SetSuccessorStaturID(const Idx id) {
	successor_status_id_ = id;
}

Idx Node::SuccessorStatusID() const {
	return successor_status_id_;
}

bool Node::Latest() const {
	return if_latest_;
}

void Node::SetLatest(const bool if_latest) {
	if_latest_ = if_latest;
}


#if USE_GHOST_DATA

void Node::SetGhostVisSet(const VisibilitySet& set) {
	// if (ghost_vis_set_.Size() > 0) {
	// 	ghost_vis_set_.Clear();
	// }

	ghost_vis_set_ = set;
}

void Node::ExtendGhostVisSet(const VisibilitySet& set) {
	ghost_vis_set_.Insert(set);
}

const VisibilitySet& Node::GhostVisSet() const {
	return ghost_vis_set_;
}

VisibilitySet& Node::GhostVisSet() {
	return ghost_vis_set_;
}

Idx Node::GhostCoverageSize() const {
	return ghost_vis_set_.Size();
}

void Node::SetGhostCost(const RealNum cost) {
	ghost_cost_ = cost;
}

void Node::IncreaseGhostCostBy(const RealNum addon_cost) {
	ghost_cost_ += addon_cost;
}

void Node::UpdateGhostCost(const RealNum cost) {
	if (cost < ghost_cost_) {
		ghost_cost_ = cost;
	}
}

RealNum Node::GhostCost() const {
	return ghost_cost_;
}

#endif

#if USE_HEURISTIC

void Node::SetHeuristic(const RealNum h) {
	h_ = h;
}

RealNum Node::Heuristic() const {
	return h_;
}

#endif
