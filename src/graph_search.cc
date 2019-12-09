#include "graph_search.h"

GraphSearch::GraphSearch(Inspection::GPtr graph) : graph_(graph) {
	virtual_graph_coverage_.Clear();
	result_.reset(new Node(0));
}

SizeType GraphSearch::ExpandVirtualGraph(SizeType new_size, bool lazy_computation) {
	if (virtual_graph_size_ == graph_->NumVertices()) {
		std::cout << "Pre-computed graph with size " << virtual_graph_size_ << " is exhausted!" << std::endl;
		return virtual_graph_size_;
	}

	if (new_size > graph_->NumVertices()) {
		new_size = graph_->NumVertices();
	}

	for (SizeType i = virtual_graph_size_; i < new_size; ++i) {
		Inspection::VPtr v = graph_->Vertex(i);
		time_vis_ += v->time_vis;
		time_build_ += v->time_build;
		virtual_graph_coverage_.Insert(v->vis);
	}

	for (SizeType i = 0; i < graph_->NumEdges(); ++i) {
		Inspection::EPtr e = graph_->Edge(i);
		if (e->source >= new_size) {
			break;
		}

		if ( (!e->in_virtual_graph) && e->target < new_size ) {
			e->in_virtual_graph = true;
			if (!lazy_computation || e->checked) {
                time_valid_ += e->time_forward_kinematics;
                time_valid_ += e->time_collision_detection;
                num_validated_++;
                e->virtual_checked = true;
			}
		}
	}

	virtual_graph_size_ = new_size;
	return virtual_graph_size_;
}

std::vector<Idx> GraphSearch::SearchVirtualGraphCompleteLazy(const RealNum p, const RealNum eps) {
    const TimePoint start = Clock::now();

#if USE_GHOST_DATA
    p_ = p;
    eps_ = eps;
#endif

    Idx source = 0;

    // search, local pointer for result
    NodePtr result_node;

    bool valid_result_found = false;
    while(!valid_result_found) {
        // traceback map
        map_.reset(new TracebackMap(virtual_graph_size_));
        for (SizeType i = 0; i < graph_->NumEdges(); ++i) {
            Inspection::EPtr e = graph_->Edge(i);
            
            // an edge should be in virtual graph then it should not be checked as invalid
            if (e->in_virtual_graph && !(e->virtual_checked && !e->valid) ) {
                map_->AddDirectEdge(e->source, e->target, e->cost);
            }
        }

#if DEBUG_MODE
        if (!map_->Connected(graph_)) {
            getchar();
        }
#endif

        result_node = nullptr;

        // reset data structures
        queue_.reset(new PriorityQueue);
        open_sets_.resize(virtual_graph_size_);
        closed_sets_.resize(virtual_graph_size_);
        
        // remember to clean up open sets and closed sets every new search
        for (Idx i = 0; i < virtual_graph_size_; ++i) {
            open_sets_[i].clear();
            closed_sets_[i].clear();
        }

        // prepare source node
        NodePtr source_node(new Node(graph_->Vertex(source)->index));
        source_node->SetVisSet(graph_->Vertex(source)->vis);
        source_node->SetChecked(true);
    #if USE_GHOST_DATA
        source_node->SetGhostVisSet(graph_->Vertex(source)->vis);
    #endif
        
        queue_->push(source_node);
        open_sets_[source].insert(source_node);

        // main search part
        bool found = false;
        while(!queue_->empty()) {
            auto n = PopFromQueueCompleteLazy();

            // not a valid node
            if (n == nullptr) { continue; }

            // update result
            if (result_node == nullptr || n->BetterThan(result_node)) {
                result_node = n;
            }

            // check for termination
            if (InGoalSet(n)) {
                found = true;
                // std::cout << "Node in goal set!" << std::endl;
                break; 
            }

            // extend a node
            ComputeAndAddSuccessorsCompleteLazy(n);
        }

        if (!found) {
            std::cerr << "Error!" << std::endl;
            getchar();
        }

        NodePtr tag = result_node;
        valid_result_found = true;
        while (tag != nullptr) {
            bool local_path_valid = ValidPath(tag->LocalPath());
            if (!local_path_valid) {
                valid_result_found = false;
                break;
            }
            tag = tag->Parent();
        }

        if (!valid_result_found) {
            std::cout << "Replan!" << std::endl;
        }
    }    

    if (result_node->BetterThan(result_)) {
        // copy result
        result_->DeepCopy(result_node);
    }
    
    // retrieve full path
    std::vector<Idx> path;
    NodePtr tag = result_node;
    path.push_back(tag->Index());
    while (tag) {
        auto super_edge = tag->LocalPath();

        for (auto i = 1; i < super_edge.size(); ++i) {
            path.push_back(super_edge[i]);
        }
        
        tag = tag->Parent();
    }

    std::reverse(path.begin(), path.end());

    time_search_ += RelativeTime(start);
    return path;

}

std::vector<Idx> GraphSearch::SearchVirtualGraph(const RealNum p, const RealNum eps) {
	const TimePoint start = Clock::now();

#if USE_GHOST_DATA
	p_ = p;
	eps_ = eps;
#endif

	Idx source = 0;

	// traceback map
	map_.reset(new TracebackMap(virtual_graph_size_));
	for (SizeType i = 0; i < graph_->NumEdges(); ++i) {
		Inspection::EPtr e = graph_->Edge(i);
		
		// an edge should be in virtual graph then it should not be checked as invalid
		if (e->in_virtual_graph && !(e->virtual_checked && !e->valid) ) {
			map_->AddDirectEdge(e->source, e->target, e->cost);
		}
	}

#if DEBUG_MODE
    if (!map_->Connected(graph_)) {
        getchar();
    }
#endif

	// search, local pointer for result
	NodePtr result_node = nullptr;

    // reset data structures
	queue_.reset(new PriorityQueue);
	open_sets_.resize(virtual_graph_size_);
	closed_sets_.resize(virtual_graph_size_);
	
	// remember to clean up open sets and closed sets every new search
	for (Idx i = 0; i < virtual_graph_size_; ++i) {
		open_sets_[i].clear();
		closed_sets_[i].clear();
	}

    // prepare source node
	NodePtr source_node(new Node(graph_->Vertex(source)->index));
	source_node->SetVisSet(graph_->Vertex(source)->vis);
	source_node->SetChecked(true);
#if USE_GHOST_DATA
	source_node->SetGhostVisSet(graph_->Vertex(source)->vis);
#endif
	
	queue_->push(source_node);
	open_sets_[source].insert(source_node);

	// main search part
	bool found = false;
	while(!queue_->empty()) {
		auto n = PopFromQueue();

		// not a valid node
		if (n == nullptr) { continue; }

		// updage result
		if (result_node == nullptr || n->BetterThan(result_node)) {
			result_node = n;
		}

		// check for termination
		if (InGoalSet(n)) {
			found = true;
			// std::cout << "Node in goal set!" << std::endl;
			break; 
		}

		// extend a node
		ComputeAndAddSuccessors(n);
	}

	if (!found) {
		std::cerr << "Error!" << std::endl;
		getchar();
	}

	if (result_node->BetterThan(result_)) {
		// copy result
		result_->DeepCopy(result_node);
	}
	
	// retrieve full path
	std::vector<Idx> path;
    NodePtr tag = result_node;
    path.push_back(tag->Index());
    while (tag) {
        auto super_edge = tag->LocalPath();

        for (auto i = 1; i < super_edge.size(); ++i) {
            path.push_back(super_edge[i]);
        }
        
        tag = tag->Parent();
    }

    std::reverse(path.begin(), path.end());

    time_search_ += RelativeTime(start);
    return path;
}

// this is important
NodePtr GraphSearch::PopFromQueue() {
    // pop a parent node
    auto node_to_pop = queue_->top();
    queue_->pop();

#if DEBUG_MODE
        std::cout << "Node popped: ";
        PrintNodeStatus(node_to_pop);
        std::cout << std::endl;
        getchar();
#endif

    // if this node is not subsumed by other nodes
    if (node_to_pop->Latest() && !node_to_pop->IsSubsumed()) {
        // if this node is the latest successor of its parent
        if (IsUpToDate(node_to_pop)) {
            if (!node_to_pop->IsChecked()) {
				bool local_path_valid = ValidPath(node_to_pop->LocalPath());
				node_to_pop->SetChecked(true);
				node_to_pop->SetValid(local_path_valid);

				if (!local_path_valid) {
					ComputeAndAddSuccessors(node_to_pop->Parent());
					return nullptr;
				}
			}

			// reset to 0, later match to its successors
			node_to_pop->ResetSuccessorStatusID();
            open_sets_[node_to_pop->Index()].erase(node_to_pop);
			closed_sets_[node_to_pop->Index()].insert(node_to_pop);
			return node_to_pop;
		}
	}

	return nullptr;
}

NodePtr GraphSearch::PopFromQueueCompleteLazy() {
    // pop a parent node
    auto node_to_pop = queue_->top();
    queue_->pop();

#if DEBUG_MODE
        std::cout << "Node popped: ";
        PrintNodeStatus(node_to_pop);
        std::cout << std::endl;
        getchar();
#endif

    // if this node is not subsumed by other nodes
    if (node_to_pop->Latest() && !node_to_pop->IsSubsumed()) {
        // if this node is the latest successor of its parent
        open_sets_[node_to_pop->Index()].erase(node_to_pop);
        closed_sets_[node_to_pop->Index()].insert(node_to_pop);
        return node_to_pop;
    }

    return nullptr;
}

bool GraphSearch::IsUpToDate(const NodePtr p) const {
    if (p->Parent() == nullptr) {
        return true;
    }
    
    if (p->IsChecked() && p->IsValid()) {
        return true;
    }
    
    auto parent_id = p->Parent()->SuccessorStatusID();
    auto self_id = p->SuccessorStatusID();
    
    return (self_id == parent_id);
}

void GraphSearch::ComputeAndAddSuccessors(const NodePtr parent) {
#if DEBUG_MODE
        std::cout << "Pre: ";
        PrintOpenSets();
        Idx i = CheckOpenSets();
        if (i > 0) {
            std::cout << "Pre: Open set is invalid at " << i << std::endl;
            getchar();
        }
        getchar();
#endif

    parent->IncreaseSuccessorStatusID();
    Idx id = parent->SuccessorStatusID();

    // auto successors = map_->Successors(parent->Index(), parent->VisSet(), graph_);
    auto successors = map_->FirstMeetSuccessors(parent->Index(), parent->VisSet(), graph_);
    for (const auto& s : successors) {
        Idx m = s[0];
        NodePtr new_node(new Node());
        new_node->CopyAsChild(parent);
        new_node->Extend(m, map_->Cost(m, parent->Index()), graph_->Vertex(m)->vis);
        new_node->SetLocalPath(s);
        new_node->SetSuccessorStaturID(id);

        if (DominatedByClosedState(new_node)) { continue; }
        if (DominatedByOpenState(new_node)) {
            if (parent->SuccessorStatusID() != id) { break; }
            continue;
        }

        if (parent->SuccessorStatusID() != id) { break; }

#if USE_HEURISTIC
        new_node->SetHeuristic(map_->ComputeHeuristic(m, new_node->VisSet(), graph_, virtual_graph_coverage_));
#endif
        open_sets_[new_node->Index()].insert(new_node);
        queue_->push(new_node);
    }

#if DEBUG_MODE
        std::cout << "Post: ";
        PrintOpenSets();
        i = CheckOpenSets();
        if (i > 0) {
            std::cout << "Post: Open set is invalid at " << i << std::endl;
            getchar();
        }
        getchar();
#endif
}

void GraphSearch::ComputeAndAddSuccessorsCompleteLazy(const NodePtr parent) {
#if DEBUG_MODE
        std::cout << "Pre: ";
        PrintOpenSets();
        getchar();
#endif

    // auto successors = map_->Successors(parent->Index(), parent->VisSet(), graph_);
    auto successors = map_->FirstMeetSuccessors(parent->Index(), parent->VisSet(), graph_);
    for (const auto& s : successors) {
        Idx m = s[0];
        NodePtr new_node(new Node());
        new_node->CopyAsChild(parent);
        new_node->Extend(m, map_->Cost(m, parent->Index()), graph_->Vertex(m)->vis);
        new_node->SetLocalPath(s);

        if (DominatedByClosedState(new_node)) { continue; }
        if (DominatedByOpenStateCompleteLazy(new_node)) { continue; }

#if USE_HEURISTIC
        new_node->SetHeuristic(map_->ComputeHeuristic(m, new_node->VisSet(), graph_, virtual_graph_coverage_));
#endif
        open_sets_[new_node->Index()].insert(new_node);
        queue_->push(new_node);
    }

#if DEBUG_MODE
        std::cout << "Post: ";
        PrintOpenSets();
        getchar();
#endif
}

void GraphSearch::PrintOpenSets() const {
    for (Idx i = 0; i < open_sets_.size(); ++i) {
        std::cout << "Open set " << i << " :" << std::endl;
        OpenSet open_set = *(open_sets_.begin()+i);
        for (auto node : open_set) {
            PrintNodeStatus(node);
        }
        std::cout << std::endl;
    }
}

void GraphSearch::PrintNodeStatus(const NodePtr node) const {
    std::cout << node->Index() << ", ";
    if (node->Parent()) {
        std::cout << node->Parent()->Index() << ", ";
    }
    else {
        std::cout << "nullptr, ";
    }
    std::cout << node->CostToCome() << ", "
    << node->CoverageSize() << ", "
    << node->GhostCost() << ", "
    << node->GhostCoverageSize() << ", "
    << node->IsSubsumed() << ", "
    << node->IsChecked() << ", "
    << node->IsValid() << ", "
    << IsUpToDate(node) << ", "
    << node->NumberOfSubsumed() << "; \t";
}

Idx GraphSearch::CheckOpenSets() const {
    for (Idx it = 0; it < open_sets_.size(); ++it) {
        OpenSet set = *(open_sets_.begin() + it);
        for (auto i = set.begin(); i != set.end(); ++i) {
            for (auto j = i; j != set.end(); ++j) {
                if (i != j && Dominates(*i, *j)) {
                    return it;
                }
            }

            NodePtr n = *i;
            if (n->NumberOfSubsumed() > 0) {
                if (!n->IsChecked()) {
                    std::cout << "not checked" << std::endl;
                    getchar();
                    return it;
                }

                if (!n->IsValid()) {
                    std::cout << "not valid" << std::endl;
                    getchar();
                    return it;
                }
            }
        }
    }

    return 0;
}

bool GraphSearch::InGoalSet(const NodePtr n) const {
#if USE_GHOST_DATA
	return (n->GhostVisSet() == virtual_graph_coverage_);
#endif

	return (n->VisSet() == virtual_graph_coverage_);
}

bool GraphSearch::DominatedByClosedState(const NodePtr node) const {
	auto states = closed_sets_[node->Index()];
	for (const auto s : states) {
#if USE_HEURISTIC
        if (s->CostToCome() < node->CostToCome() && s->VisSet().Contains(node->VisSet())) {
            return true;
        }
#else
		if (s->VisSet().Contains(node->VisSet())) {
			return true;
		}
#endif
	}

	return false;
}

bool GraphSearch::DominatedByOpenState(NodePtr& node) {
    // this is a copy
    Idx index = node->Index();
    auto states = open_sets_[index];
    
    // for (auto s : states) {
    for (auto it = states.begin(); it != states.end(); ++it) {
        NodePtr s = *it;
        // remove out of date nodes
        if (!IsUpToDate(s)) {
            open_sets_[index].erase(s);
            continue;
        }

        // early return for duplicates
        if (node->Parent() == s->Parent() && fabs(node->CostToCome() - s->CostToCome()) < EPS) {
            s->SetSuccessorStaturID(node->SuccessorStatusID());
            return true;
        }

        if (Dominates(s, node)) {
            if (!s->IsChecked()) {
                bool local_path_valid = ValidPath(s->LocalPath());
                s->SetChecked(true);
                s->SetValid(local_path_valid);

                if (!local_path_valid) {
                    open_sets_[index].erase(s);
                    ComputeAndAddSuccessors(s->Parent());

                    return DominatedByOpenState(node);
                }
            }

#if DEBUG_MODE
            std::cout << s->Index() << " as child of ";
            if (s->Parent()) {
                std::cout << s->Parent()->Index();
            } 
            else {
                std::cout << " nullptr ";
            }
            std::cout << " subsumes " << node->Index() << " as child of ";
            if (node->Parent()) {
                std::cout << node->Parent()->Index();
            } 
            else {
                std::cout << " nullptr ";
            }

            std::cout << std::endl;
            std::cout << s->IsChecked() << std::endl;
            getchar();
#endif

#if USE_GHOST_COST_AS_KEY
            auto cost_before = s->GhostCost();
#endif

            s->Subsume(node);

#if USE_GHOST_COST_AS_KEY
            auto cost_after = s->GhostCost();
            if (cost_after < cost_before) {
                NodePtr updated(new Node(s));
                s->SetLatest(false);
                open_sets_[index].erase(s);
                open_sets_[index].insert(updated);
                queue_->push(updated);
            }
#endif

            return true;
        }

        if (!IsUpToDate(node)) { return true; }

        if (Dominates(node, s)) {
            if (!node->IsChecked()) {
                bool local_path_valid = ValidPath(node->LocalPath());
                node->SetChecked(true);
                node->SetValid(local_path_valid);

                if (!local_path_valid) {
                    ComputeAndAddSuccessors(node->Parent());
                    return true;
                }
            }

#if DEBUG_MODE
            std::cout << node->Index() << " as child of ";
            if (node->Parent()) {
                std::cout << node->Parent()->Index();
            } 
            else {
                std::cout << " nullptr ";
            }
            std::cout << " subsumes " << s->Index() << " as child of ";
            if (s->Parent()) {
                std::cout << s->Parent()->Index();
            } 
            else {
                std::cout << " nullptr ";
            }

            std::cout << std::endl;
            std::cout << node->IsChecked() << std::endl;
            getchar();
#endif

            node->Subsume(s);
            open_sets_[index].erase(s);
        }
    }

    return false;
}

bool GraphSearch::DominatedByOpenStateCompleteLazy(NodePtr& node) {
    // this is a copy
    Idx index = node->Index();
    auto states = open_sets_[index];
    
    // for (auto s : states) {
    for (auto it = states.begin(); it != states.end(); ++it) {
        NodePtr s = *it;

        if (Dominates(s, node)) {
#if USE_GHOST_COST_AS_KEY
            auto cost_before = s->GhostCost();
#endif

            s->Subsume(node);

#if USE_GHOST_COST_AS_KEY
            auto cost_after = s->GhostCost();
            if (cost_after < cost_before) {
                NodePtr updated(new Node(s));
                s->SetLatest(false);
                open_sets_[index].erase(s);
                open_sets_[index].insert(updated);
                queue_->push(updated);
            }
#endif

            return true;
        }

        if (Dominates(node, s)) {
            node->Subsume(s);
            open_sets_[index].erase(s);
        }
    }

    return false;
}

bool GraphSearch::Dominates(const NodePtr n1, const NodePtr n2) const{
#if USE_GHOST_DATA
	VisibilitySet union_set = n2->GhostVisSet();
	union_set.Insert(n1->GhostVisSet());

	return ( (n1->CostToCome() <= (1+eps_)*(n2->GhostCost()))
			&& (n1->CoverageSize()/(RealNum)union_set.Size() >= p_) );
#else
	return (n1->VisSet().Contains(n2->VisSet())
			&& n1->CostToCome() <= n2->CostToCome());
#endif
}

bool GraphSearch::ValidPath(const std::vector<Idx>& path) {
	SizeType len = path.size();
	if (len > 0) {
		for (Idx i = len-1; i > 0; --i) {
			Inspection::EPtr edge = graph_->FindEdge(path[i-1], path[i]);
			// for an up to date successor, there is no invalid but checked edge
			if (!edge->virtual_checked) {
				time_valid_ += edge->time_forward_kinematics;
			        time_valid_ += edge->time_collision_detection;
			        num_validated_++;
				edge->virtual_checked = true;
				if (!edge->valid) {
					map_->RemoveDirectEdge(edge->source, edge->target);
					return false;
				}
			}
		}
	}

	return true;
}

SizeType GraphSearch::VirtualGraphCoverageSize() const {
	return virtual_graph_coverage_.Size();
}

SizeType GraphSearch::ResultCoverageSize() const {
	return result_->VisSet().Size();
}

RealNum GraphSearch::ResultCost() const {
	return result_->CostToCome();
}

void GraphSearch::PrintTitle(std::ostream &out) const {
    out << "GraphSize "
        << "GraphCoverage"
        << "P "
        << "EPS "
        << "Coverage "
        << "Cost "
        << "CoverRatio "
        << "VisTime "
        << "BuildTime "
        << "ValidationTime "
        << "SearchTime "
        << "TotalTime "
        << "NumEdgeValidated "
        << std::endl;
}

void GraphSearch::PrintResult(std::ostream &out) const {
	out << virtual_graph_size_ << " "
	<< virtual_graph_coverage_.Size() << " "
	<< p_ << " "
	<< eps_ << " "
	<< ResultCoverageSize() << " "
	<< ResultCost() << " "
	<< ResultCoverageSize()/(RealNum)MAX_COVERAGE_SIZE << " "
	<< time_vis_ << " "
	<< time_build_ << " "
	<< time_valid_ << " "
	<< time_search_ << " "
	<< time_vis_+time_build_+time_valid_+time_search_ << " "
	<< num_validated_ << " "
	<< std::endl;
}

SizeType GraphSearch::RelativeTime(const TimePoint start) const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start).count();
}

SizeType GraphSearch::TotalTime() const {
    return time_vis_+time_build_+time_valid_+time_search_;
}

void GraphSearch::SetMaxTimeAllowed(const SizeType& time) {
    max_time_allowed_ = time;
}

bool GraphSearch::CheckTermination() const {
    if (TotalTime() > max_time_allowed_) {
        std::cout << "Exceeding maximum time allowed!" << std::endl;
        PrintResult(std::cout);
        return true;
    }

    return false;
}
