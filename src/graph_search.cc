#include "graph_search.h"

GraphSearch::GraphSearch(const Inspection::GPtr graph) : graph_(graph) {
    virtual_graph_coverage_.Clear();
    open_sets_.clear();
    closed_sets_.clear();
}

void GraphSearch::SetLazinessMode(const Idx mode_id) {
    if (mode_id >= kLazinessMap.size()) {
        std::cerr << "Invalid laziness mode! Keep using old mode "
                  << kLazinessMap.at(laziness_mode_)
                  << std::endl;
        return;
    }

#if KEEP_SUBSUMING_HISTORY == 0

    if (kLazinessMap.at(laziness_mode_) == "LazyA*") {
        std::cerr << "LazyA* cannot work without subsuming history"
                  << ", keep using "
                  << kSuccessorMap.at(successor_mode_)
                  << std::endl;
        return;
    }

#endif

    laziness_mode_ = mode_id;
    std::cout << "Set laziness mode to "
              << kLazinessMap.at(mode_id)
              << std::endl;
}

void GraphSearch::SetSuccessorMode(const Idx mode_id) {
    if (mode_id >= kSuccessorMap.size()) {
        std::cerr << "Invalid successor mode! Keep using old mode "
                  << kSuccessorMap.at(successor_mode_)
                  << std::endl;
        return;
    }

#if USE_NODE_REUSE

    if (kSuccessorMap.at(mode_id) != "direct") {
        std::cerr << "Node reusing does not support successor mode "
                  << kSuccessorMap.at(mode_id)
                  << ", keep using "
                  << kSuccessorMap.at(successor_mode_)
                  << std::endl;

        return;
    }

#endif

    if (mode_id > 0 && laziness_mode_ > 1) {
        std::cerr << kLazinessMap.at(laziness_mode_)
                  << " does not support successor mode "
                  << kSuccessorMap.at(mode_id)
                  << ", keep using "
                  << kSuccessorMap.at(successor_mode_)
                  << std::endl;

        return;
    }

    successor_mode_ = mode_id;
    std::cout << "Set successor mode to "
              << kSuccessorMap.at(mode_id)
              << std::endl;
}

void GraphSearch::SetSourceIndex(const Idx source) {
    source_idx_ = source;
}

void GraphSearch::UpdateApproximationParameters(const RealNum eps, const RealNum p) {
#if USE_GHOST_DATA
    eps_ = eps < 0.0 ? 0.0 : eps;
    p_ = p > 1.0 ? 1.0 : p;
#endif
}

SizeType GraphSearch::ExpandVirtualGraph(SizeType new_size) {
    if (virtual_graph_size_ == graph_->NumVertices()) {
        std::cout << "Pre-computed graph with size " << virtual_graph_size_ << " is exhausted!" <<
                  std::endl;
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
            virtual_graph_num_edges_++;

            if (kLazinessMap.at(laziness_mode_) == "no lazy" || e->checked) {
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

std::vector<Idx> GraphSearch::SearchVirtualGraph() {
    const TimePoint start = Clock::now();

    NodePtr result_node = nullptr;
    bool valid_result_found = false;
    SizeType num_replan = 0;
    num_nodes_extended_ = 0;
    num_nodes_generated_ = 0;
    num_nodes_remained_ = 0;

    while(!valid_result_found) {
        // Reset result node.
        result_node = nullptr;

        InitDataStructures();

        // Search starts.
        bool found = false;

        while(!queue_->empty()) {
            NodePtr n = PopFromPriorityQueue();

            // Not a valid node.
            if (n == nullptr) {
                continue;
            }

            // Updage result.
            if (result_node == nullptr || n->BetterThan(result_node)) {
                result_node = n;
            }

            // Check for termination.
            if (InGoalSet(n)) {
                found = true;
                break;
            }

            open_sets_[n->Index()].erase(n);

            // Extend a node.
            num_nodes_extended_++;
            Extend(n);

#if USE_NODE_REUSE

            if (n->ReusedFromClosedSet()) {
                // Node is already in closed set.
                continue;
            }

#endif
            n->SetSearchID(search_id_);
            closed_sets_[n->Index()].insert(n);
        }

        if (!found) {
            std::cerr << "[ERROR] Search terminated without finding a valid result!" << std::endl;
            exit(1);
        }

        valid_result_found = true;

        if (kLazinessMap.at(laziness_mode_) == "LazySP") {
            // If using completely lazy, should check result plan.
            NodePtr tag = result_node;

            while (tag != nullptr) {
                bool local_path_valid = ValidPath(tag->LocalPath());

                if (!local_path_valid) {
                    valid_result_found = false;
                    break;
                }

                tag = tag->Parent();
            }

            if (!valid_result_found) {
                std::cout << "\rReplan: " << ++num_replan << std::flush;
            }
            else {
                std::cout << std::endl;
            }
        }
    }

    prev_graph_size_ = virtual_graph_size_;
    search_id_++;

    // Update global result.
    if (result_ == nullptr) {
        result_.reset(new Node(result_node));
    }
    else if(result_node->BetterThan(result_)) {
        result_->DeepCopy(result_node);
    }

    // Retrieve full path.
    std::vector<Idx> path;
    auto tag = result_node;
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

void GraphSearch::InitDataStructures() {
    // Prepare traceback map.
    map_.reset(new TracebackMap(virtual_graph_size_));

    for (SizeType i = 0; i < graph_->NumEdges(); ++i) {
        Inspection::EPtr e = graph_->Edge(i);

        // An edge should be in virtual graph and it should not be checked as invalid.
        if (e->in_virtual_graph && !(e->virtual_checked && !e->valid) ) {
            map_->AddDirectEdge(e->source, e->target, e->cost);
        }
    }

    // Reset data structures.
    queue_.reset(new PriorityQueue);
    open_sets_.resize(virtual_graph_size_);
    closed_sets_.resize(virtual_graph_size_);

#if USE_NODE_REUSE
    this->UpdateUnboundedNodes();
#else

    for (SizeType i = 0; i < virtual_graph_size_; ++i) {
        open_sets_[i].clear();
        closed_sets_[i].clear();
    }

#endif

    if (queue_->empty()) {
        // Prepare source node.
        NodePtr source_node(new Node(graph_->Vertex(source_idx_)->index));
        source_node->SetVisSet(graph_->Vertex(source_idx_)->vis);
        source_node->SetChecked(true);
#if USE_GHOST_DATA
        source_node->SetGhostVisSet(graph_->Vertex(source_idx_)->vis);
#endif
        queue_->push(source_node);
        open_sets_[source_idx_].insert(source_node);
    }
}

NodePtr GraphSearch::PopFromPriorityQueue() {
    // Pop from priority queue;
    auto node_to_pop = queue_->top();
    queue_->pop();

#if USE_NODE_REUSE

    if (node_to_pop->ReusedFromClosedSet()) {
        // A reused node must be latest and not subsumed.
        return node_to_pop;
    }

#endif

    if (!node_to_pop->Latest()) {
        // This is not the latest version of the node.
        return nullptr;
    }

    if (node_to_pop->IsSubsumed()) {
        // This node is subsumed by another node.
        return nullptr;
    }

    if (kLazinessMap.at(laziness_mode_) == "LazyA* modified") {
        if (kSuccessorMap.at(successor_mode_) == "direct") {
            if (!node_to_pop->IsChecked()) {
                if (!Valid(node_to_pop)) {
                    // Invalid node is discarded.
                    open_sets_[node_to_pop->Index()].erase(node_to_pop);
                    return nullptr;
                }
            }
        }
        else {
            std::cerr << "[ERROR] Not implemented!" << std::endl;
            exit(1);
        }
    }
    else if (kLazinessMap.at(laziness_mode_) == "LazyA*") {
        if (!Valid(node_to_pop)) {
            // Invalid node is discarded.
            open_sets_[node_to_pop->Index()].erase(node_to_pop);
#if SAVE_PREDECESSOR
            Idx index = node_to_pop->Index();
#endif

            for (auto n : node_to_pop->SubsumedNodes()) {
#if SAVE_PREDECESSOR

                if (!map_->EdgeExists(index, n->Index())
                        || n->SearchID() < search_id_) {
                    continue;
                }

                NodePtr new_node(new Node());
                new_node->CopyAsChild(n);
                new_node->Extend(index, map_->EdgeCost(index, n->Index()), graph_->Vertex(index)->vis);
                new_node->SetLocalPath(std::vector<Idx> {index, n->Index()});

                AddNode(new_node);
#else
                n->SetSubsumed(false);
                RecursivelyAddNode(n);
#endif
            }

            node_to_pop->ClearSubsumeHistory();
            return nullptr;
        }
    }

    return node_to_pop;
}

void GraphSearch::Extend(NodePtr n) {
    std::vector<std::vector<Idx>> successors;

    if (kSuccessorMap.at(successor_mode_) == "direct") {
        successors = map_->NeighboringSuccessors(n->Index());
    }
    else if (kSuccessorMap.at(successor_mode_) == "expanded") {
        successors = map_->Successors(n->Index(), n->VisSet(), graph_);
    }
    else if (kSuccessorMap.at(successor_mode_) == "first-meet") {
        successors = map_->FirstMeetSuccessors(n->Index(), n->VisSet(), graph_);
    }

    for (const auto& s : successors) {
#if USE_NODE_REUSE

        if (n->ReusedFromClosedSet() && !NewNodesInvolved(n, s)) {
            continue;
        }

#endif

        Idx v = s[0];
        NodePtr new_node(new Node());
        new_node->CopyAsChild(n);
        new_node->Extend(v, map_->Cost(v, n->Index()), graph_->Vertex(v)->vis);
        new_node->SetLocalPath(s);
        num_nodes_generated_++;

#if USE_HEURISTIC
        new_node->SetHeuristic(map_->ComputeHeuristic(v, new_node->VisSet(), graph_,
                               virtual_graph_coverage_));
#endif

        if (AddNode(new_node)) {
            num_nodes_remained_++;
        }
    }

    n->SetExtendedGraphSize(virtual_graph_size_);

//     if (kLazinessMap.at(laziness_mode_) == "LazyA* modified") {
// #if KEEP_SUBSUMING_HISTORY
//                 ComputeAndAddSuccessorsCompleteLazy(n);
// #else
//                 ComputeAndAddSuccessors(n);
// #endif
//             }
//             else {
//                 ComputeAndAddSuccessorsCompleteLazy(n);
//             }
}

bool GraphSearch::AddNode(NodePtr n, const bool skip_queue_operations) {
    if (DominatedByClosedState(n)) {
        return false;
    }

    if (SubsumedByOpenState(n, skip_queue_operations)) {
        return false;
    }

    open_sets_[n->Index()].insert(n);

    if (!skip_queue_operations) {
        queue_->push(n);
    }

    return true;
}

void GraphSearch::RecursivelyAddNode(NodePtr n, const bool skip_queue_operations) {
    if (!map_->EdgeExists(n->Parent()->Index(), n->Index())
            || n->Parent()->SearchID() < search_id_) {
        for (auto s : n->SubsumedNodes()) {
            s->SetSubsumed(false);
            RecursivelyAddNode(s, skip_queue_operations);
        }
    }
    else {
        AddNode(n, skip_queue_operations);
    }
}

bool GraphSearch::SubsumedByOpenState(NodePtr node, const bool skip_queue_operations) {
    auto index = node->Index();

    for (auto it = open_sets_[index].begin(); it != open_sets_[index].end();) {
        NodePtr s = *it;

#if USE_GHOST_DATA

        if (kLazinessMap.at(laziness_mode_) == "LazyA* modified"
                && this->StronglyDominates(s->CostToCome(), s->VisSet(), node->GhostCost(), node->GhostVisSet())) {
            if (!s->IsChecked()) {
                if (!Valid(s)) {
                    open_sets_[index].erase(it++);
                    continue;
                }
            }

            return true;
        }

#endif

        if (Dominates(s, node)) {
            if (kLazinessMap.at(laziness_mode_) == "LazyA* modified" && !s->IsChecked()) {
                if (!Valid(s)) {
                    open_sets_[index].erase(it++);
                    continue;
                }
            }

            bool need_update_queue = false;

            if (!skip_queue_operations) {
#if USE_GHOST_DATA

#if USE_GHOST_COST_AS_KEY

                if (node->GhostCost() < s->GhostCost() || !s->GhostVisSet().Contains(node->GhostVisSet())) {
                    need_update_queue = true;
                }

#else

                if (!s->GhostVisSet().Contains(node->GhostVisSet())) {
                    need_update_queue = true;
                }

#endif

#endif
            }

            if (need_update_queue) {
                NodePtr updated(new Node(s));
                s->SetLatest(false);
                updated->Subsume(node);
                open_sets_[index].erase(s);
                open_sets_[index].insert(updated);
                queue_->push(updated);
            }
            else {
                s->Subsume(node);
            }

            return true;
        }

#if USE_GHOST_DATA

        if (kLazinessMap.at(laziness_mode_) == "LazyA* modified"
                && this->StronglyDominates(node->CostToCome(), node->VisSet(), s->GhostCost(), s->GhostVisSet())) {
            if (!node->IsChecked()) {
                if (!Valid(node)) {
                    return true;
                }
            }

            s->SetSubsumed(true);
            open_sets_[index].erase(it++);
            continue;
        }

#endif

        if (Dominates(node, s)) {
            if (kLazinessMap.at(laziness_mode_) == "LazyA* modified" && !node->IsChecked()) {
                if (!Valid(node)) {
                    return true;
                }
            }

            node->Subsume(s);
            open_sets_[index].erase(it++);
            continue;
        }

        ++it;
    }

    return false;
}

bool GraphSearch::Valid(NodePtr n) {
    bool local_path_valid = ValidPath(n->LocalPath());
    n->SetChecked(true);
    n->SetValid(local_path_valid);

    return local_path_valid;
}

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

    if (!node_to_pop->Latest()) {
        // This node has been updated to a separate node.
        return nullptr;
    }

    // if this node is not subsumed by other nodes
    if (!node_to_pop->IsSubsumed()) {
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

#if USE_NODE_REUSE

    if (node_to_pop->ReusedFromClosedSet()) {
        return node_to_pop;
    }

#endif

    if (!node_to_pop->Latest()) {
        // This node has been updated to a separate node.
        return nullptr;
    }

    // if this node is not subsumed by other nodes
    if (!node_to_pop->IsSubsumed()) {
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


    std::vector<std::vector<Idx>> successors;

    if (kSuccessorMap.at(successor_mode_) == "expanded") {
        successors = map_->Successors(parent->Index(), parent->VisSet(), graph_);
    }
    else if (kSuccessorMap.at(successor_mode_) == "first-meet") {
        successors = map_->FirstMeetSuccessors(parent->Index(), parent->VisSet(), graph_);
    }
    else if (kSuccessorMap.at(successor_mode_) == "direct") {
        successors = map_->NeighboringSuccessors(parent->Index());
    }

    for (const auto& s : successors) {
        Idx m = s[0];
        NodePtr new_node(new Node());
        new_node->CopyAsChild(parent);
        new_node->Extend(m, map_->Cost(m, parent->Index()), graph_->Vertex(m)->vis);
        new_node->SetLocalPath(s);
        new_node->SetSuccessorStaturID(id);
        num_nodes_generated_++;

        if (DominatedByClosedState(new_node)) {
            continue;
        }

        if (kSuccessorMap.at(successor_mode_) != "direct") {
            if (DominatedByOpenState(new_node)) {
                if (parent->SuccessorStatusID() != id) {
                    break;
                }

                continue;
            }

            if (parent->SuccessorStatusID() != id) {
                break;
            }
        }
        else {
            // Do not need to recompute successors.
            if (DominatedByOpenState2(new_node)) {
                continue;
            }
        }

#if USE_HEURISTIC
        new_node->SetHeuristic(map_->ComputeHeuristic(m, new_node->VisSet(), graph_,
                               virtual_graph_coverage_));
#endif

        open_sets_[new_node->Index()].insert(new_node);
        queue_->push(new_node);
        num_nodes_remained_++;
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

bool GraphSearch::NewNodesInvolved(const NodePtr parent, const std::vector<Idx>& successor) const {
    for (const auto& i : successor) {
        if (i >= parent->ExtendedGraphSize()) {
            return true;
        }
    }

    return false;
}

void GraphSearch::ComputeAndAddSuccessorsCompleteLazy(const NodePtr parent) {
#if DEBUG_MODE
    std::cout << "Pre: ";
    PrintOpenSets();
    getchar();
#endif

    std::vector<std::vector<Idx>> successors;

    if (kSuccessorMap.at(successor_mode_) == "expanded") {
        successors = map_->Successors(parent->Index(), parent->VisSet(), graph_);
    }
    else if (kSuccessorMap.at(successor_mode_) == "first-meet") {
        successors = map_->FirstMeetSuccessors(parent->Index(), parent->VisSet(), graph_);
    }
    else if (kSuccessorMap.at(successor_mode_) == "direct") {
        successors = map_->NeighboringSuccessors(parent->Index());
    }

    // std::cout << parent->Index() << ", " << parent->ReusedFromClosedSet() << std::endl;
    // for (const auto& s : successors) {
    //     std::cout << s[0] << " ";
    // }
    // std::cout << std::endl;
    // getchar();

    // PrintClosedSets();
    // PrintOpenSets();
    // getchar();

#if USE_NODE_REUSE
    const bool& add_only_new_successors = parent->ReusedFromClosedSet();
#endif

    for (const auto& s : successors) {
#if USE_NODE_REUSE

        if (add_only_new_successors && !NewNodesInvolved(parent, s)) {
            continue;
        }

#endif

        Idx m = s[0];
        NodePtr new_node(new Node());
        new_node->CopyAsChild(parent);
        new_node->Extend(m, map_->Cost(m, parent->Index()), graph_->Vertex(m)->vis);
        new_node->SetLocalPath(s);
        num_nodes_generated_++;

        if (DominatedByClosedState(new_node)) {
            // std::cout << "Dominated by closed states" << std::endl;
            continue;
        }

        if (DominatedByOpenStateCompleteLazy(new_node)) {
            // std::cout << "Dominated by open states" << std::endl;
            continue;
        }

#if USE_HEURISTIC
        new_node->SetHeuristic(map_->ComputeHeuristic(m, new_node->VisSet(), graph_,
                               virtual_graph_coverage_));
#endif
        // std::cout << "Added to open set" << std::endl;

        open_sets_[new_node->Index()].insert(new_node);
        queue_->push(new_node);
        num_nodes_remained_++;
    }

    parent->SetExtendedGraphSize(virtual_graph_size_);

#if DEBUG_MODE
    std::cout << "Post: ";
    PrintOpenSets();
    getchar();
#endif
}

NodePtr GraphSearch::ComputeNearestSuccessor(const NodePtr parent) {
    auto successor = map_->NearestSuccessor(parent->Index(), parent->VisSet(), graph_);

    Idx m = successor[0];
    NodePtr new_node(new Node());
    new_node->CopyAsChild(parent);
    new_node->Extend(m, map_->Cost(m, parent->Index()), graph_->Vertex(m)->vis);
    new_node->SetLocalPath(successor);

    return new_node;
}

void GraphSearch::PrintClosedSets() const {
    for (auto i = 0; i < closed_sets_.size(); ++i) {
        std::cout << "Closed set " << i << " :" << std::endl;
        ClosedSet closed_set = *(closed_sets_.begin()+i);

        for (auto node : closed_set) {
            PrintNodeStatus(node);
        }

        std::cout << std::endl;
    }
}

void GraphSearch::PrintOpenSets() const {
    unsigned i = 0;

    for (auto open_set : open_sets_) {
        std::cout << "Open set " << i << " :" << std::endl;

        for (auto node : open_set) {
            PrintNodeStatus(node);
        }

        std::cout << std::endl;
        ++i;
    }
}

void GraphSearch::PrintNodeStatus(const NodePtr node, std::ostream& out) const {
    if (node == nullptr) {
        out << "Node is null.\t";
        return;
    }

    out << node->Index() << ", ";

    if (node->Parent()) {
        out << node->Parent()->Index() << ", ";
    }
    else {
        out << "nullptr, ";
    }

    out << node->CostToCome() << ", "
        << node->CoverageSize() << ", "
        << node->GhostCost() << ", "
        << node->GhostCoverageSize() << ", "
        << node->IsSubsumed() << ", "
        << node->IsChecked() << ", "
        << node->IsValid() << ", "
        << node->Latest() << ", "
        << node->ReusedFromClosedSet() << ", "
        << node->SearchID() << ", "
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
    // return (n->VisSet().Size() >= p_*virtual_graph_coverage_.Size());
    return (n->GhostVisSet() == virtual_graph_coverage_);
#endif

    return (n->VisSet() == virtual_graph_coverage_);
}

bool GraphSearch::StronglyDominates(const RealNum& l1, const VisibilitySet& s1, const RealNum& l2,
                                    const VisibilitySet& s2) const {
    if (l1 > l2) {
        return false;
    }

    if (!s1.Contains(s2)) {
        return false;
    }

    return true;
}

bool GraphSearch::DominatedByClosedState(const NodePtr node) const {
    auto states = closed_sets_[node->Index()];

    for (const auto s : states) {
#if USE_GHOST_DATA

        if (this->StronglyDominates(s->CostToCome(), s->VisSet(), node->GhostCost(), node->GhostVisSet())) {
            // New state is completely dominated.
            return true;
        }

        if (this->StronglyDominates(s->GhostCost(), s->GhostVisSet(), node->GhostCost(),
                                    node->GhostVisSet())) {
            s->Subsume(node, true);
            return true;
        }

#else

        if (this->StronglyDominates(s->CostToCome(), s->VisSet(), node->CostToCome(), node->VisSet())) {
            return true;
        }

#endif
    }

    return false;
}

bool GraphSearch::DominatedByOpenState(const NodePtr node) {
    // this is a copy
    Idx index = node->Index();
    auto states = open_sets_[index];

    // for (auto s : states) {
    for (auto it = states.begin(); it != states.end();) {
        NodePtr s = *it;

        // early return for duplicates
        if (node->Parent() == s->Parent() && fabs(node->CostToCome() - s->CostToCome()) < EPS) {
            s->SetSuccessorStaturID(node->SuccessorStatusID());
            return true;
        }

        // remove out of date nodes
        if (!IsUpToDate(s)) {
            open_sets_[index].erase(it++);
            continue;
        }

        if (Dominates(s, node)) {
            if (!s->IsChecked()) {
                bool local_path_valid = ValidPath(s->LocalPath());
                s->SetChecked(true);
                s->SetValid(local_path_valid);

                if (!local_path_valid) {
                    open_sets_[index].erase(it++);

                    ComputeAndAddSuccessors(s->Parent());
                    return DominatedByOpenState(node);
                }
            }

            bool need_update_queue = false;

#if USE_GHOST_DATA

#if USE_GHOST_COST_AS_KEY

            if (node->GhostCost() < s->GhostCost() || !s->GhostVisSet().Contains(node->GhostVisSet())) {
                need_update_queue = true;
            }

#else

            if (!s->GhostVisSet().Contains(node->GhostVisSet())) {
                need_update_queue = true;
            }

#endif

#endif

            if (need_update_queue) {
                NodePtr updated(new Node(s));
                s->SetLatest(false);
                updated->Subsume(node);
                open_sets_[index].erase(s);
                open_sets_[index].insert(updated);
                queue_->push(updated);
            }
            else {
                s->Subsume(node);
            }

            return true;
        }

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

            node->Subsume(s);
            open_sets_[index].erase(it++);
            continue;
        }

        ++it;
    }

    return false;
}

bool GraphSearch::DominatedByOpenState2(const NodePtr node) {
    // this is a copy
    auto index = node->Index();

    for (auto it = open_sets_[index].begin(); it != open_sets_[index].end();) {
        NodePtr s = *it;

#if USE_GHOST_DATA

        if (this->StronglyDominates(s->CostToCome(), s->VisSet(), node->GhostCost(), node->GhostVisSet())) {
            if (!s->IsChecked()) {
                bool local_path_valid = ValidPath(s->LocalPath());
                s->SetChecked(true);
                s->SetValid(local_path_valid);

                if (!local_path_valid) {
                    open_sets_[index].erase(it++);
                    continue;
                }
            }

            return true;
        }

#endif

        if (Dominates(s, node)) {
            if (!s->IsChecked()) {
                bool local_path_valid = ValidPath(s->LocalPath());
                s->SetChecked(true);
                s->SetValid(local_path_valid);

                if (!local_path_valid) {
                    open_sets_[index].erase(it++);
                    continue;
                }
            }

            bool need_update_queue = false;

#if USE_GHOST_DATA

#if USE_GHOST_COST_AS_KEY

            if (node->GhostCost() < s->GhostCost() || !s->GhostVisSet().Contains(node->GhostVisSet())) {
                need_update_queue = true;
            }

#else

            if (!s->GhostVisSet().Contains(node->GhostVisSet())) {
                need_update_queue = true;
            }

#endif

#endif

            if (need_update_queue) {
                NodePtr updated(new Node(s));
                s->SetLatest(false);
                updated->Subsume(node);
                open_sets_[index].erase(s);
                open_sets_[index].insert(updated);
                queue_->push(updated);
            }
            else {
                s->Subsume(node);
            }

            return true;
        }

#if USE_GHOST_DATA

        if (this->StronglyDominates(node->CostToCome(), node->VisSet(), s->GhostCost(), s->GhostVisSet())) {
            if (!node->IsChecked()) {
                bool local_path_valid = ValidPath(node->LocalPath());
                node->SetChecked(true);
                node->SetValid(local_path_valid);

                if (!local_path_valid) {
                    return true;
                }
            }

            s->SetSubsumed(true);
            open_sets_[index].erase(it++);
            continue;
        }

#endif

        if (Dominates(node, s)) {
            if (!node->IsChecked()) {
                bool local_path_valid = ValidPath(node->LocalPath());
                node->SetChecked(true);
                node->SetValid(local_path_valid);

                if (!local_path_valid) {
                    return true;
                }
            }

            node->Subsume(s);
            open_sets_[index].erase(it++);
            continue;
        }

        ++it;
    }

    return false;
}

bool GraphSearch::DominatedByOpenStateCompleteLazy(const NodePtr node,
        const bool skip_queue_operations) {
    // this is a copy
    auto index = node->Index();

    for (auto it = open_sets_[index].begin(); it != open_sets_[index].end();) {
        NodePtr s = *it;

#if USE_GHOST_DATA

        if (this->StronglyDominates(s->CostToCome(), s->VisSet(), node->GhostCost(), node->GhostVisSet())) {
            return true;
        }

#endif

        if (Dominates(s, node)) {
            bool need_update_queue = false;

            if (!skip_queue_operations) {
#if USE_GHOST_DATA

#if USE_GHOST_COST_AS_KEY

                if (node->GhostCost() < s->GhostCost() || !s->GhostVisSet().Contains(node->GhostVisSet())) {
                    need_update_queue = true;
                }

#else

                if (!s->GhostVisSet().Contains(node->GhostVisSet())) {
                    need_update_queue = true;
                }

#endif

#endif
            }

            if (need_update_queue) {
                NodePtr updated(new Node(s));
                s->SetLatest(false);
                updated->Subsume(node);
                open_sets_[index].erase(s);
                open_sets_[index].insert(updated);
                queue_->push(updated);
            }
            else {
                s->Subsume(node);
            }

            return true;
        }

#if USE_GHOST_DATA

        if (this->StronglyDominates(node->CostToCome(), node->VisSet(), s->GhostCost(), s->GhostVisSet())) {
            s->SetSubsumed(true);
            open_sets_[index].erase(it++);
            continue;
        }

#endif

        if (Dominates(node, s)) {
            node->Subsume(s);
            open_sets_[index].erase(it++);
            continue;
        }

        ++it;
    }

    return false;
}

bool GraphSearch::Dominates(const NodePtr n1, const NodePtr n2) const {
#if USE_GHOST_DATA
    VisibilitySet union_set = n2->GhostVisSet();
    union_set.Insert(n1->GhostVisSet());

    if ((1 + eps_)*(n2->GhostCost()) < n1->CostToCome()) {
        return false;
    }

    if (union_set.Size()*p_ > n1->CoverageSize()) {
        return false;
    }

    return true;
#else
    return this->StronglyDominates(n1->CostToCome(), n1->VisSet(), n2->CostToCome(), n2->VisSet());
#endif
}

bool GraphSearch::ValidPath(const std::vector<Idx>& path) {
    SizeType len = path.size();

    if (len > 0) {
        for (auto i = len-1; i > 0; --i) {
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

const VisibilitySet& GraphSearch::VirtualGraphCoverage() const {
    return virtual_graph_coverage_;
}


SizeType GraphSearch::ResultCoverageSize() const {
    if (result_ == nullptr) {
        return 0;
    }

    return result_->VisSet().Size();
}

RealNum GraphSearch::ResultCost() const {
    return result_->CostToCome();
}

void GraphSearch::PrintTitle(std::ostream& out) const {
    out << "GraphSize "
        << "GraphCoverage "
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
        << "NumEdgesOnGraph "
        << "NumNodesExtended "
        << "NumNodesGenerated "
        << "NumNodesRemained "
        << std::endl;
}

void GraphSearch::PrintResult(std::ostream& out) const {
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
        << virtual_graph_num_edges_ << " "
        << num_nodes_extended_ << " "
        << num_nodes_generated_ << " "
        << num_nodes_remained_ << " "
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

SizeType GraphSearch::VirtualGraphNumEdges() const {
    return virtual_graph_num_edges_;
}

void GraphSearch::ReconstructNode(const NodePtr node) const {
    auto idx = node->Index();
    node->CopyAsChild(node->Parent());
    node->Extend(idx, node->LocalPathCost(), graph_->Vertex(idx)->vis);
}

void GraphSearch::TraceFirstUnboundedNode(const NodePtr node, std::queue<NodePtr>& recycle_bin) {
    NodePtr tag = node;
    std::stack<NodePtr> stack;

    while(tag != nullptr && tag->SearchID() < search_id_) {
        stack.push(tag);
        tag = tag->Parent();
    }

    bool found_first_unbounded = true;

    if (tag == nullptr || tag->ReusedFromClosedSet()) {
        found_first_unbounded = false;
    }

    while(!stack.empty()) {
        tag = stack.top();
        stack.pop();

        tag->SetSearchID(search_id_);

        if (!found_first_unbounded) {
            if (!tag->IsBounded(p_, eps_)) {
                found_first_unbounded = true;
            }
        }

        if (found_first_unbounded) {
            tag->SetReuseFromClosedSet(false);
        }
        else {
            tag->SetReuseFromClosedSet(true);
        }
    }
}

void GraphSearch::RecycleSubsumedNodes(NodePtr node, std::queue<NodePtr>& recycle_bin,
                                       const bool force_recycle_all) {
    auto tmp_history = node->SubsumedNodes();
    node->ClearSubsumeHistory();

#if SAVE_PREDECESSOR
    Idx index = node->Index();
#endif

    for (auto n : tmp_history) {
#if SAVE_PREDECESSOR

        if (!map_->EdgeExists(index, n->Index())
                || n->SearchID() < search_id_
                || !n->ReusedFromClosedSet()) {
            continue;
        }

        NodePtr new_node(new Node());
        new_node->CopyAsChild(n);
        new_node->Extend(index, map_->EdgeCost(index, n->Index()), graph_->Vertex(index)->vis);
        new_node->SetLocalPath(std::vector<Idx> {index, n->Index()});

        if (!force_recycle_all && Dominates(node, new_node)) {
            node->Subsume(new_node);
        }
        else {
            new_node->SetSubsumed(false);
            recycle_bin.push(new_node);
        }

#else

        if (!force_recycle_all
                && n->Parent()->SearchID() == search_id_
                && n->Parent()->ReusedFromClosedSet()
                && map_->EdgeExists(n->Parent()->Index(), n->Index())
                && Dominates(node, n)) {
            node->Subsume(n);
        }
        else {
            n->SetSubsumed(false);
            recycle_bin.push(n);
        }

#endif
    }
}

void GraphSearch::UpdateUnboundedNodes() {
    std::queue<NodePtr> recycle_bin;

    // Parse closed set.
    for (SizeType i = 0; i < virtual_graph_size_; ++i) {
        auto& closed_set = closed_sets_[i];

        for (auto it = closed_set.begin(); it != closed_set.end(); ++it) {
            auto node = *it;
            this->TraceFirstUnboundedNode(node, recycle_bin);
        }
    }

    std::cout << "Parsed closed set" << std::flush;

    // Find boundary nodes.
    std::unordered_set<NodePtr> reopened_nodes;

    for (SizeType i = 0; i < virtual_graph_size_; ++i) {
        auto& closed_set = closed_sets_[i];

        for (auto it = closed_set.begin(); it != closed_set.end();) {
            auto node = *it;

            if (node->ReusedFromClosedSet()) {
                ++it;
                continue;
            }

            if (node->Parent() && node->Parent()->ReusedFromClosedSet()) {
                // Boundary node.
                this->ReconstructNode(node);
                this->RecycleSubsumedNodes(node, recycle_bin, true);
                reopened_nodes.insert(node);
            }
            else {
                // Non-reusable node.
                this->RecycleSubsumedNodes(node, recycle_bin, true);
            }

            closed_set.erase(it++);
        }
    }

    std::cout << "\rFound boundary nodes" << std::flush;

    // Update open sets.
    for (SizeType i = 0; i < virtual_graph_size_; ++i) {
        auto& open_set = open_sets_[i];

        for (auto it = open_set.begin(); it != open_set.end();) {
            auto node = *it;

            if (node->Parent() == nullptr) {
                ++it;
                continue;
            }

            if (node->Parent()->ReusedFromClosedSet()
                    && map_->EdgeExists(node->Parent()->Index(), node->Index())) {
                if (!node->IsBounded(p_, eps_)) {
                    // Boundary node.
                    this->ReconstructNode(node);
                    this->RecycleSubsumedNodes(node, recycle_bin);
                }

                ++it;
                continue;
            }

            // Non-reusable node.
            this->RecycleSubsumedNodes(node, recycle_bin, true);
            open_set.erase(it++);
        }
    }

    std::cout << "\rUpdated open set" << std::flush;

    for (auto node : reopened_nodes) {
        open_sets_[node->Index()].insert(node);
    }

    std::cout << "\rMoved reopened nodes to open set" << std::flush;

    std::vector<NodePtr> nodes_to_add;

    while (!recycle_bin.empty()) {
        auto node = recycle_bin.front();
        recycle_bin.pop();

        if (node->Parent()->ReusedFromClosedSet()
                && node->Parent()->SearchID() == search_id_) {
            if (!node->IsBounded(p_, eps_)) {
                // Boundary node.
                this->ReconstructNode(node);
                this->RecycleSubsumedNodes(node, recycle_bin, true);
            }

            nodes_to_add.push_back(node);
            continue;
        }

        // Non-reusable node.
        this->RecycleSubsumedNodes(node, recycle_bin, true);
    }

    for (auto node : nodes_to_add) {
        AddNode(node, true);
    }

    std::cout << "\rInserted recycle bin" << std::flush;

    for (SizeType i = 0; i < virtual_graph_size_; ++i) {
        auto& closed_set = closed_sets_[i];

        for (auto node : closed_set) {
            queue_->push(node);
        }

        auto& open_set = open_sets_[i];

        for (auto node : open_set) {
            queue_->push(node);
        }
    }

    std::cout << "\rUpdated queue" << std::flush;
}
