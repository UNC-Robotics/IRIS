// BSD 3-Clause License

// Copyright (c) 2019, The University of North Carolina at Chapel Hill
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//! @author Mengyu Fu

#ifndef GRAPH_SEARCH_H_
#define GRAPH_SEARCH_H_

#include <list>
#include <queue>
#include <set>
#include <unordered_set>

#include "inspection_graph.h"
#include "node.h"
#include "traceback_map.h"

class GraphSearch {
    static RealNum KeyBase(const NodePtr n) {
#if USE_GHOST_DATA
#if USE_GHOST_COST_AS_KEY
        return n->GhostCost();
#endif
#endif
        return n->CostToCome();
    }

    static RealNum Key(const NodePtr n) {
#if USE_HEURISTIC
        return KeyBase(n) + HEUR_BIAS*n->Heuristic();
#endif
        return KeyBase(n);
    }

    struct QueueCmp {
        // This is greater operator.
        // If false is returned, n1 comes before n2; if true is returned, n2 comes before n1.
        bool operator() (const NodePtr n1, const NodePtr n2) const {
            return Key(n1) > Key(n2);

            // smaller key comes first
            const auto& key_1 = Key(n1);
            const auto& key_2 = Key(n2);

            if (std::fabs(key_1 - key_2) > EPS) {
                return key_1 > key_2;
            }

#if USE_GHOST_DATA
            const auto& size_1 = n1->GhostCoverageSize();
            const auto& size_2 = n2->GhostCoverageSize();

            if (size_1 != size_2) {
                return size_1 < size_2;
            }

#endif
            return n1->CoverageSize() < n2->CoverageSize();
        }
    };

    struct CoverageCmp {
        bool operator() (const NodePtr n1, const NodePtr n2) const {
            // larger coverage comes first
            return n1->CoverageSize() < n2->CoverageSize();
        }
    };

    using PriorityQueue = std::priority_queue<NodePtr, std::vector<NodePtr>, QueueCmp>;
    // using OpenSet = std::set<NodePtr, CoverageCmp>;
    using OpenSet = std::unordered_set<NodePtr>;
    // using ClosedSet = std::set<NodePtr, CoverageCmp>;
    using ClosedSet = std::unordered_set<NodePtr>;


    inline static const std::map<Idx, String> kLazinessMap = {{0, "no lazy"},
        {1, "LazySP"},
        {2, "LazyA* modified"},
        {3, "LazyA*"}
    };

    inline static const std::map<Idx, String> kSuccessorMap = {{0, "direct"},
        {1, "expanded"},
        {2, "first-meet"}
    };

  public:
    GraphSearch(const Inspection::GPtr graph);

    void SetLazinessMode(const Idx mode_id);
    void SetSuccessorMode(const Idx mode_id);
    void SetSourceIndex(const Idx source);
    void UpdateApproximationParameters(const RealNum eps, const RealNum p);

    SizeType ExpandVirtualGraph(SizeType new_size);
    std::vector<Idx> SearchVirtualGraph();
    void InitDataStructures();
    NodePtr PopFromPriorityQueue();
    void Extend(NodePtr n);
    bool AddNode(NodePtr n, const bool skip_queue_operations=false);
    void RecursivelyAddNode(NodePtr n, const bool skip_queue_operations=false);
    bool SubsumedByOpenState(NodePtr node, const bool skip_queue_operations);
    bool Valid(NodePtr n);
    SizeType VirtualGraphCoverageSize() const;
    const VisibilitySet& VirtualGraphCoverage() const;
    SizeType ResultCoverageSize() const;
    RealNum ResultCost() const;
    void PrintResult(std::ostream& out) const;
    void PrintTitle(std::ostream& out) const;
    SizeType TotalTime() const;
    void SetMaxTimeAllowed(const SizeType& time);
    SizeType VirtualGraphNumEdges() const;

  private:
    Inspection::GPtr graph_{nullptr};
    SizeType virtual_graph_size_{0};
    SizeType prev_graph_size_{0};
    VisibilitySet virtual_graph_coverage_;
    NodePtr result_{nullptr};
    RealNum greedy_cost_{0};

#if USE_GHOST_DATA
    RealNum p_ {1.0};
    RealNum eps_{0.0};
#endif
    SizeType time_build_ {0};
    SizeType time_vis_{0};
    SizeType time_valid_{0};
    SizeType time_search_{0};
    SizeType num_validated_{0};
    SizeType virtual_graph_num_edges_{0};
    SizeType max_time_allowed_{10000000};
    SizeType num_nodes_extended_{0};
    SizeType num_nodes_generated_{0};
    SizeType num_nodes_remained_{0};
    std::shared_ptr<TracebackMap> map_{nullptr};

    std::shared_ptr<PriorityQueue> queue_{nullptr};
    std::vector<OpenSet> open_sets_;
    std::vector<ClosedSet> closed_sets_;

    Idx successor_mode_{0};
    Idx laziness_mode_{0};
    Idx source_idx_{0};
    Idx search_id_{0};

    NodePtr PopFromQueue();
    NodePtr PopFromQueueCompleteLazy();
    bool IsUpToDate(const NodePtr p) const;
    bool NewNodesInvolved(const NodePtr parent, const std::vector<Idx>& successor) const;
    void ComputeAndAddSuccessors(const NodePtr p);
    void ComputeAndAddSuccessorsCompleteLazy(const NodePtr p);
    NodePtr ComputeNearestSuccessor(const NodePtr parent);
    bool InGoalSet(const NodePtr n) const;
    bool StronglyDominates(const RealNum& l1, const VisibilitySet& s1, const RealNum& l2,
                           const VisibilitySet& s2) const;
    bool DominatedByClosedState(const NodePtr node) const;
    bool DominatedByOpenState(const NodePtr node);
    bool DominatedByOpenState2(const NodePtr node);
    bool DominatedByOpenStateCompleteLazy(const NodePtr node, const bool skip_queue_operations=false);
    bool Dominates(const NodePtr n1, const NodePtr n2) const;
    bool ValidPath(const std::vector<Idx>& path);
    SizeType RelativeTime(const TimePoint start) const;
    void PrintClosedSets() const;
    void PrintOpenSets() const;
    void PrintNodeStatus(const NodePtr node, std::ostream& out=std::cout) const;
    Idx CheckOpenSets() const;
    bool CheckTermination() const;
    void ReconstructNode(const NodePtr node) const;
    void TraceFirstUnboundedNode(const NodePtr node, std::queue<NodePtr>& recycle_bin);
    void UpdateUnboundedNodes();
    void RecycleSubsumedNodes(NodePtr node, std::queue<NodePtr>& recycle_bin,
                              const bool force_recycle_all=false);
};


#endif // GRAPH_SEARCH_H_
