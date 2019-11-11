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
    static RealNum Key(const NodePtr n) {
#if USE_HEURISTIC
	return n->CostToCome() + HEUR_BIAS*n->Heuristic();
#endif
	return n->CostToCome();
    }
    
    struct Cmp {
    	bool operator() (const NodePtr n1, const NodePtr n2) const {
    	    // smaller key comes first
    	    return Key(n1) > Key(n2);
    	}
    };
    
    struct CoverageCmp {
    	bool operator() (const NodePtr n1, const NodePtr n2) const {
    	    // larger coverage comes first
    	    return n1->CoverageSize() < n2->CoverageSize();
    	}
    };
    
    using PriorityQueue = std::priority_queue<NodePtr, std::vector<NodePtr>, Cmp>;
    using OpenSet = std::unordered_set<NodePtr>;
    using ClosedSet = std::set<NodePtr, CoverageCmp>;
public:
    GraphSearch(Inspection::GPtr graph);
    
    SizeType ExpandVirtualGraph(SizeType new_size, bool lazy_computation=true);
    std::vector<Idx> SearchVirtualGraphCompleteLazy(const RealNum p=1.0, const RealNum eps=0.0);
    std::vector<Idx> SearchVirtualGraph(const RealNum p=1.0, const RealNum eps=0.0);
    SizeType VirtualGraphCoverageSize() const;
    SizeType ResultCoverageSize() const;
    RealNum ResultCost() const;
    void PrintResult(std::ostream &out) const;
    void PrintTitle(std::ostream &out) const;
    SizeType TotalTime() const;
    void SetMaxTimeAllowed(const SizeType& time);

private:
    Inspection::GPtr graph_;
    SizeType virtual_graph_size_{0};
    VisibilitySet virtual_graph_coverage_;
    NodePtr result_{nullptr};

#if USE_GHOST_DATA
    RealNum p_{1};
    RealNum eps_{0};
#endif
    SizeType time_build_{0};
    SizeType time_vis_{0};
    SizeType time_valid_{0};
    SizeType time_search_{0};
    SizeType num_validated_{0};
    SizeType max_time_allowed_{10000000};
    std::shared_ptr<TracebackMap> map_;

    std::shared_ptr<PriorityQueue> queue_;
    std::vector<OpenSet> open_sets_;
    std::vector<ClosedSet> closed_sets_;

    NodePtr PopFromQueue();
    NodePtr PopFromQueueCompleteLazy();
    bool IsUpToDate(const NodePtr p) const;
    void ComputeAndAddSuccessors(const NodePtr p);
    void ComputeAndAddSuccessorsCompleteLazy(const NodePtr p);
    bool InGoalSet(const NodePtr n) const;
    bool DominatedByClosedState(const NodePtr node) const;
    bool DominatedByOpenState(NodePtr& node);
    bool DominatedByOpenStateCompleteLazy(NodePtr& node);
    bool Dominates(const NodePtr n1, const NodePtr n2) const;
    bool ValidPath(const std::vector<Idx>& path);
    SizeType RelativeTime(const TimePoint start) const;

    void PrintOpenSets() const;
    void PrintNodeStatus(const NodePtr node) const;
    Idx CheckOpenSets() const;
    bool CheckTermination() const;

};


#endif // GRAPH_SEARCH_H_
