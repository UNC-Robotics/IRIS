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
