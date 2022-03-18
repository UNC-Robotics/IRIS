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

#ifndef TRACEBACK_MAP_H_
#define TRACEBACK_MAP_H_

#include <iostream>
#include <list>
#include <queue>
#include <set>

#include "global_common.h"
#include "inspection_graph.h"

class TracebackMap {
    using State = std::pair<Idx, RealNum>;
    struct Cmp {
        bool operator()(const State& s1, const State& s2) {
            // smaller cost comes first
            return s1.second > s2.second;
        }
    };

    using DijkstraQueue = std::priority_queue<State, std::vector<State>, Cmp>;

    using StateNew = std::tuple<Idx, RealNum, bool>;
    struct CmpNew {
        bool operator()(const StateNew& s1, const StateNew& s2) {
            // smaller cost comes first
            return std::get<1>(s1) > std::get<1>(s2);
        }
    };

    using DijkstraQueueNew = std::priority_queue<StateNew, std::vector<StateNew>, CmpNew>;

  public:
    TracebackMap(const SizeType num_vertices);
    ~TracebackMap();

    void Print() const;

    void AddTrace(const Idx i, const Idx j, const Idx i_to_j);
    void AddCost(const Idx i, const Idx j, const RealNum cost);
    void AddEdgeCost(const Idx i, const Idx j, const RealNum cost);
    void AddNeighbor(const Idx i, const Idx j);
    void AddDirectEdge(const Idx i, const Idx j, const RealNum cost);
    void RemoveDirectEdge(const Idx i, const Idx j);

    Idx Trace(const Idx i, const Idx j) const;
    RealNum Cost(const Idx i, const Idx j) const;
    RealNum EdgeCost(const Idx i, const Idx j) const;
    bool EdgeExists(const Idx i, const Idx j) const;
    std::set<Idx> NeighborList(const Idx i) const;

    std::vector<std::vector<Idx>> Successors(const Idx source, const VisibilitySet& current_vis_set,
                               const Inspection::GPtr graph);
    std::vector<RealNum> SingleSourceShortestDistance(const Idx source, bool query=false,
            const Idx target=0) const;
    std::vector<std::vector<Idx>> FirstMeetSuccessors(const Idx source,
                               const VisibilitySet& current_vis_set, const Inspection::GPtr graph);
    std::vector<std::vector<Idx>> NeighboringSuccessors(const Idx source);
    std::vector<Idx> NearestSuccessor(const Idx source, const VisibilitySet& current_vis_set,
                                      const Inspection::GPtr graph);
    std::list<Idx> FullPath(const std::vector<Idx> milestones) const;
    bool Connected(const Inspection::GPtr& graph) const;
    void CheckLowerBound(const Idx source, const String message) const;
    void CheckAllLowerBound(const String message) const;

    void LockMap() {
        map_locked_ = true;
    }
    void UnlockMap() {
        map_locked_ = false;
    }

#if USE_HEURISTIC
    RealNum ComputeHeuristic(const Idx source, const VisibilitySet& current_vis_set,
                             const Inspection::GPtr graph, const VisibilitySet& graph_coverage);
#endif

  private:
    bool map_locked_{false};
    const SizeType n_;
    IdxArray2 map_;
    std::vector<RealNum> cost_array_;
    std::vector<RealNum> edge_cost_array_;
    std::vector<std::set<Idx>> neighbors_;

    void Reset();
    SizeType FlatIndex(const SizeType i, const SizeType j) const;

};

#endif // TRACEBACK_MAP_H_
