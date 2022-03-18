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

#ifndef PRIORITY_QUEUE_H_
#define PRIORITY_QUEUE_H_

#include <set>

#include "node.h"

class PriorityQueue {
  public:
    // reference key to sort the set
    static RealNum Key(const NodePtr n) {
#if USE_HEURISTIC
        return n->CostToCome() + HEUR_BIAS*n->Heuristic();
#endif
        return n->CostToCome();
    }

    // comparison inside a NodeSet
    struct Cmp {
        bool operator() (const NodePtr n1, const NodePtr n2) const {
            // smaller key comes first
            return Key(n1) > Key(n2);
        }
    };

    using NodeSet = std::set<NodePtr, Cmp>;
    using NodeSetPtr = std::shared_ptr<NodeSet>;

    // comparison between NodeSets
    struct QueueCmp {
        bool operator() (const NodeSetPtr s1, const NodeSetPtr s2) const {
            if (s2->empty()) {
                return true;
            }
            else if (s1->empty()) {
                return false;
            }

            // smaller key comes first
            NodePtr n1 = *s1->begin();
            NodePtr n2 = *s2->begin();
            return Key(n1) > Key(n2);
        }
    };

    using Queue = std::set<NodeSetPtr, QueueCmp>;

    PriorityQueue() = default;
    ~PriorityQueue() {
        queue_.clear();
    }

    void Push(const NodeSetPtr node_set) {
        queue_.insert(node_set)
    }

    NodeSetPtr Top() {
        return *queue_.begin();
    }

    void Pop() {
        if (queue_.size() > 0) {
            queue_.erase(queue_.begin());
        }
        else {
            std::cerr << "Queue is empty!" << std::endl;
        }
    }

    void Retrieve(const NodeSetPtr node_set) {
        Pop() {
            if (queue_.size() > 0) {
                queue_.find(NodeSetPtr);
            }
            else {
                std::cerr << "Queue is empty!" << std::endl;
            }
        }

        NodeSetPtr
private:
        Queue queue_;

    };

#endif // PRIORITY_QUEUE_H_