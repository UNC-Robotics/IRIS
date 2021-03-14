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