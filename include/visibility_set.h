#ifndef VISIBILITY_SET_H_
#define VISIBILITY_SET_H_

#include <bitset>

#include "global_common.h"

class VisibilitySet {
    using Bitset = std::bitset<MAX_COVERAGE_SIZE>;

  public:
    VisibilitySet();
    VisibilitySet(const VisibilitySet& other);

    VisibilitySet& operator=(const VisibilitySet& other);
    bool operator[](Idx i) const;
    bool operator==(const VisibilitySet& other) const;
    bool operator<(const VisibilitySet& other) const;
    bool operator>(const VisibilitySet& other) const;

    void Clear();
    void SetAll();

    bool At(Idx i) const;
    void Insert(Idx i);
    void Insert(const VisibilitySet& other);
    void Remove(const VisibilitySet& other);
    bool IsExpending(const VisibilitySet& other) const;
    bool Contains(const VisibilitySet& other) const;
    bool IsContainedIn(const VisibilitySet& other) const;
    SizeType Size() const;
    const Bitset& bitset() const;

  private:
    Bitset bitset_;

};

#endif // VISIBILITY_SET_H
