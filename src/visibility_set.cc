#include <iostream>

#include "visibility_set.h"

VisibilitySet::VisibilitySet() {
    Clear();
}

VisibilitySet::VisibilitySet(const VisibilitySet& other)
    : bitset_(other.bitset()) {}

VisibilitySet& VisibilitySet::operator=(const VisibilitySet& other) {
    if (this != &other) {
        bitset_ = other.bitset();
    }

    return *this;
}

bool VisibilitySet::operator[](Idx i) const {
    return (bitset_[i] == 1);
}

bool VisibilitySet::operator==(const VisibilitySet& other) const {
    return (bitset_ == other.bitset());
}

bool VisibilitySet::operator<(const VisibilitySet& other) const {
    return (this->Size() < other.Size());
}

bool VisibilitySet::operator>(const VisibilitySet& other) const {
    return (this->Size() > other.Size());
}

void VisibilitySet::Clear() {
    bitset_.reset();
}

void VisibilitySet::SetAll() {
    bitset_.set();
}

bool VisibilitySet::At(Idx i) const {
    return (bitset_[i] == 1);
}

void VisibilitySet::Insert(Idx i) {
    bitset_.set(i, true);
}

void VisibilitySet::Insert(const VisibilitySet& other) {
    bitset_ |= other.bitset();
}

void VisibilitySet::Remove(const VisibilitySet& other) {
    bitset_ &= (~other.bitset());
}

bool VisibilitySet::IsExpending(const VisibilitySet& other) const {
    return (bitset_ != (bitset_ & other.bitset()));
}

bool VisibilitySet::Contains(const VisibilitySet& other) const {
    return (other.bitset() == (bitset_ & other.bitset()));
}

bool VisibilitySet::IsContainedIn(const VisibilitySet& other) const {
    return (bitset_ == (bitset_ & other.bitset()));
}

SizeType VisibilitySet::Size() const {
    return bitset_.count();
}

const VisibilitySet::Bitset& VisibilitySet::bitset() const {
    return bitset_;
}
