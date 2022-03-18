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
