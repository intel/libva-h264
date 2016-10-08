// Copyright (c) 2012 The Chromium Authors. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//    * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
//    * Neither the name of Google Inc. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef LIBVA_H264_RANGES_H_
#define LIBVA_H264_RANGES_H_

#include <stddef.h>
#include <stdint.h>

#include <algorithm>
#include <ostream>
#include <vector>

namespace libva_h264 {

// Ranges allows holding an ordered list of ranges of [start,end) intervals.
// The canonical example use-case is holding the list of ranges of buffered
// bytes or times in a <video> tag.
template <class T>  // Endpoint type; typically a base::TimeDelta or an int64_t.
class Ranges {
 public:
  // Allow copy & assign.

  // Add (start,end) to this object, coallescing overlaps as appropriate.
  // Returns the number of stored ranges, post coallescing.
  size_t Add(T start, T end);

  // Return the number of disjoint ranges.
  size_t size() const;

  // Return the "i"'th range's start & end (0-based).
  T start(size_t i) const;
  T end(size_t i) const;

  // Clear all ranges.
  void clear();

  // Computes the intersection between this range and |other|.
  Ranges<T> IntersectionWith(const Ranges<T>& other) const;

 private:
  // Disjoint, in increasing order of start.
  std::vector<std::pair<T, T> > ranges_;
};

//////////////////////////////////////////////////////////////////////
// EVERYTHING BELOW HERE IS IMPLEMENTATION DETAIL!!
//////////////////////////////////////////////////////////////////////

template<class T>
size_t Ranges<T>::Add(T start, T end) {
  if (start == end)  // Nothing to be done with empty ranges.
    return ranges_.size();

  size_t i;
  // Walk along the array of ranges until |start| is no longer larger than the
  // current interval's end.
  for (i = 0; i < ranges_.size() && ranges_[i].second < start; ++i) {
    // Empty body
  }

  // Now we know |start| belongs in the i'th slot.
  // If i is the end of the range, append new range and done.
  if (i == ranges_.size()) {
    ranges_.push_back(std::make_pair(start, end));
    return ranges_.size();
  }

  // If |end| is less than i->first, then [start,end) is a new (non-overlapping)
  // i'th entry pushing everyone else back, and done.
  if (end < ranges_[i].first) {
    ranges_.insert(ranges_.begin() + i, std::make_pair(start, end));
    return ranges_.size();
  }

  // Easy cases done.  Getting here means there is overlap between [start,end)
  // and the existing ranges.

  // Now: start <= i->second && i->first <= end
  if (start < ranges_[i].first)
    ranges_[i].first = start;
  if (ranges_[i].second < end)
    ranges_[i].second = end;

  // Now: [start,end) is contained in the i'th range, and we'd be done, except
  // for the fact that the newly-extended i'th range might now overlap
  // subsequent ranges.  Merge until discontinuities appear.  Note that there's
  // no need to test/merge previous ranges, since needing that would mean the
  // original loop went too far.
  while ((i + 1) < ranges_.size() &&
         ranges_[i + 1].first <= ranges_[i].second) {
    ranges_[i].second = std::max(ranges_[i].second, ranges_[i + 1].second);
    ranges_.erase(ranges_.begin() + i + 1);
  }

  return ranges_.size();
}

template<class T>
size_t Ranges<T>::size() const {
  return ranges_.size();
}

template<class T>
T Ranges<T>::start(size_t i) const {
  return ranges_[i].first;
}

template<class T>
T Ranges<T>::end(size_t i) const {
  return ranges_[i].second;
}

template<class T>
void Ranges<T>::clear() {
  ranges_.clear();
}

template<class T>
Ranges<T> Ranges<T>::IntersectionWith(const Ranges<T>& other) const {
  Ranges<T> result;

  size_t i = 0;
  size_t j = 0;

  while (i < size() && j < other.size()) {
    T max_start = std::max(start(i), other.start(j));
    T min_end = std::min(end(i), other.end(j));

    // Add an intersection range to the result if the ranges overlap.
    if (max_start < min_end)
      result.Add(max_start, min_end);

    if (end(i) < other.end(j))
      ++i;
    else
      ++j;
  }

  return result;
}

}  // namespace libva_h264

#endif  // LIBVA_H264_RANGES_H_
