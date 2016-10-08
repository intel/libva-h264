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
//
// This file contains an implementation of an H.264 Decoded Picture Buffer
// used in H264 decoders.

#ifndef LIBVA_H264_H264_DPB_H_
#define LIBVA_H264_H264_DPB_H_

#include <stddef.h>

#include <memory>
#include <vector>

#include "h264_parser.h"
#include "va_h264_parser.h"

namespace libva_h264 {

// A picture (a frame or a field) in the H.264 spec sense.
// See spec at http://www.itu.int/rec/T-REC-H.264
class H264Picture : public std::enable_shared_from_this<H264Picture> {
 public:
  using Vector = std::vector<std::shared_ptr<H264Picture>>;

  enum Field {
    FIELD_NONE,
    FIELD_TOP,
    FIELD_BOTTOM,
  };

  H264Picture();

  // Values calculated per H.264 specification or taken from slice header.
  // See spec for more details on each (some names have been converted from
  // CamelCase in spec to Chromium-style names).
  int pic_order_cnt_type;
  int top_field_order_cnt;
  int bottom_field_order_cnt;
  int pic_order_cnt;
  int pic_order_cnt_msb;
  int pic_order_cnt_lsb;
  int delta_pic_order_cnt_bottom;
  int delta_pic_order_cnt0;
  int delta_pic_order_cnt1;

  int pic_num;
  int long_term_pic_num;
  int frame_num;  // from slice header
  int frame_num_offset;
  int frame_num_wrap;
  int long_term_frame_idx;

  H264SliceHeader::Type type;
  int nal_ref_idc;
  bool idr;        // IDR picture?
  int idr_pic_id;  // Valid only if idr == true.
  bool ref;        // reference picture?
  bool long_term;  // long term reference picture?
  bool outputted;
  // Does memory management op 5 needs to be executed after this
  // picture has finished decoding?
  bool mem_mgmt_5;

  // Created by the decoding process for gaps in frame_num.
  // Not for decode or output.
  bool nonexisting;

  Field field;

  // Values from slice_hdr to be used during reference marking and
  // memory management after finishing this picture.
  bool long_term_reference_flag;
  bool adaptive_ref_pic_marking_mode_flag;
  struct va_h264_dec_ref_pic_marking ref_pic_marking;

  // Position in DPB (i.e. index in DPB).
  int dpb_position;

  virtual ~H264Picture();

 private:
  H264Picture(const H264Picture&) = delete;
  H264Picture& operator=(const H264Picture&) = delete;
};

// DPB - Decoded Picture Buffer.
// Stores decoded pictures that will be used for future display
// and/or reference.
class H264DPB {
 public:
  H264DPB();
  ~H264DPB();

  void set_max_num_pics(size_t max_num_pics);
  size_t max_num_pics() const { return max_num_pics_; }

  // Remove unused (not reference and already outputted) pictures from DPB
  // and free it.
  void DeleteUnused();

  // Remove a picture by its pic_order_cnt and free it.
  void DeleteByPOC(int poc);

  // Clear DPB.
  void Clear();

  // Store picture in DPB. DPB takes ownership of its resources.
  void StorePic(const std::shared_ptr<H264Picture>& pic);

  // Return the number of reference pictures in DPB.
  int CountRefPics();

  // Mark all pictures in DPB as unused for reference.
  void MarkAllUnusedForRef();

  // Return a short-term reference picture by its pic_num.
  std::shared_ptr<H264Picture> GetShortRefPicByPicNum(int pic_num);

  // Return a long-term reference picture by its long_term_pic_num.
  std::shared_ptr<H264Picture> GetLongRefPicByLongTermPicNum(int pic_num);

  // Return the short reference picture with lowest frame_num. Used for sliding
  // window memory management.
  std::shared_ptr<H264Picture> GetLowestFrameNumWrapShortRefPic();

  // Append all pictures that have not been outputted yet to the passed |out|
  // vector, sorted by lowest pic_order_cnt (in output order).
  void GetNotOutputtedPicsAppending(H264Picture::Vector* out);

  // Append all short term reference pictures to the passed |out| vector.
  void GetShortTermRefPicsAppending(H264Picture::Vector* out);

  // Append all long term reference pictures to the passed |out| vector.
  void GetLongTermRefPicsAppending(H264Picture::Vector* out);

  // Iterators for direct access to DPB contents.
  // Will be invalidated after any of Remove* calls.
  H264Picture::Vector::iterator begin() { return pics_.begin(); }
  H264Picture::Vector::iterator end() { return pics_.end(); }
  H264Picture::Vector::const_iterator begin() const { return pics_.begin(); }
  H264Picture::Vector::const_iterator end() const { return pics_.end(); }
  H264Picture::Vector::const_reverse_iterator rbegin() const {
    return pics_.rbegin();
  }
  H264Picture::Vector::const_reverse_iterator rend() const {
    return pics_.rend();
  }

  size_t size() const { return pics_.size(); }
  bool IsFull() const { return pics_.size() == max_num_pics_; }

  // Per H264 spec, increase to 32 if interlaced video is supported.
  enum {
    kDPBMaxSize = 16,
  };

 private:
  void UpdatePicPositions();

  H264Picture::Vector pics_;
  size_t max_num_pics_;

  H264DPB(const H264DPB&) = delete;
  H264DPB& operator=(const H264DPB&) = delete;
};

}  // namespace libva_h264

#endif  // LIBVA_H264_H264_DPB_H_
