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

#include <string.h>

#include <algorithm>

#include "h264_dpb.h"

namespace libva_h264 {

H264Picture::H264Picture()
    : pic_order_cnt_type(0),
      top_field_order_cnt(0),
      bottom_field_order_cnt(0),
      pic_order_cnt(0),
      pic_order_cnt_msb(0),
      pic_order_cnt_lsb(0),
      delta_pic_order_cnt_bottom(0),
      delta_pic_order_cnt0(0),
      delta_pic_order_cnt1(0),
      pic_num(0),
      long_term_pic_num(0),
      frame_num(0),
      frame_num_offset(0),
      frame_num_wrap(0),
      long_term_frame_idx(0),
      type(H264SliceHeader::kPSlice),
      nal_ref_idc(0),
      idr(false),
      idr_pic_id(0),
      ref(false),
      long_term(false),
      outputted(false),
      mem_mgmt_5(false),
      nonexisting(false),
      field(FIELD_NONE),
      long_term_reference_flag(false),
      adaptive_ref_pic_marking_mode_flag(false),
      dpb_position(0) {
  memset(&ref_pic_marking, 0, sizeof(ref_pic_marking));
}

H264Picture::~H264Picture() {}

H264DPB::H264DPB() : max_num_pics_(0) {}
H264DPB::~H264DPB() {}

void H264DPB::Clear() {
  pics_.clear();
}

void H264DPB::set_max_num_pics(size_t max_num_pics) {
  max_num_pics_ = max_num_pics;
  if (pics_.size() > max_num_pics_)
    pics_.resize(max_num_pics_);
}

void H264DPB::UpdatePicPositions() {
  size_t i = 0;
  for (auto& pic : pics_) {
    pic->dpb_position = i;
    ++i;
  }
}

void H264DPB::DeleteByPOC(int poc) {
  for (H264Picture::Vector::iterator it = pics_.begin(); it != pics_.end();
       ++it) {
    if ((*it)->pic_order_cnt == poc) {
      pics_.erase(it);
      UpdatePicPositions();
      return;
    }
  }
}

void H264DPB::DeleteUnused() {
  for (H264Picture::Vector::iterator it = pics_.begin(); it != pics_.end();) {
    if ((*it)->outputted && !(*it)->ref)
      it = pics_.erase(it);
    else
      ++it;
  }
  UpdatePicPositions();
}

void H264DPB::StorePic(const std::shared_ptr<H264Picture>& pic) {
  pic->dpb_position = pics_.size();
  pics_.push_back(pic);
}

int H264DPB::CountRefPics() {
  int ret = 0;
  for (size_t i = 0; i < pics_.size(); ++i) {
    if (pics_[i]->ref)
      ++ret;
  }
  return ret;
}

void H264DPB::MarkAllUnusedForRef() {
  for (size_t i = 0; i < pics_.size(); ++i)
    pics_[i]->ref = false;
}

std::shared_ptr<H264Picture> H264DPB::GetShortRefPicByPicNum(int pic_num) {
  for (const auto& pic : pics_) {
    if (pic->ref && !pic->long_term && pic->pic_num == pic_num)
      return pic;
  }

  return nullptr;
}

std::shared_ptr<H264Picture> H264DPB::GetLongRefPicByLongTermPicNum(int pic_num) {
  for (const auto& pic : pics_) {
    if (pic->ref && pic->long_term && pic->long_term_pic_num == pic_num)
      return pic;
  }

  return nullptr;
}

std::shared_ptr<H264Picture> H264DPB::GetLowestFrameNumWrapShortRefPic() {
  std::shared_ptr<H264Picture> ret;
  for (const auto& pic : pics_) {
    if (pic->ref && !pic->long_term &&
        (!ret || pic->frame_num_wrap < ret->frame_num_wrap))
      ret = pic;
  }
  return ret;
}

void H264DPB::GetNotOutputtedPicsAppending(H264Picture::Vector* out) {
  for (const auto& pic : pics_) {
    if (!pic->outputted)
      out->push_back(pic);
  }
}

void H264DPB::GetShortTermRefPicsAppending(H264Picture::Vector* out) {
  for (const auto& pic : pics_) {
    if (pic->ref && !pic->long_term)
      out->push_back(pic);
  }
}

void H264DPB::GetLongTermRefPicsAppending(H264Picture::Vector* out) {
  for (const auto& pic : pics_) {
    if (pic->ref && pic->long_term)
      out->push_back(pic);
  }
}

}  // namespace libva_h264
