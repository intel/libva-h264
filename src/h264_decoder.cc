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

#include <algorithm>
#include <limits>
#include <cstring>

#include "h264_decoder.h"

namespace libva_h264 {

template <typename T, size_t N> char (&ArraySizeHelper(T (&array)[N]))[N];
#define arraysize(array) (sizeof(ArraySizeHelper(array)))

H264Decoder::H264Accelerator::H264Accelerator() {}

H264Decoder::H264Accelerator::~H264Accelerator() {}

H264Decoder::H264Decoder(H264Accelerator* accelerator)
    : max_frame_num_(0),
      max_pic_num_(0),
      max_long_term_frame_idx_(0),
      max_num_reorder_frames_(0),
      accelerator_(accelerator) {
  Reset();
  state_ = kNeedStreamMetadata;
}

H264Decoder::~H264Decoder() {}

void H264Decoder::Reset() {
  curr_pic_ = nullptr;
  curr_pp_ = nullptr;
  curr_slice_hdr_ = nullptr;
  curr_poc_ = nullptr;
  curr_ref_pic_list_mod_ = nullptr;
  curr_dec_ref_pic_marking_ = nullptr;

  prev_frame_num_ = -1;
  prev_ref_frame_num_ = -1;
  prev_frame_num_offset_ = -1;
  prev_has_memmgmnt5_ = false;

  prev_ref_has_memmgmnt5_ = false;
  prev_ref_top_field_order_cnt_ = -1;
  prev_ref_pic_order_cnt_msb_ = -1;
  prev_ref_pic_order_cnt_lsb_ = -1;
  prev_ref_field_ = H264Picture::FIELD_NONE;

  ref_pic_list_p0_.clear();
  ref_pic_list_b0_.clear();
  ref_pic_list_b1_.clear();
  dpb_.Clear();
  parser_.Reset();
  accelerator_->Reset();
  last_output_poc_ = std::numeric_limits<int>::min();

  // If we are in kDecoding, we can resume without processing an SPS.
  if (state_ == kDecoding)
    state_ = kAfterReset;
}

void H264Decoder::PrepareRefPicLists() {
  ConstructReferencePicListsP();
  ConstructReferencePicListsB();
}

bool H264Decoder::ModifyReferencePicLists(H264Picture::Vector* ref_pic_list0,
                                          H264Picture::Vector* ref_pic_list1) {
  ref_pic_list0->clear();
  ref_pic_list1->clear();

  // Fill reference picture lists for B and S/SP slices.
  if (curr_slice_hdr_->slice_type % 5 == 0 || curr_slice_hdr_->slice_type % 5 == 3) {
    *ref_pic_list0 = ref_pic_list_p0_;
    return ModifyReferencePicList(0, ref_pic_list0);
  } else if (curr_slice_hdr_->slice_type % 5 == 1) {
    *ref_pic_list0 = ref_pic_list_b0_;
    *ref_pic_list1 = ref_pic_list_b1_;
    return ModifyReferencePicList(0, ref_pic_list0) &&
           ModifyReferencePicList(1, ref_pic_list1);
  }

  return true;
}

bool H264Decoder::DecodePicture() {
  return accelerator_->SubmitDecode(curr_pic_);
}

bool H264Decoder::InitNonexistingPicture(std::shared_ptr<H264Picture> pic,
                                         int frame_num) {
  pic->nonexisting = true;
  pic->nal_ref_idc = 1;
  pic->frame_num = pic->pic_num = frame_num;
  pic->adaptive_ref_pic_marking_mode_flag = false;
  pic->ref = true;
  pic->long_term_reference_flag = false;
  pic->field = H264Picture::FIELD_NONE;

  return CalculatePicOrderCounts(pic);
}

bool H264Decoder::InitCurrPicture() {
  curr_pic_->idr = curr_dec_ref_pic_marking_->idr_pic_flag;
  if (curr_pic_->idr)
    curr_pic_->idr_pic_id = curr_dec_ref_pic_marking_->idr_pic_id;

  if (curr_pp_->pic_fields.bits.field_pic_flag) {
    curr_pic_->field = curr_pp_->CurrPic.flags & VA_PICTURE_H264_BOTTOM_FIELD ?
        H264Picture::FIELD_BOTTOM : H264Picture::FIELD_TOP;
  } else {
    curr_pic_->field = H264Picture::FIELD_NONE;
  }

  if (curr_pic_->field != H264Picture::FIELD_NONE) {
    return false;
  }

  curr_pic_->nal_ref_idc = curr_pp_->pic_fields.bits.reference_pic_flag ? 1 : 0;
  curr_pic_->ref = curr_pp_->pic_fields.bits.reference_pic_flag == 0 ? false : true;
  // This assumes non-interlaced stream.
  curr_pic_->frame_num = curr_pic_->pic_num = curr_pp_->frame_num;

  curr_pic_->pic_order_cnt_type = curr_poc_->pic_order_cnt_type;
  switch (curr_pic_->pic_order_cnt_type) {
    case 0:
      curr_pic_->pic_order_cnt_lsb = curr_poc_->pic_order_cnt_lsb;
      curr_pic_->delta_pic_order_cnt_bottom =
          curr_poc_->delta_pic_order_cnt_bottom;
      break;

    case 1:
      curr_pic_->delta_pic_order_cnt0 = curr_poc_->delta_pic_order_cnt[0];
      curr_pic_->delta_pic_order_cnt1 = curr_poc_->delta_pic_order_cnt[1];
      break;

    case 2:
      break;

    default:
      return false;
  }

  if (!CalculatePicOrderCounts(curr_pic_))
    return false;

  curr_pic_->long_term_reference_flag = (curr_pp_->CurrPic.flags & VA_PICTURE_H264_LONG_TERM_REFERENCE) != 0;
  curr_pic_->adaptive_ref_pic_marking_mode_flag =
      curr_dec_ref_pic_marking_->adaptive_ref_pic_marking_mode_flag;

  // If the slice header indicates we will have to perform reference marking
  // process after this picture is decoded, store required data for that
  // purpose.
  if (curr_dec_ref_pic_marking_->adaptive_ref_pic_marking_mode_flag) {
    static_assert(sizeof(curr_pic_->ref_pic_marking) ==
                      sizeof(*curr_dec_ref_pic_marking_),
                  "Array sizes of ref pic marking do not match.");
    std::memcpy(&curr_pic_->ref_pic_marking, curr_dec_ref_pic_marking_,
           sizeof(curr_pic_->ref_pic_marking));
  }

  return true;
}

bool H264Decoder::CalculatePicOrderCounts(std::shared_ptr<H264Picture> pic) {
  switch (pic->pic_order_cnt_type) {
    case 0: {
      // See spec 8.2.1.1.
      int prev_pic_order_cnt_msb, prev_pic_order_cnt_lsb;

      if (pic->idr) {
        prev_pic_order_cnt_msb = prev_pic_order_cnt_lsb = 0;
      } else {
        if (prev_ref_has_memmgmnt5_) {
          if (prev_ref_field_ != H264Picture::FIELD_BOTTOM) {
            prev_pic_order_cnt_msb = 0;
            prev_pic_order_cnt_lsb = prev_ref_top_field_order_cnt_;
          } else {
            prev_pic_order_cnt_msb = 0;
            prev_pic_order_cnt_lsb = 0;
          }
        } else {
          prev_pic_order_cnt_msb = prev_ref_pic_order_cnt_msb_;
          prev_pic_order_cnt_lsb = prev_ref_pic_order_cnt_lsb_;
        }
      }

      int max_pic_order_cnt_lsb =
          1 << (curr_poc_->log2_max_pic_order_cnt_lsb_minus4 + 4);
      if ((pic->pic_order_cnt_lsb < prev_pic_order_cnt_lsb) &&
          (prev_pic_order_cnt_lsb - pic->pic_order_cnt_lsb >=
           max_pic_order_cnt_lsb / 2)) {
        pic->pic_order_cnt_msb = prev_pic_order_cnt_msb + max_pic_order_cnt_lsb;
      } else if ((pic->pic_order_cnt_lsb > prev_pic_order_cnt_lsb) &&
                 (pic->pic_order_cnt_lsb - prev_pic_order_cnt_lsb >
                  max_pic_order_cnt_lsb / 2)) {
        pic->pic_order_cnt_msb = prev_pic_order_cnt_msb - max_pic_order_cnt_lsb;
      } else {
        pic->pic_order_cnt_msb = prev_pic_order_cnt_msb;
      }

      if (pic->field != H264Picture::FIELD_BOTTOM) {
        pic->top_field_order_cnt =
            pic->pic_order_cnt_msb + pic->pic_order_cnt_lsb;
      }

      if (pic->field != H264Picture::FIELD_TOP) {
        if (pic->field == H264Picture::FIELD_NONE) {
          pic->bottom_field_order_cnt =
              pic->top_field_order_cnt + pic->delta_pic_order_cnt_bottom;
        } else {
          pic->bottom_field_order_cnt =
              pic->pic_order_cnt_msb + pic->pic_order_cnt_lsb;
        }
      }
      break;
    }

    case 1: {
      // See spec 8.2.1.2.
      if (prev_has_memmgmnt5_)
        prev_frame_num_offset_ = 0;

      if (pic->idr)
        pic->frame_num_offset = 0;
      else if (prev_frame_num_ > pic->frame_num)
        pic->frame_num_offset = prev_frame_num_offset_ + max_frame_num_;
      else
        pic->frame_num_offset = prev_frame_num_offset_;

      int abs_frame_num = 0;
      if (curr_poc_->num_ref_frames_in_pic_order_cnt_cycle != 0)
        abs_frame_num = pic->frame_num_offset + pic->frame_num;
      else
        abs_frame_num = 0;

      if (pic->nal_ref_idc == 0 && abs_frame_num > 0)
        --abs_frame_num;

      int expected_pic_order_cnt = 0;
      if (abs_frame_num > 0) {
        if (curr_poc_->num_ref_frames_in_pic_order_cnt_cycle == 0) {
          return false;
        }

        int pic_order_cnt_cycle_cnt =
            (abs_frame_num - 1) / curr_poc_->num_ref_frames_in_pic_order_cnt_cycle;
        int frame_num_in_pic_order_cnt_cycle =
            (abs_frame_num - 1) % curr_poc_->num_ref_frames_in_pic_order_cnt_cycle;

        expected_pic_order_cnt = pic_order_cnt_cycle_cnt *
                                 curr_poc_->expected_delta_per_pic_order_cnt_cycle;
        // frame_num_in_pic_order_cnt_cycle is verified < 255 in parser
        for (int i = 0; i <= frame_num_in_pic_order_cnt_cycle; ++i)
          expected_pic_order_cnt += curr_poc_->offset_for_ref_frame[i];
      }

      if (!pic->nal_ref_idc)
        expected_pic_order_cnt += curr_poc_->offset_for_non_ref_pic;

      if (pic->field == H264Picture::FIELD_NONE) {
        pic->top_field_order_cnt =
            expected_pic_order_cnt + pic->delta_pic_order_cnt0;
        pic->bottom_field_order_cnt = pic->top_field_order_cnt +
                                      curr_poc_->offset_for_top_to_bottom_field +
                                      pic->delta_pic_order_cnt1;
      } else if (pic->field != H264Picture::FIELD_BOTTOM) {
        pic->top_field_order_cnt =
            expected_pic_order_cnt + pic->delta_pic_order_cnt0;
      } else {
        pic->bottom_field_order_cnt = expected_pic_order_cnt +
                                      curr_poc_->offset_for_top_to_bottom_field +
                                      pic->delta_pic_order_cnt0;
      }
      break;
    }

    case 2: {
      // See spec 8.2.1.3.
      if (prev_has_memmgmnt5_)
        prev_frame_num_offset_ = 0;

      if (pic->idr)
        pic->frame_num_offset = 0;
      else if (prev_frame_num_ > pic->frame_num)
        pic->frame_num_offset = prev_frame_num_offset_ + max_frame_num_;
      else
        pic->frame_num_offset = prev_frame_num_offset_;

      int temp_pic_order_cnt;
      if (pic->idr) {
        temp_pic_order_cnt = 0;
      } else if (!pic->nal_ref_idc) {
        temp_pic_order_cnt = 2 * (pic->frame_num_offset + pic->frame_num) - 1;
      } else {
        temp_pic_order_cnt = 2 * (pic->frame_num_offset + pic->frame_num);
      }

      if (pic->field == H264Picture::FIELD_NONE) {
        pic->top_field_order_cnt = temp_pic_order_cnt;
        pic->bottom_field_order_cnt = temp_pic_order_cnt;
      } else if (pic->field == H264Picture::FIELD_BOTTOM) {
        pic->bottom_field_order_cnt = temp_pic_order_cnt;
      } else {
        pic->top_field_order_cnt = temp_pic_order_cnt;
      }
      break;
    }

    default:
      return false;
  }

  switch (pic->field) {
    case H264Picture::FIELD_NONE:
      pic->pic_order_cnt =
          std::min(pic->top_field_order_cnt, pic->bottom_field_order_cnt);
      break;
    case H264Picture::FIELD_TOP:
      pic->pic_order_cnt = pic->top_field_order_cnt;
      break;
    case H264Picture::FIELD_BOTTOM:
      pic->pic_order_cnt = pic->bottom_field_order_cnt;
      break;
  }

  return true;
}

void H264Decoder::UpdatePicNums(int frame_num) {
  for (auto& pic : dpb_) {
    if (!pic->ref)
      continue;

    // 8.2.4.1. Assumes non-interlaced stream.
    if (pic->long_term) {
      pic->long_term_pic_num = pic->long_term_frame_idx;
    } else {
      if (pic->frame_num > frame_num)
        pic->frame_num_wrap = pic->frame_num - max_frame_num_;
      else
        pic->frame_num_wrap = pic->frame_num;

      pic->pic_num = pic->frame_num_wrap;
    }
  }
}

struct PicNumDescCompare {
  bool operator()(const std::shared_ptr<H264Picture>& a,
                  const std::shared_ptr<H264Picture>& b) const {
    return a->pic_num > b->pic_num;
  }
};

struct LongTermPicNumAscCompare {
  bool operator()(const std::shared_ptr<H264Picture>& a,
                  const std::shared_ptr<H264Picture>& b) const {
    return a->long_term_pic_num < b->long_term_pic_num;
  }
};

void H264Decoder::ConstructReferencePicListsP() {
  // RefPicList0 (8.2.4.2.1) [[1] [2]], where:
  // [1] shortterm ref pics sorted by descending pic_num,
  // [2] longterm ref pics by ascending long_term_pic_num.
  ref_pic_list_p0_.clear();

  // First get the short ref pics...
  dpb_.GetShortTermRefPicsAppending(&ref_pic_list_p0_);
  size_t num_short_refs = ref_pic_list_p0_.size();

  // and sort them to get [1].
  std::sort(ref_pic_list_p0_.begin(), ref_pic_list_p0_.end(),
            PicNumDescCompare());

  // Now get long term pics and sort them by long_term_pic_num to get [2].
  dpb_.GetLongTermRefPicsAppending(&ref_pic_list_p0_);
  std::sort(ref_pic_list_p0_.begin() + num_short_refs, ref_pic_list_p0_.end(),
            LongTermPicNumAscCompare());
}

struct POCAscCompare {
  bool operator()(const std::shared_ptr<H264Picture>& a,
                  const std::shared_ptr<H264Picture>& b) const {
    return a->pic_order_cnt < b->pic_order_cnt;
  }
};

struct POCDescCompare {
  bool operator()(const std::shared_ptr<H264Picture>& a,
                  const std::shared_ptr<H264Picture>& b) const {
    return a->pic_order_cnt > b->pic_order_cnt;
  }
};

void H264Decoder::ConstructReferencePicListsB() {
  // RefPicList0 (8.2.4.2.3) [[1] [2] [3]], where:
  // [1] shortterm ref pics with POC < curr_pic's POC sorted by descending POC,
  // [2] shortterm ref pics with POC > curr_pic's POC by ascending POC,
  // [3] longterm ref pics by ascending long_term_pic_num.
  ref_pic_list_b0_.clear();
  ref_pic_list_b1_.clear();
  dpb_.GetShortTermRefPicsAppending(&ref_pic_list_b0_);
  size_t num_short_refs = ref_pic_list_b0_.size();

  // First sort ascending, this will put [1] in right place and finish [2].
  std::sort(ref_pic_list_b0_.begin(), ref_pic_list_b0_.end(), POCAscCompare());

  // Find first with POC > curr_pic's POC to get first element in [2]...
  H264Picture::Vector::iterator iter;
  iter = std::upper_bound(ref_pic_list_b0_.begin(), ref_pic_list_b0_.end(),
                          curr_pic_, POCAscCompare());

  // and sort [1] descending, thus finishing sequence [1] [2].
  std::sort(ref_pic_list_b0_.begin(), iter, POCDescCompare());

  // Now add [3] and sort by ascending long_term_pic_num.
  dpb_.GetLongTermRefPicsAppending(&ref_pic_list_b0_);
  std::sort(ref_pic_list_b0_.begin() + num_short_refs, ref_pic_list_b0_.end(),
            LongTermPicNumAscCompare());

  // RefPicList1 (8.2.4.2.4) [[1] [2] [3]], where:
  // [1] shortterm ref pics with POC > curr_pic's POC sorted by ascending POC,
  // [2] shortterm ref pics with POC < curr_pic's POC by descending POC,
  // [3] longterm ref pics by ascending long_term_pic_num.

  dpb_.GetShortTermRefPicsAppending(&ref_pic_list_b1_);
  num_short_refs = ref_pic_list_b1_.size();

  // First sort by descending POC.
  std::sort(ref_pic_list_b1_.begin(), ref_pic_list_b1_.end(), POCDescCompare());

  // Find first with POC < curr_pic's POC to get first element in [2]...
  iter = std::upper_bound(ref_pic_list_b1_.begin(), ref_pic_list_b1_.end(),
                          curr_pic_, POCDescCompare());

  // and sort [1] ascending.
  std::sort(ref_pic_list_b1_.begin(), iter, POCAscCompare());

  // Now add [3] and sort by ascending long_term_pic_num
  dpb_.GetShortTermRefPicsAppending(&ref_pic_list_b1_);
  std::sort(ref_pic_list_b1_.begin() + num_short_refs, ref_pic_list_b1_.end(),
            LongTermPicNumAscCompare());

  // If lists identical, swap first two entries in RefPicList1 (spec 8.2.4.2.3)
  if (ref_pic_list_b1_.size() > 1 &&
      std::equal(ref_pic_list_b0_.begin(), ref_pic_list_b0_.end(),
                 ref_pic_list_b1_.begin()))
    std::swap(ref_pic_list_b1_[0], ref_pic_list_b1_[1]);
}

// See 8.2.4
int H264Decoder::PicNumF(const std::shared_ptr<H264Picture>& pic) {
  if (!pic)
    return -1;

  if (!pic->long_term)
    return pic->pic_num;
  else
    return max_pic_num_;
}

// See 8.2.4
int H264Decoder::LongTermPicNumF(const std::shared_ptr<H264Picture>& pic) {
  if (pic->ref && pic->long_term)
    return pic->long_term_pic_num;
  else
    return 2 * (max_long_term_frame_idx_ + 1);
}

// Shift elements on the |v| starting from |from| to |to|, inclusive,
// one position to the right and insert pic at |from|.
static void ShiftRightAndInsert(H264Picture::Vector* v,
                                int from,
                                int to,
                                const std::shared_ptr<H264Picture>& pic) {
  v->resize(to + 2);

  for (int i = to + 1; i > from; --i)
    (*v)[i] = (*v)[i - 1];

  (*v)[from] = pic;
}

bool H264Decoder::ModifyReferencePicList(int list,
                                         H264Picture::Vector* ref_pic_listx) {
  bool ref_pic_list_modification_flag_lX;
  int num_ref_idx_lX_active_minus1;
  struct va_h264_ref_pic_list_mod_op *list_mod;

  // This can process either ref_pic_list0 or ref_pic_list1, depending on
  // the list argument. Set up pointers to proper list to be processed here.
  if (list == 0) {
    ref_pic_list_modification_flag_lX =
        curr_ref_pic_list_mod_->ref_pic_list_modification_flag_l0;
    num_ref_idx_lX_active_minus1 = curr_slice_hdr_->num_ref_idx_l0_active_minus1;
    list_mod = curr_ref_pic_list_mod_->ref_pic_list_modification_l0;
  } else {
    ref_pic_list_modification_flag_lX =
        curr_ref_pic_list_mod_->ref_pic_list_modification_flag_l1;
    num_ref_idx_lX_active_minus1 = curr_slice_hdr_->num_ref_idx_l1_active_minus1;
    list_mod = curr_ref_pic_list_mod_->ref_pic_list_modification_l1;
  }

  // Resize the list to the size requested in the slice header.
  // Note that per 8.2.4.2 it's possible for num_ref_idx_lX_active_minus1 to
  // indicate there should be more ref pics on list than we constructed.
  // Those superfluous ones should be treated as non-reference and will be
  // initialized to nullptr, which must be handled by clients.
  ref_pic_listx->resize(num_ref_idx_lX_active_minus1 + 1);

  if (!ref_pic_list_modification_flag_lX)
    return true;

  // Spec 8.2.4.3:
  // Reorder pictures on the list in a way specified in the stream.
  int pic_num_lx_pred = curr_pic_->pic_num;
  int ref_idx_lx = 0;
  int pic_num_lx_no_wrap;
  int pic_num_lx;
  bool done = false;
  std::shared_ptr<H264Picture> pic;
  for (unsigned int i = 0; i < arraysize(curr_ref_pic_list_mod_->ref_pic_list_modification_l0) && !done; ++i) {
    switch (list_mod->modification_of_pic_nums_idc) {
      case 0:
      case 1:
        // Modify short reference picture position.
        if (list_mod->modification_of_pic_nums_idc == 0) {
          // Subtract given value from predicted PicNum.
          pic_num_lx_no_wrap =
              pic_num_lx_pred -
              (static_cast<int>(list_mod->abs_diff_pic_num_minus1) + 1);
          // Wrap around max_pic_num_ if it becomes < 0 as result
          // of subtraction.
          if (pic_num_lx_no_wrap < 0)
            pic_num_lx_no_wrap += max_pic_num_;
        } else {
          // Add given value to predicted PicNum.
          pic_num_lx_no_wrap =
              pic_num_lx_pred +
              (static_cast<int>(list_mod->abs_diff_pic_num_minus1) + 1);
          // Wrap around max_pic_num_ if it becomes >= max_pic_num_ as result
          // of the addition.
          if (pic_num_lx_no_wrap >= max_pic_num_)
            pic_num_lx_no_wrap -= max_pic_num_;
        }

        // For use in next iteration.
        pic_num_lx_pred = pic_num_lx_no_wrap;

        if (pic_num_lx_no_wrap > curr_pic_->pic_num)
          pic_num_lx = pic_num_lx_no_wrap - max_pic_num_;
        else
          pic_num_lx = pic_num_lx_no_wrap;

        pic = dpb_.GetShortRefPicByPicNum(pic_num_lx);
        if (!pic) {
          return false;
        }
        ShiftRightAndInsert(ref_pic_listx, ref_idx_lx,
                            num_ref_idx_lX_active_minus1, pic);
        ref_idx_lx++;

        for (int src = ref_idx_lx, dst = ref_idx_lx;
             src <= num_ref_idx_lX_active_minus1 + 1; ++src) {
          if (PicNumF((*ref_pic_listx)[src]) != pic_num_lx)
            (*ref_pic_listx)[dst++] = (*ref_pic_listx)[src];
        }
        break;

      case 2:
        // Modify long term reference picture position.
        pic = dpb_.GetLongRefPicByLongTermPicNum(list_mod->long_term_pic_num);
        if (!pic) {
          return false;
        }
        ShiftRightAndInsert(ref_pic_listx, ref_idx_lx,
                            num_ref_idx_lX_active_minus1, pic);
        ref_idx_lx++;

        for (int src = ref_idx_lx, dst = ref_idx_lx;
             src <= num_ref_idx_lX_active_minus1 + 1; ++src) {
          if (LongTermPicNumF((*ref_pic_listx)[src]) !=
              static_cast<int>(list_mod->long_term_pic_num))
            (*ref_pic_listx)[dst++] = (*ref_pic_listx)[src];
        }
        break;

      case 3:
        // End of modification list.
        done = true;
        break;

      default:
        // May be recoverable.
        break;
    }

    ++list_mod;
  }

  // Per NOTE 2 in 8.2.4.3.2, the ref_pic_listx size in the above loop is
  // temporarily made one element longer than the required final list.
  // Resize the list back to its required size.
  ref_pic_listx->resize(num_ref_idx_lX_active_minus1 + 1);

  return true;
}

void H264Decoder::OutputPic(std::shared_ptr<H264Picture> pic) {
  pic->outputted = true;

  if (pic->nonexisting) {
    return;
  }

  last_output_poc_ = pic->pic_order_cnt;

  accelerator_->OutputPicture(pic);
}

void H264Decoder::ClearDPB() {
  // Clear DPB contents, marking the pictures as unused first.
  dpb_.Clear();
  last_output_poc_ = std::numeric_limits<int>::min();
}

bool H264Decoder::OutputAllRemainingPics() {
  // Output all pictures that are waiting to be outputted.
  FinishPrevFrameIfPresent();
  H264Picture::Vector to_output;
  dpb_.GetNotOutputtedPicsAppending(&to_output);
  // Sort them by ascending POC to output in order.
  std::sort(to_output.begin(), to_output.end(), POCAscCompare());

  for (auto& pic : to_output)
    OutputPic(pic);

  return true;
}

bool H264Decoder::Flush() {
  if (!OutputAllRemainingPics())
    return false;

  ClearDPB();
  return true;
}

bool H264Decoder::StartNewFrame() {
  // TODO posciak: add handling of max_num_ref_frames per spec.
  max_frame_num_ = 1 << (curr_pp_->seq_fields.bits.log2_max_frame_num_minus4 + 4);
  int frame_num = curr_pp_->frame_num;
  if (curr_dec_ref_pic_marking_->idr_pic_flag)
    prev_ref_frame_num_ = 0;

  // 7.4.3
  if (frame_num != prev_ref_frame_num_ &&
      frame_num != (prev_ref_frame_num_ + 1) % max_frame_num_) {
    if (!HandleFrameNumGap(frame_num))
      return false;
  }

  if (!InitCurrPicture())
    return false;

  UpdatePicNums(frame_num);
  PrepareRefPicLists();

  if (!accelerator_->SubmitFrameMetadata(curr_pp_, dpb_, ref_pic_list_p0_,
                                         ref_pic_list_b0_, ref_pic_list_b1_,
                                         curr_pic_))
    return false;

  return true;
}

bool H264Decoder::HandleMemoryManagementOps(std::shared_ptr<H264Picture> pic) {
  // 8.2.5.4
  for (size_t i = 0; i < arraysize(pic->ref_pic_marking.ops); ++i) {
    // Code below does not support interlaced stream (per-field pictures).
    auto ref_pic_marking = &pic->ref_pic_marking.ops[i];
    std::shared_ptr<H264Picture> to_mark;
    int pic_num_x;

    switch (ref_pic_marking->memory_management_control_operation) {
      case 0:
        // Normal end of operations' specification.
        return true;

      case 1:
        // Mark a short term reference picture as unused so it can be removed
        // if outputted.
        pic_num_x =
            pic->pic_num - (ref_pic_marking->difference_of_pic_nums_minus1 + 1);
        to_mark = dpb_.GetShortRefPicByPicNum(pic_num_x);
        if (to_mark) {
          to_mark->ref = false;
        } else {
          return false;
        }
        break;

      case 2:
        // Mark a long term reference picture as unused so it can be removed
        // if outputted.
        to_mark = dpb_.GetLongRefPicByLongTermPicNum(
            ref_pic_marking->long_term_pic_num);
        if (to_mark) {
          to_mark->ref = false;
        } else {
          return false;
        }
        break;

      case 3:
        // Mark a short term reference picture as long term reference.
        pic_num_x =
            pic->pic_num - (ref_pic_marking->difference_of_pic_nums_minus1 + 1);
        to_mark = dpb_.GetShortRefPicByPicNum(pic_num_x);
        if (to_mark) {
          to_mark->long_term = true;
          to_mark->long_term_frame_idx = ref_pic_marking->long_term_frame_idx;
        } else {
          return false;
        }
        break;

      case 4: {
        // Unmark all reference pictures with long_term_frame_idx over new max.
        max_long_term_frame_idx_ =
            ref_pic_marking->max_long_term_frame_idx_plus1 - 1;
        H264Picture::Vector long_terms;
        dpb_.GetLongTermRefPicsAppending(&long_terms);
        for (size_t i = 0; i < long_terms.size(); ++i) {
          std::shared_ptr<H264Picture>& long_term_pic = long_terms[i];
          // Ok to cast, max_long_term_frame_idx is much smaller than 16bit.
          if (long_term_pic->long_term_frame_idx >
              static_cast<int>(max_long_term_frame_idx_))
            long_term_pic->ref = false;
        }
        break;
      }

      case 5:
        // Unmark all reference pictures.
        dpb_.MarkAllUnusedForRef();
        max_long_term_frame_idx_ = -1;
        pic->mem_mgmt_5 = true;
        break;

      case 6: {
        // Replace long term reference pictures with current picture.
        // First unmark if any existing with this long_term_frame_idx...
        H264Picture::Vector long_terms;
        dpb_.GetLongTermRefPicsAppending(&long_terms);
        for (size_t i = 0; i < long_terms.size(); ++i) {
          std::shared_ptr<H264Picture>& long_term_pic = long_terms[i];
          // Ok to cast, long_term_frame_idx is much smaller than 16bit.
          if (long_term_pic->long_term_frame_idx ==
              static_cast<int>(ref_pic_marking->long_term_frame_idx))
            long_term_pic->ref = false;
        }

        // and mark the current one instead.
        pic->ref = true;
        pic->long_term = true;
        pic->long_term_frame_idx = ref_pic_marking->long_term_frame_idx;
        break;
      }

      default:
        // Would indicate a bug in parser.
        break;
    }
  }

  return true;
}

// This method ensures that DPB does not overflow, either by removing
// reference pictures as specified in the stream, or using a sliding window
// procedure to remove the oldest one.
// It also performs marking and unmarking pictures as reference.
// See spac 8.2.5.1.
bool H264Decoder::ReferencePictureMarking(std::shared_ptr<H264Picture> pic) {
  // If the current picture is an IDR, all reference pictures are unmarked.
  if (pic->idr) {
    dpb_.MarkAllUnusedForRef();

    if (pic->long_term_reference_flag) {
      pic->long_term = true;
      pic->long_term_frame_idx = 0;
      max_long_term_frame_idx_ = 0;
    } else {
      pic->long_term = false;
      max_long_term_frame_idx_ = -1;
    }

    return true;
  }

  // Not an IDR. If the stream contains instructions on how to discard pictures
  // from DPB and how to mark/unmark existing reference pictures, do so.
  // Otherwise, fall back to default sliding window process.
  if (pic->adaptive_ref_pic_marking_mode_flag) {
    return HandleMemoryManagementOps(pic);
  } else {
    return SlidingWindowPictureMarking();
  }
}

bool H264Decoder::SlidingWindowPictureMarking() {
  // 8.2.5.3. Ensure the DPB doesn't overflow by discarding the oldest picture.
  int num_ref_pics = dpb_.CountRefPics();
  if (num_ref_pics == std::max<int>(curr_pp_->num_ref_frames, 1)) {
    // Max number of reference pics reached, need to remove one of the short
    // term ones. Find smallest frame_num_wrap short reference picture and mark
    // it as unused.
    std::shared_ptr<H264Picture> to_unmark =
        dpb_.GetLowestFrameNumWrapShortRefPic();
    if (!to_unmark) {
      return false;
    }

    to_unmark->ref = false;
  }

  return true;
}

bool H264Decoder::FinishPicture(std::shared_ptr<H264Picture> pic) {
  // Finish processing the picture.
  // Start by storing previous picture data for later use.
  if (pic->ref) {
    ReferencePictureMarking(pic);
    prev_ref_has_memmgmnt5_ = pic->mem_mgmt_5;
    prev_ref_top_field_order_cnt_ = pic->top_field_order_cnt;
    prev_ref_pic_order_cnt_msb_ = pic->pic_order_cnt_msb;
    prev_ref_pic_order_cnt_lsb_ = pic->pic_order_cnt_lsb;
    prev_ref_field_ = pic->field;
    prev_ref_frame_num_ = pic->frame_num;
  }
  prev_frame_num_ = pic->frame_num;
  prev_has_memmgmnt5_ = pic->mem_mgmt_5;
  prev_frame_num_offset_ = pic->frame_num_offset;

  // Remove unused (for reference or later output) pictures from DPB, marking
  // them as such.
  dpb_.DeleteUnused();

  // The ownership of pic will either be transferred to DPB - if the picture is
  // still needed (for output and/or reference) - or we will release it
  // immediately if we manage to output it here and won't have to store it for
  // future reference.

  // Get all pictures that haven't been outputted yet.
  H264Picture::Vector not_outputted;
  dpb_.GetNotOutputtedPicsAppending(&not_outputted);
  // Include the one we've just decoded.
  not_outputted.push_back(pic);

  // Sort in output order.
  std::sort(not_outputted.begin(), not_outputted.end(), POCAscCompare());

  // Try to output as many pictures as we can. A picture can be output,
  // if the number of decoded and not yet outputted pictures that would remain
  // in DPB afterwards would at least be equal to max_num_reorder_frames.
  // If the outputted picture is not a reference picture, it doesn't have
  // to remain in the DPB and can be removed.
  H264Picture::Vector::iterator output_candidate = not_outputted.begin();
  size_t num_remaining = not_outputted.size();
  while (num_remaining > max_num_reorder_frames_ ||
         // If the condition below is used, this is an invalid stream. We should
         // not be forced to output beyond max_num_reorder_frames in order to
         // make room in DPB to store the current picture (if we need to do so).
         // However, if this happens, ignore max_num_reorder_frames and try
         // to output more. This may cause out-of-order output, but is not
         // fatal, and better than failing instead.
         ((dpb_.IsFull() && (!pic->outputted || pic->ref)) && num_remaining)) {
    OutputPic(*output_candidate);

    if (!(*output_candidate)->ref) {
      // Current picture hasn't been inserted into DPB yet, so don't remove it
      // if we managed to output it immediately.
      int outputted_poc = (*output_candidate)->pic_order_cnt;
      if (outputted_poc != pic->pic_order_cnt)
        dpb_.DeleteByPOC(outputted_poc);
    }

    ++output_candidate;
    --num_remaining;
  }

  // If we haven't managed to output the picture that we just decoded, or if
  // it's a reference picture, we have to store it in DPB.
  if (!pic->outputted || pic->ref) {
    if (dpb_.IsFull()) {
      // If we haven't managed to output anything to free up space in DPB
      // to store this picture, it's an error in the stream.
      return false;
    }

    dpb_.StorePic(pic);
  }

  return true;
}

static int LevelToMaxDpbMbs(int level) {
  // See table A-1 in spec.
  switch (level) {
    case 10:
      return 396;
    case 11:
      return 900;
    case 12:  //  fallthrough
    case 13:  //  fallthrough
    case 20:
      return 2376;
    case 21:
      return 4752;
    case 22:  //  fallthrough
    case 30:
      return 8100;
    case 31:
      return 18000;
    case 32:
      return 20480;
    case 40:  //  fallthrough
    case 41:
      return 32768;
    case 42:
      return 34816;
    case 50:
      return 110400;
    case 51:  //  fallthrough
    case 52:
      return 184320;
    default:
      return 0;
  }
}

bool H264Decoder::UpdateMaxNumReorderFrames() {
  if (curr_sps_->vui_parameters_present_flag && curr_sps_->bitstream_restriction_flag) {
    max_num_reorder_frames_ =
        static_cast<size_t>(curr_sps_->max_num_reorder_frames);
    if (max_num_reorder_frames_ > dpb_.max_num_pics()) {
      max_num_reorder_frames_ = 0;
      return false;
    }
    return true;
  }

  // max_num_reorder_frames not present, infer from profile/constraints
  // (see VUI semantics in spec).
  if (curr_sps_->constraint_set3_flag) {
    switch (curr_sps_->profile_idc) {
      case 44:
      case 86:
      case 100:
      case 110:
      case 122:
      case 244:
        max_num_reorder_frames_ = 0;
        break;
      default:
        max_num_reorder_frames_ = dpb_.max_num_pics();
        break;
    }
  } else {
    max_num_reorder_frames_ = dpb_.max_num_pics();
  }

  return true;
}

bool H264Decoder::ProcessSPS() {
  if (curr_pp_->seq_fields.bits.frame_mbs_only_flag == 0) {
    return false;
  }

  // Calculate picture height/width in macroblocks and pixels
  // (spec 7.4.2.1.1, 7.4.3).
  int width_mb = curr_pp_->picture_width_in_mbs_minus1 + 1;
  int height_mb = curr_pp_->picture_height_in_mbs_minus1 + 1;

  if (width_mb > std::numeric_limits<int>::max() / 16 ||
      height_mb > std::numeric_limits<int>::max() / 16) {
    return false;
  }

  Size new_pic_size(16 * width_mb, 16 * height_mb);
  if (new_pic_size.IsEmpty()) {
    return false;
  }

  if (!pic_size_.IsEmpty() && new_pic_size == pic_size_) {
    // Already have surfaces and this SPS keeps the same resolution,
    // no need to request a new set.
    return true;
  }

  pic_size_ = new_pic_size;

  int level = curr_sps_->level_idc;
  int max_dpb_mbs = LevelToMaxDpbMbs(level);
  if (max_dpb_mbs == 0)
    return false;

  size_t max_dpb_size = std::min(max_dpb_mbs / (width_mb * height_mb),
                                 static_cast<int>(H264DPB::kDPBMaxSize));
  if (max_dpb_size == 0) {
    return false;
  }

  dpb_.set_max_num_pics(max_dpb_size);

  if (!UpdateMaxNumReorderFrames())
    return false;

  return true;
}

bool H264Decoder::FinishPrevFrameIfPresent() {
  // If we already have a frame waiting to be decoded, decode it and finish.
  if (curr_pic_) {
    if (!DecodePicture())
      return false;

    std::shared_ptr<H264Picture> pic = curr_pic_;
    curr_pic_ = nullptr;
    return FinishPicture(pic);
  }

  return true;
}

bool H264Decoder::HandleFrameNumGap(int frame_num) {
  if (!curr_pp_->seq_fields.bits.gaps_in_frame_num_value_allowed_flag) {
    return false;
  }

  // 7.4.3/7-23
  int unused_short_term_frame_num = (prev_ref_frame_num_ + 1) % max_frame_num_;
  while (unused_short_term_frame_num != frame_num) {
    std::shared_ptr<H264Picture> pic = std::make_shared<H264Picture>();
    if (!InitNonexistingPicture(pic, unused_short_term_frame_num))
      return false;

    UpdatePicNums(unused_short_term_frame_num);

    if (!FinishPicture(pic))
      return false;

    unused_short_term_frame_num++;
    unused_short_term_frame_num %= max_frame_num_;
  }

  return true;
}

bool H264Decoder::PreprocessFirstSlice() {
  // If the new picture is an IDR, flush DPB.
  if (curr_dec_ref_pic_marking_->idr_pic_flag) {
    // Output all remaining pictures, unless we are explicitly instructed
    // not to do so.
    if (!curr_dec_ref_pic_marking_->no_output_of_prior_pics_flag) {
      if (!Flush())
        return false;
    }
    dpb_.Clear();
    last_output_poc_ = std::numeric_limits<int>::min();
  }

  return true;
}

bool H264Decoder::ProcessCurrentSlice() {
  if (curr_pp_->pic_fields.bits.field_pic_flag == 0)
    max_pic_num_ = max_frame_num_;
  else
    max_pic_num_ = 2 * max_frame_num_;

  H264Picture::Vector ref_pic_list0, ref_pic_list1;
  if (!ModifyReferencePicLists(&ref_pic_list0, &ref_pic_list1))
    return false;

  if (!accelerator_->SubmitSlice(curr_pp_, curr_slice_hdr_, ref_pic_list0,
      ref_pic_list1, curr_pic_, nullptr, 0))
    return false;

  return true;
}

#define SET_ERROR_AND_RETURN()         \
  do {                                 \
    state_ = kError;                   \
    return H264Decoder::kDecodeError;  \
  } while (0)

void H264Decoder::SetStream(const uint8_t* ptr, size_t size) {
  parser_.SetStream(ptr, size);
}

H264Decoder::DecodeResult H264Decoder::Decode() {
/*
  if (state_ == kError) {
    return kDecodeError;
  }

  while (1) {
    H264Parser::Result par_res;

    if (!curr_nalu_) {
      curr_nalu_.reset(new H264NALU());
      par_res = parser_.AdvanceToNextNALU(curr_nalu_.get());
      if (par_res == H264Parser::kEOStream)
        return kRanOutOfStreamData;
      else if (par_res != H264Parser::kOk)
        SET_ERROR_AND_RETURN();
    }

    switch (curr_nalu_->nal_unit_type) {
      case H264NALU::kNonIDRSlice:
        // We can't resume from a non-IDR slice.
        if (state_ != kDecoding)
          break;

      // else fallthrough
      case H264NALU::kIDRSlice: {
        // TODO(posciak): the IDR may require an SPS that we don't have
        // available. For now we'd fail if that happens, but ideally we'd like
        // to keep going until the next SPS in the stream.
        if (state_ == kNeedStreamMetadata) {
          // We need an SPS, skip this IDR and keep looking.
          break;
        }

        // If after reset, we should be able to recover from an IDR.
        state_ = kDecoding;

        if (!curr_slice_hdr_) {
          curr_slice_hdr_.reset(new H264SliceHeader());
          par_res =
              parser_.ParseSliceHeader(*curr_nalu_, curr_slice_hdr_.get());
          if (par_res != H264Parser::kOk)
            SET_ERROR_AND_RETURN();

          if (!PreprocessCurrentSlice())
            SET_ERROR_AND_RETURN();
        }

        if (!curr_pic_) {
          // New picture/finished previous one, try to start a new one
          // or tell the client we need more surfaces.
          curr_pic_ = accelerator_->CreateH264Picture();
          if (!curr_pic_)
            return kRanOutOfSurfaces;

          if (!StartNewFrame(curr_slice_hdr_.get()))
            SET_ERROR_AND_RETURN();
        }

        if (!ProcessCurrentSlice())
          SET_ERROR_AND_RETURN();

        curr_slice_hdr_.reset();
        break;
      }

      case H264NALU::kSPS: {
        int sps_id;

        if (!FinishPrevFrameIfPresent())
          SET_ERROR_AND_RETURN();

        par_res = parser_.ParseSPS(&sps_id);
        if (par_res != H264Parser::kOk)
          SET_ERROR_AND_RETURN();

        bool need_new_buffers = false;
        if (!ProcessSPS(sps_id, &need_new_buffers))
          SET_ERROR_AND_RETURN();

        if (state_ == kNeedStreamMetadata)
          state_ = kAfterReset;

        if (need_new_buffers) {
          if (!Flush())
            return kDecodeError;

          curr_pic_ = nullptr;
          curr_nalu_ = nullptr;
          ref_pic_list_p0_.clear();
          ref_pic_list_b0_.clear();
          ref_pic_list_b1_.clear();

          return kAllocateNewSurfaces;
        }
        break;
      }

      case H264NALU::kPPS: {
        int pps_id;

        if (!FinishPrevFrameIfPresent())
          SET_ERROR_AND_RETURN();

        par_res = parser_.ParsePPS(&pps_id);
        if (par_res != H264Parser::kOk)
          SET_ERROR_AND_RETURN();

        break;
      }

      case H264NALU::kAUD:
      case H264NALU::kEOSeq:
      case H264NALU::kEOStream:
        if (state_ != kDecoding)
          break;

        if (!FinishPrevFrameIfPresent())
          SET_ERROR_AND_RETURN();

        break;

      default:
        break;
    }

    curr_nalu_.reset();
  }
*/
  return kDecodeError;
}

Size H264Decoder::GetPicSize() const {
  return pic_size_;
}

size_t H264Decoder::GetRequiredNumOfPictures() const {
  return dpb_.max_num_pics() + kPicsInPipeline;
}

}  // namespace libva_h264
