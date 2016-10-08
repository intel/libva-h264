/* Copyright (c) 2016 Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 *
 * 1. Redistributions of source code must retain the above
 * copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of
 * its contributors may be used to endorse or promote products
 * derived from this software without specific prior written
 * permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "va_h264_dpb.h"

#include "h264_decoder.h"

#include <deque>

using namespace libva_h264;

struct numbered_picture : public H264Picture {
  static int curr_number;
  static std::deque<int> unused;
  int number;

  numbered_picture();
  ~numbered_picture();
};

int numbered_picture::curr_number = 0;
std::deque<int> numbered_picture::unused;

numbered_picture::numbered_picture() {
  if (unused.empty()) {
    number = curr_number++;
  } else {
    number = unused.front();
    unused.pop_front();
  }
}

numbered_picture::~numbered_picture() { unused.push_back(number); }

struct va_h264_dpb : public H264Decoder::H264Accelerator {
  H264Decoder decoder_;
  std::vector<int> to_output_;
  std::vector<int> output_before_decode_;
  int curr_pic_number_;

  va_h264_dpb() : decoder_(this){};

  bool update_one_au(struct va_buffer_create_info *au, int au_len,
                     int **pics_to_output_before_decode);
  void end_of_stream(int **pics_to_output);

  std::shared_ptr<H264Picture> CreateH264Picture() override;
  bool SubmitFrameMetadata(VAPictureParameterBufferH264 *pp,
                           const H264DPB &dpb,
                           const H264Picture::Vector &ref_pic_listp0,
                           const H264Picture::Vector &ref_pic_listb0,
                           const H264Picture::Vector &ref_pic_listb1,
                           const std::shared_ptr<H264Picture> &pic) override;
  bool SubmitSlice(VAPictureParameterBufferH264 *pp,
                   VASliceParameterBufferH264 *slice,
                   const H264Picture::Vector &ref_pic_list0,
                   const H264Picture::Vector &ref_pic_list1,
                   const std::shared_ptr<H264Picture> &pic, const uint8_t *data,
                   size_t size) override;
  bool SubmitDecode(const std::shared_ptr<H264Picture> &pic) override;
  bool OutputPicture(const std::shared_ptr<H264Picture> &pic) override;
  void Reset() override;
};

std::shared_ptr<H264Picture> va_h264_dpb::CreateH264Picture() {
  return std::make_shared<numbered_picture>();
}

bool va_h264_dpb::SubmitFrameMetadata(VAPictureParameterBufferH264 *pp,
                                      const H264DPB &dpb,
                                      const H264Picture::Vector &ref_pic_listp0,
                                      const H264Picture::Vector &ref_pic_listb0,
                                      const H264Picture::Vector &ref_pic_listb1,
                                      const std::shared_ptr<H264Picture> &pic) {
  pp->ReferenceFrames[0].flags = ~VA_PICTURE_H264_INVALID;
  return true;
}

void h264pic_to_va(VAPictureH264 &va, H264Picture &pic) {
  va.picture_id = reinterpret_cast<numbered_picture &>(pic).number;
  if (pic.ref) {
    if (pic.long_term_reference_flag) {
      va.flags = VA_PICTURE_H264_LONG_TERM_REFERENCE;
      va.frame_idx = pic.long_term_frame_idx;
    } else {
      va.flags = VA_PICTURE_H264_SHORT_TERM_REFERENCE;
      va.frame_idx = pic.frame_num;
    }
  } else {
    va.flags = 0;
    va.frame_idx = 0;
  }
  if (pic.field == H264Picture::FIELD_TOP) {
    va.flags |= VA_PICTURE_H264_TOP_FIELD;
    va.TopFieldOrderCnt = pic.top_field_order_cnt;
    va.BottomFieldOrderCnt = 0;
  } else if (pic.field == H264Picture::FIELD_BOTTOM) {
    va.flags |= VA_PICTURE_H264_BOTTOM_FIELD;
    va.TopFieldOrderCnt = 0;
    va.BottomFieldOrderCnt = pic.bottom_field_order_cnt;
  } else {
    va.TopFieldOrderCnt = pic.top_field_order_cnt;
    va.BottomFieldOrderCnt = pic.bottom_field_order_cnt;
  }
}

void invalid_va(VAPictureH264 &va) {
  va.picture_id = VA_INVALID_SURFACE;
  va.frame_idx = 0;
  va.flags = VA_PICTURE_H264_INVALID;
  va.TopFieldOrderCnt = 0;
  va.BottomFieldOrderCnt = 0;
}

bool va_h264_dpb::SubmitSlice(VAPictureParameterBufferH264 *pp,
                              VASliceParameterBufferH264 *slice,
                              const H264Picture::Vector &ref_pic_list0,
                              const H264Picture::Vector &ref_pic_list1,
                              const std::shared_ptr<H264Picture> &pic,
                              const uint8_t *data, size_t size) {
  unsigned int i;
  if (pp->ReferenceFrames[0].flags == ~((unsigned int)VA_PICTURE_H264_INVALID)) {
    h264pic_to_va(pp->CurrPic, *pic);
    for (i = 0; i < 16 && i < ref_pic_list0.size(); ++i)
      h264pic_to_va(pp->ReferenceFrames[i], *ref_pic_list0[i]);
    for (auto j = ref_pic_list1.cbegin(); j < ref_pic_list1.cend() && i < 16; ++j)
      if (std::find(ref_pic_list0.cbegin(), ref_pic_list0.cend(), *j) == ref_pic_list0.cend())
        h264pic_to_va(pp->ReferenceFrames[i++], **j);
    for (; i < 16; i++)
      invalid_va(pp->ReferenceFrames[i]);
  }
  for (i = 0; i < 32 && i < ref_pic_list0.size(); ++i)
    h264pic_to_va(slice->RefPicList0[i], *ref_pic_list0[i]);
  for (; i < 32; ++i)
    invalid_va(slice->RefPicList0[i]);
  for (i = 0; i < 32 && i < ref_pic_list1.size(); ++i)
    h264pic_to_va(slice->RefPicList1[i], *ref_pic_list1[i]);
  for (; i < 32; ++i)
    invalid_va(slice->RefPicList1[i]);
  return true;
}

bool va_h264_dpb::SubmitDecode(const std::shared_ptr<H264Picture> &pic) {
  curr_pic_number_ = reinterpret_cast<const numbered_picture &>(*pic).number;
  to_output_.push_back(-1);
  return true;
}

bool va_h264_dpb::OutputPicture(const std::shared_ptr<H264Picture> &pic) {
  int pic_number = reinterpret_cast<const numbered_picture &>(*pic).number;
  to_output_.push_back(pic_number);
  return true;
}

void va_h264_dpb::Reset() { return; }

bool va_h264_dpb::update_one_au(struct va_buffer_create_info *au, int au_len,
                                int **pics_to_output_before_decode) {

  for (auto buf = au; buf < au + au_len; ++buf) {
    if (buf->type != VAPictureParameterBufferType)
      continue;
    decoder_.curr_pp_ = (VAPictureParameterBufferH264 *)au->data;
  }

  for (auto buf = au; buf < au + au_len; ++buf) {
    if (buf->type != VASliceParameterBufferType)
      continue;
    auto _data = (VASliceParameterBufferH264 *)buf->data;
    for (auto i = _data; i < _data + buf->num_elements; ++i) {
      decoder_.curr_slice_hdr_ = i;
      decoder_.curr_sps_ = va_slice_parameter_h264_sps(i);
      decoder_.curr_poc_ = va_slice_parameter_h264_poc(i);
      decoder_.curr_ref_pic_list_mod_ =
          va_slice_parameter_h264_ref_pic_list_mod(i);
      decoder_.curr_dec_ref_pic_marking_ =
          va_slice_parameter_h264_dec_ref_pic_marking(i);

      if (i == _data) {
        decoder_.curr_pic_.reset();
        if (!decoder_.ProcessSPS())
          return false;
        if (!decoder_.PreprocessFirstSlice())
          return false;
        decoder_.curr_pic_ = CreateH264Picture();
        if (!decoder_.StartNewFrame())
          return false;
      }
      if (!decoder_.ProcessCurrentSlice())
        return false;
    }
  }
  if (!decoder_.FinishPrevFrameIfPresent())
    return false;

  output_before_decode_.clear();
  auto decode = std::find(to_output_.begin(), to_output_.end(), -1);
  std::copy(to_output_.begin(), decode,
            std::back_inserter(output_before_decode_));
  output_before_decode_.push_back(-1);
  *pics_to_output_before_decode = output_before_decode_.data();

  if (decode != to_output_.end())
    ++decode;

  to_output_.erase(to_output_.begin(), decode);

  return true;
}

void va_h264_dpb::end_of_stream(int **pics_to_output) {
  decoder_.Flush();
  output_before_decode_.clear();
  std::copy(to_output_.begin(), to_output_.end(),
            std::back_inserter(output_before_decode_));
  to_output_.clear();
  output_before_decode_.push_back(-1);
  *pics_to_output = output_before_decode_.data();
}

struct va_h264_dpb *va_h264_dpb_new(void) {
  return new va_h264_dpb();
}

void va_h264_dpb_free(struct va_h264_dpb *d) { delete d; }

bool va_h264_dpb_update_one_au(struct va_h264_dpb *d,
                               struct va_buffer_create_info *au, int au_len,
                               int **pics_to_output_before_decode) {
  return d->update_one_au(au, au_len, pics_to_output_before_decode);
}

void va_h264_dpb_end_of_stream(struct va_h264_dpb *d, int **pics_to_output) {
  d->end_of_stream(pics_to_output);
}
