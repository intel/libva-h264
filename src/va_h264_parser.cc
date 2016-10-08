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

#include "va_h264_parser.h"
#include "h264_parser.h"

#include <cstring>
#include <memory>
#include <vector>

static_assert(sizeof(struct va_h264_sps) + sizeof(struct va_h264_poc) +
                      sizeof(struct va_h264_ref_pic_list_mod) +
                      sizeof(struct va_h264_dec_ref_pic_marking) <=
                  offsetof(VASliceParameterBufferH264, luma_log2_weight_denom) -
                      offsetof(VASliceParameterBufferH264, RefPicList0),
              "extra parsing structs need to fit in RefPicList0, RefPicList1");

using namespace libva_h264;

const uint8_t zigzag_index_4x4[] = {
    0, 1, 5, 6, 2, 4, 7, 12, 3, 8, 11, 13, 9, 10, 14, 15,
};

const uint8_t zigzag_index_8x8[] = {
    0,  1,  5,  6,  14, 15, 27, 28, 2,  4,  7,  13, 16, 26, 29, 42,
    3,  8,  12, 17, 25, 30, 41, 43, 9,  11, 18, 24, 31, 40, 44, 53,
    10, 19, 23, 32, 39, 45, 52, 54, 20, 22, 33, 38, 46, 51, 55, 60,
    21, 34, 37, 47, 50, 56, 59, 61, 35, 36, 48, 49, 57, 58, 62, 63,
};

struct va_h264_au {
  va_h264_au();

  bool pp_set_;
  VAPictureParameterBufferH264 pp_;
  VAIQMatrixBufferH264 iq_;
  std::vector<VASliceParameterBufferH264> slices_;
  std::array<struct va_buffer_create_info, 4> bufs_;
  std::array<int32_t, 256> offset_for_ref_frame_;

  void add_slice(const H264SPS *sps, const H264PPS *pps,
                 const H264SliceHeader *slice, const uint8_t *stream);
  void set_pic_param(const H264SPS *sps, const H264PPS *pps,
                     const H264SliceHeader *slice);
  int finish_bufs(struct va_buffer_create_info **buffers);
};

struct va_h264_parser {
  va_h264_parser() : curr_slice_(nullptr), last_slice_(nullptr) {}

  H264Parser parser_;
  const uint8_t *stream_;
  size_t stream_size_;
  std::unique_ptr<struct va_h264_au> au_;
  H264SliceHeader slices_[2];
  H264SliceHeader *curr_slice_;
  H264SliceHeader *last_slice_;

  int parse_one_au(struct va_buffer_create_info **_buffers);
  void set_stream(const uint8_t *stream, off_t stream_size);
  bool is_first_vcl(const H264SPS *sps);
};

va_h264_au::va_h264_au() : pp_set_(false) {
  bufs_[0].type = VAPictureParameterBufferType;
  bufs_[0].size = sizeof(pp_);
  bufs_[0].num_elements = 1;
  bufs_[0].data = &pp_;

  bufs_[1].type = VAIQMatrixBufferType;
  bufs_[1].size = sizeof(iq_);
  bufs_[1].num_elements = 1;
  bufs_[1].data = &iq_;

  bufs_[2].type = VASliceParameterBufferType;
  bufs_[2].size = sizeof(VASliceParameterBufferH264);
  bufs_[2].num_elements = 0;
  bufs_[2].data = nullptr;

  bufs_[3].type = VASliceDataBufferType;
  bufs_[3].size = 0;
  bufs_[3].num_elements = 1;
  bufs_[3].data = nullptr;
}

void va_h264_au::set_pic_param(const H264SPS *sps, const H264PPS *pps,
                               const H264SliceHeader *slice) {
  if (slice->field_pic_flag)
    pp_.CurrPic.flags = slice->bottom_field_flag ? VA_PICTURE_H264_BOTTOM_FIELD
                                                 : VA_PICTURE_H264_TOP_FIELD;
  else
    pp_.CurrPic.flags = 0;
  if (slice->nal_ref_idc != 0) {
    if (slice->long_term_reference_flag)
      pp_.CurrPic.flags |= VA_PICTURE_H264_LONG_TERM_REFERENCE;
    else
      pp_.CurrPic.flags |= VA_PICTURE_H264_SHORT_TERM_REFERENCE;
  }

  pp_.picture_width_in_mbs_minus1 = sps->pic_width_in_mbs_minus1;
  pp_.picture_height_in_mbs_minus1 =
      (sps->frame_mbs_only_flag ? 1 : 2) *
          (sps->pic_height_in_map_units_minus1 + 1) -
      1;
  pp_.bit_depth_luma_minus8 = sps->bit_depth_luma_minus8;
  pp_.bit_depth_chroma_minus8 = sps->bit_depth_chroma_minus8;
  pp_.num_ref_frames = sps->max_num_ref_frames;
  pp_.seq_fields.bits.chroma_format_idc = sps->chroma_format_idc;
  pp_.seq_fields.bits.residual_colour_transform_flag =
      sps->separate_colour_plane_flag;
  pp_.seq_fields.bits.gaps_in_frame_num_value_allowed_flag =
      sps->gaps_in_frame_num_value_allowed_flag;
  pp_.seq_fields.bits.frame_mbs_only_flag = sps->frame_mbs_only_flag;
  pp_.seq_fields.bits.mb_adaptive_frame_field_flag =
      sps->mb_adaptive_frame_field_flag;
  pp_.seq_fields.bits.direct_8x8_inference_flag =
      sps->direct_8x8_inference_flag;
  pp_.seq_fields.bits.MinLumaBiPredSize8x8 = sps->level_idc >= 31;
  pp_.seq_fields.bits.log2_max_frame_num_minus4 =
      sps->log2_max_frame_num_minus4;
  pp_.seq_fields.bits.pic_order_cnt_type = sps->pic_order_cnt_type;
  pp_.seq_fields.bits.log2_max_pic_order_cnt_lsb_minus4 =
      sps->log2_max_pic_order_cnt_lsb_minus4;
  pp_.seq_fields.bits.delta_pic_order_always_zero_flag =
      sps->delta_pic_order_always_zero_flag;
  pp_.num_slice_groups_minus1 = pps->num_slice_groups_minus1;
  pp_.slice_group_map_type = 0;
  pp_.slice_group_change_rate_minus1 = 0;
  pp_.pic_init_qp_minus26 = pps->pic_init_qp_minus26;
  pp_.pic_init_qs_minus26 = pps->pic_init_qs_minus26;
  pp_.chroma_qp_index_offset = pps->chroma_qp_index_offset;
  pp_.second_chroma_qp_index_offset = pps->second_chroma_qp_index_offset;
  pp_.pic_fields.bits.entropy_coding_mode_flag = pps->entropy_coding_mode_flag;
  pp_.pic_fields.bits.weighted_pred_flag = pps->weighted_pred_flag;
  pp_.pic_fields.bits.weighted_bipred_idc = pps->weighted_bipred_idc;
  pp_.pic_fields.bits.transform_8x8_mode_flag = pps->transform_8x8_mode_flag;
  pp_.pic_fields.bits.field_pic_flag = slice->field_pic_flag;
  pp_.pic_fields.bits.constrained_intra_pred_flag =
      pps->constrained_intra_pred_flag;
  pp_.pic_fields.bits.pic_order_present_flag =
      pps->bottom_field_pic_order_in_frame_present_flag;
  pp_.pic_fields.bits.deblocking_filter_control_present_flag =
      pps->deblocking_filter_control_present_flag;
  pp_.pic_fields.bits.redundant_pic_cnt_present_flag =
      pps->redundant_pic_cnt_present_flag;
  pp_.pic_fields.bits.reference_pic_flag = slice->nal_ref_idc == 0 ? 0 : 1;
  pp_.frame_num = slice->frame_num;

  for (int i = 0; i < 6; i++)
    for (int j = 0; j < kH264ScalingList4x4Length; j++)
      if (pps->pic_scaling_matrix_present_flag)
        iq_.ScalingList4x4[i][j] = pps->scaling_list4x4[i][zigzag_index_4x4[j]];
      else
        iq_.ScalingList4x4[i][j] = sps->scaling_list4x4[i][zigzag_index_4x4[j]];

  for (int i = 0; i < 2; i++)
    for (int j = 0; j < kH264ScalingList8x8Length; j++)
      if (pps->pic_scaling_matrix_present_flag)
        iq_.ScalingList8x8[i][j] = pps->scaling_list8x8[i][zigzag_index_8x8[j]];
      else
        iq_.ScalingList8x8[i][j] = sps->scaling_list8x8[i][zigzag_index_8x8[j]];

  for (int i = 0; i < sps->num_ref_frames_in_pic_order_cnt_cycle; i++)
    offset_for_ref_frame_[i] = sps->offset_for_ref_frame[i];

  pp_set_ = true;
}

void va_h264_au::add_slice(const H264SPS *sps, const H264PPS *pps,
                           const H264SliceHeader *s, const uint8_t *stream) {
  if (!pp_set_)
    set_pic_param(sps, pps, s);

  slices_.emplace_back();
  auto &&slice = slices_.back();
  auto out_sps = va_slice_parameter_h264_sps(&slice);
  auto poc = va_slice_parameter_h264_poc(&slice);
  auto ref_pic_list_mod = va_slice_parameter_h264_ref_pic_list_mod(&slice);
  auto dec_ref_pic_marking =
      va_slice_parameter_h264_dec_ref_pic_marking(&slice);

  bufs_[3].data = const_cast<uint8_t *>(stream);

  slice.slice_data_size = s->nalu_size;
  slice.slice_data_offset = s->nalu_data - stream;
  slice.slice_data_flag = VA_SLICE_DATA_FLAG_ALL;

  slice.slice_data_bit_offset = s->header_bit_size;
  slice.first_mb_in_slice = s->first_mb_in_slice;
  slice.slice_type = s->slice_type % 5;
  slice.direct_spatial_mv_pred_flag = s->direct_spatial_mv_pred_flag;
  slice.num_ref_idx_l0_active_minus1 = s->num_ref_idx_l0_active_minus1;
  slice.num_ref_idx_l1_active_minus1 = s->num_ref_idx_l1_active_minus1;
  slice.cabac_init_idc = s->cabac_init_idc;
  slice.slice_qp_delta = s->slice_qp_delta;
  slice.disable_deblocking_filter_idc = s->disable_deblocking_filter_idc;
  slice.slice_alpha_c0_offset_div2 = s->slice_alpha_c0_offset_div2;
  slice.slice_beta_offset_div2 = s->slice_beta_offset_div2;
  slice.luma_log2_weight_denom = s->luma_log2_weight_denom;
  slice.chroma_log2_weight_denom = s->chroma_log2_weight_denom;
  slice.luma_weight_l0_flag = 1;
  slice.chroma_weight_l0_flag = 1;
  for (int i = 0; i < s->num_ref_idx_l0_active_minus1 + 1; ++i) {
    slice.luma_weight_l0[i] = s->pred_weight_table_l0.luma_weight[i];
    slice.luma_offset_l0[i] = s->pred_weight_table_l0.luma_offset[i];
    slice.chroma_weight_l0[i][0] = s->pred_weight_table_l0.chroma_weight[i][0];
    slice.chroma_offset_l0[i][0] = s->pred_weight_table_l0.chroma_offset[i][0];
    slice.chroma_weight_l0[i][1] = s->pred_weight_table_l0.chroma_weight[i][1];
    slice.chroma_offset_l0[i][1] = s->pred_weight_table_l0.chroma_offset[i][1];
  }
  if (s->IsBSlice()) {
    slice.luma_weight_l1_flag = 1;
    slice.chroma_weight_l1_flag = 1;
    for (int i = 0; i < s->num_ref_idx_l1_active_minus1 + 1; ++i) {
      slice.luma_weight_l1[i] = s->pred_weight_table_l1.luma_weight[i];
      slice.luma_offset_l1[i] = s->pred_weight_table_l1.luma_offset[i];
      slice.chroma_weight_l1[i][0] =
          s->pred_weight_table_l1.chroma_weight[i][0];
      slice.chroma_offset_l1[i][0] =
          s->pred_weight_table_l1.chroma_offset[i][0];
      slice.chroma_weight_l1[i][1] =
          s->pred_weight_table_l1.chroma_weight[i][1];
      slice.chroma_offset_l1[i][1] =
          s->pred_weight_table_l1.chroma_offset[i][1];
    }
  }

  out_sps->profile_idc = sps->profile_idc;
  out_sps->constraint_set3_flag = sps->constraint_set3_flag;
  out_sps->level_idc = sps->level_idc;
  out_sps->vui_parameters_present_flag = sps->vui_parameters_present_flag;
  if (out_sps->vui_parameters_present_flag) {
    out_sps->bitstream_restriction_flag = sps->bitstream_restriction_flag;
    if (out_sps->bitstream_restriction_flag)
      out_sps->max_num_reorder_frames = sps->max_num_reorder_frames;
  }

  poc->pic_order_cnt_type = sps->pic_order_cnt_type;
  if (poc->pic_order_cnt_type == 0) {
    poc->log2_max_pic_order_cnt_lsb_minus4 =
        sps->log2_max_pic_order_cnt_lsb_minus4;
    poc->pic_order_cnt_lsb = s->pic_order_cnt_lsb;
    poc->delta_pic_order_cnt_bottom = s->delta_pic_order_cnt_bottom;
  } else if (poc->pic_order_cnt_type == 1) {
    poc->delta_pic_order_always_zero_flag =
        sps->delta_pic_order_always_zero_flag;
    poc->offset_for_non_ref_pic = sps->offset_for_non_ref_pic;
    poc->offset_for_top_to_bottom_field = sps->offset_for_top_to_bottom_field;
    poc->num_ref_frames_in_pic_order_cnt_cycle =
        sps->num_ref_frames_in_pic_order_cnt_cycle;
    poc->offset_for_ref_frame = offset_for_ref_frame_.data();
    poc->delta_pic_order_cnt[0] = s->delta_pic_order_cnt0;
    poc->delta_pic_order_cnt[1] = s->delta_pic_order_cnt1;
    poc->expected_delta_per_pic_order_cnt_cycle =
        sps->expected_delta_per_pic_order_cnt_cycle;
  }

  if (!s->IsISlice() && !s->IsSISlice()) {
    ref_pic_list_mod->ref_pic_list_modification_flag_l0 =
        s->ref_pic_list_modification_flag_l0;
    if (ref_pic_list_mod->ref_pic_list_modification_flag_l0) {
      for (int i = 0; i < 32; ++i) {
        auto &&dest = ref_pic_list_mod->ref_pic_list_modification_l0[i];
        auto &&src = s->ref_list_l0_modifications[i];
        dest.modification_of_pic_nums_idc = src.modification_of_pic_nums_idc;
        switch (dest.modification_of_pic_nums_idc) {
        case 0:
        case 1:
          dest.abs_diff_pic_num_minus1 = src.abs_diff_pic_num_minus1;
          continue;
        case 2:
          dest.long_term_pic_num = src.long_term_pic_num;
          continue;
        }
        break;
      }
    }
  }

  dec_ref_pic_marking->idr_pic_flag = s->idr_pic_flag;
  dec_ref_pic_marking->idr_pic_id = s->idr_pic_id;
  if (s->idr_pic_flag) {
    dec_ref_pic_marking->no_output_of_prior_pics_flag =
        s->no_output_of_prior_pics_flag;
    dec_ref_pic_marking->long_term_reference_flag = s->long_term_reference_flag;
  } else {
    dec_ref_pic_marking->adaptive_ref_pic_marking_mode_flag =
        s->adaptive_ref_pic_marking_mode_flag;
    for (int i = 0; i < 32; ++i) {
      auto op = dec_ref_pic_marking->ops + i;
      auto op_num = s->ref_pic_marking[i].memory_mgmnt_control_operation;
      op->memory_management_control_operation = op_num;
      if (op_num == 0)
        break;

      if (op_num == 1 || op_num == 3)
        op->difference_of_pic_nums_minus1 =
            s->ref_pic_marking[i].difference_of_pic_nums_minus1;
      if (op_num == 2)
        op->long_term_pic_num = s->ref_pic_marking[i].long_term_pic_num;
      if (op_num == 3 || op_num == 6)
        op->long_term_frame_idx = s->ref_pic_marking[i].long_term_frame_idx;
      if (op_num == 4)
        op->max_long_term_frame_idx_plus1 =
            s->ref_pic_marking[i].max_long_term_frame_idx_plus1;
    }
  }
}

int va_h264_au::finish_bufs(struct va_buffer_create_info **buffers) {
  unsigned int start = std::numeric_limits<unsigned int>::max(),
               end = std::numeric_limits<unsigned int>::min();
  for (auto &&slice : slices_) {
    if (slice.slice_data_offset < start)
      start = slice.slice_data_offset;
    if (slice.slice_data_offset + slice.slice_data_size > end)
      end = slice.slice_data_offset + slice.slice_data_size;
  }
  bufs_[3].data = reinterpret_cast<char *>(bufs_[3].data) + start;
  bufs_[3].size = end - start;

  for (auto &&slice : slices_)
    slice.slice_data_offset -= start;

  bufs_[2].num_elements = slices_.size();
  bufs_[2].data = slices_.data();

  *buffers = bufs_.data();
  return bufs_.size();
}

void va_h264_parser::set_stream(const uint8_t *stream, off_t stream_size) {
  stream_ = stream;
  stream_size_ = stream_size;
  parser_.SetStream(stream, stream_size);
  curr_slice_ = nullptr;
  last_slice_ = nullptr;
}

bool va_h264_parser::is_first_vcl(const H264SPS *sps) {

#define RETURN_TRUE_IF_DIFFERS(field)                                          \
  if (curr_slice_->field != last_slice_->field)                                \
  return true
  RETURN_TRUE_IF_DIFFERS(frame_num);
  RETURN_TRUE_IF_DIFFERS(pic_parameter_set_id);
  RETURN_TRUE_IF_DIFFERS(field_pic_flag);
  RETURN_TRUE_IF_DIFFERS(bottom_field_flag);
  if (curr_slice_->nal_ref_idc == 0 || last_slice_->nal_ref_idc == 0)
    RETURN_TRUE_IF_DIFFERS(nal_ref_idc);
  if (sps->pic_order_cnt_type == 0) {
    RETURN_TRUE_IF_DIFFERS(pic_order_cnt_lsb);
    RETURN_TRUE_IF_DIFFERS(delta_pic_order_cnt_bottom);
  }
  if (sps->pic_order_cnt_type == 1) {
    RETURN_TRUE_IF_DIFFERS(delta_pic_order_cnt0);
    RETURN_TRUE_IF_DIFFERS(delta_pic_order_cnt1);
  }
  RETURN_TRUE_IF_DIFFERS(idr_pic_flag);
  if (curr_slice_->idr_pic_flag)
    RETURN_TRUE_IF_DIFFERS(idr_pic_id);
#undef RETURN_TRUE_IF_DIFFERS

  return false;
}

int va_h264_parser::parse_one_au(struct va_buffer_create_info **buffers) {
  H264NALU nalu;
  int _ign;
  const H264SPS *sps = nullptr;
  const H264PPS *pps = nullptr;
  au_.reset(new struct va_h264_au());

  last_slice_ = nullptr;
  if (curr_slice_ != nullptr) {
    pps = parser_.GetPPS(curr_slice_->pic_parameter_set_id);
    sps = parser_.GetSPS(pps->seq_parameter_set_id);
    au_->add_slice(sps, pps, curr_slice_, stream_);
    last_slice_ = curr_slice_;
    curr_slice_ = nullptr;
  }

  while (true) {
    switch (parser_.AdvanceToNextNALU(&nalu)) {
    case H264Parser::kInvalidStream:
    case H264Parser::kUnsupportedStream:
      return -1;
    case H264Parser::kEOStream:
      goto done;
    default:
      break;
    }

    switch (nalu.nal_unit_type) {
    case H264NALU::kAUD:
      if (last_slice_ != nullptr)
        goto done;
      break;
    case H264NALU::kSPS:
      if (last_slice_ != nullptr)
        goto done;
      if (parser_.ParseSPS(&_ign) != H264Parser::kOk)
        return -1;
      break;
    case H264NALU::kPPS:
      if (last_slice_ != nullptr)
        goto done;
      if (parser_.ParsePPS(&_ign) != H264Parser::kOk)
        return -1;
      break;
    case H264NALU::kSEIMessage:
      if (last_slice_ != nullptr)
        goto done;
      break;
    case H264NALU::kReserved14:
    case H264NALU::kReserved15:
    case H264NALU::kReserved16:
    case H264NALU::kReserved17:
    case H264NALU::kReserved18:
      if (last_slice_ != nullptr)
        goto done;
      break;
    case H264NALU::kIDRSlice:
    case H264NALU::kNonIDRSlice:
      curr_slice_ = last_slice_ != slices_ ? slices_ : slices_ + 1;
      if (parser_.ParseSliceHeader(nalu, curr_slice_) != H264Parser::kOk)
        return -1;
      pps = parser_.GetPPS(curr_slice_->pic_parameter_set_id);
      sps = parser_.GetSPS(pps->seq_parameter_set_id);
      if (last_slice_ != nullptr && is_first_vcl(sps))
        goto done;
      au_->add_slice(sps, pps, curr_slice_, stream_);
      last_slice_ = curr_slice_;
      curr_slice_ = nullptr;
      break;
    default:
      break;
    }
  }

done:
  if (au_->slices_.size() == 0) {
    *buffers = nullptr;
    return 0;
  } else
    return au_.release()->finish_bufs(buffers);
}

struct va_h264_parser *va_h264_parser_new(void) {
  return new struct va_h264_parser();
}

void va_h264_parser_free(struct va_h264_parser *p) { delete p; }

void va_h264_parser_set_stream(struct va_h264_parser *p, const uint8_t *stream,
                               size_t stream_size) {
  p->set_stream(stream, stream_size);
}

int va_h264_parser_parse_one_au(struct va_h264_parser *p,
                                struct va_buffer_create_info **_buffers) {
  return p->parse_one_au(_buffers);
}

void va_h264_parser_free_au(struct va_buffer_create_info *buffers) {
  struct va_h264_au *au =
      (struct va_h264_au *)((char *)buffers -
                            offsetof(struct va_h264_au, bufs_));
  delete au;
}
