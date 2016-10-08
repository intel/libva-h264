/*! \file va_h264_parser.h
 *  H.264 Annex B parser for use with libva API.
 *
 *  \copyright © 2016 Intel Corporation. All rights reserved.
 *
 *  \par
 *    Redistribution and use in source and binary forms, with or
 *    without modification, are permitted provided that the
 *    following conditions are met:
 *
 *  \par
 *    1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer. 
 *
 *  \par
 *    2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials
 *    provided with the distribution. 
 *
 *  \par
 *    3. Neither the name of the copyright holder nor the names of
 *    its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
 *
 *  \par
 *    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *    CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *    INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *    MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *    CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *    NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *    HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *    OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
 */
#ifndef LIBVA_H264_VA_H264_PARSER_H_
#define LIBVA_H264_VA_H264_PARSER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <va/va.h>

#ifdef __cplusplus
extern "C" {
#endif

/*! \brief Opaque type representing an H.264 parser. */
struct va_h264_parser;

/*! \brief A representation of a buffer that should be created
 *         using vaCreateBuffer().
 */
typedef struct va_buffer_create_info {
  VABufferType type;
  unsigned int size;
  unsigned int num_elements;
  void *data;
} va_buffer_create_info;

/*! \brief SPS syntax elements. */
struct va_h264_sps {
  uint8_t profile_idc;
  bool constraint_set3_flag;
  uint8_t level_idc;
  bool vui_parameters_present_flag;
  bool bitstream_restriction_flag;
  uint32_t max_num_reorder_frames;
};

/*! \brief Syntax elements related to pic order count. */
struct va_h264_poc {
  uint8_t pic_order_cnt_type;
  union {
    struct {
      int32_t log2_max_pic_order_cnt_lsb_minus4;
      uint32_t pic_order_cnt_lsb;
      int32_t delta_pic_order_cnt_bottom;
    };
    struct {
      bool delta_pic_order_always_zero_flag;
      int32_t offset_for_non_ref_pic;
      int32_t offset_for_top_to_bottom_field;
      uint8_t num_ref_frames_in_pic_order_cnt_cycle;
      int32_t *offset_for_ref_frame;
      int32_t delta_pic_order_cnt[2];
      int32_t expected_delta_per_pic_order_cnt_cycle;
    };
  };
};

struct va_h264_ref_pic_list_mod_op {
  uint8_t modification_of_pic_nums_idc;
  union {
    uint32_t abs_diff_pic_num_minus1;
    uint32_t long_term_pic_num;
  };
};

/*! \brief Syntax elements related to reference list
 *          modification. */
struct va_h264_ref_pic_list_mod {
  bool ref_pic_list_modification_flag_l0;
  struct va_h264_ref_pic_list_mod_op ref_pic_list_modification_l0[32];
  bool ref_pic_list_modification_flag_l1;
  struct va_h264_ref_pic_list_mod_op ref_pic_list_modification_l1[32];
};

/*! \brief Syntax elements related to reference pic marking */
struct va_h264_dec_ref_pic_marking {
  bool idr_pic_flag;
  uint16_t idr_pic_id;
  union {
    struct {
      bool no_output_of_prior_pics_flag;
      bool long_term_reference_flag;
    };
    struct {
      bool adaptive_ref_pic_marking_mode_flag;
      struct {
        uint8_t memory_management_control_operation;
        union {
          struct {
            uint32_t difference_of_pic_nums_minus1;
            uint32_t long_term_frame_idx;
          };
          uint32_t long_term_pic_num;
          uint32_t max_long_term_frame_idx_plus1;
        };
      } ops[32];
    };
  };
};

/*! \brief Returns the embedded additional slice header
 *         information from a slice parameter buffer.
 */
static inline struct va_h264_sps *
va_slice_parameter_h264_sps(VASliceParameterBufferH264 *b) {
  return (struct va_h264_sps *)&b->RefPicList0;
}

/*! \brief Returns the embedded pic order count information from a
 *         slice parameter buffer.
 */
static inline struct va_h264_poc *
va_slice_parameter_h264_poc(VASliceParameterBufferH264 *b) {
  return (struct va_h264_poc *)(va_slice_parameter_h264_sps(b) + 1);
}

/*! \brief Returns the embedded reference list modification
 *         information from a slice parameter buffer.
 */
static inline struct va_h264_ref_pic_list_mod *
va_slice_parameter_h264_ref_pic_list_mod(VASliceParameterBufferH264 *b) {
  return (struct va_h264_ref_pic_list_mod *)(va_slice_parameter_h264_poc(b) +
                                             1);
}

/*! \brief Returns the embedded reference pic marking information
 *         from a slice parameter buffer.
 */
static inline struct va_h264_dec_ref_pic_marking *
va_slice_parameter_h264_dec_ref_pic_marking(VASliceParameterBufferH264 *b) {
  return (struct va_h264_dec_ref_pic_marking
              *)(va_slice_parameter_h264_ref_pic_list_mod(b) + 1);
}

/*! \brief Create a new h264 parser.
 *
 * Creates and initializes a new h264 parser with no input.
 *
 * \return a new h264 parser or NULL on error
 */
struct va_h264_parser *va_h264_parser_new(void);

/*! \brief Frees an h264 parser.
 *
 * Release and invalidate an h264 parser.
 */
void va_h264_parser_free(struct va_h264_parser *);

/*! \brief Set the input stream for the parser.
 *
 * Sets the data stream for the parser and sets the internal input
 * cursor at the start of the stream.
 */
void va_h264_parser_set_stream(struct va_h264_parser *, const uint8_t *stream,
                               size_t stream_size);

/*! \brief Parse and return one Access Unit.
 *
 * A single AU from current the input stream is parsed and
 * returned. Any slice parameter buffers will contain embedded pic
 * order count, reference list modification, and reference pic
 * marking syntax elements. These syntax elements can be accessed
 * using va_slice_parameter_h264_poc(),
 * va_slice_parameter_h264_ref_pic_list_mod(), and
 * va_slice_parameter_h264_dec_ref_pic_marking() respectively.
 * These syntax elements cohabitate the space used for reference
 * lists in the slice parameter buffers so should be extracted by
 * the decoding process before writing the reference lists.
 *
 * \param[out] au Set to an array of create_info. The length of
 * the array is the return value of the function.  \return < 0 on
 * error; \n 0 on end-of-stream condition; \n > 0 the number of
 * elements in au
 */
int va_h264_parser_parse_one_au(struct va_h264_parser *,
                                struct va_buffer_create_info **au);

/*! \brief Free an AU returned by parse_one_au.
 *
 * Release and invalid an AU returned by parse_one_au.
 */
void va_h264_parser_free_au(struct va_buffer_create_info *);

#ifdef __cplusplus
}
#endif

#endif // LIBVA_H264_VA_H264_PARSER_H_
