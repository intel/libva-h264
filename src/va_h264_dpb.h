/*! \file va_h264_dpb.h
 *  H.264 dpb for use with libva API.
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
#ifndef LIBVA_H264_VA_H264_DPB_H_
#define LIBVA_H264_VA_H264_DPB_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <va_h264_parser.h>

struct va_h264_dpb;

/*! \brief Create a new h264 dpb.
 *
 * Creates and initializes a new h264 dpb.
 *
 * \return a new h264 dpb or NULL on error
 */
struct va_h264_dpb *va_h264_dpb_new(void);

/*! \brief Frees an h264 dpb.
 *
 * Release and invalidate an h264 dpb.
 */
void va_h264_dpb_free(struct va_h264_dpb *);

bool va_h264_dpb_update_one_au(struct va_h264_dpb *d,
                               struct va_buffer_create_info *au, int au_len,
                               int **pics_to_output_before_decode);

void va_h264_dpb_end_of_stream(struct va_h264_dpb *,
                               int **pics_to_output_before_decode);

#ifdef __cplusplus
}
#endif

#endif // LIBVA_H264_VA_H264_DPB_H_
