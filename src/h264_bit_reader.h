// Copyright 2014 The Chromium Authors. All rights reserved.
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
// This file contains an implementation of an H264 Annex-B video stream parser.

#ifndef LIBVA_H264_H264_BIT_READER_H_
#define LIBVA_H264_H264_BIT_READER_H_

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>


namespace libva_h264 {

// A class to provide bit-granularity reading of H.264 streams.
// This is not a generic bit reader class, as it takes into account
// H.264 stream-specific constraints, such as skipping emulation-prevention
// bytes and stop bits. See spec for more details.
class H264BitReader {
 public:
  H264BitReader();
  ~H264BitReader();

  // Initialize the reader to start reading at |data|, |size| being size
  // of |data| in bytes.
  // Return false on insufficient size of stream..
  // TODO(posciak,fischman): consider replacing Initialize() with
  // heap-allocating and creating bit readers on demand instead.
  bool Initialize(const uint8_t* data, off_t size);

  // Read |num_bits| next bits from stream and return in |*out|, first bit
  // from the stream starting at |num_bits| position in |*out|.
  // |num_bits| may be 1-32, inclusive.
  // Return false if the given number of bits cannot be read (not enough
  // bits in the stream), true otherwise.
  bool ReadBits(int num_bits, int* out);

  // Return the number of bits left in the stream.
  off_t NumBitsLeft();

  // See the definition of more_rbsp_data() in spec.
  bool HasMoreRBSPData();

  // Return the number of emulation prevention bytes already read.
  size_t NumEmulationPreventionBytesRead();

 private:
  // Advance to the next byte, loading it into curr_byte_.
  // Return false on end of stream.
  bool UpdateCurrByte();

  // Pointer to the next unread (not in curr_byte_) byte in the stream.
  const uint8_t* data_;

  // Bytes left in the stream (without the curr_byte_).
  off_t bytes_left_;

  // Contents of the current byte; first unread bit starting at position
  // 8 - num_remaining_bits_in_curr_byte_ from MSB.
  int curr_byte_;

  // Number of bits remaining in curr_byte_
  int num_remaining_bits_in_curr_byte_;

  // Used in emulation prevention three byte detection (see spec).
  // Initially set to 0xffff to accept all initial two-byte sequences.
  int prev_two_bytes_;

  // Number of emulation preventation bytes (0x000003) we met.
  size_t emulation_prevention_bytes_;

  H264BitReader(const H264BitReader&) = delete;
  H264BitReader& operator=(const H264BitReader&) = delete;
};

}  // namespace libva_h264

#endif  // LIBVA_H264_H264_BIT_READER_H_
