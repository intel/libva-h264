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

#include "h264_bit_reader.h"

namespace libva_h264 {

H264BitReader::H264BitReader()
    : data_(NULL),
      bytes_left_(0),
      curr_byte_(0),
      num_remaining_bits_in_curr_byte_(0),
      prev_two_bytes_(0),
      emulation_prevention_bytes_(0) {}

H264BitReader::~H264BitReader() {}

bool H264BitReader::Initialize(const uint8_t* data, off_t size) {

  if (size < 1)
    return false;

  data_ = data;
  bytes_left_ = size;
  num_remaining_bits_in_curr_byte_ = 0;
  // Initially set to 0xffff to accept all initial two-byte sequences.
  prev_two_bytes_ = 0xffff;
  emulation_prevention_bytes_ = 0;

  return true;
}

bool H264BitReader::UpdateCurrByte() {
  if (bytes_left_ < 1)
    return false;

  // Emulation prevention three-byte detection.
  // If a sequence of 0x000003 is found, skip (ignore) the last byte (0x03).
  if (*data_ == 0x03 && (prev_two_bytes_ & 0xffff) == 0) {
    // Detected 0x000003, skip last byte.
    ++data_;
    --bytes_left_;
    ++emulation_prevention_bytes_;
    // Need another full three bytes before we can detect the sequence again.
    prev_two_bytes_ = 0xffff;

    if (bytes_left_ < 1)
      return false;
  }

  // Load a new byte and advance pointers.
  curr_byte_ = *data_++ & 0xff;
  --bytes_left_;
  num_remaining_bits_in_curr_byte_ = 8;

  prev_two_bytes_ = ((prev_two_bytes_ & 0xff) << 8) | curr_byte_;

  return true;
}

// Read |num_bits| (1 to 31 inclusive) from the stream and return them
// in |out|, with first bit in the stream as MSB in |out| at position
// (|num_bits| - 1).
bool H264BitReader::ReadBits(int num_bits, int* out) {
  int bits_left = num_bits;
  *out = 0;

  while (num_remaining_bits_in_curr_byte_ < bits_left) {
    // Take all that's left in current byte, shift to make space for the rest.
    *out |= (curr_byte_ << (bits_left - num_remaining_bits_in_curr_byte_));
    bits_left -= num_remaining_bits_in_curr_byte_;

    if (!UpdateCurrByte())
      return false;
  }

  *out |= (curr_byte_ >> (num_remaining_bits_in_curr_byte_ - bits_left));
  *out &= ((1u << num_bits) - 1u);
  num_remaining_bits_in_curr_byte_ -= bits_left;

  return true;
}

off_t H264BitReader::NumBitsLeft() {
  return (num_remaining_bits_in_curr_byte_ + bytes_left_ * 8);
}

bool H264BitReader::HasMoreRBSPData() {
  // Make sure we have more bits, if we are at 0 bits in current byte and
  // updating current byte fails, we don't have more data anyway.
  if (num_remaining_bits_in_curr_byte_ == 0 && !UpdateCurrByte())
    return false;

  // If there is no more RBSP data, then |curr_byte_| contains the stop bit and
  // zero padding. Check to see if there is other data instead.
  // (We don't actually check for the stop bit itself, instead treating the
  // invalid case of all trailing zeros identically).
  if ((curr_byte_ & ((1 << (num_remaining_bits_in_curr_byte_ - 1)) - 1)) != 0)
    return true;

  // While the spec disallows it (7.4.1: "The last byte of the NAL unit shall
  // not be equal to 0x00"), some streams have trailing null bytes anyway. We
  // don't handle emulation prevention sequences because HasMoreRBSPData() is
  // not used when parsing slices (where cabac_zero_word elements are legal).
  for (off_t i = 0; i < bytes_left_; i++) {
    if (data_[i] != 0)
      return true;
  }

  bytes_left_ = 0;
  return false;
}

size_t H264BitReader::NumEmulationPreventionBytesRead() {
  return emulation_prevention_bytes_;
}

}  // namespace libva_h264
