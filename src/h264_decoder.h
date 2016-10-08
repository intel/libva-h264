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

#ifndef LIBVA_H264_H264_DECODER_H_
#define LIBVA_H264_H264_DECODER_H_

#include <stddef.h>
#include <stdint.h>

#include <memory>
#include <vector>

#include "va_h264_parser.h"
#include "h264_parser.h"
#include "h264_dpb.h"

namespace libva_h264 {

struct Size {
  Size() : width_(0), height_(0) {}
  Size(int width, int height) : width_(width), height_(height) {}
  bool IsEmpty() const { return width_ == 0 || height_ == 0; }
  bool operator==(const Size& s) { return width_ == s.width_ && height_ == s.height_; }
  int width_;
  int height_;
};

// Clients of this class are expected to pass H264 Annex-B byte stream
// and are expected to provide an implementation of H264Accelerator for
// offloading final steps of the decoding process.
//
// This class must be created, called and destroyed on a single thread, and
// does nothing internally on any other thread.
class H264Decoder {
 public:
  enum DecodeResult {
    kDecodeError,  // Error while decoding.
    // TODO(posciak): unsupported streams are currently treated as error
    // in decoding; in future it could perhaps be possible to fall back
    // to software decoding instead.
    // kStreamError,  // Error in stream.
    kAllocateNewSurfaces,  // Need a new set of surfaces to be allocated.
    kRanOutOfStreamData,   // Need more stream data to proceed.
    kRanOutOfSurfaces,     // Waiting for the client to free up output surfaces.
  };

  class H264Accelerator {
   public:
    H264Accelerator();
    virtual ~H264Accelerator();

    // Create a new H264Picture that the decoder client can use for decoding
    // and pass back to this accelerator for decoding or reference.
    // When the picture is no longer needed by decoder, it will just drop
    // its reference to it, and it may do so at any time.
    // Note that this may return nullptr if accelerator is not able to provide
    // any new pictures at given time. The decoder is expected to handle
    // this situation as normal and return from Decode() with kRanOutOfSurfaces.
    virtual std::shared_ptr<H264Picture> CreateH264Picture() = 0;

    // Submit metadata for the current frame, providing the current |sps| and
    // |pps| for it, |dpb| has to contain all the pictures in DPB for current
    // frame, and |ref_pic_p0/b0/b1| as specified in the H264 spec. Note that
    // depending on the frame type, either p0, or b0 and b1 are used. |pic|
    // contains information about the picture for the current frame.
    // Note that this does not run decode in the accelerator and the decoder
    // is expected to follow this call with one or more SubmitSlice() calls
    // before calling SubmitDecode().
    // Return true if successful.
    virtual bool SubmitFrameMetadata(VAPictureParameterBufferH264* pp,
                                     const H264DPB& dpb,
                                     const H264Picture::Vector& ref_pic_listp0,
                                     const H264Picture::Vector& ref_pic_listb0,
                                     const H264Picture::Vector& ref_pic_listb1,
                                     const std::shared_ptr<H264Picture>& pic) = 0;

    // Submit one slice for the current frame, passing the current |pps| and
    // |pic| (same as in SubmitFrameMetadata()), the parsed header for the
    // current slice in |slice_hdr|, and the reordered |ref_pic_listX|,
    // as per H264 spec.
    // |data| pointing to the full slice (including the unparsed header| of
    // |size| in bytes.
    // This must be called one or more times per frame, before SubmitDecode().
    // Note that |data| does not have to remain valid after this call returns.
    // Return true if successful.
    virtual bool SubmitSlice(VAPictureParameterBufferH264* pp,
                             VASliceParameterBufferH264* slice,
                             const H264Picture::Vector& ref_pic_list0,
                             const H264Picture::Vector& ref_pic_list1,
                             const std::shared_ptr<H264Picture>& pic,
                             const uint8_t* data,
                             size_t size) = 0;

    // Execute the decode in hardware for |pic|, using all the slices and
    // metadata submitted via SubmitFrameMetadata() and SubmitSlice() since
    // the previous call to SubmitDecode().
    // Return true if successful.
    virtual bool SubmitDecode(const std::shared_ptr<H264Picture>& pic) = 0;

    // Schedule output (display) of |pic|. Note that returning from this
    // method does not mean that |pic| has already been outputted (displayed),
    // but guarantees that all pictures will be outputted in the same order
    // as this method was called for them. Decoder may drop its reference
    // to |pic| after calling this method.
    // Return true if successful.
    virtual bool OutputPicture(const std::shared_ptr<H264Picture>& pic) = 0;

    // Reset any current state that may be cached in the accelerator, dropping
    // any cached parameters/slices that have not been committed yet.
    virtual void Reset() = 0;

   private:
    H264Accelerator(const H264Accelerator&) = delete;
    H264Accelerator& operator=(const H264Accelerator&) = delete;
  };

  H264Decoder(H264Accelerator* accelerator);
  ~H264Decoder();

  // AcceleratedVideoDecoder implementation.
  bool Flush();
  void Reset();
  void SetStream(const uint8_t* ptr, size_t size);
  DecodeResult Decode();
  Size GetPicSize() const;
  size_t GetRequiredNumOfPictures() const;

  // We need to keep at most kDPBMaxSize pictures in DPB for
  // reference/to display later and an additional one for the one currently
  // being decoded. We also ask for some additional ones since VDA needs
  // to accumulate a few ready-to-output pictures before it actually starts
  // displaying and giving them back. +2 instead of +1 because of subjective
  // smoothness improvement during testing.
  enum {
    kPicsInPipeline = 6,
    kMaxNumReqPictures = H264DPB::kDPBMaxSize + kPicsInPipeline,
  };

  // Internal state of the decoder.
  enum State {
    kNeedStreamMetadata,  // After initialization, need an SPS.
    kDecoding,            // Ready to decode from any point.
    kAfterReset,          // After Reset(), need a resume point.
    kError,               // Error in decode, can't continue.
  };

  // Process H264 stream structures.
  bool ProcessSPS();
  // Process current slice header to discover if we need to start a new picture,
  // finishing up the current one.
  bool PreprocessFirstSlice();
  // Process current slice as a slice of the current picture.
  bool ProcessCurrentSlice();

  // Initialize the current picture according to data in |slice_hdr|.
  bool InitCurrPicture();

  // Initialize |pic| as a "non-existing" picture (see spec) with |frame_num|,
  // to be used for frame gap concealment.
  bool InitNonexistingPicture(std::shared_ptr<H264Picture> pic, int frame_num);

  // Calculate picture order counts for |pic| on initialization
  // of a new frame (see spec).
  bool CalculatePicOrderCounts(std::shared_ptr<H264Picture> pic);

  // Update PicNum values in pictures stored in DPB on creation of
  // a picture with |frame_num|.
  void UpdatePicNums(int frame_num);

  bool UpdateMaxNumReorderFrames();

  // Prepare reference picture lists for the current frame.
  void PrepareRefPicLists();
  // Prepare reference picture lists for the given slice.
  bool ModifyReferencePicLists(H264Picture::Vector* ref_pic_list0,
                               H264Picture::Vector* ref_pic_list1);

  // Construct initial reference picture lists for use in decoding of
  // P and B pictures (see 8.2.4 in spec).
  void ConstructReferencePicListsP();
  void ConstructReferencePicListsB();

  // Helper functions for reference list construction, per spec.
  int PicNumF(const std::shared_ptr<H264Picture>& pic);
  int LongTermPicNumF(const std::shared_ptr<H264Picture>& pic);

  // Perform the reference picture lists' modification (reordering), as
  // specified in spec (8.2.4).
  //
  // |list| indicates list number and should be either 0 or 1.
  bool ModifyReferencePicList(int list,
                              H264Picture::Vector* ref_pic_listx);

  // Perform reference picture memory management operations (marking/unmarking
  // of reference pictures, long term picture management, discarding, etc.).
  // See 8.2.5 in spec.
  bool HandleMemoryManagementOps(std::shared_ptr<H264Picture> pic);
  bool ReferencePictureMarking(std::shared_ptr<H264Picture> pic);
  bool SlidingWindowPictureMarking();

  // Handle a gap in frame_num in the stream up to |frame_num|, by creating
  // "non-existing" pictures (see spec).
  bool HandleFrameNumGap(int frame_num);

  // Start processing a new frame.
  bool StartNewFrame();

  // All data for a frame received, process it and decode.
  bool FinishPrevFrameIfPresent();

  // Called after we are done processing |pic|. Performs all operations to be
  // done after decoding, including DPB management, reference picture marking
  // and memory management operations.
  // This will also output pictures if any have become ready to be outputted
  // after processing |pic|.
  bool FinishPicture(std::shared_ptr<H264Picture> pic);

  // Clear DPB contents and remove all surfaces in DPB from *in_use_ list.
  // Cleared pictures will be made available for decode, unless they are
  // at client waiting to be displayed.
  void ClearDPB();

  // Commits all pending data for HW decoder and starts HW decoder.
  bool DecodePicture();

  // Notifies client that a picture is ready for output.
  void OutputPic(std::shared_ptr<H264Picture> pic);

  // Output all pictures in DPB that have not been outputted yet.
  bool OutputAllRemainingPics();

  // Decoder state.
  State state_;

  // Parser in use.
  H264Parser parser_;

  // DPB in use.
  H264DPB dpb_;

  // Picture currently being processed/decoded.
  std::shared_ptr<H264Picture> curr_pic_;

  // Reference picture lists, constructed for each frame.
  H264Picture::Vector ref_pic_list_p0_;
  H264Picture::Vector ref_pic_list_b0_;
  H264Picture::Vector ref_pic_list_b1_;

  // Global state values, needed in decoding. See spec.
  int max_frame_num_;
  int max_pic_num_;
  int max_long_term_frame_idx_;
  size_t max_num_reorder_frames_;

  int prev_frame_num_;
  int prev_ref_frame_num_;
  int prev_frame_num_offset_;
  bool prev_has_memmgmnt5_;

  // Values related to previously decoded reference picture.
  bool prev_ref_has_memmgmnt5_;
  int prev_ref_top_field_order_cnt_;
  int prev_ref_pic_order_cnt_msb_;
  int prev_ref_pic_order_cnt_lsb_;
  H264Picture::Field prev_ref_field_;

  // Current NALU and slice header being processed.
  VAPictureParameterBufferH264 *curr_pp_;
  VASliceParameterBufferH264 *curr_slice_hdr_;
  struct va_h264_sps *curr_sps_;
  struct va_h264_poc *curr_poc_;
  struct va_h264_ref_pic_list_mod *curr_ref_pic_list_mod_;
  struct va_h264_dec_ref_pic_marking *curr_dec_ref_pic_marking_;

  // Output picture size.
  Size pic_size_;

  // PicOrderCount of the previously outputted frame.
  int last_output_poc_;

  H264Accelerator* accelerator_;

  H264Decoder(const H264Decoder&) = delete;
  H264Decoder& operator=(const H264Decoder&) = delete;
};

}  // namespace libva_h264

#endif  // LIBVA_H264_H264_DECODER_H_
