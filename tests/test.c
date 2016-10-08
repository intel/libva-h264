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

#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <err.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <X11/Xlib.h>

#include <va_h264_dpb.h>
#include <va_h264_parser.h>
#include <va/va_x11.h>

Display *d;
VADisplay disp = NULL;
VAContextID context = VA_INVALID_ID;
Window window;
VASurfaceID *surfaces = NULL;
int num_surfaces = 0;

void grow_surfaces (void)
{
  int new_num_surfaces = num_surfaces == 0 ? 1 : num_surfaces * 2;
  VASurfaceID *new_surfaces = realloc (surfaces, sizeof (VASurfaceID) *
      new_num_surfaces);
  vaCreateSurfaces (disp, VA_RT_FORMAT_YUV420, 1920, 1088, new_surfaces +
      num_surfaces, new_num_surfaces - num_surfaces, NULL, 0);
  surfaces = new_surfaces;
  num_surfaces = new_num_surfaces;
}

void create_display (void)
{
  int _ign;
  VAConfigID config;
  d = XOpenDisplay (NULL);
  window = XCreateSimpleWindow (d, DefaultRootWindow (d), 0, 0, 1920, 1080, 0,
      BlackPixel (d, DefaultScreen (d)), WhitePixel (d, DefaultScreen (d)));
  XMapWindow (d, window);
  XSelectInput (d, window, StructureNotifyMask);
  for (;;) {
    XEvent e;
    XNextEvent (d, &e);
    if (e.type == MapNotify)
      break;
  }
  disp = vaGetDisplay (d);
  vaInitialize (disp, &_ign, &_ign);
  vaCreateConfig (disp, VAProfileH264High, VAEntrypointVLD, NULL, 0, &config);
  vaCreateContext (disp, config, 1920, 1088, VA_PROGRESSIVE, NULL, 0, &context);
}

void decode (struct va_buffer_create_info *bufs, int nr_bufs)
{
  VABufferID vbufs[nr_bufs];
  VASurfaceID surface;

  if (disp == NULL)
    create_display ();

  for (int i = 0; i < nr_bufs; i++) {
    VAPictureParameterBufferH264 *b = bufs[i].data;
    if (bufs[i].type != VAPictureParameterBufferType)
      continue;

    if (b->CurrPic.picture_id + 1 > num_surfaces)
      grow_surfaces ();
    b->CurrPic.picture_id = surfaces[b->CurrPic.picture_id];
    surface = b->CurrPic.picture_id;
    for (int j = 0; j < 16; j++) {
      if (b->ReferenceFrames[j].flags & VA_PICTURE_H264_INVALID)
        break;
      b->ReferenceFrames[j].picture_id = surfaces[b->ReferenceFrames[j].picture_id];
    }
  }

  for (int i = 0; i < nr_bufs; i++) {
    VASliceParameterBufferH264 *slices = bufs[i].data;
    if (bufs[i].type != VASliceParameterBufferType)
      continue;

    for (VASliceParameterBufferH264 *s = slices;
        s < slices + bufs[i].num_elements; ++s) {
      for (int j = 0; j < 32; j++) {
        if (s->RefPicList0[j].flags & VA_PICTURE_H264_INVALID)
          break;
        s->RefPicList0[j].picture_id = surfaces[s->RefPicList0[j].picture_id];
      }
      for (int j = 0; j < 32; j++) {
        if (s->RefPicList1[j].flags & VA_PICTURE_H264_INVALID)
          break;
        s->RefPicList1[j].picture_id = surfaces[s->RefPicList1[j].picture_id];
      }
    }
  }

  for (int i = 0; i < nr_bufs; i++)
    vaCreateBuffer (disp, context, bufs[i].type, bufs[i].size,
        bufs[i].num_elements, bufs[i].data, vbufs + i);

  vaBeginPicture (disp, context, surface);
  vaRenderPicture (disp, context, vbufs, nr_bufs);
  vaEndPicture (disp, context);
//  vaSyncSurface (disp, surface);
}

void output (int nr)
{
  vaPutSurface (disp, surfaces[nr], window, 0, 0, 1920, 1080, 0, 0, 1920, 1080,
      NULL, 0, 0);
}

int main (int argc, char **argv)
{
  struct va_h264_parser *p;
  struct va_h264_dpb *d;
  int fd;
  struct stat stat;
  void *f;

  if (argc != 2)
    errx (1, "USAGE: %s [filename.h264-annex-b]", argv[0]);
  if ((fd = open (argv[1], O_RDONLY)) < 0)
    err (1, "open");
  if (fstat (fd, &stat))
    err (1, "fstat");
  if ((f = mmap(NULL, stat.st_size, PROT_READ, MAP_SHARED, fd, 0))
      == MAP_FAILED)
    err (1, "mmap");

  p = va_h264_parser_new ();
  d = va_h264_dpb_new ();
  va_h264_parser_set_stream (p, f, stat.st_size);
  
  struct va_buffer_create_info *ci = NULL;
  int bufs;
  int *outputs;
  while ((bufs = va_h264_parser_parse_one_au (p, &ci)) > 0) {
    if (!va_h264_dpb_update_one_au (d, ci, bufs, &outputs))
      errx (1, "error: update_one_au\n");
    for (; *outputs != -1; outputs++)
      output (*outputs);
    decode (ci, bufs);

    va_h264_parser_free_au (ci);
    ci = NULL;
  }
  va_h264_dpb_end_of_stream(d, &outputs);
  for (; *outputs != -1; outputs++)
    output (*outputs);

  return 0;
}
