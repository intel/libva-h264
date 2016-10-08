# Copyright (c) 2016 Intel Corporation. All rights reserved.
#
# Redistribution and use in source and binary forms, with or
# without modification, are permitted provided that the following
# conditions are met:
#
# 1. Redistributions of source code must retain the above
# copyright notice, this list of conditions and the following
# disclaimer.
#
# 2. Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of
# its contributors may be used to endorse or promote products
# derived from this software without specific prior written
# permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
# NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import collections
import ctypes
import gi
import itertools
import mmap
import os
import sys

gi.require_version ('Va', '1.0')
gi.require_version ('VaX11', '1.0')
gi.require_version ('GVa', '0.1')
gi.require_version ('VaH264', '0.1')
gi.require_version ('GVaH264', '0.1')

from gi.repository import Va
from gi.repository import VaX11
from gi.repository import GVa
from gi.repository import VaH264
from gi.repository import GVaH264

X11 = ctypes.CDLL ('libX11.so.6')
X11.XOpenDisplay.argtypes = [ctypes.c_char_p]
X11.XOpenDisplay.restype = ctypes.c_void_p
X11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
X11.XDefaultRootWindow.restype = ctypes.c_int32
X11.XDefaultScreen.argtypes = [ctypes.c_void_p]
X11.XDefaultScreen.restype = ctypes.c_int
X11.XBlackPixel.argtypes = [ctypes.c_void_p, ctypes.c_int]
X11.XBlackPixel.restype = ctypes.c_ulong
X11.XWhitePixel.argtypes = [ctypes.c_void_p, ctypes.c_int]
X11.XWhitePixel.restype = ctypes.c_ulong
X11.XCreateSimpleWindow.argtypes = [ctypes.c_void_p, ctypes.c_int32,
    ctypes.c_int, ctypes.c_int, ctypes.c_uint, ctypes.c_uint, ctypes.c_uint,
    ctypes.c_ulong, ctypes.c_ulong]
X11.XCreateSimpleWindow.restype = ctypes.c_int32
X11.XMapWindow.argtypes = [ctypes.c_void_p, ctypes.c_int32]
X11.XMapWindow.restype = ctypes.c_int
X11.XSelectInput.argtypes = [ctypes.c_void_p, ctypes.c_int32, ctypes.c_long]
X11.XSelectInput.restype = ctypes.c_int
X11.XNextEvent.argtypes = [ctypes.c_void_p, ctypes.c_long*24]
X11.XNextEvent.restype = ctypes.c_int
VaX = ctypes.CDLL ('libva-x11.so')
VaX.vaGetDisplay.argtypes = [ctypes.c_void_p]
VaX.vaGetDisplay.restype = ctypes.c_void_p

display = X11.XOpenDisplay (b':0')
window = X11.XCreateSimpleWindow (display, X11.XDefaultRootWindow (display), 0,
    0, 1920, 1080, 0, X11.XBlackPixel (display, X11.XDefaultScreen (display)),
    X11.XWhitePixel (display, X11.XDefaultScreen (display)))
X11.XMapWindow (display, window)
X11.XSelectInput (display, window, 0x20000)
while True:
  event = (ctypes.c_long * 24) ()
  X11.XNextEvent (display, event)
  if ctypes.cast (event, ctypes.POINTER (ctypes.c_int))[0] == 19:
    break
va_disp = GVa.Display.new (VaX.vaGetDisplay (display))
config = va_disp.create_config (Va.Profile.H264_MAIN, Va.Entrypoint.VLD, [])
context = config.create_context (1920, 1080, Va.PROGRESSIVE, [])

infile_map = mmap.mmap (os.open (sys.argv[1], os.O_RDWR), 0)
infile = ctypes.addressof (ctypes.c_ubyte.from_buffer (infile_map))
parser = GVaH264.Parser.new ()
dpb = GVaH264.DPB.new ()
parser.set_stream (infile, infile_map.size ())
surfaces = collections.defaultdict (lambda: va_disp.create_surface (
    Va.RT_FORMAT_YUV420, 1920, 1080, []))

def update_surface_ids (au):
  pp = au.pic_param ()
  s = surfaces[pp.CurrPic.picture_id]
  pp.CurrPic.picture_id = s.id
  for frame_nr in range (16):
    pic = GVaH264.Util.pic_param_reference_frame (pp, frame_nr)
    if pic.flags & Va.PICTURE_H264_INVALID != 0:
      break
    pic.picture_id = surfaces[pic.picture_id].id
  for slice_nr in itertools.count ():
    slice = au.slice_param (slice_nr)
    if slice is None:
      break
    for reflist in (0, 1):
      for i in range (32):
        pic = GVaH264.Util.slice_param_ref_pic_list (slice, reflist, i)
        if pic.flags & Va.PICTURE_H264_INVALID != 0:
          break
        pic.picture_id = surfaces[pic.picture_id].id
  return s

while True:
  au = parser.parse_one_au ()
  if not au:
    break
  _,to_output = dpb.update_one_au (au)
  for o in to_output:
    VaX11.put_surface (va_disp.disp, surfaces[o].id, window, 0, 0, 1920, 1080,
        0, 0, 1920, 1080, Va.Rectangle (), 0, 0)
  surface = update_surface_ids (au)
  buffers = au.make_buffers (context)
  context.begin_picture (surface)
  context.render_picture (buffers)
  context.end_picture ()
