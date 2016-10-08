.. _libva-h264-parser:

##########
  Parser
##########

The ``va_h264_parser`` parses an H.264 Annex B bytestream to
libva API data structures. The opaque struct ``va_h264_parser``
represents the parser itself:

.. doxygenstruct:: va_h264_parser

A new ``va_h264_parser`` is created with:

.. doxygenfunction:: va_h264_parser_new

When a ``va_h264_parser`` is no longer needed it should be
released with:

.. doxygenfunction:: va_h264_parser_free

The ``va_h264_parser`` starts with an empty input stream. To
change the input stream call:

.. doxygenfunction:: va_h264_parser_set_stream

H.264 Access Units can then be parsed and returned with:

.. doxygenfunction:: va_h264_parser_parse_one_au

The slice parameter buffers will contain extra syntax elements
embedded in them beyond what is required to be given to the libva
API. The decoder will access this information using these three
functions:

.. doxygenfunction:: va_slice_parameter_h264_poc
.. doxygenfunction:: va_slice_parameter_h264_ref_pic_list_mod
.. doxygenfunction:: va_slice_parameter_h264_dec_ref_pic_marking
