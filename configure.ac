#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])
AC_INIT([libva-h264], [0.1], [BUG-REPORT-ADDRESS])
AC_CONFIG_MACRO_DIRS([m4])
AC_CONFIG_SRCDIR([src/va_h264_parser.cc])
AM_INIT_AUTOMAKE([foreign subdir-objects dist-xz no-dist-gzip])
AM_SILENT_RULES([yes])
LT_INIT

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC
AM_PROG_VALAC
DX_XML_FEATURE([ON])
DX_HTML_FEATURE([OFF])
DX_INIT_DOXYGEN([libva-h264], [doc/Doxyfile], [doxygen-out])
AM_CONDITIONAL([DX_COND_xml], [test x$DX_FLAG_xml = x1])
AC_CHECK_PROGS([SPHINX_BUILD], [sphinx-build sphinx-build-2 sphinx-build-3])
AM_CONDITIONAL([HAVE_SPHINX], [test x$SPHINX_BUILD != x])
GOBJECT_INTROSPECTION_REQUIRE([1.48.0])
VAPIGEN_CHECK

# Checks for libraries.
AM_PATH_GLIB_2_0([], [], [], [gobject])
PKG_CHECK_MODULES([LIBVA], [libva])
PKG_CHECK_MODULES([GVA_0_1], [gva-0.1])

# Checks for header files.
AC_CHECK_HEADERS([stddef.h stdint.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_CHECK_HEADER_STDBOOL
AC_C_INLINE
AC_TYPE_INT32_T
AC_TYPE_INT64_T
AC_TYPE_OFF_T
AC_TYPE_SIZE_T
AC_TYPE_UINT32_T
AC_TYPE_UINT8_T

# Checks for library functions.
AC_CHECK_FUNCS([memset])

AC_CONFIG_FILES([Makefile doc/Doxyfile doc/conf.py])
AC_OUTPUT
