ACLOCAL_AMFLAGS = -I m4

lib_LTLIBRARIES = libva-h264.la libgva-h264-0.1.la
libva_h264_la_SOURCES = \
	src/h264_bit_reader.cc \
	src/h264_bit_reader.h \
	src/h264_decoder.cc \
	src/h264_decoder.h \
	src/h264_dpb.cc \
	src/h264_dpb.h \
	src/h264_parser.cc \
	src/h264_parser.h \
	src/ranges.h \
	src/va_h264_dpb.cc \
	src/va_h264_dpb.h \
	src/va_h264_parser.cc \
	src/va_h264_parser.h
libgva_h264_0_1_la_SOURCES = \
	src/au.vala \
	src/dpb.vala \
	src/parser.vala \
	src/util.vala
include_HEADERS = \
	src/va_h264_parser.h \
	src/va_h264_dpb.h \
	gva-h264-0.1.h
libva_h264_la_CFLAGS = $(LIBVA_CFLAGS)
libva_h264_la_CXXFLAGS = $(LIBVA_CFLAGS)
libva_h264_la_CPPFLAGS = -I$(srcdir)/src
libva_h264_la_LDFLAGS = $(LIBVA_LIBS)
libgva_h264_0_1_la_VALAFLAGS = --pkg libva --pkg gva-0.1
libgva_h264_0_1_la_VALAFLAGS += --vapidir $(abs_builddir)/interface
libgva_h264_0_1_la_VALAFLAGS += --pkg libva-h264
libgva_h264_0_1_la_VALAFLAGS += --vapi interface/gvah264-0.1.vapi
libgva_h264_0_1_la_CFLAGS = $(GLIB_CFLAGS) $(LIBVA_CFLAGS) $(GVA_0_1_CFLAGS)
libgva_h264_0_1_la_CFLAGS += -I$(srcdir)/src
libgva_h264_0_1_la_LDFLAGS = $(GLIB_LIBS) $(LIBVA_LIBS) $(GVA_0_1_CFLAGS)
libgva_h264_0_1_la_LIBADD = libva-h264.la

include $(INTROSPECTION_MAKEFILE)
INTROSPECTION_GIRS = interface/VaH264-0.1.gir.pre
interface/VaH264-0.1.gir.pre: | libva-h264.la
interface_VaH264_0_1_gir_pre_NAMESPACE = VaH264
interface_VaH264_0_1_gir_pre_VERSION = 0.1
interface_VaH264_0_1_gir_pre_LIBS = libva-h264.la
interface_VaH264_0_1_gir_pre_CFLAGS = -I$(srcdir)/src
interface_VaH264_0_1_gir_pre_FILES = src/va_h264_parser.h src/va_h264_dpb.h
interface_VaH264_0_1_gir_pre_INCLUDES = Va-1.0
interface_VaH264_0_1_gir_pre_SCANNERFLAGS = --accept-unprefixed
interface_VaH264_0_1_gir_pre_SCANNERFLAGS += --identifier-prefix=va_h264_
interface_VaH264_0_1_gir_pre_SCANNERFLAGS += $$(patsubst %,--c-include=%,$$(notdir $$^))
girdir = $(datadir)/gir-1.0
dist_gir_DATA = interface/VaH264-0.1.gir interface/GVaH264-0.1.gir
typelibdir = $(libdir)/girepository-1.0
typelib_DATA = interface/VaH264-0.1.typelib interface/GVaH264-0.1.typelib
vapidir = $(datadir)/vala/vapi
dist_vapi_DATA = interface/libva-h264.vapi interface/gvah264-0.1.vapi

CLEANFILES = $(dist_gir_DATA) $(typelib_DATA)
CLEANFILES += interface/VaH264-0.1.gir.pre interface/4vapi-VaH264-0.1.gir interface/libva-h264.vapi

include $(VAPIGEN_MAKEFILE)
VAPIGEN_VAPIS = interface/libva-h264.vapi
interface_libva_h264_vapi_GIR = interface/4vapi-VaH264-0.1.gir
interface_libva_h264_vapi_FILES = $(interface_libva_h264_vapi_GIR)
interface_libva_h264_vapi_DEPS = libva
interface_libva_h264_vapi_METADATADIRS = $(srcdir)/interface/metadata

EXTRA_DIST = doc/contents.rst doc/parser.rst doc/dpb.rst scripts/gir4vapi.py \
	interface/metadata/4vapi-VaH264-0.1.metadata
MOSTLYCLEANFILES = $(DX_CLEANFILES)

$(srcdir)/libgva_h264_0_1_la_vala.stamp: interface/libva-h264.vapi

interface:
	mkdir $@

interface/VaH264-0.1.gir: interface/VaH264-0.1.gir.pre
	$(AM_V_at)$(AM_V_GEN)sed -e 's/introspectable="0" //' \
	-e 's/identifier-prefixes="va_h264_/identifier-prefixes="VaH264/g' \
	-e 's/symbol-prefixes="va_h264_/symbol-prefixes="va_h264/g' \
	-e 's/c:type="VABufferType"/name="Va.BufferType" &/' < $< > $@

interface/4vapi-VaH264-0.1.gir: interface/VaH264-0.1.gir
	$(AM_V_at)$(AM_V_GEN)$(srcdir)/scripts/gir4vapi.py < $< > $@

interface/GVaH264-0.1.gir: | interface/libva-h264.vapi
interface/GVaH264-0.1.gir: $(libgva_h264_0_1_la_SOURCES)
	$(AM_V_at)$(AM_V_GEN)$(VALAC) $(wildcard $(srcdir)/src/*.vala) \
	--gir $@ --shared-library libgva-h264-0.1.so --library gva-h264-0.1 \
	--header gva-h264-0.1.h -Xc -shared $(libgva_h264_0_1_la_VALAFLAGS) \
	-Xc -I$(srcdir)/src -o /dev/null
	$(AM_V_at)sed -i \
	-e 's/name="gpointer" c:type="VAPictureP/name="Va.PictureParameterBufferH264" c:type="VAPictureP/' \
	-e 's/name="gpointer" c:type="VAPictureH/name="Va.PictureH264" c:type="VAPictureH/' \
	-e 's/name="gpointer" c:type="VASli/name="Va.SliceParameterBufferH264" c:type="VASli/' \
	$@

gva-h264-0.1.h: interface/GVaH264-0.1.gir

if DX_COND_xml
if HAVE_SPHINX

@DX_RULES@

clean-local:
	rm -rf doc-out

.PHONY: sphinx-doc
sphinx-doc: doxygen-run
	$(SPHINX_BUILD) -c doc $(top_srcdir)/doc doc-out

endif HAVE_SPHINX
endif DX_COND_xml
