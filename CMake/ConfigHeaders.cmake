# CMake/ConfigHeaders.cmake
#
# Auto-generate the per-library "config.h" headers that the vendored
# libsigrok4DSL / libsigrokdecode4DSL sub-libraries expect. Upstream builds
# these headers with autotools (configure), but when the libraries are
# vendored into this top-level CMake build that step is missing and the build
# fails with undeclared PACKAGE*/SRD_* macros.
#
# All version numbers are derived from the project-wide DS_VERSION_* variables
# (defined in the top-level CMakeLists.txt), so there is a single source of
# truth and nothing has to be kept in sync by hand.

#-------------------------------------------------------------------------------
# libsigrok4DSL
#
# Its output modules (csv.c, gnuplot.c, vcd.c) include the header with a
# relative path ("../config.h"), which the compiler resolves against the
# source tree and cannot be redirected through include directories. Therefore
# the header must be written into the source tree (this is also what autotools
# does upstream). It is covered by .gitignore.
#-------------------------------------------------------------------------------
set(PACKAGE "libsigrok4DSL")
set(PACKAGE_VERSION "${DS_VERSION_STRING}")
set(PACKAGE_STRING "${PACKAGE} ${DS_VERSION_STRING}")

configure_file(
	${PROJECT_SOURCE_DIR}/libsigrok4DSL/config.h.in
	${PROJECT_SOURCE_DIR}/libsigrok4DSL/config.h
)

#-------------------------------------------------------------------------------
# libsigrokdecode4DSL
#
# Its sources include the header with a plain "#include \"config.h\"" (no
# relative ".."), so it can be generated into the binary directory and exposed
# through an include directory. This keeps the source tree clean.
#-------------------------------------------------------------------------------
set(SRD_PACKAGE_TARNAME "libsigrokdecode4DSL")
set(SRD_PACKAGE_VERSION_MAJOR ${DS_VERSION_MAJOR})
set(SRD_PACKAGE_VERSION_MINOR ${DS_VERSION_MINOR})
set(SRD_PACKAGE_VERSION_MICRO ${DS_VERSION_MICRO})
set(SRD_PACKAGE_VERSION_STRING "${DS_VERSION_STRING}")
set(SRD_LIB_VERSION_CURRENT 1)
set(SRD_LIB_VERSION_REVISION 0)
set(SRD_LIB_VERSION_AGE 0)
set(SRD_LIB_VERSION_STRING "${SRD_LIB_VERSION_CURRENT}.${SRD_LIB_VERSION_REVISION}.${SRD_LIB_VERSION_AGE}")

configure_file(
	${PROJECT_SOURCE_DIR}/libsigrokdecode4DSL/config.h.in
	${PROJECT_BINARY_DIR}/libsigrokdecode4DSL/config.h
)
include_directories(${PROJECT_BINARY_DIR}/libsigrokdecode4DSL)
