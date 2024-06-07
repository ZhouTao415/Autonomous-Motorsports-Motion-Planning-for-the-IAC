cmake_minimum_required(VERSION 3.2)

project(BLAS Fortran)

set(BLAS_MAJOR_VERSION 3)
set(BLAS_MINOR_VERSION 12)
set(BLAS_PATCH_VERSION 0)
set(
  BLAS_VERSION
  ${BLAS_MAJOR_VERSION}.${BLAS_MINOR_VERSION}.${BLAS_PATCH_VERSION}
  )

# Add the CMake directory for custom CMake modules
set(CMAKE_MODULE_PATH "${BLAS_SOURCE_DIR}/CMAKE" ${CMAKE_MODULE_PATH})

# Export all symbols on Windows when building shared libraries
SET(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

# Set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo" "Coverage")
endif()

# Coverage
set(_is_coverage_build 0)
set(_msg "Checking if build type is 'Coverage'")
message(STATUS "${_msg}")
if(NOT CMAKE_CONFIGURATION_TYPES)
  string(TOLOWER ${CMAKE_BUILD_TYPE} _build_type_lc)
  if(${_build_type_lc} STREQUAL "coverage")
    set(_is_coverage_build 1)
  endif()
endif()
message(STATUS "${_msg}: ${_is_coverage_build}")

if(_is_coverage_build)
  message(STATUS "Adding coverage")
  find_package(codecov)
endif()

# By default static library
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

# By default build index32 library
option(BUILD_INDEX64 "Build Index-64 API libraries" OFF)
if(BUILD_INDEX64)
  set(BLASLIB "blas64")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DWeirdNEC -DLAPACK_ILP64 -DHAVE_LAPACK_CONFIG_H")
  set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -fdefault-integer-8")
else()
  set(BLASLIB "blas")
endif()

# --------------------------------------------------
set(BLAS_INSTALL_EXPORT_NAME ${BLASLIB}-targets)

macro(blas_install_library lib)
  install(TARGETS ${lib}
    EXPORT ${BLAS_INSTALL_EXPORT_NAME}
    ARCHIVE DESTINATION lib COMPONENT Development
    LIBRARY DESTINATION lib COMPONENT RuntimeLibraries
    RUNTIME DESTINATION lib COMPONENT RuntimeLibraries
  )
endmacro()

set(PKG_CONFIG_DIR lib/pkgconfig)

# --------------------------------------------------
# Precision to build
# By default all precisions are generated
option(BUILD_SINGLE "Build single precision real" ON)
option(BUILD_DOUBLE "Build double precision real" ON)
option(BUILD_COMPLEX "Build single precision complex" ON)
option(BUILD_COMPLEX16 "Build double precision complex" ON)
message(STATUS "Build single precision real: ${BUILD_SINGLE}")
message(STATUS "Build double precision real: ${BUILD_DOUBLE}")
message(STATUS "Build single precision complex: ${BUILD_COMPLEX}")
message(STATUS "Build double precision complex: ${BUILD_COMPLEX16}")

if(NOT (BUILD_SINGLE OR BUILD_DOUBLE OR BUILD_COMPLEX OR BUILD_COMPLEX16))
  message(FATAL_ERROR "Nothing to build, no precision selected.
  Please enable at least one of these:
  BUILD_SINGLE, BUILD_DOUBLE, BUILD_COMPLEX, BUILD_COMPLEX16.")
endif()

include(GNUInstallDirs)

# Updated OSX RPATH settings
# In response to CMake 3.0 generating warnings regarding policy CMP0042,
# the OSX RPATH settings have been updated per recommendations found
# in the CMake Wiki:
#  http://www.cmake.org/Wiki/CMake_RPATH_handling#Mac_OS_X_and_the_RPATH
set(CMAKE_MACOSX_RPATH ON)
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES ${CMAKE_INSTALL_FULL_LIBDIR} isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
  set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_FULL_LIBDIR})
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
endif()


include(PreventInSourceBuilds)
include(PreventInBuildInstalls)

# --------------------------------------------------
# Check for any necessary platform specific compiler flags
include(CheckLAPACKCompilerFlags)
CheckLAPACKCompilerFlags("-recursive" _recursiveFlag)
CheckLAPACKCompilerFlags("-frecursive" _frecursiveFlag)
CheckLAPACKCompilerFlags("-Mrecursive" _MrecursiveFlag)

# Add recursive flag
if(_recursiveFlag)
  string(REGEX MATCH "-recursive" output_test <string> "${CMAKE_Fortran_FLAGS}")
  if(NOT output_test)
    set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -recursive"
      CACHE STRING "Recursive flag must be set" FORCE)
  endif()
elseif(_frecursiveFlag)
  string(REGEX MATCH "-frecursive" output_test <string> "${CMAKE_Fortran_FLAGS}")
  if(NOT output_test)
    set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -frecursive"
    CACHE STRING "Recursive flag must be set" FORCE)
  endif()
elseif(_MrecursiveFlag)
  string(REGEX MATCH "-Mrecursive" output_test <string> "${CMAKE_Fortran_FLAGS}")
  if(NOT output_test)
    set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -Mrecursive"
    CACHE STRING "Recursive flag must be set" FORCE)
  endif()
endif()


set(BLAS_LIBRARIES ${BLASLIB})

# --------------------------------------------------
# Testing
option(BUILD_TESTING "Build tests" ${_is_coverage_build})
include(CTest)
message(STATUS "Build tests: ${BUILD_TESTING}")

#######################################################################
#  This is the makefile to create a library for the BLAS.
#  The files are grouped as follows:
#
#       SBLAS1 -- Single precision real BLAS routines
#       CBLAS1 -- Single precision complex BLAS routines
#       DBLAS1 -- Double precision real BLAS routines
#       ZBLAS1 -- Double precision complex BLAS routines
#
#       CB1AUX -- Real BLAS routines called by complex routines
#       ZB1AUX -- D.P. real BLAS routines called by d.p. complex
#                 routines
#
#      ALLBLAS -- Auxiliary routines for Level 2 and 3 BLAS
#
#       SBLAS2 -- Single precision real BLAS2 routines
#       CBLAS2 -- Single precision complex BLAS2 routines
#       DBLAS2 -- Double precision real BLAS2 routines
#       ZBLAS2 -- Double precision complex BLAS2 routines
#
#       SBLAS3 -- Single precision real BLAS3 routines
#       CBLAS3 -- Single precision complex BLAS3 routines
#       DBLAS3 -- Double precision real BLAS3 routines
#       ZBLAS3 -- Double precision complex BLAS3 routines
#
#######################################################################

#---------------------------------------------------------
#  Level 1 BLAS
#---------------------------------------------------------

set(SBLAS1 isamax.f sasum.f saxpy.f scopy.f sdot.f snrm2.f90
	srot.f srotg.f90 sscal.f sswap.f sdsdot.f srotmg.f srotm.f)

set(CBLAS1 scabs1.f scasum.f scnrm2.f90 icamax.f caxpy.f ccopy.f
	cdotc.f cdotu.f csscal.f crotg.f90 cscal.f cswap.f csrot.f)

set(DBLAS1 idamax.f dasum.f daxpy.f dcopy.f ddot.f dnrm2.f90
	drot.f drotg.f90 dscal.f dsdot.f dswap.f drotmg.f drotm.f)

set(ZBLAS1 dcabs1.f dzasum.f dznrm2.f90 izamax.f zaxpy.f zcopy.f
	zdotc.f zdotu.f zdscal.f zrotg.f90 zscal.f zswap.f zdrot.f)

set(CB1AUX isamax.f sasum.f saxpy.f scopy.f snrm2.f90 sscal.f)

set(ZB1AUX idamax.f dasum.f daxpy.f dcopy.f dnrm2.f90 dscal.f)

#---------------------------------------------------------------------
#  Auxiliary routines needed by both the Level 2 and Level 3 BLAS
#---------------------------------------------------------------------
set(ALLBLAS lsame.f xerbla.f xerbla_array.f)

#---------------------------------------------------------
#  Level 2 BLAS
#---------------------------------------------------------
set(SBLAS2 sgemv.f sgbmv.f ssymv.f ssbmv.f sspmv.f
	strmv.f stbmv.f stpmv.f strsv.f stbsv.f stpsv.f
	sger.f ssyr.f sspr.f ssyr2.f sspr2.f)

set(CBLAS2 cgemv.f cgbmv.f chemv.f chbmv.f chpmv.f
	ctrmv.f ctbmv.f ctpmv.f ctrsv.f ctbsv.f ctpsv.f
	cgerc.f cgeru.f cher.f chpr.f cher2.f chpr2.f)

set(DBLAS2 dgemv.f dgbmv.f dsymv.f dsbmv.f dspmv.f
	dtrmv.f dtbmv.f dtpmv.f dtrsv.f dtbsv.f dtpsv.f
	dger.f dsyr.f dspr.f dsyr2.f dspr2.f)

set(ZBLAS2 zgemv.f zgbmv.f zhemv.f zhbmv.f zhpmv.f
	ztrmv.f ztbmv.f ztpmv.f ztrsv.f ztbsv.f ztpsv.f
	zgerc.f zgeru.f zher.f zhpr.f zher2.f zhpr2.f)

#---------------------------------------------------------
#  Level 3 BLAS
#---------------------------------------------------------
set(SBLAS3 sgemm.f ssymm.f ssyrk.f ssyr2k.f strmm.f strsm.f)

set(CBLAS3 cgemm.f csymm.f csyrk.f csyr2k.f ctrmm.f ctrsm.f
	chemm.f cherk.f cher2k.f)

set(DBLAS3 dgemm.f dsymm.f dsyrk.f dsyr2k.f dtrmm.f dtrsm.f)

set(ZBLAS3 zgemm.f zsymm.f zsyrk.f zsyr2k.f ztrmm.f ztrsm.f
	zhemm.f zherk.f zher2k.f)


set(SOURCES)
if(BUILD_SINGLE)
  list(APPEND SOURCES ${SBLAS1} ${ALLBLAS} ${SBLAS2} ${SBLAS3})
endif()
if(BUILD_DOUBLE)
  list(APPEND SOURCES ${DBLAS1} ${ALLBLAS} ${DBLAS2} ${DBLAS3})
endif()
if(BUILD_COMPLEX)
  list(APPEND SOURCES ${CBLAS1} ${CB1AUX} ${ALLBLAS} ${CBLAS2} ${CBLAS3})
endif()
if(BUILD_COMPLEX16)
  list(APPEND SOURCES ${ZBLAS1} ${ZB1AUX} ${ALLBLAS} ${ZBLAS2} ${ZBLAS3})
endif()
list(REMOVE_DUPLICATES SOURCES)

add_library(${BLASLIB} ${SOURCES})
set_target_properties(
  ${BLASLIB} PROPERTIES
  VERSION ${BLAS_VERSION}
  SOVERSION ${BLAS_MAJOR_VERSION}
  )

blas_install_library(${BLASLIB})


if(BUILD_TESTING)
  add_subdirectory(TESTING)
endif()
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/blas.pc.in ${CMAKE_CURRENT_BINARY_DIR}/${BLASLIB}.pc @ONLY)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${BLASLIB}.pc
  DESTINATION ${PKG_CONFIG_DIR}
  COMPONENT Development
  )
