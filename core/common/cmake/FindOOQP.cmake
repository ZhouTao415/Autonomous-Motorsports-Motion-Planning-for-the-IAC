# Try to find the include directory
find_path(OOQP_INCLUDE_DIR
  NAMES QpGenData.h
  HINTS ${PROJECT_SOURCE_DIR}/thirdparty/ooqp/include/ooqp
)

if(OOQP_INCLUDE_DIR)
  set(OOQP_FOUND_INCLUDE TRUE)
  set(OOQP_INCLUDE_DIRS ${OOQP_INCLUDE_DIR})
  message(STATUS "Found OOQP include dirs: ${OOQP_INCLUDE_DIRS}")
else()
  message(STATUS "Could not find OOQP include dir")
endif()

# Try to find the libraries
set(OOQP_LIBS_LIST
  ooqpgensparse ooqpsparse ooqpgondzio ooqpbase ma27
)

set(OOQP_LIBRARIES "")
set(OOQP_FOUND_LIBS TRUE)
foreach(LIB ${OOQP_LIBS_LIST})
  find_library(OOQP_LIB_${LIB}
    NAMES ${LIB}
    HINTS ${PROJECT_SOURCE_DIR}/thirdparty/ooqp/lib
  )
  if(OOQP_LIB_${LIB})
    list(APPEND OOQP_LIBRARIES ${OOQP_LIB_${LIB}})
  else()
    set(OOQP_FOUND_LIBS FALSE)
    message(STATUS "Could not find OOQP library: ${LIB}")
  endif()
endforeach()

list(APPEND OOQP_LIBRARIES blas gfortran)

# Print OOQP_LIBRARIES
if(OOQP_FOUND_LIBS)
  message(STATUS "Found OOQP libraries: ${OOQP_LIBRARIES}")
else()
  message(STATUS "Could not find all OOQP libraries")
endif()

# Success if both the libraries and the include directories were found
if(OOQP_FOUND_INCLUDE AND OOQP_FOUND_LIBS)
  set(OOQP_FOUND TRUE)
  message(STATUS "Found OOQP")
else()
  set(OOQP_FOUND FALSE)
  message(STATUS "Could not find OOQP")
endif()

# Set package variables
if(OOQP_FOUND)
  set(OOQP_INCLUDE_DIRS ${OOQP_INCLUDE_DIR})
  set(OOQP_LIBRARIES ${OOQP_LIBRARIES})
endif()
