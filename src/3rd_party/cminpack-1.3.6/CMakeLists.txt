# The name of our project is "CMINPACK". CMakeLists files in this project can
# refer to the root source directory of the project as ${CMINPACK_SOURCE_DIR} and
# to the root binary directory of the project as ${CMINPACK_BINARY_DIR}.
cmake_minimum_required (VERSION 3.1)
project (CMINPACK)

string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

set (CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include(${PROJECT_SOURCE_DIR}/cmake/cminpack_utils.cmake)
# Set version and OS-specific settings
set(CMINPACK_VERSION 1.3.6 CACHE STRING "CMinpack version")
set(CMINPACK_SOVERSION 1 CACHE STRING "CMinpack API version")
GET_OS_INFO()

# Add an "uninstall" target
#CONFIGURE_FILE ("${PROJECT_SOURCE_DIR}/cmake/uninstall_target.cmake.in"
#    "${PROJECT_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY)
#ADD_CUSTOM_TARGET (uninstall "${CMAKE_COMMAND}" -P
#    "${PROJECT_BINARY_DIR}/uninstall_target.cmake")

enable_testing()

option (BUILD_SHARED_LIBS "Build shared libraries instead of static." OFF)
if (BUILD_SHARED_LIBS)
  message (STATUS "Building shared libraries.")
else ()
  message (STATUS "Building static libraries.")
#  set(CMAKE_RELEASE_POSTFIX _s)
#  set(CMAKE_RELWITHDEBINFO_POSTFIX _s)
#  set(CMAKE_DEBUG_POSTFIX _s)
#  set(CMAKE_MINSIZEREL_POSTFIX _s)
  if(WIN32)
    add_definitions(-DCMINPACK_NO_DLL)
  endif(WIN32)
endif ()

option(USE_BLAS "Compile cminpack using cblas library if possible" OFF)

#set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/../build)

if(NOT "${CMAKE_PREFIX_PATH}" STREQUAL "")
  include_directories(${CMAKE_PREFIX_PATH}/include)
endif()

set (cminpack_srcs
  cminpack.h cminpackP.h
  chkder.c  enorm.c   hybrd1.c  hybrj.c   lmdif1.c  lmstr1.c  qrfac.c   r1updt.c
  dogleg.c  fdjac1.c  hybrd.c   lmder1.c  lmdif.c   lmstr.c   qrsolv.c  rwupdt.c
  dpmpar.c  fdjac2.c  hybrj1.c  lmder.c   lmpar.c   qform.c   r1mpyq.c  covar.c covar1.c
  minpack.h
  chkder_.c enorm_.c  hybrd1_.c hybrj_.c  lmdif1_.c lmstr1_.c qrfac_.c  r1updt_.c
  dogleg_.c fdjac1_.c hybrd_.c  lmder1_.c lmdif_.c  lmstr_.c  qrsolv_.c rwupdt_.c
  dpmpar_.c fdjac2_.c hybrj1_.c lmder_.c  lmpar_.c  qform_.c  r1mpyq_.c covar_.c
  )
set (cminpack_hdrs
    cminpack.h minpack.h)

add_library (3rd_cminpack ${cminpack_srcs})

if (${CMAKE_SYSTEM_NAME} STREQUAL "FreeBSD")
  TARGET_LINK_LIBRARIES(3rd_cminpack m)
endif()


include (CheckLibraryExists)
include (CheckFunctionExists)
check_function_exists (sqrt HAVE_SQRT_NO_LIBM)
if (NOT HAVE_SQRT_NO_LIBM)
  check_library_exists ("m" sqrt "" HAVE_LIBM)
  if (HAVE_LIBM)
    target_link_libraries(3rd_cminpack PUBLIC m)
  endif()
endif ()

# Link with CBLAS library if requested
if (USE_BLAS)
  find_package (CBLAS)
  if (CBLAS_FOUND)
    target_link_libraries(3rd_cminpack PUBLIC ${CBLAS_LIBRARIES})
    set_target_properties(3rd_cminpack PROPERTIES LINK_FLAGS "${CBLAS_LINKER_FLAGS}")
    target_compile_definitions(3rd_cminpack PUBLIC USE_CBLAS)
  endif()
endif()

set_target_properties (3rd_cminpack PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_subdirectory (cmake)
#add_subdirectory (examples)
