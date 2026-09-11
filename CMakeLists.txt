cmake_minimum_required(VERSION 3.16)
project(physx_pack VERSION 2.0.0 LANGUAGES CXX)

include(CTest)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)

option(PHYSX_PACK_BUILD_EXAMPLES "Build the public API example" ON)
option(PHYSX_PACK_BUILD_BENCHMARKS "Build the deterministic query benchmark" OFF)
set(PHYSX_PACK_SANITIZER "none" CACHE STRING "Runtime sanitizer: none, address, or thread")
set_property(CACHE PHYSX_PACK_SANITIZER PROPERTY STRINGS none address thread)

add_library(physx_pack STATIC
  src/pack_reader.cpp src/geometry_reader.cpp src/ray_query.cpp
  src/package.cpp src/scene.cpp src/visibility_query.cpp)
add_library(physx_pack::physx_pack ALIAS physx_pack)
target_compile_features(physx_pack PUBLIC cxx_std_17)
target_include_directories(physx_pack
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(physx_pack PUBLIC ZLIB::ZLIB Threads::Threads)

if(NOT PHYSX_PACK_SANITIZER STREQUAL "none")
  if(NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    message(FATAL_ERROR "Sanitizers require GCC or Clang")
  endif()
  if(PHYSX_PACK_SANITIZER STREQUAL "address")
    set(physx_sanitize_flags -fsanitize=address,undefined -fno-omit-frame-pointer)
  elseif(PHYSX_PACK_SANITIZER STREQUAL "thread")
    set(physx_sanitize_flags -fsanitize=thread -fno-omit-frame-pointer)
  else()
    message(FATAL_ERROR "Unknown PHYSX_PACK_SANITIZER: ${PHYSX_PACK_SANITIZER}")
  endif()
  target_compile_options(physx_pack PUBLIC ${physx_sanitize_flags})
  target_link_options(physx_pack PUBLIC ${physx_sanitize_flags})
endif()

if(PHYSX_PACK_BUILD_EXAMPLES)
  add_executable(physx_visibility_example examples/visibility_query_main.cpp)
  target_link_libraries(physx_visibility_example PRIVATE physx_pack::physx_pack)
endif()

if(BUILD_TESTING)
  function(physx_test name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE physx_pack::physx_pack)
    target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    add_test(NAME ${name} COMMAND ${name})
    set_tests_properties(${name} PROPERTIES TIMEOUT 120)
  endfunction()
  physx_test(physx_public_api tests/public_api_compile.cpp)
  physx_test(physx_visibility tests/visibility_query_test.cpp)
  physx_test(physx_geometry tests/geometry_reader_test.cpp)
  physx_test(physx_pack_reader tests/pack_reader_test.cpp)
  if(NOT CMAKE_CROSSCOMPILING)
    add_test(NAME physx_install_consumer COMMAND ${CMAKE_COMMAND}
      -DROOT_BUILD=${CMAKE_BINARY_DIR} -DROOT_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}
      -DTEST_CONFIG=$<CONFIG> -DTEST_COMPILER=${CMAKE_CXX_COMPILER}
      -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/install_consumer.cmake)
    set_tests_properties(physx_install_consumer PROPERTIES TIMEOUT 120)
  endif()
endif()

if(PHYSX_PACK_BUILD_BENCHMARKS)
  add_executable(physx_query_benchmark tests/query_benchmark.cpp)
  target_link_libraries(physx_query_benchmark PRIVATE physx_pack::physx_pack)
endif()

install(TARGETS physx_pack EXPORT physx_packTargets ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES include/physx_pack/visibility_query.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/physx_pack)
install(EXPORT physx_packTargets NAMESPACE physx_pack:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/physx_pack)
configure_package_config_file(cmake/physx_packConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/physx_packConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/physx_pack)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/physx_packConfigVersion.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/physx_packConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/physx_packConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/physx_pack)
