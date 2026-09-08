cmake_minimum_required(VERSION 3.16)

project(physx_pack_visibility LANGUAGES CXX)

include(CTest)

find_package(ZLIB REQUIRED)

add_library(physx_pack STATIC
  src/pack_reader.cpp
  src/geometry_reader.cpp
  src/ray_query.cpp
  examples/visibility_query.cpp
)
target_compile_features(physx_pack PUBLIC cxx_std_17)
target_include_directories(physx_pack
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/examples
)
target_link_libraries(physx_pack PUBLIC ZLIB::ZLIB)

add_executable(physx_visibility_example
  examples/visibility_query_main.cpp
)
target_compile_features(physx_visibility_example PRIVATE cxx_std_17)
target_link_libraries(physx_visibility_example PRIVATE physx_pack)

if(BUILD_TESTING)
  add_executable(physx_visibility_smoke tests/visibility_query_smoke.cpp)
  target_compile_features(physx_visibility_smoke PRIVATE cxx_std_17)
  target_link_libraries(physx_visibility_smoke PRIVATE physx_pack)
  add_test(NAME physx_visibility_smoke COMMAND physx_visibility_smoke)
endif()
