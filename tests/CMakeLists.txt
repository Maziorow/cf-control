cmake_minimum_required(VERSION 3.8)
project(flatness_tests CXX)

# ── Compiler settings ─────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ─────────────────────────────────────────────────────────
find_package(GTest REQUIRED)

# ── Test executable ───────────────────────────────────────────────────────
add_executable(test_flatness test_flatness.cpp)

target_include_directories(test_flatness PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../cf_control/include)

# Inject the absolute path to test_sets/ so the test can open the CSV
# regardless of the working directory it is run from.
target_compile_definitions(test_flatness PRIVATE
  TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test_sets")

target_link_libraries(test_flatness
  GTest::gtest
  GTest::gtest_main)

# ── CTest integration ─────────────────────────────────────────────────────
include(GoogleTest)
gtest_discover_tests(test_flatness)
