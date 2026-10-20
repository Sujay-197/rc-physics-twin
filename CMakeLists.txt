cmake_minimum_required(VERSION 3.20)

# rc-physics-twin native build: the vendored SlipX Core (core/) plus the
# rc_twin C ABI (native/) that Unity and the headless harness load.
project(rc_physics_twin VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

# SlipX's determinism rules apply to everything that compiles Core headers.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/core/cmake")
include(SlipxDeterminism)
slipx_assert_determinism_flags()

include(CTest)

# Only the Core library; SlipX's own tests and upper layers build from core/
# itself (see Docs/slipx-windows-build.md).
set(SLIPX_BUILD_TESTS OFF)
add_subdirectory(core/src/core/slipx_core)

add_subdirectory(native)
