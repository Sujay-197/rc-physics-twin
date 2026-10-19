# rc_twin: shared library exposing the Core through a C ABI (include/rc_twin.h).

add_library(rc_twin SHARED src/rc_twin.cpp src/rc_twin_road.cpp)
target_include_directories(rc_twin PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_definitions(rc_twin PRIVATE RC_TWIN_BUILDING)
target_link_libraries(rc_twin PRIVATE slipx_core)
slipx_apply_determinism_flags(rc_twin)
set_target_properties(rc_twin PROPERTIES
  PREFIX ""                      # rc_twin.dll, not librc_twin.dll
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON
)
# Everything lands in bin/, so tests find rc_twin.dll next to them.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set_target_properties(rc_twin PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
if(MINGW OR CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(rc_twin PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
endif()

install(TARGETS rc_twin RUNTIME DESTINATION bin LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
install(FILES include/rc_twin.h DESTINATION include)

if(BUILD_TESTING)
  # Plain C, through the DLL only: proves the header is C and the exports work.
  add_executable(test_capi tests/test_capi.c)
  target_link_libraries(test_capi PRIVATE rc_twin)
  set_target_properties(test_capi PROPERTIES C_STANDARD 99)
  add_test(NAME rc_twin.capi COMMAND test_capi)

  add_executable(test_ground_capi tests/test_ground_capi.c)
  target_link_libraries(test_ground_capi PRIVATE rc_twin)
  set_target_properties(test_ground_capi PROPERTIES C_STANDARD 99)
  add_test(NAME rc_twin.ground COMMAND test_ground_capi)

  add_executable(test_road tests/test_road.c)
  target_link_libraries(test_road PRIVATE rc_twin)
  set_target_properties(test_road PROPERTIES C_STANDARD 99)
  add_test(NAME rc_twin.road COMMAND test_road)

  # The headless harness (tools/drive_road.py), through the same DLL.
  find_package(Python3 COMPONENTS Interpreter)
  if(Python3_Interpreter_FOUND)
    add_test(NAME tools.drive_road
             COMMAND ${Python3_EXECUTABLE} -I ${PROJECT_SOURCE_DIR}/tools/test_drive_road.py)
    add_test(NAME tools.edge_profiler
             COMMAND ${Python3_EXECUTABLE} -I ${PROJECT_SOURCE_DIR}/tools/test_edge_profiler.py)
    add_test(NAME sim_py.env
             COMMAND ${Python3_EXECUTABLE} -I ${PROJECT_SOURCE_DIR}/sim_py/test_rc_twin_env.py)
    add_test(NAME sim_py.campus
             COMMAND ${Python3_EXECUTABLE} -I ${PROJECT_SOURCE_DIR}/sim_py/test_campus.py)
    set_tests_properties(tools.drive_road tools.edge_profiler sim_py.env sim_py.campus PROPERTIES
      ENVIRONMENT "RC_TWIN_LIB=$<TARGET_FILE:rc_twin>")
  endif()

  # C ABI against the Core called directly, bit for bit.
  add_executable(test_equivalence tests/test_equivalence.cpp)
  target_link_libraries(test_equivalence PRIVATE rc_twin slipx_core)
  slipx_apply_determinism_flags(test_equivalence)
  add_test(NAME rc_twin.equivalence COMMAND test_equivalence)

endif()

# Copy the plugin into the Unity project: cmake --build build --target unity_plugin
# Close the Unity editor first: it keeps native plugins loaded until it exits.
set(RC_TWIN_UNITY_PLUGIN_DIR
    "${PROJECT_SOURCE_DIR}/unity/RcTwin/Assets/Plugins/x86_64" CACHE PATH
    "Where unity_plugin copies rc_twin.dll")
add_custom_target(unity_plugin
  COMMAND ${CMAKE_COMMAND} -E make_directory "${RC_TWIN_UNITY_PLUGIN_DIR}"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:rc_twin> "${RC_TWIN_UNITY_PLUGIN_DIR}"
  DEPENDS rc_twin
  COMMENT "Copying rc_twin.dll to ${RC_TWIN_UNITY_PLUGIN_DIR}")
