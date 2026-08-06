cmake_minimum_required(VERSION 3.10)
project(patrol_car LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(ICAR_BUILD_LOGIC_TESTS "Build hardware-independent control tests" OFF)
option(ICAR_LOGIC_TESTS_ONLY "Build only hardware-independent control tests" OFF)

function(icar_target_defaults target_name)
  target_compile_features(${target_name} PRIVATE cxx_std_17)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${target_name} PRIVATE
      -Wall
      -Wextra
      -Wpedantic
      $<$<CONFIG:Release>:-O3>)
    if(NOT ICAR_LOGIC_TESTS_ONLY)
      target_compile_options(${target_name} PRIVATE $<$<CONFIG:Release>:-flto>)
      # target_link_options requires CMake 3.13, and passing -flto through
      # target_link_libraries would mix keyword/plain signatures. Use the
      # old-style LINK_FLAGS_RELEASE property (CMake 3.10 compatible).
      set_property(TARGET ${target_name} APPEND_STRING
        PROPERTY LINK_FLAGS_RELEASE " -flto")
    endif()
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64)")
      target_compile_options(${target_name} PRIVATE -mcpu=native)
    endif()
  endif()
endfunction()

if(ICAR_BUILD_LOGIC_TESTS OR ICAR_LOGIC_TESTS_ONLY)
  enable_testing()
  if(ICAR_LOGIC_TESTS_ONLY)
    message(STATUS "Logic tests: hardware-free build using OpenCV stubs")
  else()
    find_package(OpenCV REQUIRED)
    message(STATUS "Logic tests: building against real OpenCV")
  endif()
  add_executable(control_algorithms_test tests/control_algorithms_test.cpp)
  target_include_directories(control_algorithms_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  icar_target_defaults(control_algorithms_test)
  add_test(NAME control_algorithms_test COMMAND control_algorithms_test)

  # Production FSM/control sources, compiled hardware-free so that logic
  # tests catch real compile errors in park.cpp / obstacle.cpp / center.cpp /
  # yfork.cpp instead of only testing the small helpers.
  add_library(icar_logic_core STATIC
    src/config/config_loader.cpp
    src/ctrl/center.cpp
    src/ctrl/control_geometry.cpp
    src/ctrl/control_geometry_validator.cpp
    src/ctrl/perception_geometry_builder.cpp
    src/ctrl/planned_geometry_builder.cpp
    src/ctrl/track.cpp
    src/fsm/obstacle.cpp
    src/fsm/park.cpp
    src/fsm/yfork.cpp
    src/fsm/park/park_observation.cpp
    src/fsm/park/park_path_planner.cpp
    src/fsm/park/park_debug_renderer.cpp)
  target_include_directories(icar_logic_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  if(ICAR_LOGIC_TESTS_ONLY)
    # OpenCV stub headers replace the real library for hardware-free builds.
    target_include_directories(icar_logic_core PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/stubs)
  else()
    # Real-OpenCV logic tests must link the same OpenCV libraries the main
    # program uses (putText/circle/rectangle/imwrite/Mat live in libopencv_*).
    target_include_directories(icar_logic_core PUBLIC ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(icar_logic_core PUBLIC ${OpenCV_LIBS})
  endif()
  icar_target_defaults(icar_logic_core)

  add_executable(production_logic_compile_test
    tests/production_logic_compile_test.cpp)
  target_include_directories(production_logic_compile_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/tests)
  target_compile_definitions(production_logic_compile_test PRIVATE
    ICAR_TEST_RES_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/res/config.json")
  target_link_libraries(production_logic_compile_test PRIVATE icar_logic_core)
  icar_target_defaults(production_logic_compile_test)
  add_test(NAME production_logic_compile_test
    COMMAND production_logic_compile_test)

  add_executable(fsm_behavior_test tests/fsm_behavior_test.cpp)
  target_include_directories(fsm_behavior_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/tests)
  target_compile_definitions(fsm_behavior_test PRIVATE
    ICAR_TEST_RES_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/res/config.json")
  target_link_libraries(fsm_behavior_test PRIVATE icar_logic_core)
  icar_target_defaults(fsm_behavior_test)
  add_test(NAME fsm_behavior_test COMMAND fsm_behavior_test)

  add_executable(geometry_behavior_test tests/geometry_behavior_test.cpp)
  target_include_directories(geometry_behavior_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/tests)
  target_compile_definitions(geometry_behavior_test PRIVATE
    ICAR_TEST_RES_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/res/config.json")
  target_link_libraries(geometry_behavior_test PRIVATE icar_logic_core)
  icar_target_defaults(geometry_behavior_test)
  add_test(NAME geometry_behavior_test COMMAND geometry_behavior_test)

  add_executable(park_chain_behavior_test
    tests/park_chain_behavior_test.cpp)
  target_include_directories(park_chain_behavior_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/tests)
  target_compile_definitions(park_chain_behavior_test PRIVATE
    ICAR_TEST_RES_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/res/config.json")
  target_link_libraries(park_chain_behavior_test PRIVATE icar_logic_core)
  icar_target_defaults(park_chain_behavior_test)
  add_test(NAME park_chain_behavior_test COMMAND park_chain_behavior_test)

  add_executable(runtime_safety_chain_test
    tests/runtime_safety_chain_test.cpp)
  target_include_directories(runtime_safety_chain_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/tests)
  target_compile_definitions(runtime_safety_chain_test PRIVATE
    ICAR_TEST_RES_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/res/config.json")
  target_link_libraries(runtime_safety_chain_test PRIVATE icar_logic_core)
  icar_target_defaults(runtime_safety_chain_test)
  add_test(NAME runtime_safety_chain_test
    COMMAND runtime_safety_chain_test)

  add_executable(track_behavior_test tests/track_behavior_test.cpp)
  target_include_directories(track_behavior_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/tests)
  target_compile_definitions(track_behavior_test PRIVATE
    ICAR_TEST_RES_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/res/config.json")
  target_link_libraries(track_behavior_test PRIVATE icar_logic_core)
  icar_target_defaults(track_behavior_test)
  add_test(NAME track_behavior_test COMMAND track_behavior_test)

  add_executable(runtime_behavior_test tests/runtime_behavior_test.cpp
    src/config/config_loader.cpp src/fsm/park/park_observation.cpp
    src/fsm/park/park_path_planner.cpp src/ctrl/planned_geometry_builder.cpp)
  target_include_directories(runtime_behavior_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  icar_target_defaults(runtime_behavior_test)
  add_test(NAME runtime_behavior_test COMMAND runtime_behavior_test)
  add_executable(logic_check_failure_test tests/logic_check_failure_test.cpp)
  target_include_directories(logic_check_failure_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
  icar_target_defaults(logic_check_failure_test)
  add_test(NAME logic_check_failure_test COMMAND logic_check_failure_test)
  set_tests_properties(logic_check_failure_test PROPERTIES WILL_FAIL TRUE)
endif()
if(ICAR_LOGIC_TESTS_ONLY)
  return()
endif()
set(INCLUDE_PATH "/usr/local/include")
set(LIB_PATH "/usr/local/lib")
include_directories(include)
#---------------------------------------------------------------------
#       [ Include、Lib  : Define ] ==> []
#---------------------------------------------------------------------
link_directories(${COMMON_LIB_DIR})
include_directories( ${COMMON_INCLUDE_DIR} )
#---------------------------------------------------------------------
#       [ Include、Lib  : Path ] ==> [ glib opencv ]
#---------------------------------------------------------------------
find_package(PkgConfig)
pkg_search_module(GLIB REQUIRED glib-2.0)
include_directories(${GLIB_INCLUDE_DIRS})

find_package( OpenCV REQUIRED )    
include_directories( ${OpenCV_INCLUDE_DIRS} )

pkg_search_module(SERIAL REQUIRED libserial)
include_directories(${SERIAL_INCLUDE_DIRS})
link_directories(${SERIAL_LIBRARY_DIRS})

# find ppnc
pkg_search_module(PPNC REQUIRED ppnc)
include_directories(${PPNC_INCLUDE_DIRS})
link_directories(${PPNC_LIBRARY_DIRS})

# find onnx
pkg_search_module(ONNX REQUIRED onnx)
include_directories(${ONNX_INCLUDE_DIRS})
link_directories(${ONNX_LIBRARY_DIRS})
#---------------------------------------------------------------------
#               [ bin ] ==> [ demo ]
#---------------------------------------------------------------------
# 开机自启程序
add_executable(boot src/tool/boot.cpp)
icar_target_defaults(boot)
target_include_directories(boot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(boot pthread ${OpenCV_LIBS} ${SERIAL_LIBRARIES})

# 相机测试
add_executable(camera src/tool/camera.cpp src/ctrl/predeal.cpp)
icar_target_defaults(camera)
target_include_directories(camera PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(camera pthread ${OpenCV_LIBS})

# 图像合成
add_executable(img2video src/tool/img2video.cpp)
icar_target_defaults(img2video)
target_include_directories(img2video PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(img2video pthread ${OpenCV_LIBS})

# 相机矫正
add_executable(calibration src/tool/calibration.cpp)
icar_target_defaults(calibration)
target_include_directories(calibration PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(calibration pthread ${OpenCV_LIBS})

# 图像测试
add_executable(imging src/tool/imging.cpp src/ctrl/predeal.cpp)
icar_target_defaults(imging)
target_include_directories(imging PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(imging pthread ${OpenCV_LIBS})

# 目标检测
add_executable(detection src/tool/detection.cpp src/ctrl/predeal.cpp)
icar_target_defaults(detection)
target_include_directories(detection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(detection pthread ${PPNC_LIBRARIES} ${ONNX_LIBRARIES} ${OpenCV_LIBS})

# 图像采集
add_executable(collection src/tool/collection.cpp src/ctrl/predeal.cpp)
icar_target_defaults(collection)
target_include_directories(collection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(collection pthread ${OpenCV_LIBS} ${SERIAL_LIBRARIES})
#---------------------------------------------------------------------
#               [ bin ] ==> [ main ]
#---------------------------------------------------------------------
set(ICAR_SOURCES
  src/icar.cpp
  src/ctrl/center.cpp
  src/ctrl/control_geometry.cpp
  src/ctrl/control_geometry_validator.cpp
  src/ctrl/perception_geometry_builder.cpp
  src/ctrl/planned_geometry_builder.cpp
  src/ctrl/predeal.cpp
  src/ctrl/track.cpp
  src/fsm/busy.cpp
  src/fsm/cross.cpp
  src/fsm/fork.cpp
  src/fsm/manualControl.cpp
  src/fsm/obstacle.cpp
  src/fsm/park.cpp
  src/fsm/park/park_observation.cpp
  src/fsm/park/park_path_planner.cpp
  src/fsm/park/park_debug_renderer.cpp
  src/fsm/slow.cpp
  src/fsm/station.cpp
  src/fsm/stop.cpp
  src/fsm/yfork.cpp
  src/runtime/alerts.cpp
  src/runtime/ai_snapshot.cpp
  src/runtime/arbitration.cpp
  src/runtime/capture.cpp
  src/runtime/control.cpp
  src/runtime/frame_safety.cpp
  src/runtime/fsm.cpp
  src/runtime/icar.cpp
  src/runtime/inference.cpp
  src/runtime/latest_frame_capture.cpp
  src/runtime/preprocess.cpp
  src/runtime/running.cpp
  src/runtime/safety.cpp
  src/runtime/state_machines.cpp
  src/runtime/telemetry.cpp
  src/runtime/vehicle_output.cpp
  src/config/config_loader.cpp
)
add_executable(icar ${ICAR_SOURCES})
icar_target_defaults(icar)
target_include_directories(icar PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(icar pthread ${PPNC_LIBRARIES} ${ONNX_LIBRARIES} ${OpenCV_LIBS} ${SERIAL_LIBRARIES})

