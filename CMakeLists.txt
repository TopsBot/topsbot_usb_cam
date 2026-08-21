cmake_minimum_required(VERSION 3.5)
project(topsbot_usb_cam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# colcon 未传 CMAKE_BUILD_TYPE 时默认为 Release，避免 MJPEG 解码等热路径以 -O0 编译
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type (Release, Debug, RelWithDebInfo)")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-Og -g)
  else()
    add_compile_options(-O3 -DNDEBUG)
  endif()
endif()

message(STATUS "topsbot_usb_cam: CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)
find_package(tb_img_msgs REQUIRED)

if(CMAKE_SYSROOT)
  set(TOPSBOT_TACV_ROOT "${CMAKE_SYSROOT}/usr/local")
else()
  set(TOPSBOT_TACV_ROOT "/usr/local")
endif()
set(TOPSBOT_TACV_INCLUDE_DIR "${TOPSBOT_TACV_ROOT}/include")
set(TOPSBOT_TACV_LIB_DIR "${TOPSBOT_TACV_ROOT}/lib")

find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec REQUIRED libavcodec)
pkg_check_modules(avutil REQUIRED libavutil)
pkg_check_modules(swscale REQUIRED libswscale)

add_library(${PROJECT_NAME}_lib SHARED
  src/topsbot_usb_cam.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  include
  ${OpenCV_INCLUDE_DIRS}
  ${TOPSBOT_TACV_INCLUDE_DIR}
)

target_link_directories(${PROJECT_NAME}_lib PUBLIC ${TOPSBOT_TACV_LIB_DIR})

target_link_libraries(${PROJECT_NAME}_lib
  ${avcodec_LIBRARIES}
  ${avutil_LIBRARIES}
  ${swscale_LIBRARIES}
  ${OpenCV_LIBRARIES}
  dmabufheap
)

ament_export_libraries(${PROJECT_NAME}_lib)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/topsbot_usb_cam_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "topsbot_usb_cam::TopsbotUsbCamNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

# 测试工具：文件回放发布器 + 轻量消费者（吞吐压力测试）
ament_auto_add_executable(${PROJECT_NAME}_jpeg_replay
  src/topsbot_jpeg_replay_node.cpp
)
target_link_libraries(${PROJECT_NAME}_jpeg_replay ${PROJECT_NAME}_lib dmabufheap)


# raw NV12 回放 + 轻量消费者

ament_auto_add_executable(${PROJECT_NAME}_img_sink
  src/topsbot_img_sink_node.cpp
)
target_link_libraries(${PROJECT_NAME}_img_sink ${PROJECT_NAME}_lib)

# R1: 纯解码转发节点（JPEG -> 硬件解码 -> M1 零拷贝转发）
ament_auto_add_executable(${PROJECT_NAME}_jpeg_to_img
  src/topsbot_jpeg_to_img_node.cpp
)
target_link_libraries(${PROJECT_NAME}_jpeg_to_img ${PROJECT_NAME}_lib dmabufheap tacocv tacosys)

add_executable(${PROJECT_NAME}
  src/topsbot_usb_cam_exec.cpp
)

target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_node)
ament_target_dependencies(${PROJECT_NAME} rclcpp)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
    launch
    config
)

install(FILES QUICKSTART.md DESTINATION share/${PROJECT_NAME})
