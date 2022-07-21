cmake_minimum_required(VERSION 3.5)	
project(visp_camera_calibration)

find_package(ament_cmake_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Internal packages
find_package(VISP REQUIRED)
find_package(visp_bridge REQUIRED)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

add_subdirectory(src)


# messages and services interface generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CalibPointArray.msg"
  "msg/CalibPoint.msg"
  "msg/ImageAndPoints.msg"
  "msg/ImagePoint.msg"
  "srv/Calibrate.srv"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
 )
# TESTS
ament_export_dependencies(rosidl_default_runtime)
#ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

 
# END TESTS
 
# visp_camera_calibration_lib_dir is passed as APPEND_LIBRARY_DIRS for each ament_add_gtest call so
# the project library that they link against is on the library path.
# This is especially important on Windows.
# This is overwritten each loop, but which one it points to doesn't really matter.
#set(visp_camera_calibration_lib_dir "$<TARGET_FILE_DIR:${PROJECT_NAME}>")

# Mark cpp header files for installation
install(DIRECTORY include DESTINATION include/${PROJECT_NAME})

ament_package()
