# add library

#add_library(${PROJECT_NAME}_common names.cpp)

#target_include_directories(${PROJECT_NAME}_common PUBLIC
#  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
#  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
#  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
#)

#target_link_libraries(${PROJECT_NAME} PUBLIC
#  ${geometry_msgs_TARGETS}
#  ${sensor_msgs_TARGETS}
#target_link_libraries(${PROJECT_NAME} PRIVATE
#  rcpputils::rcpputils)

#ament_target_dependencies(${PROJECT_NAME}_common rclcpp )
#ament_export_dependencies(${PROJECT_NAME}_common rclcpp sensor_msgs geometry_msgs )


#add_executable(${PROJECT_NAME}_calibrator
#  calibrator.cpp
#  calibrator_main.cpp)
  
#rosidl_target_interfaces(${PROJECT_NAME}_calibrator "rosidl_typesupport_cpp")

#target_include_directories(${PROJECT_NAME}_calibrator PUBLIC
#  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
#  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
#  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
#  "/home/vagrant/ros2_ws/build/visp_camera_calibration/rosidl_generator_c/"
##  ${VISP_INCLUDE_DIRS}
#)

#target_link_libraries(${PROJECT_NAME}_calibrator
#  ${PROJECT_NAME}_common
#)  

#ament_target_dependencies(${PROJECT_NAME}_calibrator rclcpp std_msgs sensor_msgs geometry_msgs visp_bridge )

#add_executable(${PROJECT_NAME}_image_processing
#  image_processing.cpp
#  image_processing_main.cpp
#  )
  
#target_link_libraries(${PROJECT_NAME}_image_processing
#  ${PROJECT_NAME}_common
#  ${roscpp_LIBRARIES}
#  ${VISP_LIBRARIES}
#  )
#ament_target_dependencies(${PROJECT_NAME}_image_processing rclcpp std_msgs sensor_msgs geometry_msgs visp_bridge camera_calibration_parsers)

#add_executable(${PROJECT_NAME}_camera
#  camera.cpp
#  camera_main.cpp)
#target_link_libraries(${PROJECT_NAME}_camera
#  ${PROJECT_NAME}_common
#  ${VISP_LIBRARIES})
#ament_target_dependencies(${PROJECT_NAME}_camera rclcpp std_msgs sensor_msgs geometry_msgs visp_bridge camera_calibration_parsers)

#foreach(target IN LISTS common calibrator image_processing camera)
#foreach(target IN LISTS common)
#  install(TARGETS 
#    ${PROJECT_NAME}_target
#    EXPORT export_${PROJECT_NAME}
#    ARCHIVE DESTINATION lib
#    LIBRARY DESTINATION lib
#    RUNTIME DESTINATION bin
#)
#endforeach()

