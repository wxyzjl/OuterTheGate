cmake_minimum_required(VERSION 2.8.3)
project(rplidar_ros)

set(RPLIDAR_SDK_PATH "./sdk/")

FILE(GLOB RPLIDAR_SDK_SRC 
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  sensor_msgs
  message_generation
)

include_directories(
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
  ${catkin_INCLUDE_DIRS}
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  safety_ori.msg
  min_dist.msg
  ave_dist.msg
)

# # Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   AddTwoInts.srv
# )//大胆预测 不需要使用srv

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

add_executable(rplidarNode src/node.cpp ${RPLIDAR_SDK_SRC})
target_link_libraries(rplidarNode ${catkin_LIBRARIES})

add_executable(rplidarNodeClient src/client.cpp)
target_link_libraries(rplidarNodeClient ${catkin_LIBRARIES})

add_executable(saferep src/reporter.cpp)
target_link_libraries(saferep ${catkin_LIBRARIES})

add_executable(rep_in_main src/rep_inmain.cpp)
target_link_libraries(rep_in_main ${catkin_LIBRARIES})

add_executable(listener4safetyori src/test_msg_lis.cpp)
target_link_libraries(listener4safetyori ${catkin_LIBRARIES})

add_executable(dispatch4msg src/test_msg_lis.cpp)
target_link_libraries(dispatch4msg ${catkin_LIBRARIES})

add_executable(srge2 src/reporterge2.cpp)
target_link_libraries(srge2 ${catkin_LIBRARIES})

add_executable(srge3 src/repge3.cpp)
target_link_libraries(srge3 ${catkin_LIBRARIES})

add_executable(ave src/ave.cpp)
target_link_libraries(ave ${catkin_LIBRARIES})


install(TARGETS rplidarNode rplidarNodeClient
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz sdk
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
