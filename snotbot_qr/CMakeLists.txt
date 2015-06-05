cmake_minimum_required(VERSION 2.8.3)
project(snotbot_qr)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  roscopter
  roslib
  std_msgs
  message_generation
)

# find_package(Boost REQUIRED COMPONENTS system)
FIND_PACKAGE(OpenCV)


# catkin_python_setup()

add_message_files(
  FILES
)

# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES snotbot_qr
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# add_library(snotbot_qr
#   src/${PROJECT_NAME}/snotbot_qr.cpp
# )

# add_executable(snotbot_qr_node src/snotbot_qr_node.cpp)

# add_dependencies(snotbot_qr_node snotbot_qr_generate_messages_cpp)

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(TARGETS snotbot_qr snotbot_qr_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# catkin_add_gtest(${PROJECT_NAME}-test test/test_snotbot_qr.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# catkin_add_nosetests(test)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(qr_vision src/video.cpp)
target_link_libraries(qr_vision ${catkin_LIBRARIES})
target_link_libraries(qr_vision ${OpenCV_LIBRARIES})
add_dependencies(qr_vision snotbot_qr_generate_messages_cpp)
add_dependencies(qr_vision $(catkin_EXPORTED_TARGETS))	