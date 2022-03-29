cmake_minimum_required(VERSION 3.0.2)
project(savepcd)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_conversions
  pcl_msgs
  pcl_ros
)
find_package(libnabo REQUIRED)
# g2o
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})
# sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
# ceres
find_package(Ceres REQUIRED)
include_directories(${Ceres_INCLUDE_DIRS})
#find_package(fmt REQUIRED)
#include_directories(${fmt_INCLUDE_DIRS})
#PCL
find_package(PCL REQUIRED)
#NICP

#
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES savepcd
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/savepcd.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/savepcd_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
add_executable(savepcd src/savepcd.cpp)
target_link_libraries(savepcd ${catkin_LIBRARIES})

add_executable(writePCD src/writePCD.cpp)
target_link_libraries(writePCD ${catkin_LIBRARIES})

add_executable(savepcd2 src/savepcd2.cpp)
target_link_libraries(savepcd2 ${catkin_LIBRARIES})

add_executable(readpcd src/readpcd.cpp)
target_link_libraries(readpcd ${catkin_LIBRARIES} libnabo::nabo)

add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES} libnabo::nabo)

add_executable(readCartTraj src/readCartTraj.cpp)
target_link_libraries(readCartTraj ${catkin_LIBRARIES} )

add_executable(SRINormal src/SRINormal.cpp)
target_link_libraries(SRINormal ${catkin_LIBRARIES} libnabo::nabo ${Sophus_LIBRARIES})

add_executable(newicp src/newicp.cpp)
target_link_libraries(newicp
        ${catkin_LIBRARIES}
        libnabo::nabo
        ${CHOLMOD_LIBRARIES}
        ${Sophus_LIBRARIES}
        g2o_core g2o_stuff
        g2o_types_slam3d
        g2o_types_slam2d
        )

add_executable(pose_graph_g2o_SE3 src/pose_graph_g2o_SE3.cpp)
target_link_libraries(pose_graph_g2o_SE3
        ${catkin_LIBRARIES}
        ${CHOLMOD_LIBRARIES}
        ${catkin_LIBRARIES}
        g2o_core g2o_stuff
        g2o_types_slam3d
        g2o_types_slam2d
        )

add_executable(icpceres src/icpceres.cpp)
target_link_libraries(icpceres
        ${catkin_LIBRARIES}
        ${CERES_LIBRARIES}
        ${Sophus_LIBRARIES}
        libnabo::nabo
        )

add_executable(trans src/multiecho2laserscan.cpp)
target_link_libraries(trans ${catkin_LIBRARIES})

add_executable(test01 src/test01.cpp)
target_link_libraries(test01
        ${catkin_LIBRARIES}
        ${CERES_LIBRARIES}
        ${Sophus_LIBRARIES}
        libnabo::nabo
        ${PCL_LIBRARIES}
        #${NICP_LIBARAIES}
        #${NICP_VIEWER_LIBRARY}
        )

add_executable(hectortraj src/hectortraj.cpp)
target_link_libraries(hectortraj ${catkin_LIBRARIES})

add_executable(hector02odom src/hector02odom.cpp)
target_link_libraries(hector02odom ${catkin_LIBRARIES})

add_executable(readodom src/readodom.cpp)
target_link_libraries(readodom ${catkin_LIBRARIES})

add_executable(gridmap src/gridmap.cpp)
target_link_libraries(gridmap ${catkin_LIBRARIES})

add_executable(hearfromPLICP src/hearfromPLICP.cpp)
target_link_libraries(hearfromPLICP ${catkin_LIBRARIES})
#add_executable(laserProj src/laserProj.cpp)
#target_link_libraries(laserProj ${catkin_LIBRARIES})
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_savepcd.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
