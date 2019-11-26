cmake_minimum_required(VERSION 2.8.3)
project(odom_proc)

#find_package(Eigen REQUIRED)
#include_directories(${Eigen_INCLUDE_DIRS})
find_package (Eigen3 REQUIRED NO_MODULE)
include_directories(${Eigen3_INCLUDE_DIRS})

find_package(catkin REQUIRED
    COMPONENTS
        nodelet
        roscpp
        sensor_msgs
        tf2
        tf2_client
        tf2_eigen
        tf2_ros
)
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS
        nodelet
        roscpp
        sensor_msgs
        tf2
        tf2_client
        tf2_eigen
        tf2_ros
    LIBRARIES ${PROJECT_NAME}_plugins
)
include_directories(include)

add_executable(odom_proc
    src/odom_proc/odom_proc.cpp
    src/odom_proc/odom_proc_node.cpp
)
target_link_libraries(odom_proc ${catkin_LIBRARIES})

add_executable(odom_to_tf
    src/odom_proc/odom_to_tf.cpp
    src/odom_proc/odom_to_tf_node.cpp
)
target_link_libraries(odom_to_tf ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_plugins
    src/odom_proc/odom_to_tf.cpp
    src/odom_proc/odom_to_tf_nodelet.cpp
)
target_link_libraries(${PROJECT_NAME}_plugins ${catkin_LIBRARIES})

install(
    TARGETS odom_to_tf
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
    TARGETS ${PROJECT_NAME}_plugins
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(
    FILES ${PROJECT_NAME}_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
