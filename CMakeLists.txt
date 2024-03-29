cmake_minimum_required(VERSION 3.10.2)
project(nav_utils)

find_package (Eigen3 REQUIRED NO_MODULE)
include_directories(${Eigen3_INCLUDE_DIRS})

set(CATKIN_DEPS
    geometry_msgs
    nav_msgs
    nodelet
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs
    tf2
    tf2_msgs
    tf2_client
    tf2_eigen
    tf2_ros
    topic_tools
)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPS} message_generation)
include_directories(${catkin_INCLUDE_DIRS})

add_message_files(FILES
    PoseInt32.msg
    PositionInt32.msg
    PositionPathInt16.msg
    PositionPathInt32.msg
    QuaternionInt16.msg
    Vector3Int16.msg
    Vector3Int32.msg
)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
    CATKIN_DEPENDS ${CATKIN_DEPS} message_runtime
    LIBRARIES ${PROJECT_NAME}_plugins
)
include_directories(include)

add_executable(odom_proc
    src/nav_utils/odom_proc.cpp
    src/nav_utils/odom_proc_node.cpp
)
target_link_libraries(odom_proc ${catkin_LIBRARIES})

add_executable(odom_twist_to_child_frame
  src/nav_utils/odom_twist_to_child_frame.cpp
  src/nav_utils/odom_twist_to_child_frame_node.cpp
)
target_link_libraries(odom_twist_to_child_frame ${catkin_LIBRARIES})

add_executable(odom_recompute_twist
    src/nav_utils/odom_recompute_twist.cpp
    src/nav_utils/odom_recompute_twist_node.cpp
)
target_link_libraries(odom_recompute_twist ${catkin_LIBRARIES})

add_executable(odom_to_tf
    src/nav_utils/odom_to_tf.cpp
    src/nav_utils/odom_to_tf_node.cpp
)
target_link_libraries(odom_to_tf ${catkin_LIBRARIES})

add_executable(tf_to_odom
    src/nav_utils/tf_to_odom.cpp
    src/nav_utils/tf_to_odom_node.cpp
)
target_link_libraries(tf_to_odom ${catkin_LIBRARIES})

add_executable(tf_to_path
    src/nav_utils/tf_to_path.cpp
    src/nav_utils/tf_to_path_node.cpp
)
target_link_libraries(tf_to_path ${catkin_LIBRARIES})

add_executable(tf_fast_repeater
    src/nav_utils/tf_fast_repeater.cpp
    src/nav_utils/tf_fast_repeater_node.cpp
)
target_link_libraries(tf_fast_repeater ${catkin_LIBRARIES})

add_executable(tf_copy
    src/nav_utils/tf_copy.cpp
    src/nav_utils/tf_copy_node.cpp
)
target_link_libraries(tf_copy ${catkin_LIBRARIES})

add_executable(path_downsample
    src/nav_utils/path_downsample.cpp
)
add_dependencies(path_downsample ${PROJECT_NAME}_generate_messages)
target_link_libraries(path_downsample ${catkin_LIBRARIES})

add_executable(path_upsample
    src/nav_utils/path_upsample.cpp
)
add_dependencies(path_upsample ${PROJECT_NAME}_generate_messages)
target_link_libraries(path_upsample ${catkin_LIBRARIES})

add_executable(odom_to_pose
        src/nav_utils/odom_to_pose.cpp
        )
add_dependencies(odom_to_pose ${PROJECT_NAME}_generate_messages)
target_link_libraries(odom_to_pose ${catkin_LIBRARIES})

add_executable(pose_to_odom
        src/nav_utils/pose_to_odom.cpp
        )
add_dependencies(pose_to_odom ${PROJECT_NAME}_generate_messages)
target_link_libraries(pose_to_odom ${catkin_LIBRARIES})

add_executable(odom_to_position
        src/nav_utils/odom_to_position.cpp
        )
add_dependencies(odom_to_position ${PROJECT_NAME}_generate_messages)
target_link_libraries(odom_to_position ${catkin_LIBRARIES})

add_executable(position_to_odom
        src/nav_utils/position_to_odom.cpp
        )
add_dependencies(position_to_odom ${PROJECT_NAME}_generate_messages)
target_link_libraries(position_to_odom ${catkin_LIBRARIES})

add_executable(orient_frame
        src/nav_utils/orient_frame.cpp
        src/nav_utils/orient_frame_node.cpp
        )
add_dependencies(orient_frame ${PROJECT_NAME}_generate_messages)
target_link_libraries(orient_frame ${catkin_LIBRARIES})

#add_executable(test_timer_pub
#    test/test_timer_pub.cpp
#)
#target_link_libraries(test_timer_pub ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_plugins
    src/nav_utils/crop_box_nodelet.cpp
    src/nav_utils/odom_proc.cpp
    src/nav_utils/odom_proc_nodelet.cpp
    src/nav_utils/odom_recompute_twist.cpp
    src/nav_utils/odom_recompute_twist_nodelet.cpp
    src/nav_utils/odom_to_tf.cpp
    src/nav_utils/odom_to_tf_nodelet.cpp
    src/nav_utils/odom_twist_to_child_frame.cpp
    src/nav_utils/odom_twist_to_child_frame_nodelet.cpp
    src/nav_utils/tf_copy.cpp
    src/nav_utils/tf_copy_nodelet.cpp
    src/nav_utils/tf_fast_repeater.cpp
    src/nav_utils/tf_fast_repeater_nodelet.cpp
    src/nav_utils/tf_to_odom.cpp
    src/nav_utils/tf_to_odom_nodelet.cpp
    src/nav_utils/tf_to_path.cpp
    src/nav_utils/tf_to_path_nodelet.cpp
)
target_link_libraries(${PROJECT_NAME}_plugins ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
    scripts/odom_subtract_initial_pose
    scripts/quaternion_to_azimuth
    scripts/tf_connect
    scripts/tf_filter
    scripts/tf_metrics
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    TARGETS
        odom_proc
        odom_recompute_twist
        odom_to_pose
        odom_to_position
        odom_to_tf
        odom_twist_to_child_frame
        orient_frame
        path_downsample
        path_upsample
        pose_to_odom
        position_to_odom
        tf_copy
        tf_fast_repeater
        tf_to_odom
        tf_to_path
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
