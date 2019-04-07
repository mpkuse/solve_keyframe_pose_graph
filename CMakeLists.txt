cmake_minimum_required(VERSION 2.8.3)
project(solve_keyframe_pose_graph)


set( CMAKE_CXX_STANDARD 11 )
find_package(Threads)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  image_transport
  cv_bridge
  #nap
  cerebro
  nav_msgs
)

list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/cmake")
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV 3 REQUIRED)
#find_package( ZLIB REQUIRED ) # Needed for cnpy (c++ support for writing .npy and .npz files). Remove this and in target if you dont need it.



set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)
set( CMAKE_CXX_FLAGS "-fpermissive -std=c++11 -O3" )
#-DEIGEN_USE_MKL_ALL")



generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  #nap
  cerebro
)



catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime image_transport cv_bridge
  #nap
  cerebro
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)



add_executable( keyframe_pose_graph_slam
        src/keyframe_pose_graph_slam_node.cpp
        src/NodeDataManager.cpp
        src/PoseGraphSLAM.cpp
        src/VizPoseGraph.cpp
        src/utils/PoseManipUtils.cpp
        src/utils/RosMarkerUtils.cpp
        src/utils/RawFileIO.cpp
        src/Worlds.cpp
        )


# this is needed to indicate this this executable depends on the messages of the pkg.
add_dependencies(keyframe_pose_graph_slam cerebro_generate_messages_cpp)


add_executable( test_disjointset_imp src/test_disjointset.cpp )
add_executable( test_bfs_imp src/test_bfs.cpp )

# Debug Node
#add_executable( debug_kf_slam_node
#    src/debug_kf_slam_node.cpp
#    src/NodeDataManager.cpp
#    src/PoseGraphSLAM.cpp
#    src/utils/PoseManipUtils.cpp
#    src/utils/RosMarkerUtils.cpp
#    src/cnpy/cnpy.cpp
#    )


target_link_libraries( keyframe_pose_graph_slam
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    ${CERES_LIBRARIES}
#    ${ZLIB_LIBRARIES}
)

target_link_libraries(  test_disjointset_imp ${catkin_LIBRARIES} )
target_link_libraries(  test_bfs_imp ${catkin_LIBRARIES} )

#target_link_libraries( debug_kf_slam_node
#    ${OpenCV_LIBRARIES}
#    ${catkin_LIBRARIES}
#    ${CMAKE_THREAD_LIBS_INIT}
#    ${CERES_LIBRARIES}
#    ${ZLIB_LIBRARIES}
#)
