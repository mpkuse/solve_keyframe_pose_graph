/** optimization_node.cpp

    This node will subscribes to camera keyframe poses from VIO.
    Also subscribes to opmode30 messages from geometry processing
    node.

    In a separate thread this does the pose graph optimization

        Author  : Manohar Kuse <mpkuse@connect.ust.hk>
        Created : 6th June, 2018

*/


#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <queue>
#include <ostream>


#include <thread>
#include <mutex>
#include <atomic>


//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

// ros
#include <ros/ros.h>
#include <ros/package.h>


// ros messages
#include <nav_msgs/Path.h>



#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
#include <opencv2/core/eigen.hpp>

using namespace std;
using namespace Eigen;


#include "NodeDataManager.h"
#include "PoseGraphSLAM.h"

void periodic_print_len( const NodeDataManager * manager )
{
    ros::Rate loop_rate(1);
    while( ros::ok() )
    {
        manager->print_nodes_lengths();

        loop_rate.sleep();
    }
}


void periodic_publish( const NodeDataManager * manager )
{
    ros::Rate loop_rate(1);
    while( ros::ok() )
    {
        manager->publishLastNNodes(50);
        manager->publishLastNEdges(50);

        loop_rate.sleep();
    }
}


void periodic_publish_optimized_poses( const NodeDataManager * manager, const PoseGraphSLAM * slam )
{
    ros::Rate loop_rate(20);
    while( ros::ok() )
    {
        vector<Matrix4d> optimized_w_T_ci;
        slam->getAllNodePose( optimized_w_T_ci );

        // Collect additional poses which are not yet added to pose graph optimization
        // cout << "[PeriodicPublish] optimized_w_T_ci.size()=" << optimized_w_T_ci.size() << " ";
        // cout << "manager->getNodeLen()="<<manager->getNodeLen() << endl;
        if( manager->getNodeLen() > optimized_w_T_ci.size() && optimized_w_T_ci.size() > 0 ) {
            Matrix4d w_T_last = optimized_w_T_ci[ optimized_w_T_ci.size() - 1 ]; // optimized pose for last corrected node
            Matrix4d w_M_last;
            manager->getNodePose(  optimized_w_T_ci.size() - 1,  w_M_last ); // ger VIO pose of the last corrected. This is use to get relative pose of current node wrt this node
            for( int a= optimized_w_T_ci.size() ; a<manager->getNodeLen() ; a++ )
            {
                Matrix4d w_M_c; // VIO pose of current.
                manager->getNodePose( a, w_M_c );

                Matrix4d last_M_c; // pose of current wrt last infered from VIO
                last_M_c = w_M_last.inverse() * w_M_c;

                Matrix4d int__w_TM_c = w_T_last * last_M_c  ; // using the corrected one for 0->last and using VIO for last->current.
                optimized_w_T_ci.push_back( int__w_TM_c );
            }
        }
        /// end of additonal poses


        manager->publishNodes( optimized_w_T_ci, "opt_kf_pose", 1.0, 0.1, 0.7 );
        // manager->publishPath( optimized_w_T_ci );

        loop_rate.sleep();
    }
}




int main( int argc, char ** argv)
{
    ros::init(argc, argv, "keyframe_pose_graph_slam_noe");
    ros::NodeHandle nh("~");


    // Setup subscribers and callbacks
    NodeDataManager * manager = new NodeDataManager(nh);

    //--- Camera VIO Pose ---//
    // string keyframes_vio_camerapose_topic =  string("/vins_estimator/camera_pose");
    string keyframes_vio_camerapose_topic =  string("/vins_estimator/keyframe_pose");
    ROS_INFO( "Subscribe to %s", keyframes_vio_camerapose_topic.c_str() );
    ros::Subscriber sub_odometry = nh.subscribe( keyframes_vio_camerapose_topic, 1000,&NodeDataManager::camera_pose_callback, manager );


    //--- Loop Closure Pose ---//
    string loopclosure_camera_rel_pose_topic = string("/colocation_chatter" );
    // string place_recognition_topic = string("/colocation");
    ROS_INFO( "Subscribed to %s", loopclosure_camera_rel_pose_topic.c_str() );
    ros::Subscriber sub_loopclosure = nh.subscribe( loopclosure_camera_rel_pose_topic, 1000, &NodeDataManager::loopclosure_pose_callback, manager );




    // Setup publishers
    //--- Marker ---//
    string marker_topic = string( "visualization_marker");
    ROS_INFO( "Publish to %s", marker_topic.c_str() );
    ros::Publisher pub = nh.advertise<visualization_msgs::Marker>( marker_topic , 1000 );
    manager->setVisualizationPublisher( pub );


    //--- Optimzied Path Publisher ---//
    string opt_path_topic = string( "opt_path");
    ROS_INFO( "Publish to %s", opt_path_topic.c_str() );
    ros::Publisher pub_path = nh.advertise<nav_msgs::Path>( opt_path_topic , 1000 );
    manager->setPathPublisher( pub_path );


    // another class for the core pose graph optimization
    PoseGraphSLAM * slam = new PoseGraphSLAM( manager );
    std::thread th_slam( &PoseGraphSLAM::optimize6DOF, slam );



    // setup manager publishers threads
    // std::thread th1( periodic_print_len, manager );
    std::thread th2( periodic_publish, manager );
    std::thread th3( periodic_publish_optimized_poses, manager, slam );





    // while(ros::ok()) publish debug stuff
    ros::Rate loop_rate(40);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    // th1.join();
    th2.join();
    th3.join();

    th_slam.join();

    return 0;
}
