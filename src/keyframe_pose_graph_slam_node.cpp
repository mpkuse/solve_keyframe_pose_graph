/** optimization_node.cpp

    This node will subscribes to camera keyframe poses from VIO.
    Also subscribes to opmode30 messages from geometry processing
    node.

    In a separate thread this does the pose graph optimization

        Author  : Manohar Kuse <mpkuse@connect.ust.hk>
        Created : 6th June, 2018
        Major Update : 2nd Jan, 2019 (removed dependence on nap, instead subscribes to cerebro::LoopEdge)

*/


#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <queue>
#include <ostream>
#include <cstdlib>


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
#include "VizPoseGraph.h"
#include "utils/RawFileIO.h"

#include "Composer.h"

void periodic_print_len( const NodeDataManager * manager )
{
    ros::Rate loop_rate(1);
    while( ros::ok() )
    {
        manager->print_nodes_lengths();

        loop_rate.sleep();
    }
}



// This thread will publish the odometry data as it is. It takes data from `manager`.
// offsets are used to publish it on rviz as side the main drama.
void periodic_publish_odoms( const NodeDataManager * manager, const VizPoseGraph * viz,
    float offset_x=30., float offset_y=0., float offset_z=0. )
{
    cout << "Start `periodic_publish_odoms`\n";
    ros::Rate loop_rate(15);
    // double offset_x = 30., offset_y=0., offset_z=0.;
    // double offset_x = 0., offset_y=0., offset_z=0.;

    map<int, vector<Matrix4d> > jmb;
    bool published_axis = false;
    while( ros::ok() )
    {
        if( manager->getNodeLen() == 0 ) {
            loop_rate.sleep();
            continue;
        }

        jmb.clear();

        // collect all
        int __i__start = manager->nodeidx_of_world_i_started(manager->n_worlds()-1); //0;
        //
        // try maximum 5 times. mili-seconds after unkidnapped the next world is not available
        // which causes a seg fault. This little fix will sleep for say 5ms and try again
        for( int _j=0 ; _j<25 ; _j++ ) {
            if( __i__start>=0 )
                break;

            cout << "[periodic_publish_odoms] sleep() for 100milis. This is done just as a precaution because next world may not be immediately available after unkidnap. It takes usually upto 500milisec for vins to reinitialize. This warning is not very critial." << endl;
            std::this_thread::sleep_for (std::chrono::milliseconds(200));
            __i__start = manager->nodeidx_of_world_i_started(manager->n_worlds()-1); //0;
        }

        // cout << "__i__start=" << __i__start << endl;

        // for( int i=0 ; i<manager->getNodeLen() ; i++ )
        for( int i=__i__start ; i<manager->getNodeLen() ; i++ )
        {
            if( i==-3 || i==-4 ) {
                cout << "break" << endl;
                break;
            }
            if( i< 0 )
                cout << TermColor::iCYAN() << "manager->getNodeTimestamp(" << i << ")" << TermColor::RESET() << endl;
            int world_id = manager->which_world_is_this( manager->getNodeTimestamp(i) );
            if( manager->nodePoseExists(i )  ) {
                auto w_T_c = manager->getNodePose( i );
                //add offset
                w_T_c(0,3) += offset_x; w_T_c(1,3) += offset_y; w_T_c(2,3) += offset_z;

                if( jmb.count( world_id ) ==  0 )
                    jmb[ world_id ] = vector<Matrix4d>();

                jmb[ world_id ].push_back( w_T_c );
            }
        }
        // cout << "Done collecting\n";



        // Publish all the odometries (unregistered) and also plot the loop edges.
        // make the follow to 1 if you need this.
        #if 0
        for( auto it=jmb.begin() ; it!=jmb.end() ; it++ ) {
            string ns = "odom-world#"+to_string( it->first );

            float c_r=0., c_g=0., c_b=0.;
            int rng = it->first; //color by world id WorldID
            if( rng >= 0 ) {
                cv::Scalar color = FalseColors::randomColor( rng );
                c_r = color[2]/255.;
                c_g = color[1]/255.;
                c_b = color[0]/255.;
            }

            viz->publishNodesAsLineStrip( it->second, ns.c_str(), 0.0, 0.7, 0.0 );
            // if( (it->second).size() > 10 ) {
            // viz->publishNodesAsLineStrip( it->second, ns.c_str(), c_r, c_g, c_b,
            // 10, 0.0, .7, 0.0, false );}
            // else {
            // viz->publishNodesAsLineStrip( it->second, ns.c_str(), c_r, c_g, c_b );
            // }

        }
        // viz->publishLastNEdges( -1 );
        #endif



        #if 1// only publish the latest odometry. Set this to zero if you dont need this.
        int to_publish_key = -1; //lets the largest key value
        for( auto it=jmb.begin() ; it!=jmb.end() ; it++ ) {

            if( it->first > to_publish_key &&  it->first >=0 )
                to_publish_key = it->first;
        }
        if( to_publish_key >= 0 )
            viz->publishNodesAsLineStrip( jmb[to_publish_key] , "latest_odometry", 0.0, 0.7, 0.0 );

        // Publish Camera visual
        Matrix4d wi_T_latest = *( jmb.at( to_publish_key ).rbegin() );
        float c_r=0., c_g=0.7, c_b=0.;
        viz->publishCameraVisualMarker( wi_T_latest, "odom-world##", c_r, c_g, c_b );


        #endif



        if( published_axis==false || rand() % 100 == 0 ) {
        // Publish Axis - publish infrequently
        Matrix4d odm_axis_pose = Matrix4d::Identity();
        odm_axis_pose(0,3) += offset_x; odm_axis_pose(1,3) += offset_y; odm_axis_pose(2,3) += offset_z;
        viz->publishXYZAxis( odm_axis_pose, "odom-axis", 0  );
        published_axis = true;
        }


        loop_rate.sleep();
    }
    cout << "END `periodic_publish_odoms`\n";
}





// set this to 0 to remove the gt code
#define __CODE___GT__ 0
///////////////////
#if __CODE___GT__
std::map< ros::Time, Vector3d > gt_map;
void ground_truth_callback( const geometry_msgs::PointStamped::ConstPtr msg )
{
    cout << TermColor::YELLOW() << "ground_truth_callback " << msg->header.stamp;
    cout << "\t" << msg->point.x << ", " << msg->point.y << ", "<< msg->point.z << "  ";
    cout << TermColor::RESET() << endl;

    gt_map[ msg->header.stamp ] = Vector3d( msg->point.x , msg->point.y , msg->point.z );
}


struct assoc_s
{
    ros::Time node_timestamp;
    Matrix4d corrected_pose;
    Matrix4d vio_pose;

    Vector3d gt_pose;
    ros::Time gt_pose_timestamp;
};
/////////////////////
#endif



int main( int argc, char ** argv)
{
    ros::init(argc, argv, "keyframe_pose_graph_slam_node");
    ros::NodeHandle nh("~");


    // Setup subscribers and callbacks
    NodeDataManager * manager = new NodeDataManager(nh);

    //--- Camera VIO Pose ---//
    string keyframes_vio_camerapose_topic =  string("/vins_estimator/camera_pose");
    // string keyframes_vio_camerapose_topic =  string("/vins_estimator/keyframe_pose");
    ROS_INFO( "Subscribe to %s", keyframes_vio_camerapose_topic.c_str() );
    ros::Subscriber sub_odometry = nh.subscribe( keyframes_vio_camerapose_topic, 1000,&NodeDataManager::camera_pose_callback, manager );


    //--- Loop Closure Pose ---//
    string loopclosure_camera_rel_pose_topic = string("/cerebro/loopedge" );
    // string place_recognition_topic = string("/colocation");
    ROS_INFO( "Subscribed to %s", loopclosure_camera_rel_pose_topic.c_str() );
    ros::Subscriber sub_loopclosure = nh.subscribe( loopclosure_camera_rel_pose_topic, 1000, &NodeDataManager::loopclosure_pose_callback, manager );


    //-- subscribe to tracked features to know if I have been kidnaped. --//
    //  False indicates -> I got kidnapped now
    //  True indicates --> I got unkidnapped.
    string rcvd_kidnap_indicator_topic = string( "/feature_tracker/rcvd_flag_header" );
    ROS_INFO( "Subscribed to kidnap_indicator aka %s", rcvd_kidnap_indicator_topic.c_str() );
    ros::Subscriber sub_kidnap_indicator = nh.subscribe( rcvd_kidnap_indicator_topic, 100, &NodeDataManager::rcvd_kidnap_indicator_callback, manager );


    #if __CODE___GT__
    //--- ground_truth_callback. Sometimes bags may contain GT data, ---//
    // this will associate the GT data to the pose graph for analysis.
    string ground_truth_topic = string( "/leica/position" );
    ROS_INFO( "Subscribed to ground_truth_topic: %s", ground_truth_topic.c_str() );
    ros::Subscriber  sub_gt_ = nh.subscribe( ground_truth_topic, 100, ground_truth_callback );
    #endif

    //-- subscribe to imu_T_cam : imu camera extrinsic calib. Will store this just in case there is a need
    //   this is published by vins_estimator node.
    string extrinsic_cam_imu_topic = string("/vins_estimator/extrinsic");
    ROS_INFO( "Subscribe to extrinsic_cam_imu_topic: %s", extrinsic_cam_imu_topic.c_str() );
    ros::Subscriber sub_cam_imu_extrinsic = nh.subscribe( extrinsic_cam_imu_topic, 1000, &NodeDataManager::extrinsic_cam_imu_callback, manager );





    // Setup publishers
    //--- Marker ---//
    string marker_topic = string( "viz/visualization_marker");
    ROS_INFO( "Publish to %s", marker_topic.c_str() );
    ros::Publisher pub = nh.advertise<visualization_msgs::Marker>( marker_topic , 1000 );

    //--- Image ---//
    string im_topic = string( "viz/disjoint_set_status_image");
    ROS_INFO( "Publish Image to %s", im_topic.c_str() );
    ros::Publisher pub_im = nh.advertise<sensor_msgs::Image>( im_topic , 1000 );


    //--- Optimzied Path Publisher ---//
    string opt_path_topic = string( "opt_path");
    ROS_INFO( "Publish to %s", opt_path_topic.c_str() );
    ros::Publisher pub_path = nh.advertise<nav_msgs::Path>( opt_path_topic , 1000 );

    //--- Optimzied Odometry Publisher ---//
    string opt_odometry_topic = string( "opt_odometry");
    ROS_INFO( "Publish to %s", opt_odometry_topic.c_str() );
    ros::Publisher pub_odometry_opt = nh.advertise<nav_msgs::Odometry>( opt_odometry_topic , 1000 );




    // another class for the core pose graph optimization
    PoseGraphSLAM * slam = new PoseGraphSLAM( manager );
    // std::thread th_slam( &PoseGraphSLAM::optimize6DOF, slam );
    // slam->new_optimize6DOF_enable();
    // std::thread th_slam( &PoseGraphSLAM::new_optimize6DOF, slam );

    slam->reinit_ceres_problem_onnewloopedge_optimize6DOF_enable();
    // slam->reinit_ceres_problem_onnewloopedge_optimize6DOF_disable();
    std::thread th_slam( &PoseGraphSLAM::reinit_ceres_problem_onnewloopedge_optimize6DOF, slam );


    // another class for viz.
    VizPoseGraph * viz = new VizPoseGraph( manager, slam );
    viz->setVisualizationPublisher( pub );
    viz->setImagePublisher( pub_im );
    viz->setPathPublisher( pub_path );
    viz->setOdometryPublisher( pub_odometry_opt );

    //----Pose Composer---//
    Composer * cmpr = new Composer( manager, slam, viz , nh);

    // ++ start the pose assember thread - This is needed for the following publish threads
    // It queries data from manager, slam and assembles the upto date data. This is threadsafe.
    cmpr->pose_assember_enable();
    // cmpr->pose_assember_disable();
    std::thread assember_th( &Composer::pose_assember_thread, cmpr, 30 );

    // ++ start bf_traj_publish_thread
    cmpr->bf_traj_publish_enable();
    // cmpr->bf_traj_publish_disable();
    std::thread bf_pub_th( &Composer::bf_traj_publish_thread, cmpr, 15 );

    // ++ start camera visual publisher thread
    cmpr->cam_visual_publish_enable();
    // cmpr->cam_visual_publish_disable();
    std::thread cam_visual_pub_th( &Composer::cam_visual_publish_thread, cmpr, 30 );

    // ++ loop edge publish thread
    cmpr->loopedge_publish_enable();
    // cmpr->loopedge_publish_disable();
    std::thread loopedge_pub_th( &Composer::loopedge_publish_thread, cmpr, 10 );

    // ++ disjointset status image
    cmpr->disjointset_statusimage_publish_enable();
    // cmpr->disjointset_statusimage_publish_disable();
    std::thread disjointset_monitor_pub_th( &Composer::disjointset_statusimage_publish_thread, cmpr, 1 );


    // TODO
    // subscribes to w_T_imu @100hz and publish the pose in my world frame at 100hz
    // implement this in composer class.
    // /vins_estimator/imu_propagate @ 200hz

    #if 1
        //++(1) Setup publisher for imu poses 200hz
        cmpr->setup_200hz_publishers();


        //++(2) Setup subscriber / callback for /vins_estimator/imu_propagate topic
        string imu_worldpose_topic = string("/vins_estimator/imu_propagate");
        ROS_INFO( "Subscribe to imu_worldpose_topic: %s", imu_worldpose_topic.c_str() );
        // ros::Subscriber sub_imu_worldpose_topic = nh.subscribe( imu_worldpose_topic, 1000, &Composer::imu_propagate_callback, this );
        ros::Subscriber sub_imu_worldpose_topic = nh.subscribe( imu_worldpose_topic, 1000, &Composer::imu_propagate_callback, cmpr );
    #endif



    //----END Pose Composer---//

    //--- setup manager publishers threads - adhoc ---//
    std::thread th4( periodic_publish_odoms, manager, viz, 30.0, 0.0, 0.0 ); //if you enable this, dont forget to join(), after spin() returns.




    // while(ros::ok()) publish debug stuff
    #if 0
    ros::Rate loop_rate(40);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    #else
    ros::spin();
    #endif

    // slam->new_optimize6DOF_disable();
    slam->reinit_ceres_problem_onnewloopedge_optimize6DOF_disable();

    cmpr->pose_assember_disable();
    cmpr->bf_traj_publish_disable();
    cmpr->cam_visual_publish_disable();
    cmpr->loopedge_publish_disable();
    cmpr->disjointset_statusimage_publish_disable();


    th4.join();

    th_slam.join();
    assember_th.join();
    bf_pub_th.join();
    cam_visual_pub_th.join();
    loopedge_pub_th.join();
    disjointset_monitor_pub_th.join();




    #define __LOGGING__ 0 // make this 1 to enable logging. 0 to disable logging. rememeber to catkin_make after this change
    #if __LOGGING__
    // Note: If using roslaunch to launch this node and when LOGGING is enabled,
    // roslaunch sends a sigterm and kills this thread when ros::ok() returns false ie.
    // when you press CTRL+C. The timeout is governed by roslaunch.
    //
    // If you wish to increase this timeout, you need to edit "/opt/ros/kinetic/lib/python2.7/dist-packages/roslaunch/nodeprocess.py"
    // then edit these two vars.
    // _TIMEOUT_SIGINT = 15.0 #seconds
    // _TIMEOUT_SIGTERM = 2.0 #seconds
    //          ^^^^ Borrowed from : https://answers.ros.org/question/11353/how-to-delay-escalation-to-sig-term-after-sending-sig-int-to-roslaunch/



    ///// Save Pose Graph for Debugging
    const string DATA_PATH = "/Bulk_Data/_tmp_posegraph/";

    // rm -rf /Bulk_Data/_tmp_posegraph
    string cmd_rm = "rm -rf "+DATA_PATH;
    cout << cmd_rm << endl;
    std::system( cmd_rm.c_str() );


    // mkdir -p /Bulk_Data/_tmp_posegraph
    string cmd_mkdir = "mkdir -p "+DATA_PATH;
    cout << cmd_mkdir  << endl;
    std::system( cmd_mkdir.c_str() );


    // save
    // manager->saveForDebug( DATA_PATH );
    manager->saveAsJSON( DATA_PATH );
    slam->saveAsJSON( DATA_PATH );
    #endif


    return 0;
}
