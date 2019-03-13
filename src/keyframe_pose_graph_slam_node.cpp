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

void periodic_print_len( const NodeDataManager * manager )
{
    ros::Rate loop_rate(1);
    while( ros::ok() )
    {
        manager->print_nodes_lengths();

        loop_rate.sleep();
    }
}



void periodic_publish_odoms( const NodeDataManager * manager, const VizPoseGraph * viz )
{
    cout << "Start `periodic_publish`\n";
    ros::Rate loop_rate(15);
    double offset_x = 30., offset_y=0., offset_z=0.;
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
            std::this_thread::sleep_for (std::chrono::milliseconds(100));
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
    cout << "END `periodic_publish`\n";
}


// also looks at slam->solvedUntil() to ot repeatedly update the entire path
void periodic_publish_optimized_poses_smart( const NodeDataManager * manager, const PoseGraphSLAM * slam, const VizPoseGraph * viz )
{
    ros::Rate loop_rate0(20);
    int prev_solved_until = 0, solved_until=0;

    vector<Matrix4d> optimized_w_T_ci;
    while( ros::ok() )
    {
        // cout << "periodic_publish_optimized_poses_smart\n";
        if( manager->getNodeLen() == 0 ) {
            loop_rate0.sleep();
            continue;
        }
        solved_until = slam->solvedUntil();


        // Solved() just executed, so get the latest values of all the nodes, (ie. clearup existing values)
        if( solved_until > prev_solved_until )
        {
            optimized_w_T_ci.clear();

            int L = slam->nNodes();
            for( int i=0 ; i<L ; i++ )
            {
                Matrix4d M = slam->getNodePose( i );
                // slam->opt_pose( i, M );
                // slam->getNodePose( i, M );

                optimized_w_T_ci.push_back( M );
            }
        }


        // fill in VIO for the rest
        if( manager->getNodeLen() > optimized_w_T_ci.size() )
        {
            if( optimized_w_T_ci.size() == 0 ) // if ceres::Solve() has not executed even once.
            {
                for( int a=0 ; a<manager->getNodeLen() ; a++ )
                {
                    Matrix4d w_M_c; // VIO pose of current.
                    manager->getNodePose( a, w_M_c );
                    optimized_w_T_ci.push_back( w_M_c ); // put the VIO poses.
                }
            }
            else
            {
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
        }



        viz->publishNodesAsLineStrip( optimized_w_T_ci, "opt_kf_pose_linestrip",
                                        1.0, 0.1, 0.7,
                                        solved_until, 1., 1.0, 0.0 );


        // in nav_msgs::Path only publish last 5
        int end = optimized_w_T_ci.size();
        int start = max( 0, end - 5 );
        viz->publishPath( optimized_w_T_ci, start, end );
        viz->publishOdometry( optimized_w_T_ci );


        loop_rate0.sleep();
        prev_solved_until = solved_until;
    }
    return ;


}


void monitor_disjoint_set_datastructure( const NodeDataManager * manager, const VizPoseGraph * viz )
{
    ros::Rate loop_rate(0.5);

    #if 1
    while( ros::ok() )
    {
        cv::Mat im_disp;
        // manager->getWorldsConstPtr()->disjoint_set_status_image(im_disp); // will get bubles as well as the text
        manager->getWorldsConstPtr()->disjoint_set_status_image(im_disp, true, false); // only bubles

        #if 1 // set this to zero to imshow the image. helpful for debugging.
        viz->publishImage( im_disp );
        #else
        cv::imshow( "disjoint_set_status_image" , im_disp );
        cv::waitKey(30);
        #endif

        loop_rate.sleep();
    }
    #else
    while( ros::ok() )
    {
        string info_msg = manager->getWorldsConstPtr()->disjoint_set_status();
        cv::Mat im_disp = cv::Mat::zeros(cv::Size(300,80), CV_8UC3);

        // cout << "[info_nsg]" << info_msg << endl;


        FalseColors::append_status_image( im_disp, info_msg );

        #if 1
        int uu = manager->getWorldsConstPtr()->n_worlds() ;
        // int uu = manager->n_worlds();
        cout << uu << endl;

        // circles
        for( int i=0 ; i<uu; i++ ) {
            // World Colors
            cv::Scalar color = FalseColors::randomColor( i );
            cout << color << endl;
            cv::Point pt = cv::Point(20*i+15, 15);
            cv::circle( im_disp, pt, 10, color, -1 );
            cv::putText( im_disp, to_string(i), pt, cv::FONT_HERSHEY_SIMPLEX,
                    0.4, cv::Scalar(255,255,255), 1.5 );



            // Set Colors
            cout << " find sedID of the world#" << i << endl;
            int setId =  manager->getWorldsConstPtr()->find_setID_of_world_i( i );
            cerr <<  "setID=" << setId << "  ";
            color = FalseColors::randomColor( setId );
            cerr << color << endl;
            pt = cv::Point(20*i+15, 45);
            cv::circle( im_disp, pt, 10, color, -1 );
            cv::putText( im_disp, to_string(setId), pt, cv::FONT_HERSHEY_SIMPLEX,
                    0.4, cv::Scalar(255,255,255), 1.5 );


        }
        #endif

        cv::imshow( "win" , im_disp );
        cv::waitKey(30);

        loop_rate.sleep();
    }
    #endif
}


// plots the corrected trajectories, different worlds will have different colored lines

#define opt_traj_publisher_colored_by_world_LINE_COLOR_STYLE 10 //< color the line with worldID
// #define opt_traj_publisher_colored_by_world_LINE_COLOR_STYLE 12 //< color the line with setID( worldID )

struct opt_traj_publisher_options
{
    // 10 //< color the line with worldID
    // 12 //< color the line with setID( worldID )
    int line_color_style=10;
};

void opt_traj_publisher_colored_by_world( const NodeDataManager * manager, const PoseGraphSLAM * slam, const VizPoseGraph * viz, const opt_traj_publisher_options& options )
{

    ros::Rate loop_rate(20);
    // ros::Rate loop_rate(5);
    map<int, vector<Matrix4d> > jmb;
    vector< Vector3d > lbm; // a corrected poses. Same index as the node. These are used for loopedges.
    while( ros::ok() )
    {
        // cout << "[opt_traj_publisher_colored_by_world]---\n";
        if( manager->getNodeLen() == 0 ) {
            // cout << "[opt_traj_publisher_colored_by_world]nothing to publish\n";
            loop_rate.sleep();
            continue;
        }

        //clear map
        jmb.clear();
        lbm.clear();

        // cerr << "[opt_traj_publisher_colored_by_world]i=0 ; i<"<< manager->getNodeLen() << " ; solvedUntil=" << slam->solvedUntil() <<"\n";
        int latest_pose_worldid = -1;
        int ____solvedUntil = slam->solvedUntil(); //note: solvedUntil is the index until which posegraph was solved
        int ____solvedUntil_worldid =  manager->which_world_is_this( manager->getNodeTimestamp(____solvedUntil) );
        bool ____solvedUntil_worldid_is_neg = false;
        if( ____solvedUntil_worldid < 0 ) { /*____solvedUntil_worldid = -____solvedUntil_worldid - 1;*/ ____solvedUntil_worldid_is_neg=true; }
        // cerr << "\t[opt_traj_publisher_colored_by_world] slam->solvedUntil=" << ____solvedUntil << "  ____solvedUntil_worldid" << ____solvedUntil_worldid << endl;

        // cout << "[opt_traj_publisher_colored_by_world] i=0" << " i<"<<manager->getNodeLen() ;
        // cout << "____solvedUntil=" << ____solvedUntil << "  ____solvedUntil_worldid=" << ____solvedUntil_worldid << endl;
        for( int i=0 ; i<manager->getNodeLen() ; i++ )
        {
            int world_id = manager->which_world_is_this( manager->getNodeTimestamp(i) );
            // cout << "i=" << i << " world#" << world_id << endl;// << " w_TM_i=" << PoseManipUtils::prettyprintMatrix4d( w_TM_i ) << endl;

            // i>=0 and i<solvedUntil()
            if( i>=0 && i<= ____solvedUntil ) {
                // Matrix4d w_T_c_optimized;
                Matrix4d w_T_c;

                // cerr << "world_id=" << world_id << "   ";
                // cerr << "slam->nodePoseExists("<< i << ") " << slam->nodePoseExists(i) << " ";
                // cerr << "manager->nodePoseExists(" << i << ") " << manager->nodePoseExists(i ) << " \n";


                // If the optimized pose exists use that else use the odometry pose
                int from_slam_or_from_odom = -1;
                if( world_id >= 0 ) {
                    if( slam->nodePoseExists(i) ) {
                        w_T_c = slam->getNodePose( i );
                        // cerr << "w_T_c_optimized=" << PoseManipUtils::prettyprintMatrix4d( w_T_c ) << endl;
                        from_slam_or_from_odom = 1;
                    } else {
                        if( manager->nodePoseExists(i )  ) {
                            w_T_c = manager->getNodePose( i );
                            // cerr << "w_T_c=" << PoseManipUtils::prettyprintMatrix4d( w_T_c ) << endl;
                            from_slam_or_from_odom = 2;
                        }
                    }
                } else {
                    // kidnapped worlds viz, -1, -2 etc.
                    // use the last pose and add the odometry to it
                    // TODO
                    from_slam_or_from_odom = 3;

                    int last_idx = manager->nodeidx_of_world_i_ended( -world_id - 1 );
                    Matrix4d w_T_last;
                    // if( slam->nodePoseExists(i)  )
                    //     w_T_last = slam->getNodePose(last_idx );
                    // else
                    //     w_T_last = manager->getNodePose(last_idx );
                    w_T_last = *(jmb.at( -world_id - 1  ).rbegin());
                    Matrix4d last_M_i = manager->getNodePose( last_idx ).inverse() * manager->getNodePose( i );
                    w_T_c = w_T_last * last_M_i;


                }


                if( jmb.count( world_id ) ==  0 )
                    jmb[ world_id ] = vector<Matrix4d>();

                jmb[ world_id ].push_back( w_T_c );
                lbm.push_back( w_T_c.col(3).topRows(3) );
                latest_pose_worldid = world_id;
                // cout << "  (from_slam_or_from_odom=" << from_slam_or_from_odom << " w_T_c=" << PoseManipUtils::prettyprintMatrix4d( w_T_c ) << endl;

            }


            // i>solvedUntil() < manager->getNodeLen() only odometry available here.

            if( i>(____solvedUntil)  ) {
                int last_idx=-1;
                Matrix4d w_TM_i;



                if( ____solvedUntil == 0 ) {
                    w_TM_i = manager->getNodePose( i );
                } else {
                    if( world_id >= 0 && ____solvedUntil_worldid == world_id ) {
                        last_idx = ____solvedUntil;}
                    else if( world_id >=0 && ____solvedUntil_worldid != world_id  ) {
                        w_TM_i = manager->getNodePose( i );}
                    else if( world_id < 0 ) {
                        // last_idx = ____solvedUntil;
                        last_idx = manager->nodeidx_of_world_i_ended( -world_id - 1 );
                        // cout << "last_idx=" << last_idx << endl;
                    } else {
                        cout << "\nopt_traj_publisher_colored_by_world impossivle\n";
                        exit(2);
                    }

                }

                if( last_idx >= 0 ) {
                    Matrix4d w_T_last;
                    if( slam->nodePoseExists(last_idx) )
                        w_T_last = slam->getNodePose(last_idx );
                    else
                        w_T_last = manager->getNodePose(last_idx );

                    Matrix4d last_M_i = manager->getNodePose( last_idx ).inverse() * manager->getNodePose( i );
                    w_TM_i = w_T_last * last_M_i;
                }



                if( jmb.count( world_id ) ==  0 )
                    jmb[ world_id ] = vector<Matrix4d>();



                jmb[ world_id ].push_back( w_TM_i );
                lbm.push_back( w_TM_i.col(3).topRows(3) );
                latest_pose_worldid = world_id;
                // cout << "  w_TM_i=" << PoseManipUtils::prettyprintMatrix4d( w_TM_i ) << "  last_idx="<<  last_idx << endl;
            }


        }



        // cout << "[opt_traj_publisher_colored_by_world]publish\n";
        // publish jmb
        // #if 0
        for( auto it=jmb.begin() ; it!=jmb.end() ; it++ ) {
            // if( it->first < 0 ) //skip publishing kidnapped part.
                // continue;
            string ns = "world#"+to_string( it->first );

            float c_r=0., c_g=0., c_b=0.;
            int rng=-1;
            if( options.line_color_style == 10 )
                rng = it->first; //color by world id WorldID
            else if( options.line_color_style == 12 )
                rng = manager->getWorldsConstPtr()->find_setID_of_world_i( it->first ); //color by setID
            else {
                cout << TermColor::RED() << "opt_traj_publisher_colored_by_world ERROR. invalid option `opt_traj_publisher_options`. expected either 10 or 12.\n";
                exit( 1 );
            }

            // #if opt_traj_publisher_colored_by_world_LINE_COLOR_STYLE == 10
            // int rng = it->first; //color by world id WorldID
            // #endif
            //
            // #if opt_traj_publisher_colored_by_world_LINE_COLOR_STYLE == 12
            // int rng = manager->getWorldsConstPtr()->find_setID_of_world_i( it->first ); //color by setID
            // #endif
            if( rng >= 0 ) {
                cv::Scalar color = FalseColors::randomColor( rng );
                c_r = color[2]/255.;
                c_g = color[1]/255.;
                c_b = color[0]/255.;
            }

            viz->publishNodesAsLineStrip( it->second, ns.c_str(), c_r, c_g, c_b );
        }
        // #endif

        // Publish Camera visual
        Matrix4d wi_T_latest = *( jmb.at( latest_pose_worldid ).rbegin() );
        float c_r=0., c_g=0., c_b=0.;
        int rng=-1;
        if( options.line_color_style == 10 )
            rng = latest_pose_worldid;
        else if( options.line_color_style == 12 )
            rng = manager->getWorldsConstPtr()->find_setID_of_world_i( latest_pose_worldid ); //color by setID
        else {
            cout << TermColor::RED() << "opt_traj_publisher_colored_by_world ERROR. invalid option `opt_traj_publisher_options`. expected either 10 or 12.\n";
            exit( 1 );
        }


        // #if opt_traj_publisher_colored_by_world_LINE_COLOR_STYLE == 10
        // int rng = latest_pose_worldid;
        // #endif
        //
        // #if opt_traj_publisher_colored_by_world_LINE_COLOR_STYLE == 12
        // int rng = manager->getWorldsConstPtr()->find_setID_of_world_i( latest_pose_worldid ); //color by setID
        // #endif
        if( rng >= 0 ) {
            cv::Scalar color = FalseColors::randomColor( rng );
            c_r = color[2]/255.;
            c_g = color[1]/255.;
            c_b = color[0]/255.;
        }
        viz->publishCameraVisualMarker( wi_T_latest, "world##", c_r, c_g, c_b );


        // Publish loop edges
        // TODO: in the future publish intra-world loopedges in different color and interworld as different color
        visualization_msgs::Marker linelist_marker;
        RosMarkerUtils::init_line_marker( linelist_marker );
        linelist_marker.ns = "loopedges_on_opt_traj"; linelist_marker.id = 0;
        linelist_marker.color.r = 0.4;linelist_marker.color.g = 0.2;linelist_marker.color.b = 0.;linelist_marker.color.a = 1.;
        linelist_marker.scale.x = 0.1;

        int nloopedgfes = manager->getEdgeLen();
        for( int it=0 ; it<nloopedgfes; it++ ) {
            auto pair = manager->getEdgeIdxInfo( it );
            int __a = pair.first;
            int __b = pair.second;
            Vector3d ____apose = lbm[__a];
            Vector3d ____bpose = lbm[__b];

            RosMarkerUtils::add_point_to_marker(  ____apose, linelist_marker, false );
            RosMarkerUtils::add_point_to_marker(  ____bpose, linelist_marker, false );
        }
        viz->publishThisVisualMarker( linelist_marker );


        // book keeping
        // cerr << "\nSLEEP\n";
        loop_rate.sleep();
    }
}


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


    //-- TODO: Also subscribe to tracked features to know if I have been kidnaped.
    //  False indicates -> I got kidnapped now
    //  True indicates --> I got unkidnapped.
    string rcvd_kidnap_indicator_topic = string( "/feature_tracker/rcvd_flag_header" );
    ROS_INFO( "Subscribed to kidnap_indicator aka %s", rcvd_kidnap_indicator_topic.c_str() );
    ros::Subscriber sub_kidnap_indicator = nh.subscribe( rcvd_kidnap_indicator_topic, 100, &NodeDataManager::rcvd_kidnap_indicator_callback, manager );



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
    std::thread th_slam( &PoseGraphSLAM::reinit_ceres_problem_onnewloopedge_optimize6DOF, slam );


    // another class for viz.
    VizPoseGraph * viz = new VizPoseGraph( manager, slam );
    viz->setVisualizationPublisher( pub );
    viz->setImagePublisher( pub_im );
    viz->setPathPublisher( pub_path );
    viz->setOdometryPublisher( pub_odometry_opt );

    // setup manager publishers threads - adhoc
    // std::thread th3( periodic_publish_optimized_poses_smart, manager, slam, viz );
    std::thread th4( periodic_publish_odoms, manager, viz );
    std::thread th5( monitor_disjoint_set_datastructure, manager, viz );

    opt_traj_publisher_options options;
    options.line_color_style = 10;
    std::thread th6( opt_traj_publisher_colored_by_world, manager, slam, viz, options );





    // while(ros::ok()) publish debug stuff
    ros::Rate loop_rate(40);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    // slam->new_optimize6DOF_disable();
    slam->reinit_ceres_problem_onnewloopedge_optimize6DOF_disable();

    // th1.join();
    // th2.join();
    // th3.join();
    th4.join();
    th5.join();
    th6.join();

    th_slam.join();


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


    #define __LOGGING__ 1 // make this 1 to enable logging. 0 to disable logging. rememeber to catkin_make after this change
    #if __LOGGING__
    // save
    // manager->saveForDebug( DATA_PATH );
    manager->saveAsJSON( DATA_PATH );
    slam->saveAsJSON( DATA_PATH );
    #endif


    return 0;
}
