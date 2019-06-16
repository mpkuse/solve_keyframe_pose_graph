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


// plots the corrected trajectories, different worlds will have different colored lines

#define opt_traj_publisher_colored_by_world_LINE_COLOR_STYLE 10 //< color the line with worldID
// #define opt_traj_publisher_colored_by_world_LINE_COLOR_STYLE 12 //< color the line with setID( worldID )

struct opt_traj_publisher_options
{
    // 10 //< color the line with worldID
    // 12 //< color the line with setID( worldID )
    int line_color_style=10;


    // The thickness of the lines
    float linewidth_multiplier=1.0;


    // Udumbe offset_y. When multiple co-ordinates exist, the offset for plotting on rviz. keep it 30.0
    float udumbe_offset_y = 30.0;
};

// comment the following #define to remove the code which checks file's exisitance before printing.
// #define __opt_traj_publisher_colored_by_world___print_on_file_exist

#ifdef __opt_traj_publisher_colored_by_world___print_on_file_exist
#include <sys/stat.h>
#include <unistd.h>
#include <string>
#include <fstream>
inline bool exists_test3 (const std::string& name) {
  struct stat buffer;
  return (stat (name.c_str(), &buffer) == 0);
}
#endif

//original code
#if 0
void opt_traj_publisher_colored_by_world( const NodeDataManager * manager, const PoseGraphSLAM * slam, const VizPoseGraph * viz, const opt_traj_publisher_options& options )
{
    #ifdef __opt_traj_publisher_colored_by_world___print_on_file_exist
    bool enable_cout = false;
    #endif



    ros::Rate loop_rate(20);
    // ros::Rate loop_rate(5);
    map<int, vector<Matrix4d> > jmb;
    vector< Vector3d > lbm; // a corrected poses. Same index as the node. These are used for loopedges.
    vector< Matrix4d > lbm_fullpose;
    bool published_axis = true;
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
        lbm_fullpose.clear();

        // cerr << "[opt_traj_publisher_colored_by_world]i=0 ; i<"<< manager->getNodeLen() << " ; solvedUntil=" << slam->solvedUntil() <<"\n";
        int latest_pose_worldid = -1;
        int ____solvedUntil = slam->solvedUntil(); //note: solvedUntil is the index until which posegraph was solved
        int ____solvedUntil_worldid =  manager->which_world_is_this( manager->getNodeTimestamp(____solvedUntil) );
        bool ____solvedUntil_worldid_is_neg = false;
        if( ____solvedUntil_worldid < 0 ) { /*____solvedUntil_worldid = -____solvedUntil_worldid - 1;*/ ____solvedUntil_worldid_is_neg=true; }
        // cerr << "\t[opt_traj_publisher_colored_by_world] slam->solvedUntil=" << ____solvedUntil << "  ____solvedUntil_worldid" << ____solvedUntil_worldid << endl;

        #ifdef __opt_traj_publisher_colored_by_world___print_on_file_exist
        if( exists_test3( "/app/xxx") )
            enable_cout = true;
        else
            enable_cout = false;

        if( enable_cout ) {
        // cout << "[opt_traj_publisher_colored_by_world] i=0" << " i<"<<manager->getNodeLen() ;
        cout << "____solvedUntil=" << ____solvedUntil << "  ____solvedUntil_worldid=" << ____solvedUntil_worldid << endl;
        }
        #endif


        for( int i=0 ; i<manager->getNodeLen() ; i++ )
        {
            int world_id = manager->which_world_is_this( manager->getNodeTimestamp(i) );
            // cout << "i=" << i << " world#" << world_id << endl;// << " w_TM_i=" << PoseManipUtils::prettyprintMatrix4d( w_TM_i ) << endl;

            // i>=0 and i<solvedUntil()
            if( i>=0 && i<= ____solvedUntil ) {
                // Matrix4d w_T_c_optimized;
                Matrix4d w_T_c;


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
                    w_T_last = *(jmb.at( -world_id - 1  ).rbegin());
                    Matrix4d last_M_i = manager->getNodePose( last_idx ).inverse() * manager->getNodePose( i );
                    w_T_c = w_T_last * last_M_i;


                }


                if( jmb.count( world_id ) ==  0 )
                    jmb[ world_id ] = vector<Matrix4d>();

                jmb[ world_id ].push_back( w_T_c );
                lbm.push_back( w_T_c.col(3).topRows(3) );
                lbm_fullpose.push_back( w_T_c );
                latest_pose_worldid = world_id;

                #ifdef __opt_traj_publisher_colored_by_world___print_on_file_exist
                if( enable_cout )
                cout << i << ":" <<  world_id << "  (from_slam_or_from_odom=" << from_slam_or_from_odom << " w_T_c=" << PoseManipUtils::prettyprintMatrix4d( w_T_c ) << endl;
                #endif

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
                        w_TM_i = manager->getNodePose( i );
                    }
                    else if( world_id < 0 ) {
                        // this is the kidnaped node
                        last_idx = manager->nodeidx_of_world_i_ended( -world_id - 1 ); // only this in working code

                        if( !( last_idx >= 0 && last_idx < manager->getNodeLen() && i >=0 && i<manager->getNodeLen()) ) {
                            cout << "ERROR. last_idx=" << last_idx << endl;
                            manager->getWorldsConstPtr()->print_summary( 2);
                            manager->print_worlds_info(2);
                            assert( last_idx >= 0 && last_idx < manager->getNodeLen() && i >=0 && i<manager->getNodeLen() );
                        }

                        w_TM_i = *(jmb[ -world_id-1 ].rbegin()) * ( manager->getNodePose( last_idx ).inverse() * manager->getNodePose( i ) ) ;
                        last_idx = -1;

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
                lbm_fullpose.push_back( w_TM_i );
                latest_pose_worldid = world_id;

                #ifdef __opt_traj_publisher_colored_by_world___print_on_file_exist
                if( enable_cout )
                cout << i << ":" << world_id << "  w_TM_i=" << PoseManipUtils::prettyprintMatrix4d( w_TM_i ) << "  last_idx="<<  last_idx << endl;
                #endif
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

            viz->publishNodesAsLineStrip( it->second, ns.c_str(), c_r, c_g, c_b, options.linewidth_multiplier );
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
        viz->publishCameraVisualMarker( wi_T_latest, "world##", c_r, c_g, c_b, options.linewidth_multiplier, 20 );


        // Publish loop edges
        // TODO: in the future publish intra-world loopedges in different color and interworld as different color
        visualization_msgs::Marker linelist_marker;
        RosMarkerUtils::init_line_marker( linelist_marker );
        linelist_marker.ns = "loopedges_on_opt_traj"; linelist_marker.id = 0;
        linelist_marker.color.r = 0.42;linelist_marker.color.g = 0.55;linelist_marker.color.b = 0.14;linelist_marker.color.a = 1.;
        linelist_marker.scale.x = 0.1*options.linewidth_multiplier;

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


        if( published_axis || rand() % 100 == 0 ) {
            Matrix4d _axis_pose = Matrix4d::Identity();
            // odm_axis_pose(0,3) += offset_x; odm_axis_pose(1,3) += offset_y; odm_axis_pose(2,3) += offset_z;
            viz->publishXYZAxis( _axis_pose, "opt_traj_axis", 0  );
            published_axis = false;
        }


        // book keeping
        // cerr << "\nSLEEP\n";
        loop_rate.sleep();
    }


    #if __CODE___GT__

    // Set this to 1 to enable loggin of final poses,
    #define __LOGGING___LBM__ 1
    #if __LOGGING___LBM__
    // Log LMB
    vector<assoc_s> _RESULT_;
    cout << "========[opt_traj_publisher_colored_by_world] logging ========\n";
    cout << "loop on gt_map\n";
    int _gt_map_i = 0;
    //---a
    for( auto it=gt_map.begin() ; it!= gt_map.end() ; it++ ) {
        cout << _gt_map_i++ << " : " << it->first << " : " << (it->second).transpose() << endl;
    }

    ///---b
    cout << "loop on lbm lbm.size() = " << lbm.size() << " lbm_fullpose.size= "<< lbm_fullpose.size() <<"\n";
    assert( lbm.size() == lbm_fullpose.size() );
    for( auto k=0 ; k<lbm.size() ; k++ ) {
        ros::Time _t = manager->getNodeTimestamp(k);
        Matrix4d _viopose = manager->getNodePose(k);
        cout << k << ": " << _t << ": "<< lbm[k].transpose() << "\t" ;
        cout << PoseManipUtils::prettyprintMatrix4d(lbm_fullpose[k]) << endl;


        assoc_s tmp;
        tmp.corrected_pose = lbm_fullpose[k];
        tmp.vio_pose = _viopose;
        tmp.node_timestamp = _t;


        //---------------
        // loop through gt_map and find the gt_pose at this t.
        // search for `_t` in gt_map.
        {
            cout << "\tsearch for `_t`= "<< _t << " in gt_map.\n";
            int it_i = -1;
            ros::Duration smallest_diff = ros::Duration( 100000 );
            int smallest_diff_i = -1;
            auto gt_map_iterator = gt_map.begin();
            bool found = false;
            for( auto it=gt_map.begin() ; it!= gt_map.end() ; it++ ) {
                it_i++;
                ros::Duration diff = it->first - _t;
                if( diff.sec < 0 ) { diff.sec = - diff.sec; diff.nsec = 1000000000 - diff.nsec; }
                // cout << "\t\tit_i=" << it_i << " diff=" << diff.sec << " " << diff.nsec << "  >>>>>> smallest_diff_i=" << smallest_diff_i << endl;

                if( diff.sec < smallest_diff.sec || (diff.sec == smallest_diff.sec && abs(diff.nsec) < abs(smallest_diff.nsec)  ) ) {
                    smallest_diff = diff;
                    smallest_diff_i = it_i;
                    gt_map_iterator = it;
                }
                if( (diff.sec == 0  &&  abs(diff.nsec) < 10000000) || (diff.sec == -1  &&  diff.nsec > (1000000000-10000000) )  ) {
                    // cout << "NodeDataManager::find_indexof_node " << i << " "<< diff.sec << " " << diff.nsec << endl
                    cout << TermColor::GREEN() << "\tfound " << it->first  << " at idx=" << it_i << endl << TermColor::RESET();
                    found = true;

                    tmp.gt_pose_timestamp = it->first;
                    tmp.gt_pose = it->second;
                    break;
                }

            }

            if( found == false ) {
                cout << TermColor::RED() << "\tNOT found, however, smallest_diff=" << smallest_diff << " at idx="<< smallest_diff_i;
                cout << endl << TermColor::RESET();
                tmp.gt_pose_timestamp = gt_map_iterator->first;
                tmp.gt_pose = gt_map_iterator->second;
            }
        }
        //---------
        // END of search onn gt_map
        //---------



        // Result:"
        _RESULT_.push_back( tmp );
        cout << TermColor::CYAN() << "Result: \n";
        cout << "node_timestamp           " << tmp.node_timestamp << endl;
        cout << "gt_pose_timestamp        " << tmp.gt_pose_timestamp << endl;
        cout << "gt_pose                  " << (tmp.gt_pose).transpose() << endl;
        cout << "vio_pose                 " << PoseManipUtils::prettyprintMatrix4d(tmp.vio_pose) << endl;
        cout << "corrected_pose           " << PoseManipUtils::prettyprintMatrix4d(tmp.corrected_pose) << endl;
        cout << TermColor::RESET();

    }


    // write CSV:
    std::stringstream buffer_gt;
    std::stringstream buffer_corrected;
    std::stringstream buffer_vio;

    for( int j=0 ; j<_RESULT_.size() ; j++ )
    {
        // stamp, tx, ty, tz
        buffer_gt << long(_RESULT_[j].node_timestamp.toSec() * 1E9) << "," << _RESULT_[j].gt_pose(0) << "," << _RESULT_[j].gt_pose(1)<< "," << _RESULT_[j].gt_pose(2) <<endl;


        // stamp, tx, ty, tz, qw, qx, qy, qz
        buffer_corrected << long(_RESULT_[j].node_timestamp.toSec() * 1E9) << "," << _RESULT_[j].corrected_pose(0,3) << "," << _RESULT_[j].corrected_pose(1,3)<< "," << _RESULT_[j].corrected_pose(2,3) << ",";
        Matrix3d __R = _RESULT_[j].corrected_pose.topLeftCorner(3,3);
        Quaterniond quat( __R );
        buffer_corrected << quat.w() << "," << quat.x() << ","<< quat.y() << "," << quat.z() << endl;


        buffer_vio << long(_RESULT_[j].node_timestamp.toSec() * 1E9) << "," << _RESULT_[j].vio_pose(0,3) << "," << _RESULT_[j].vio_pose(1,3)<< "," << _RESULT_[j].vio_pose(2,3) << ",";
        Matrix3d __R2 = _RESULT_[j].vio_pose.topLeftCorner(3,3);
        Quaterniond quat2( __R );
        buffer_vio << quat2.w() << "," << quat2.x() << ","<< quat2.y() << "," << quat2.z() << endl;
    }
    const string DATA_PATH = "/Bulk_Data/_tmp_posegraph/";
    RawFileIO::write_string( DATA_PATH+"/gt.csv", buffer_gt.str() );
    RawFileIO::write_string( DATA_PATH+"/corrected.csv", buffer_corrected.str() );
    RawFileIO::write_string( DATA_PATH+"/vio_pose.csv", buffer_vio.str() );



    #endif // __LOGGING___LBM__

    #endif  //__CODE___GT__




}
#endif



void opt_traj_publisher_colored_by_world( const NodeDataManager * manager, const PoseGraphSLAM * slam, const VizPoseGraph * viz, const opt_traj_publisher_options& options )
{
    #ifdef __opt_traj_publisher_colored_by_world___print_on_file_exist
    bool enable_cout = false;
    #endif



    ros::Rate loop_rate(10);
    // ros::Rate loop_rate(5);
    map<int, vector<Matrix4d> > jmb;
    vector< Vector3d > lbm; // a corrected poses. Same index as the node. These are used for loopedges.
    vector< Matrix4d > lbm_fullpose;
    bool published_axis = true;
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
        lbm_fullpose.clear();

        // cerr << "[opt_traj_publisher_colored_by_world]i=0 ; i<"<< manager->getNodeLen() << " ; solvedUntil=" << slam->solvedUntil() <<"\n";
        int latest_pose_worldid = -1;
        int ____solvedUntil = slam->solvedUntil(); //note: solvedUntil is the index until which posegraph was solved
        int ____solvedUntil_worldid =  manager->which_world_is_this( manager->getNodeTimestamp(____solvedUntil) );
        bool ____solvedUntil_worldid_is_neg = false;
        if( ____solvedUntil_worldid < 0 ) { /*____solvedUntil_worldid = -____solvedUntil_worldid - 1;*/ ____solvedUntil_worldid_is_neg=true; }
        // cerr << "\t[opt_traj_publisher_colored_by_world] slam->solvedUntil=" << ____solvedUntil << "  ____solvedUntil_worldid" << ____solvedUntil_worldid << endl;

        #ifdef __opt_traj_publisher_colored_by_world___print_on_file_exist
        if( exists_test3( "/app/xxx") )
            enable_cout = true;
        else
            enable_cout = false;

        if( enable_cout ) {
        // cout << "[opt_traj_publisher_colored_by_world] i=0" << " i<"<<manager->getNodeLen() ;
        cout << "____solvedUntil=" << ____solvedUntil << "\t";
        cout << "____solvedUntil_worldid=" << ____solvedUntil_worldid << "\t";
        cout << "__slam->get_reinit_ceres_problem_onnewloopedge_optimize6DOF_status=" << slam->get_reinit_ceres_problem_onnewloopedge_optimize6DOF_status() << "\t";
        cout << endl;
        }
        #endif

        ElapsedTime _time_jmb;
        _time_jmb.tic();
        for( int i=0 ; i<manager->getNodeLen() ; i++ )
        {
            int world_id = manager->which_world_is_this( manager->getNodeTimestamp(i) );
            // cout << "i=" << i << " world#" << world_id << endl;// << " w_TM_i=" << PoseManipUtils::prettyprintMatrix4d( w_TM_i ) << endl;


            /*
            if( slam->get_reinit_ceres_problem_onnewloopedge_optimize6DOF_status() == 2 ) {
                cout << TermColor::iYELLOW() << "[main::opt_traj_publisher_colored_by_world] I have detected ceres::Solve() is in progress. This is the cause of thread blocking....avoiding this by break" << TermColor::RESET() << endl;
                break;
            }
            */

            // const Matrix4d ___slam_getNodePose_i = slam->getNodePose( i );
            // bool ___slam_nodePoseExists_i = slam->nodePoseExists(i);


            // i>=0 and i<solvedUntil()
            if( i>=0 && i<= ____solvedUntil ) {
                // Matrix4d w_T_c_optimized;
                Matrix4d w_T_c;


                // If the optimized pose exists use that else use the odometry pose
                int from_slam_or_from_odom = -1;
                if( world_id >= 0 ) {
                    if( slam->nodePoseExists(i) /*___slam_nodePoseExists_i*/ ) {
                        w_T_c = slam->getNodePose( i );
                        /*w_T_c = ___slam_getNodePose_i;*/
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
                    w_T_last = *(jmb.at( -world_id - 1  ).rbegin());
                    Matrix4d last_M_i = manager->getNodePose( last_idx ).inverse() * manager->getNodePose( i );
                    w_T_c = w_T_last * last_M_i;


                }


                if( jmb.count( world_id ) ==  0 )
                    jmb[ world_id ] = vector<Matrix4d>();

                jmb[ world_id ].push_back( w_T_c );
                lbm.push_back( w_T_c.col(3).topRows(3) );
                lbm_fullpose.push_back( w_T_c );
                latest_pose_worldid = world_id;

                #ifdef __opt_traj_publisher_colored_by_world___print_on_file_exist
                if( enable_cout )
                cout << TermColor::RED() << i << ":" <<  world_id << "  (from_slam_or_from_odom=" << from_slam_or_from_odom << " w_T_c=" << PoseManipUtils::prettyprintMatrix4d( w_T_c ) << endl << TermColor::RESET();
                #endif

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
                        w_TM_i = manager->getNodePose( i );
                    }
                    else if( world_id < 0 ) {
                        // this is the kidnaped node
                        last_idx = manager->nodeidx_of_world_i_ended( -world_id - 1 ); // only this in working code

                        if( !( last_idx >= 0 && last_idx < manager->getNodeLen() && i >=0 && i<manager->getNodeLen()) ) {
                            cout << "ERROR. last_idx=" << last_idx << endl;
                            manager->getWorldsConstPtr()->print_summary( 2);
                            manager->print_worlds_info(2);
                            assert( last_idx >= 0 && last_idx < manager->getNodeLen() && i >=0 && i<manager->getNodeLen() );
                        }

                        w_TM_i = *(jmb[ -world_id-1 ].rbegin()) * ( manager->getNodePose( last_idx ).inverse() * manager->getNodePose( i ) ) ;
                        last_idx = -1;

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
                lbm_fullpose.push_back( w_TM_i );
                latest_pose_worldid = world_id;

                #ifdef __opt_traj_publisher_colored_by_world___print_on_file_exist
                if( enable_cout )
                cout << i << ":" << world_id << "  w_TM_i=" << PoseManipUtils::prettyprintMatrix4d( w_TM_i ) << "  last_idx="<<  last_idx << endl;
                #endif
            }


        }

        cout << TermColor::BLUE() << "[opt_traj_publisher_colored_by_world] Took " << _time_jmb.toc_milli() << "ms to compute #nodes=" << manager->getNodeLen() << TermColor::RESET() << endl;
        //-----------------------------------------------------------------------------------------------//
        //------------------------- After this only uses jmb and lmb to publish -------------------------//
        //-----------------------------------------------------------------------------------------------//

        //---
        //--- Decide offset's (for plotting) different co-ordinate systems
        //---
        ElapsedTime _time_publih;
        _time_publih.tic();
        #if 1
        map< int, int > setids_to_udumbes;
        if( jmb.size() > 0 )
        {
            int h=0;
            for( auto it=jmb.begin() ; it!=jmb.end() ; it++ ) {
                int setid = manager->getWorldsConstPtr()->find_setID_of_world_i( it->first );
                if( setid < 0 ) //ignore negative setids
                    continue;
                if( setids_to_udumbes.count(setid) == 0 ) {
                    setids_to_udumbes[ setid ] = h;
                    h++;
                }
            }

            #if 0
            cout << TermColor::MAGENTA() << "---\n";
            for( auto it=setids_to_udumbes.begin() ; it!=setids_to_udumbes.end() ; it++ ) {
                cout << "setid=" << it->first << "\tudumbe=" << it->second << endl;
            }
            cout << TermColor::RESET();
            #endif

        }
        #endif




        //---
        //--- Publish jmb.
        //          Note: jmb's keys are `worldIDs` and jmb's values are `vector<Matrix4d>& w_T_ci`
        //---
        if( jmb.size() == 0 )
        {
                // cout << "[opt_traj_publisher_colored_by_world] not publishing because jmb.size is zero\n";
        }
        else
        {
                // cout << "[opt_traj_publisher_colored_by_world]publish\n";

            // collect data to publish
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

                int curr_set_id = manager->getWorldsConstPtr()->find_setID_of_world_i( it->first );
                float offset_x=0., offset_y=0., offset_z=0.;
                if( curr_set_id >= 0 ) {
                    if ( setids_to_udumbes.count(curr_set_id) > 0 ) {
                        // cout << "rng=" << rng << "setids_to_udumbes"<<  setids_to_udumbes.at( rng ) << endl;
                        offset_y = setids_to_udumbes.at( curr_set_id )*options.udumbe_offset_y;
                    }
                } else {
                    curr_set_id = manager->getWorldsConstPtr()->find_setID_of_world_i( -it->first-1);
                    if ( setids_to_udumbes.count(curr_set_id) > 0 ) {
                        offset_y = setids_to_udumbes.at( curr_set_id )*options.udumbe_offset_y;
                    }
                }

                viz->publishNodesAsLineStrip( it->second, ns.c_str(), c_r, c_g, c_b, options.linewidth_multiplier, offset_x, offset_y, offset_z );
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


            int curr_set_id = manager->getWorldsConstPtr()->find_setID_of_world_i( latest_pose_worldid );
            float offset_x=0., offset_y=0., offset_z=0.;
            if( curr_set_id >= 0 ) {
                if ( setids_to_udumbes.count(curr_set_id) > 0 ) {
                    // cout << "rng=" << rng << "setids_to_udumbes"<<  setids_to_udumbes.at( rng ) << endl;
                    offset_y = setids_to_udumbes.at( curr_set_id )*options.udumbe_offset_y;
                }
            } else {
                curr_set_id = manager->getWorldsConstPtr()->find_setID_of_world_i( -latest_pose_worldid-1 );
                if ( setids_to_udumbes.count(curr_set_id) > 0 ) {
                    offset_y = setids_to_udumbes.at( curr_set_id )*options.udumbe_offset_y;
                }
            }

            viz->publishCameraVisualMarker( wi_T_latest, "world##", c_r, c_g, c_b,
                    options.linewidth_multiplier, 20,
                    offset_x, offset_y, offset_z );


            // Publish loop edges
            // TODO: in the future publish intra-world loopedges in different color and interworld as different color
            visualization_msgs::Marker linelist_marker;
            RosMarkerUtils::init_line_marker( linelist_marker );
            linelist_marker.ns = "loopedges_on_opt_traj"; linelist_marker.id = 0;
            linelist_marker.color.r = 0.42;linelist_marker.color.g = 0.55;linelist_marker.color.b = 0.14;linelist_marker.color.a = 1.;
            linelist_marker.scale.x = 0.1*options.linewidth_multiplier;

            int nloopedgfes = manager->getEdgeLen();
            for( int it=0 ; it<nloopedgfes; it++ ) {
                auto pair = manager->getEdgeIdxInfo( it );
                int __a = pair.first;
                int __b = pair.second;

                int __a_worldid = manager->which_world_is_this( manager->getNodeTimestamp(__a) );
                int __a_setid = manager->getWorldsConstPtr()->find_setID_of_world_i( __a_worldid );
                int __b_worldid = manager->which_world_is_this( manager->getNodeTimestamp(__b) );
                int __b_setid = manager->getWorldsConstPtr()->find_setID_of_world_i( __b_worldid );
                #if 0
                cout << TermColor::CYAN() ;
                cout << "it=" << it;
                cout << "__a=" << __a << " __a_worldid=" << __a_worldid << " __a_setid=" << __a_setid << "\t|";
                cout << "__b=" << __b << " __b_worldid=" << __b_worldid << " __b_setid=" << __b_setid << "\n";
                cout << TermColor::RESET();
                #endif

                int __a_udumbe = 0, __b_udumbe=0;
                if( setids_to_udumbes.count(__a_setid) > 0 )
                    __a_udumbe = setids_to_udumbes.at( __a_setid );
                if( setids_to_udumbes.count(__b_setid) > 0 )
                    int __b_udumbe = setids_to_udumbes.at( __b_setid  );

                Vector3d ____apose = lbm[__a] + Vector3d( 0., options.udumbe_offset_y*__a_udumbe , 0.0  );
                Vector3d ____bpose = lbm[__b] + Vector3d( 0., options.udumbe_offset_y*__a_udumbe , 0.0  );

                RosMarkerUtils::add_point_to_marker(  ____apose, linelist_marker, false );
                RosMarkerUtils::add_point_to_marker(  ____bpose, linelist_marker, false );
            }
            viz->publishThisVisualMarker( linelist_marker );


            // if( published_axis || rand() % 100 == 0 ) {
            if( published_axis || rand() % 100 == 0 ) {
                // odm_axis_pose(0,3) += offset_x; odm_axis_pose(1,3) += offset_y; odm_axis_pose(2,3) += offset_z;
                // viz->publishXYZAxis( _axis_pose, "opt_traj_axis", 0  );

                for( int p=0 ; p<setids_to_udumbes.size() ; p++ ) {
                    Matrix4d _axis_pose = Matrix4d::Identity();
                    _axis_pose(0,3) += 0.0; _axis_pose(1,3) += p*options.udumbe_offset_y; _axis_pose(2,3) += 0.0;
                    viz->publishXYZAxis( _axis_pose, "opt_traj_axis", p, 2.0  );
                }

                if( rand() %1000 == 0 ) {
                    // once in a while flush the unused co-ordinates
                    Matrix4d _axis_pose = Matrix4d::Identity();
                    for( int p=setids_to_udumbes.size() ; p<20;  p++ ) {
                        viz->publishXYZAxis( _axis_pose, "opt_traj_axis", p, 0.0  );
                    }
                }

                published_axis = false;
            }

        }

        cout << TermColor::BLUE() << "[opt_traj_publisher_colored_by_world] Publish took " << _time_publih.toc_milli() << " ms" << endl;
        // book keeping
        // cerr << "\nSLEEP\n";
        loop_rate.sleep();
    }


    #if __CODE___GT__

    // Set this to 1 to enable loggin of final poses,
    #define __LOGGING___LBM__ 1
    #if __LOGGING___LBM__
    // Log LMB
    vector<assoc_s> _RESULT_;
    cout << "========[opt_traj_publisher_colored_by_world] logging ========\n";
    cout << "loop on gt_map\n";
    int _gt_map_i = 0;
    //---a
    for( auto it=gt_map.begin() ; it!= gt_map.end() ; it++ ) {
        cout << _gt_map_i++ << " : " << it->first << " : " << (it->second).transpose() << endl;
    }

    ///---b
    cout << "loop on lbm lbm.size() = " << lbm.size() << " lbm_fullpose.size= "<< lbm_fullpose.size() <<"\n";
    assert( lbm.size() == lbm_fullpose.size() );
    for( auto k=0 ; k<lbm.size() ; k++ ) {
        ros::Time _t = manager->getNodeTimestamp(k);
        Matrix4d _viopose = manager->getNodePose(k);
        cout << k << ": " << _t << ": "<< lbm[k].transpose() << "\t" ;
        cout << PoseManipUtils::prettyprintMatrix4d(lbm_fullpose[k]) << endl;


        assoc_s tmp;
        tmp.corrected_pose = lbm_fullpose[k];
        tmp.vio_pose = _viopose;
        tmp.node_timestamp = _t;


        //---------------
        // loop through gt_map and find the gt_pose at this t.
        // search for `_t` in gt_map.
        {
            cout << "\tsearch for `_t`= "<< _t << " in gt_map.\n";
            int it_i = -1;
            ros::Duration smallest_diff = ros::Duration( 100000 );
            int smallest_diff_i = -1;
            auto gt_map_iterator = gt_map.begin();
            bool found = false;
            for( auto it=gt_map.begin() ; it!= gt_map.end() ; it++ ) {
                it_i++;
                ros::Duration diff = it->first - _t;
                if( diff.sec < 0 ) { diff.sec = - diff.sec; diff.nsec = 1000000000 - diff.nsec; }
                // cout << "\t\tit_i=" << it_i << " diff=" << diff.sec << " " << diff.nsec << "  >>>>>> smallest_diff_i=" << smallest_diff_i << endl;

                if( diff.sec < smallest_diff.sec || (diff.sec == smallest_diff.sec && abs(diff.nsec) < abs(smallest_diff.nsec)  ) ) {
                    smallest_diff = diff;
                    smallest_diff_i = it_i;
                    gt_map_iterator = it;
                }
                if( (diff.sec == 0  &&  abs(diff.nsec) < 10000000) || (diff.sec == -1  &&  diff.nsec > (1000000000-10000000) )  ) {
                    // cout << "NodeDataManager::find_indexof_node " << i << " "<< diff.sec << " " << diff.nsec << endl
                    cout << TermColor::GREEN() << "\tfound " << it->first  << " at idx=" << it_i << endl << TermColor::RESET();
                    found = true;

                    tmp.gt_pose_timestamp = it->first;
                    tmp.gt_pose = it->second;
                    break;
                }

            }

            if( found == false ) {
                cout << TermColor::RED() << "\tNOT found, however, smallest_diff=" << smallest_diff << " at idx="<< smallest_diff_i;
                cout << endl << TermColor::RESET();
                tmp.gt_pose_timestamp = gt_map_iterator->first;
                tmp.gt_pose = gt_map_iterator->second;
            }
        }
        //---------
        // END of search onn gt_map
        //---------



        // Result:"
        _RESULT_.push_back( tmp );
        cout << TermColor::CYAN() << "Result: \n";
        cout << "node_timestamp           " << tmp.node_timestamp << endl;
        cout << "gt_pose_timestamp        " << tmp.gt_pose_timestamp << endl;
        cout << "gt_pose                  " << (tmp.gt_pose).transpose() << endl;
        cout << "vio_pose                 " << PoseManipUtils::prettyprintMatrix4d(tmp.vio_pose) << endl;
        cout << "corrected_pose           " << PoseManipUtils::prettyprintMatrix4d(tmp.corrected_pose) << endl;
        cout << TermColor::RESET();

    }


    // write CSV:
    std::stringstream buffer_gt;
    std::stringstream buffer_corrected;
    std::stringstream buffer_vio;

    for( int j=0 ; j<_RESULT_.size() ; j++ )
    {
        // stamp, tx, ty, tz
        buffer_gt << long(_RESULT_[j].node_timestamp.toSec() * 1E9) << "," << _RESULT_[j].gt_pose(0) << "," << _RESULT_[j].gt_pose(1)<< "," << _RESULT_[j].gt_pose(2) <<endl;


        // stamp, tx, ty, tz, qw, qx, qy, qz
        buffer_corrected << long(_RESULT_[j].node_timestamp.toSec() * 1E9) << "," << _RESULT_[j].corrected_pose(0,3) << "," << _RESULT_[j].corrected_pose(1,3)<< "," << _RESULT_[j].corrected_pose(2,3) << ",";
        Matrix3d __R = _RESULT_[j].corrected_pose.topLeftCorner(3,3);
        Quaterniond quat( __R );
        buffer_corrected << quat.w() << "," << quat.x() << ","<< quat.y() << "," << quat.z() << endl;


        buffer_vio << long(_RESULT_[j].node_timestamp.toSec() * 1E9) << "," << _RESULT_[j].vio_pose(0,3) << "," << _RESULT_[j].vio_pose(1,3)<< "," << _RESULT_[j].vio_pose(2,3) << ",";
        Matrix3d __R2 = _RESULT_[j].vio_pose.topLeftCorner(3,3);
        Quaterniond quat2( __R );
        buffer_vio << quat2.w() << "," << quat2.x() << ","<< quat2.y() << "," << quat2.z() << endl;
    }
    const string DATA_PATH = "/Bulk_Data/_tmp_posegraph/";
    RawFileIO::write_string( DATA_PATH+"/gt.csv", buffer_gt.str() );
    RawFileIO::write_string( DATA_PATH+"/corrected.csv", buffer_corrected.str() );
    RawFileIO::write_string( DATA_PATH+"/vio_pose.csv", buffer_vio.str() );



    #endif // __LOGGING___LBM__

    #endif  //__CODE___GT__




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
    // std::thread th4( periodic_publish_odoms, manager, viz );
    std::thread th5( monitor_disjoint_set_datastructure, manager, viz );

    opt_traj_publisher_options options;
    // 10 //< color the line with worldID
    // 12 //< color the line with setID( worldID )
    options.line_color_style = 10;
    options.linewidth_multiplier = 3; //0.25; //8
    options.udumbe_offset_y = 30.0;
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
    // th4.join();
    th5.join();
    th6.join();

    th_slam.join();



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
