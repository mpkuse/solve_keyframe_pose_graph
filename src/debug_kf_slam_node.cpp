/**

This node is supposed to do what `keyframe_pose_graph_slam_node.cpp` does. But the pose graph
is loaded from file (.npz) . Purpose is to analyze the effect of each of the edges to pose graph.
I should see converginf trajectories and not jumping around.


    Author  : Manohar Kuse <mpkuse@connect.ust.hk>
    Created : 14th June, 2018

*/

#include <iostream>
#include <vector>


#include <thread>
#include <mutex>
#include <atomic>


// ros
#include <ros/ros.h>
#include <ros/package.h>


#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
#include <opencv2/core/eigen.hpp>

using namespace std;
using namespace Eigen;

#include "NodeDataManager.h"
#include "PoseGraphSLAM.h"




void periodic_publish_once( const NodeDataManager * manager, const PoseGraphSLAM* slam )
{
    //
    // VIO Poses
    int vioL = manager->getNodeLen();
    vector<Matrix4d> vio_w_T_ci;
    vio_w_T_ci.clear();
    for( int i=0 ; i<vioL ; i++ )
    {
        Matrix4d w_M_c;
        bool status = manager->getNodePose( i, w_M_c );
        assert( status );

        vio_w_T_ci.push_back( w_M_c );
    }
    // manager->publishNodes( vio_w_T_ci, "vio_kf_pose", 1.0, 1.0, 0.0 );
    // manager->publishNodesAsTextLabels( vio_w_T_ci, "vio_kf_pose_labels", 1.0, 1.0, 0.0, .05 );
    manager->publishNodesAsLineStrip( vio_w_T_ci, "vio_kf_pose_line_strip", 1.0, 1.0, 0.0 );

    // manager->publishLastNEdges( -1 );
    manager->publishEdgesAsLineArray( -1 );


    //
    // Optimized Nodes (from SLAM)
    vector<Matrix4d> optimized_w_T_ci;
    optimized_w_T_ci.clear();

    int L = slam->nNodes();
    for( int i=0 ; i<L ; i++ )
    {
        Matrix4d M;
        slam->opt_pose( i, M );
        optimized_w_T_ci.push_back( M );
    }
    // manager->publishNodes( optimized_w_T_ci, "opt_kf_pose", 1.0, 0.1, 0.7 );
    // manager->publishNodesAsTextLabels( optimized_w_T_ci, "opt_kf_pose_labels", 1.0, 0.1, 0.7, 0.05  );
    manager->publishNodesAsLineStrip( optimized_w_T_ci, "opt_kf_pose_line_strip", 1.0, 0.1, 0.7  );

}

// Simple function to brute-force publish all nodes
// void periodic_publish( const NodeDataManager * manager, const PoseGraphSLAM * slam )
void periodic_publish( const NodeDataManager * manager, const PoseGraphSLAM* slam )
{
    ros::Rate loop_rate0(20);
    while( ros::ok() )
    {
        periodic_publish_once( manager, slam );
        loop_rate0.sleep();
    }
    return ;
}


NodeDataManager * global_manager;
PoseGraphSLAM * global_slam;
int BOX_WIDTH = 150;
int BOX_HEIGHT = 100;
int N_rows;
int N_cols;
vector<bool> edge_mask;
bool edge_mask_has_changed = true;


int page_num = 0;
int n_pages = -1;
int items_per_page = 12;
void make_gui_image( cv::Mat& im_toreturn, const NodeDataManager* manager )
{


    // N_rows = (int) sqrt( manager->getEdgeLen() );
    // N_cols = ceil( (float)manager->getEdgeLen() / N_rows);
    N_rows = sqrt( items_per_page );
    N_cols = ceil( (float)items_per_page / N_rows);

    // cout << "make_gui_image #nodes = " << manager->getNodeLen() << " ";
    // cout << "make_gui_image #edges = " << manager->getEdgeLen() << endl;
    // cout << "N_rows=" << N_rows << "\tN_cols=" << N_cols << endl;

    cv::Mat im = cv::Mat( N_rows*BOX_HEIGHT, N_cols*BOX_WIDTH, CV_8UC3, cv::Scalar(255,255,255) );


    int c = items_per_page*page_num;
    for( int i=0 ; i<N_rows ; i++ )
    {
        for( int j=0 ; j<N_cols ; j++ )
        {
            cv::Point pt( j*BOX_WIDTH, i*BOX_HEIGHT ); //x, y
            cv::circle( im, pt + cv::Point(42,42), 40 , cv::Scalar( 169, 169, 169 ), -1);

            cv::Scalar text_color;
            if( edge_mask[c] == true )
                text_color = cv::Scalar(0,255,0);
            else
                text_color = cv::Scalar(0,0,255);

            cv::putText( im, to_string(c), pt + cv::Point(22,12), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, text_color, 1 );

            std::pair<int,int> p;
            manager->getEdgeIdxInfo(c, p);
            string str = to_string(p.first)+"<-->"+to_string(p.second);
            cv::putText( im, str, pt + cv::Point(5,25), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,0,0), 1 );


            cv::putText( im, to_string(manager->getEdgeWeight(c)), pt + cv::Point(5,38), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar(0,0,0), 1 );


            Matrix4d p_T_c;
            manager->getEdgePose(c, p_T_c);
            cv::putText( im,  PoseManipUtils::prettyprintMatrix4d_YPR(p_T_c)  , pt + cv::Point(5,51), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar(0,0,0), 1 );
            cv::putText( im,  PoseManipUtils::prettyprintMatrix4d_t(p_T_c)  , pt + cv::Point(5,64), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar(0,0,0), 1 );


            c++;
            if( c > min( items_per_page*page_num + items_per_page-1, manager->getEdgeLen()-1 ) ) {
                goto outside_the_loop;
                break;
            }
        }
    }

    outside_the_loop:
    // cout << "c="<< c << endl;


    // edges idx
    int start = items_per_page*page_num;
    int end = min( items_per_page*page_num + items_per_page-1, manager->getEdgeLen() - 1 );

    // Status Image
    cv::Mat status_image = cv::Mat( 90, N_cols*BOX_WIDTH, CV_8UC3, cv::Scalar(10,10,10) );
    string core_msg = "Page: "+to_string(page_num+1)+" of "+to_string(n_pages);
    core_msg += "      #Showing Edges [" + to_string(start) + "," + to_string(end) + "]  of "+to_string( manager->getEdgeLen());
    cv::putText( status_image, core_msg , cv::Point(15,15), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255,255,255), 1.5 );

    // keys
    cv::putText( status_image, "`n`: next page. `p` : previous page.", cv::Point(15,35), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255,255,255), 1.5 );
    cv::putText( status_image, "`a` : all on this page set true.", cv::Point(15,55), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255,255,255), 1.5 );
    cv::putText( status_image, "`z` : all on this page set false.", cv::Point(15,75), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255,255,255), 1.5 );


    cv::vconcat(im, status_image, im_toreturn);

}

void cv_callback_function( int event, int x, int y, int flags )
{
    switch( event )
    {
        case CV_EVENT_LBUTTONDOWN:
            cout << "CV_EVENT_LBUTTONDOWN "<< x << " "<< y << endl;

            cout << "page_num=" << page_num << "   n_pages=" << n_pages << ";\t";
            cout << "N_rows, N_cols: " << N_rows << "  " << N_cols << ";\t";
            cout << "grid x,y: " << x/BOX_WIDTH << " " << y/BOX_HEIGHT << ";\t";
            cout << endl;
            if( x/BOX_WIDTH >= N_cols || y/BOX_HEIGHT >= N_rows ) {
                cout << "clicked in status bar...ignore click\n";
                break;
            }

            int idx = y/BOX_HEIGHT * N_cols  +   x/BOX_WIDTH    +   page_num*items_per_page ;
            cout << "idx=" << idx <<endl;

            if( idx >= edge_mask.size() ) {
                cout << "IDX is beyond maximum....ignore click\n";
                break;
            }




            // Togle mask
            edge_mask[ idx  ] = !edge_mask[  idx  ];
            edge_mask_has_changed = true;

            cout << "changed_mask:len="<<edge_mask.size() << "  ";
            for( int i=0 ; i<edge_mask.size() ; i++)
                cout << edge_mask[i];
            cout << endl;
            break;
    }
}

int main( int argc, char ** argv)
{
    const string DATA_PATH = "/Bulk_Data/_tmp_posegraph/";

    ros::init(argc, argv, "debug_pose_graph_solver");
    ros::NodeHandle nh("~");

    NodeDataManager * manager = new NodeDataManager(nh);
    PoseGraphSLAM * slam = new PoseGraphSLAM( manager );
    global_manager =  new NodeDataManager(nh);
    // global_slam = slam;



    // Setup subscribers
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


    // Load file (Pose Graph) all edges
    edge_mask.clear();
    global_manager->loadFromDebug(DATA_PATH, edge_mask ); // load all edges

    for( int i=0 ; i<global_manager->getEdgeLen() ; i++ ) {
        // edge_mask.push_back( false );
        edge_mask.push_back( false );
    }
    cout << "edge_mask.size = " << edge_mask.size() << endl;


    // Make an Image for GUI
    cv::Mat gui_image;
    cv::namedWindow("debug GUI", 1 );
    cv::setMouseCallback( "debug GUI", cv_callback_function );


    // std::thread th( periodic_publish, manager, slam );


    // Solve 6DOF pose graph optimization
    slam->set_inf_loop( false );
    slam->optimize6DOF( );
    cout <<"Done 6DOF\n";


    ros::Rate loop_rate(40);
    n_pages = (int)ceil( global_manager->getEdgeLen() / (double)items_per_page ) ;
    page_num = 0;
    while(ros::ok())
    {

        make_gui_image( gui_image, global_manager );
        cv::imshow( "debug GUI", gui_image );
        char l = cv::waitKey(10);
        switch( l )
        {
            case 'n':
                cout << "Display Next Page\n";
                if( page_num+1 < n_pages )
                    page_num++;
                break;
            case 'p':
                cout << "Display Prev Page\n";
                if( page_num > 0 )
                    page_num--;
                break;
            case 'a':
                cout << "Set all true in current page=" << page_num << endl;
                for( int k=page_num*items_per_page ; k< (page_num+1)*items_per_page ; k++ )
                {
                    cout << "edge_mask[" << k << "] = true\t";
                    edge_mask[ k ] = true;
                }
                edge_mask_has_changed = true;
                cout << endl;
                break;
            case 'z':
                cout << "Set all true in current page=" << page_num << endl;
                for( int k=page_num*items_per_page ; k< (page_num+1)*items_per_page ; k++ )
                {
                    cout << "edge_mask[" << k << "] = true\t";
                    edge_mask[ k ] = false;
                }
                edge_mask_has_changed = true;
                cout << endl;
                break;
        }

        // Load Pose graph in manager considering edge_mask
        if( edge_mask_has_changed )
        {
            manager->loadFromDebug(DATA_PATH, edge_mask ); // load all edges
            slam->deallocate_optimization_variables();
            slam->optimize6DOF( );
            edge_mask_has_changed = false;
        }
        // else
            // cout << "edge_mask has not changed\n";


        periodic_publish_once( manager, slam );

        ros::spinOnce();
    //     // manager->publishLastNNodes( -1 );
        loop_rate.sleep();
    }

    // th.join();
}
