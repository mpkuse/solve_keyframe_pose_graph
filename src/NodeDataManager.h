#pragma once
/** class NodeDataManager

    Subscribes to VIO camera pose of the keyframes and stores poses at the node
    as is.

    Will have access functions which will be used by ceres to get data.
    Make sure these access functions will be thread safe.

        Author  : Manohar Kuse <mpkuse@connect.ust.hk>
        Created : 6th June, 2018.

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
#include <nap/NapMsg.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>


#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
#include <opencv2/core/eigen.hpp>

using namespace std;
using namespace Eigen;

#include "pose_manip_utils/PoseManipUtils.h"



class NodeDataManager
{
public:
    NodeDataManager( const ros::NodeHandle& nh );
    void setFileOut( const string& fname ) { fp = fopen(fname.c_str(), "w" ); }


    // Callbacks core
    void camera_pose_callback( const nav_msgs::Odometry::ConstPtr& msg );
    void loopclosure_pose_callback( const nap::NapMsg::ConstPtr& msg  );

    // internal node queue length info
    void print_nodes_lengths()
    {
        cout << "pose,cov,stamps " << node_pose.size() << "," << node_timestamps.size() << "," << node_pose_covariance.size() ;
        cout << "\tedges " << loopclosure_edges.size() << "," << loopclosure_edges_goodness.size() << "," << loopclosure_p_T_c.size();
        cout << endl;
    }



    // public Publishing functions
    void setVisualizationPublisher( const ros::Publisher& pub );
    void setPathPublisher( const ros::Publisher& pub );


    void publishLastNNodes( int n);
    void publishLastNEdges( int n );
    // void publishNodes( vector<Matrix4d> w_T_ci );
    void publishNodes( vector<Matrix4d> w_T_ci, const string& ns, float r, float g, float b );
    void publishNodes( vector<Matrix4d> w_T_ci, const string& ns, float r, float g, float b, int idx_partition, float r1, float g1, float b1 );

    void publishNodesAsLineStrip( vector<Matrix4d> w_T_ci, const string& ns, float r, float g, float b );
    void publishNodesAsLineStrip( vector<Matrix4d> w_T_ci, const string& ns, float r, float g, float b, int idx_partition, float r1, float g1, float b1, bool enable_camera_visual=true  );


    void publishPath( vector<Matrix4d> w_T_ci  );




    // Public interface
    int getNodeLen();
    int getEdgeLen();
    bool getNodePose( int i, Matrix4d& w_T_cam );
    bool getNodeCov( int i, Matrix<double,6,6>& cov );
    bool getEdgePose( int i, Matrix4d& p_T_c );
    bool getEdgeIdxInfo( int i, std::pair<int,int>& p );


private:
    const ros::NodeHandle& nh;

    FILE * fp = stdout;
    fwrite( const string str ) { fprintf( fp, "%s", str.c_str()); }

    // Node
    std::mutex node_mutex;
    vector<Matrix4d> node_pose;
    vector<ros::Time> node_timestamps;
    vector<Matrix<double,6,6>> node_pose_covariance;


    // Closure-Edges
    std::mutex edge_mutex;
    vector<std::pair<int,int>> loopclosure_edges;
    vector<double> loopclosure_edges_goodness;
    vector<Matrix4d> loopclosure_p_T_c;


    // Publish Marker
    ros::Publisher pub_pgraph; //< Marker
    ros::Publisher pub_path_opt; //< Path

    // Utilities
    int find_indexof_node( const vector<ros::Time>& global_nodes_stamps, const ros::Time& stamp );
    void init_camera_marker( visualization_msgs::Marker& marker, float cam_size );
    void setpose_to_marker( const Matrix4d& w_T_c, visualization_msgs::Marker& marker );
    void setcolor_to_marker( float r, float g, float b, visualization_msgs::Marker& marker  );

    void init_line_marker( visualization_msgs::Marker &marker, const Vector3d& p1, const Vector3d& p2 );
    void init_line_marker( visualization_msgs::Marker &marker );


};
