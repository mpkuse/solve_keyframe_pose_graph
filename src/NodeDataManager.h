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
// #include <nap/NapMsg.h>
#include <cerebro/LoopEdge.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>


#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
#include <opencv2/core/eigen.hpp>

using namespace std;
using namespace Eigen;

#include "utils/PoseManipUtils.h"
#include "utils/RosMarkerUtils.h"

#include "cnpy/cnpy.h"

#include "nlohmann/json.hpp"
using json = nlohmann::json;


class NodeDataManager
{
public:
    NodeDataManager( const ros::NodeHandle& nh );
    void setFileOut( const string& fname ) { fp = fopen(fname.c_str(), "w" ); }


    // Callbacks core
    void camera_pose_callback( const nav_msgs::Odometry::ConstPtr& msg );
    // void loopclosure_pose_callback( const nap::NapMsg::ConstPtr& msg  );
    void loopclosure_pose_callback( const cerebro::LoopEdge::ConstPtr& msg  );

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

    void publishEdgesAsLineArray( int n );

    void publishNodesAsLineStrip( vector<Matrix4d> w_T_ci, const string& ns, float r, float g, float b );
    void publishNodesAsLineStrip( vector<Matrix4d> w_T_ci, const string& ns, float r, float g, float b, int idx_partition, float r1, float g1, float b1, bool enable_camera_visual=true  );


    void publishPath( vector<Matrix4d> w_T_ci, int start, int end  );
    void publishPath( nav_msgs::Path path );




    // Public interface
    // Node Getters
    int getNodeLen() const;
    bool getNodePose( int i, Matrix4d& w_T_cam ) const; // TODO removal
    const Matrix4d& getNodePose( int i ) const;
    bool getNodeCov( int i, Matrix<double,6,6>& cov ) const ; //TODO removal
    const Matrix<double,6,6>& getNodeCov( int i ) const;
    bool getNodeTimestamp( int i, ros::Time& stamp ) const;
    const ros::Time getNodeTimestamp( int i ) const ;

    // Edge Getters
    int getEdgeLen() const ;
    bool getEdgePose( int i, Matrix4d& p_T_c ) const; //TODO removal
    const Matrix4d& getEdgePose( int i ) const ;
    bool getEdgeIdxInfo( int i, std::pair<int,int>& p ) const; //TODO removal
    const std::pair<int,int>& getEdgeIdxInfo( int i ) const ;
    double getEdgeWeight( int i ) const;
    const string getEdgeDescriptionString( int i ) const ;


    /// Will save the Variables `node_pose`, `node_timestamps`, `node_pose_covariance`; loopclosure_edges, loopclosure_edges_goodness, loopclosure_p_T_c
    bool saveForDebug( const string& base_path );
    bool saveAsJSON( const string& base_path );
    bool loadFromDebug( const string& base_path, const vector<bool>& edge_mask ); //< Loads what saveForDebug() writes. edge_mask: a vector of 0s, 1s indicating if this edge has to be included or not. edge_mask.size() == 0 will load all


    void reset_edge_info_data();
    void reset_node_info_data();

private:
    const ros::NodeHandle& nh;

    FILE * fp = stdout;
    fwrite( const string str ) { fprintf( fp, "%s", str.c_str()); }

    // Node
    mutable std::mutex node_mutex;
    vector<Matrix4d> node_pose;
    vector<ros::Time> node_timestamps;
    vector<Matrix<double,6,6>> node_pose_covariance;


    // Closure-Edges
    mutable std::mutex edge_mutex;
    vector<std::pair<int,int>> loopclosure_edges;
    vector<double> loopclosure_edges_goodness;
    vector<Matrix4d> loopclosure_p_T_c;
    vector<string> loopclosure_description;


    // Publish Marker
    ros::Publisher pub_pgraph; //< Marker
    ros::Publisher pub_path_opt; //< Path

    // Utilities
    int find_indexof_node( const vector<ros::Time>& global_nodes_stamps, const ros::Time& stamp );


    void _print_info_on_npyarray( const cnpy::NpyArray& arr );

};
