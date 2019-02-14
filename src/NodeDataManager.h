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
#include <std_msgs/Bool.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
#include <opencv2/core/eigen.hpp>

using namespace std;
using namespace Eigen;

#include "utils/PoseManipUtils.h"
#include "utils/RosMarkerUtils.h"
#include "utils/TermColor.h"

// #include "cnpy/cnpy.h"

#include "nlohmann/json.hpp"
using json = nlohmann::json;


class NodeDataManager
{
public:
    NodeDataManager( const ros::NodeHandle& nh );


    // Callbacks core
    void camera_pose_callback( const nav_msgs::Odometry::ConstPtr& msg );
    // void loopclosure_pose_callback( const nap::NapMsg::ConstPtr& msg  );
    void loopclosure_pose_callback( const cerebro::LoopEdge::ConstPtr& msg  );

    void print_nodes_lengths();


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
    // bool saveForDebug( const string& base_path );
    // bool loadFromDebug( const string& base_path, const vector<bool>& edge_mask ); //< Loads what saveForDebug() writes. edge_mask: a vector of 0s, 1s indicating if this edge has to be included or not. edge_mask.size() == 0 will load all
    bool saveAsJSON( const string& base_path );
    bool loadFromJSON( const string& base_path, const vector<bool>& edge_mask ); //< Loads what saveForDebug() writes. edge_mask: a vector of 0s, 1s indicating if this edge has to be included or not. edge_mask.size() == 0 will load all


    void reset_edge_info_data();
    void reset_node_info_data();

private:
    const ros::NodeHandle& nh;


    // Node
    mutable std::mutex node_mutex;
    vector<Matrix4d> node_pose;
    vector<ros::Time> node_timestamps;
    vector<Matrix<double,6,6>> node_pose_covariance;


    // Closure-Edges
    mutable std::mutex edge_mutex;
    vector<std::pair<int,int>> loopclosure_edges;
    vector<std::pair<ros::Time,ros::Time>> loopclosure_edges_timestamps;
    vector<double> loopclosure_edges_goodness;
    vector<Matrix4d> loopclosure_p_T_c;
    vector<string> loopclosure_description;


    // Utilities
    int find_indexof_node( const vector<ros::Time>& global_nodes_stamps, const ros::Time& stamp );

    // void _print_info_on_npyarray( const cnpy::NpyArray& arr );

    ////// Kidnap related
    // TODO: Would be better if I move the World world object here, instead of current in slam class. 
public:
    void rcvd_kidnap_indicator_callback( const std_msgs::HeaderConstPtr& rcvd_ );

    const ros::Time last_kidnap_ended();
    const ros::Time last_kidnap_started();
    bool curr_kidnap_status() { return current_kidnap_status; }


    ros::Time stamp_of_kidnap_i_started( int i );
    ros::Time stamp_of_kidnap_i_ended( int i );
    int n_kidnaps();


    int nodeidx_of_world_i_started( int i );
    int nodeidx_of_world_i_ended( int i );
    int n_worlds();

    // Give me a timestamp and I will tell you which world co-ordinate system
    // this time is.
    //-----------|       |------------------|         |--------------
    //  ^^w0        ^-1       ^^w1               ^^-2           ^^w2
    int which_world_is_this( const ros::Time _t );

private:
    mutable std::mutex mutex_kidnap;
    vector<ros::Time> kidnap_starts;
    vector<ros::Time> kidnap_ends;
    atomic<bool> current_kidnap_status; ///< false indicates `bussiness as usual`. true means kidnapped


    void mark_as_kidnapped( const ros::Time _t );
    void mark_as_unkidnapped( const ros::Time _t );




};
