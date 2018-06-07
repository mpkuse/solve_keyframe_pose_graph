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


#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
#include <opencv2/core/eigen.hpp>

using namespace std;
using namespace Eigen;



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
        cout << "\tedges " << loopclosure_edges.size() << "," << loopclosure_edges_goodness.size() ;
        cout << endl;
    }



    // public Publishing functions
    void setVisualizationPublisher( const ros::Publisher& pub );
    void publishLastNNodes( int n);
    void publishLastNEdges( int n );

    // Public interface
    int getNodeLen()
    {
        node_mutex.lock();
        int n = node_pose.size();
        node_mutex.unlock();
        return n;
    }

    int getEdgeLen()
    {
        edge_mutex.lock();
        int n = loopclosure_edges.size();
        edge_mutex.unlock();
        return n;
    }

    // returns w_T_cam
    bool getNodePose( int i, Matrix4d& w_T_cam )
    {
        bool status;
        node_mutex.lock();
        if( i>=0 && i< node_pose.size() )
        {
            w_T_cam = node_pose[i];
            status = true;
        }
        else
        {
            status = false;
        }
        node_mutex.unlock();

        return status;
    }

    bool getNodeCov( int i, Matrix<double,6,6>& cov )
    {
        bool status;
        node_mutex.lock();
        if( i>=0 && i< node_pose_covariance.size() )
        {
            cov = node_pose_covariance[i];
            status = true;
        }
        else
        {
            status = false;
        }
        node_mutex.unlock();

        return status;
    }


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


    // Publish Marker
    ros::Publisher pub_pgraph;

    // Utilities
    int find_indexof_node( const vector<ros::Time>& global_nodes_stamps, const ros::Time& stamp );
    void init_camera_marker( visualization_msgs::Marker& marker, float cam_size );
    void setpose_to_marker( const Matrix4d& w_T_c, visualization_msgs::Marker& marker );
    void setcolor_to_marker( float r, float g, float b, visualization_msgs::Marker& marker  );

    void init_line_marker( visualization_msgs::Marker &marker, const Vector3d& p1, const Vector3d& p2 );


};
