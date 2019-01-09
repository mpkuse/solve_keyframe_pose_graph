#pragma once

/* Move all the visualization from NodeDataManager to here

    Author  : Manohar Kuse <mpkuse@connect.ust.hk>
    Created : 5th Jan, 2019
*/

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <queue>
#include <ostream>

//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>


// ros
#include <ros/ros.h>
#include <ros/package.h>

#include "NodeDataManager.h"

#include "utils/PoseManipUtils.h"
#include "utils/RosMarkerUtils.h"
#include "utils/TermColor.h"

using namespace std;
using namespace Eigen;


class VizPoseGraph
{
public:
    VizPoseGraph( const NodeDataManager* _manager ) : manager(_manager) {};
    void setVisualizationPublisher( const ros::Publisher& pub );
    void setPathPublisher( const ros::Publisher& pub ); //< TODO: removal
    void setOdometryPublisher( const ros::Publisher& pub );

    void publishLastNNodes( int n );
    void publishNodesAsLineStrip( const vector<Matrix4d>& w_T_ci, const string& ns, float r, float g, float b );
    void publishNodesAsLineStrip( const vector<Matrix4d>& w_T_ci, const string& ns,
        float r, float g, float b,
        int idx_partition,
        float r1, float g1, float b1, bool enable_camera_visual=true  );
    void publishEdgesAsLineArray( int n );
    void publishNodes( const vector<Matrix4d>& w_T_ci, const string& ns, float r, float g, float b );
    void publishNodes( const vector<Matrix4d>& w_T_ci, const string& ns, float r, float g, float b, int idx_partition, float r1, float g1, float b1 );
    void publishPath( const vector<Matrix4d>& w_T_ci, int start, int end  );
    void publishPath( const nav_msgs::Path& path );
    void publishOdometry( const vector<Matrix4d>& w_T_ci );
    void publishLastNEdges( int n );


private:
    const NodeDataManager * manager=NULL;

    // Publish Marker
    ros::Publisher pub_pgraph; //< Marker
    ros::Publisher pub_path_opt; //< Path, TODO: Removal
    ros::Publisher pub_odometry_opt; // nav_msgs::Odometry
};
