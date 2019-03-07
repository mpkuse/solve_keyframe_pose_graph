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
#include "PoseGraphSLAM.h"

#include "utils/PoseManipUtils.h"
#include "utils/RosMarkerUtils.h"
#include "utils/TermColor.h"
#include "utils/FalseColors.h"
using namespace std;
using namespace Eigen;


class VizPoseGraph
{
public:
    VizPoseGraph( const NodeDataManager* _manager, const PoseGraphSLAM* _slam ) : manager(_manager), slam(_slam){};
    void setVisualizationPublisher( const ros::Publisher& pub );
    void setImagePublisher( const ros::Publisher& pub );

    void setPathPublisher( const ros::Publisher& pub ); //< TODO: removal
    void setOdometryPublisher( const ros::Publisher& pub );

    // void publishLastNNodes( int n ); //< Removal
    void publishNodesAsLineStrip( const vector<Matrix4d>& w_T_ci, const string& ns, float r, float g, float b ) const;
    void publishNodesAsLineStrip( const vector<Matrix4d>& w_T_ci, const string& ns,
        float r, float g, float b,
        int idx_partition,
        float r1, float g1, float b1, bool enable_camera_visual=true  ) const;
    // void publishEdgesAsLineArray( int n ); //< removal
    void publishNodes( const vector<Matrix4d>& w_T_ci, const string& ns, float r, float g, float b ) const;
    void publishNodes( const vector<Matrix4d>& w_T_ci, const string& ns, float r, float g, float b, int idx_partition, float r1, float g1, float b1 ) const ;
    void publishPath( const vector<Matrix4d>& w_T_ci, int start, int end  ) const;
    void publishPath( const nav_msgs::Path& path ) const;
    void publishOdometry( const vector<Matrix4d>& w_T_ci ) const;
    void publishLastNEdges( int n ) const;

    void publishSlamResidueVisual( int n ) const;
    void publishCameraVisualMarker( const Matrix4d& wTc, const string& ns, float r, float g, float b );

    void publishXYZAxis( const Matrix4d& wT_axis, const string ns, int id );
    void publishThisVisualMarker( const visualization_msgs::Marker& the_marker );
    void publishImage( const cv::Mat& im );

private:
    const NodeDataManager * manager=NULL;
    const PoseGraphSLAM * slam = NULL;

    // Publish Marker
    ros::Publisher pub_pgraph; //< Marker
    ros::Publisher pub_image; //< publishes image.
    ros::Publisher pub_path_opt; //< Path, TODO: Removal
    ros::Publisher pub_odometry_opt; // nav_msgs::Odometry //TODO Removal
};
