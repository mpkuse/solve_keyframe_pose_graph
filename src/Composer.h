#pragma once
// This class will run several threads. The ask of each of these is to
// query manager or the slam object and produce the final poses.


#include <iostream>
#include <random>

#include "NodeDataManager.h"
#include "PoseGraphSLAM.h"
#include "VizPoseGraph.h"

// Threading
#include <thread>
#include <mutex>
#include <atomic>

// ROS


// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
#include <opencv2/core/eigen.hpp>

using namespace std;

#include "utils/PoseManipUtils.h"
#include "utils/RosMarkerUtils.h"
#include "utils/TermColor.h"

// For saveStateToDisk
#include "nlohmann/json.hpp"
using json = nlohmann::json;
#include "utils/RawFileIO.h"

#include <nav_msgs/Path.h>
#include <std_msgs/String.h>

class Composer
{
public:
    Composer( const NodeDataManager * manager,
              const PoseGraphSLAM * slam,
              const VizPoseGraph * viz,
              ros::NodeHandle& _nh
             )
    : manager(manager), slam(slam), viz(viz), nh( _nh )
    {
        b_pose_assember = false;
        b_bf_traj_publish = false;
        b_cam_visual_publish = false;
        b_loopedge_publish = false;
        b_disjointset_statusimage_publish = false;
    }

private:
    mutable std::mutex mx;

    //--- External Ptr
    // TODO: Better convert to shared_ptr
    const NodeDataManager * manager;
    const PoseGraphSLAM * slam;
    const VizPoseGraph * viz ;
    ros::NodeHandle nh;



    //----------------------//
    //--- Assember Thread---//
    //----------------------//
    //      This thread runs say at 30Hz and queries the slam, mgr and forms the jmb.
    //      it acquires the lock as it write the data to jmb.
public:
    void pose_assember_thread( int looprate = 30 );
    void pose_assember_enable() { b_pose_assember = true;}
    void pose_assember_disable() {b_pose_assember = false; }

    // Returns the last element in `global_lmb` along with the timestamp
    // Returns -1 when len(global_lmb) is zero, else return the posegraph node index of it.
    int get_last_known_camerapose( Matrix4d& w_T_lastcam, ros::Time& stamp_of_it );

private:
    atomic<bool> b_pose_assember;
    map<int, vector<Matrix4d> > global_jmb; // key: worldID, value: vector of poses
    vector< Matrix4d > global_lmb; // a corrected poses. Same index as the node. These are used for loopedges.
    int global_latest_pose_worldid = -1; //node-index of the latest pose's worldid



    //-------------------------------------------//
    //--- A Brute force viz all trajactories ---//
    //-------------------------------------------//
    //  makes use of global_jmb and publish all
public:
    void bf_traj_publish_thread( int looprate=15 ) const;
    void bf_traj_publish_enable() { b_bf_traj_publish = true;}
    void bf_traj_publish_disable() {b_bf_traj_publish = false; }

private:
    atomic<bool> b_bf_traj_publish;




    //---------------------//
    //--- Camera visual ---//
    //---------------------//
    //  makes use of global_jmb and publish last cam
public:
    void cam_visual_publish_thread( int looprate = 30 ) const;
    //    vvvv I am publishing just as it is, this might need work when having multiple co-ordinate systems
    void path_publish_thread( int looprate=30 );
    void detailed_path_publish_thread( int looprate=30 );
    void w0_T_w1_publish_thread( int looprate=3 ); //if world exists(0,1) will publish else nothing
    void cam_visual_publish_enable() { b_cam_visual_publish = true;}
    void cam_visual_publish_disable() {b_cam_visual_publish = false; }

private:
    atomic<bool> b_cam_visual_publish;



    //-------------------------//
    //--- Loop Edges Thread ---//
    //-------------------------//
    //  makes use of  manager->getEdgeLen() and global_lmb and publish edges.
public:
    void loopedge_publish_thread( int looprate = 10 ) const;
    void loopedge_publish_enable() { b_loopedge_publish = true;}
    void loopedge_publish_disable() {b_loopedge_publish = false; }

private:
        atomic<bool> b_loopedge_publish;




    //--------------------------------//
    //--- Disjointset Status Image ---//
    //--------------------------------//
    //  makes use of  manager->getEdgeLen() and global_lmb and publish edges.
public:
    void disjointset_statusimage_publish_thread( int looprate = 10 ) const;
    void disjointset_statusjson_publish_thread( int looprate = 10 ) ; //since i define the ros::Publisher inside this thread, I cannot set this as const. :(
    void disjointset_statusimage_publish_enable() { b_disjointset_statusimage_publish = true;}
    void disjointset_statusimage_publish_disable() {b_disjointset_statusimage_publish = false; }

private:
        atomic<bool> b_disjointset_statusimage_publish;


    //-----------------------------------//
    //--- 200 Hz imu pose publication ---//
    //-----------------------------------//
    // Note: The publisher and subscribers are private to this thread itself.
public:
    void setup_200hz_publishers();
    void imu_propagate_callback( const nav_msgs::Odometry::ConstPtr& msg );

private:
    ros::Publisher pub_hz200_marker;
    ros::Publisher pub_hz200_pose, pub_hz200_posestamped;



    /////////////// Save State ////////////////////
public:
    bool saveStateToDisk( string save_dir_path );
    bool loadStateFromDisk( string save_dir_path );
};
