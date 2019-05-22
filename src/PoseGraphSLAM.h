#pragma once
/** class PoseGraphSLAM

        This runs in a separate thread. Retrives data from NodeDataManager
        on the node's poses and closure edges. Does the non-linear
        least squares.

        Author  : Manohar Kuse <mpkuse@connect.ust.hk>
        Created : 6th, June, 2018

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
#include <chrono>


//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

//ceres
#include <ceres/ceres.h>


// ros
#include <ros/ros.h>
#include <ros/package.h>

#include "NodeDataManager.h"
#include "CeresResidues.h"

#include "Worlds.h"
#include "ScreenColors.h"

#include "utils/PoseManipUtils.h"
#include "utils/TermColor.h"
#include "utils/ElapsedTime.h"

using namespace std;
using namespace Eigen;


#include "nlohmann/json.hpp"
using json = nlohmann::json;

class PoseGraphSLAM
{
public:
    // PoseGraphSLAM( const NodeDataManager* _manager );
    PoseGraphSLAM( NodeDataManager* _manager );
    ~PoseGraphSLAM( ) { deallocate_optimization_variables(); }
    // void stop() { run = false; }

    // This is intended to be run in a separate thread.
    // void optimize6DOF();

    // #define __new_optimize6DOF__ //comment this line to remove the older pose graph implementation
    #ifdef __new_optimize6DOF__
    // This is the newer implementation of optimize6DOF().
    // Don't over engineer this. It runs an inf-loop and monitors manager->getNodeLen() and manager->getEdgeLen().
    // Based on those queues it sets up (and solves) the pose-graph-slam problem.
    void new_optimize6DOF();
    void new_optimize6DOF_enable() { new_optimize6DOF_isEnabled=true; }
    void new_optimize6DOF_disable() { new_optimize6DOF_isEnabled=false; }
    #endif

    // This is the newer (Feb22 2019) implementation of `new_optimize6DOF`.
    // It is an infinite loop and triggers the solve when there are new
    // loop edges in the manager.
    void reinit_ceres_problem_onnewloopedge_optimize6DOF();
    void reinit_ceres_problem_onnewloopedge_optimize6DOF_enable() { reinit_ceres_problem_onnewloopedge_optimize6DOF_isEnabled=true; }
    void reinit_ceres_problem_onnewloopedge_optimize6DOF_disable() { reinit_ceres_problem_onnewloopedge_optimize6DOF_isEnabled=false; }
    int get_reinit_ceres_problem_onnewloopedge_optimize6DOF_status() { return reinit_ceres_problem_onnewloopedge_optimize6DOF_status; }

private:
    // private stuff for thread `reinit_ceres_problem_onnewloopedge_optimize6DOF`.
    atomic<bool> reinit_ceres_problem_onnewloopedge_optimize6DOF_isEnabled;

    // -1 : Nothing happening
    // 0  : Sleeping
    // 1  : Setting up the problem
    // 2  : ceres::Solve in progress
    // 3  : ceres::Solve finished
    atomic<int> reinit_ceres_problem_onnewloopedge_optimize6DOF_status;
    void print_worlds_info( int verbosity );


public:

    // Get the optimized pose at node i. This function is thread-safe
    const Matrix4d getNodePose( int i ) const; //< this gives the pose from the optimization variable
    bool nodePoseExists( int i ) const; //< returns if ith node pose exist
    // bool getNodePose( int i, Matrix4d& ) const; //TODO: removal
    int nNodes() const;
    void getAllNodePose( vector<Matrix4d>& vec_w_T_ci ) const;



    int solvedUntil() const {
        std::lock_guard<std::mutex> lk(mutex_opt_vars);
        return solved_until;
    } // returns the node index until which solve() has operated. Thread-safe with atomics


    void deallocate_optimization_variables();

    // Writes out the optimization variables. Writes the file: `log_optimized_poses.json`
    bool saveAsJSON(const string base_path);

    //TODO: loadFromJSON: also give it an option to load nodes until.

private:
    // global variables
    atomic<bool> new_optimize6DOF_isEnabled;
    // const NodeDataManager * manager;
    NodeDataManager * manager;

    // Optimization variables
    // #define __USE_YPR_REP
    // ^^^^^ comment this out to use the default quaternion representation.
    //      This has some issues, more work needed to correctly get this working.
    //      The 4DOF should be wrt to imu poses and not camera poses.
    //      For now pitch_and_roll regularized 6DOF is alright. 

    mutable std::mutex  mutex_opt_vars;

    #ifdef __USE_YPR_REP
    double * _opt_ypr_ = NULL;
    #else
    double * _opt_quat_ = NULL; //// stored as x,y,z,w
    #endif
    double * _opt_t_ = NULL;
    int _opt_len_=0; ///< this is the current number of nodes. The actual lengths of opt_quat is 4 times this number and that of opt_t is 3 times this number.


    // step-1: new i_opt_quat[5], new i_opt_t[5]
    // step-2: pose--> i_opt_quat, i_opt_t
    // step-3: opt_quat.push_back( i_opt_quat ); opt_t.push_back( i_opt_t )
    void allocate_and_append_new_opt_variable_withpose( const Matrix4d& pose );
    const int n_opt_variables( ) const;
    #ifdef __USE_YPR_REP
    double * get_raw_ptr_to_opt_variable_ypr( int i ) const;
    #else
    double * get_raw_ptr_to_opt_variable_q( int i ) const;
    #endif
    double * get_raw_ptr_to_opt_variable_t( int i ) const;
    bool update_opt_variable_with( int i, const Matrix4d& pose ); //< this will set opt_quad[i] and opt_t[i]. Will return false for invalid i


    // Optimization variables - Loop Edge Switching Constrainsts
    double * _opt_switch_ = NULL;
    int _opt_switch_len_ = 0;


    const int n_opt_switch() const;
    double * get_raw_ptr_to_opt_switch( int i ) const;
    void allocate_and_append_new_edge_switch_var() ;

    // atomic<int> solved_until;
    int solved_until;




    // The optimization problem - CERES
    ceres::Problem problem;
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::LocalParameterization * eigenquaternion_parameterization=NULL;
    ceres::LocalParameterization * qin_angle_local_paramterization = NULL;
    ceres::LossFunction * robust_norm = NULL;
    void init_ceres_optimization_problem();


    // List of residues - These are populated when when you add a residue term
    mutable std::mutex mutex_residue_info;
    vector< std::tuple<int,int,float,string> > odometry_edges_terms; //< a,b, weight, debug string
    vector< std::tuple<int,int,float,string, string> > loop_edges_terms; //< a,b, weight, debug string

    void push_back_odomedge_residue_info( std::tuple<int,int,float,string> );
    void push_back_loopedge_residue_info( std::tuple<int,int,float,string, string> );

public:
    const std::tuple<int,int,float,string>& get_odomedge_residue_info(int i ) const;
    int get_odomedge_residue_info_size() const;


    const std::tuple<int,int,float,string, string>& get_loopedge_residue_info(int i) const;
    int get_loopedge_residue_info_size() const;
    // this i is index of loopedge and not index of node
    double get_loopedge_switching_variable_val( int i ) const { return *( get_raw_ptr_to_opt_switch(i) ); }



};
