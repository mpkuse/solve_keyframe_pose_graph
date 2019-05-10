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

    // This is the newer implementation of optimize6DOF().
    // Don't over engineer this. It runs an inf-loop and monitors manager->getNodeLen() and manager->getEdgeLen().
    // Based on those queues it sets up (and solves) the pose-graph-slam problem.
    void new_optimize6DOF();
    void new_optimize6DOF_enable() { new_optimize6DOF_isEnabled=true; }
    void new_optimize6DOF_disable() { new_optimize6DOF_isEnabled=false; }


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
    bool nodePoseExists__nolock( int i ) const; //< returns if ith node pose exist
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
    #define ___OPT_AS_DOUBLE_STAR // comment this out for having optimization variables as vector<double*>.
    mutable std::mutex  mutex_opt_vars;

    #if defined(___OPT_AS_DOUBLE_STAR)
    double * _opt_quat_ = NULL; //// stored as x,y,z,w
    double * _opt_t_ = NULL;
    int _opt_len_=0; ///< this is the current number of nodes. The actual lengths of opt_quat is 4 times this number and that of opt_t is 3 times this number.
    #else
    vector<double*> opt_quat; // stored as x,y,z,w
    vector<double*> opt_t;
    #endif


    // step-1: new i_opt_quat[5], new i_opt_t[5]
    // step-2: pose--> i_opt_quat, i_opt_t
    // step-3: opt_quat.push_back( i_opt_quat ); opt_t.push_back( i_opt_t )
    void allocate_and_append_new_opt_variable_withpose( const Matrix4d& pose );
    const int n_opt_variables( ) const;
    double * get_raw_ptr_to_opt_variable_q( int i ) const;
    double * get_raw_ptr_to_opt_variable_t( int i ) const;
    bool update_opt_variable_with( int i, const Matrix4d& pose ); //< this will set opt_quad[i] and opt_t[i]. Will return false for invalid i


    // Optimization variables - Loop Edge Switching Constrainsts
    #if defined(___OPT_AS_DOUBLE_STAR)
    double * _opt_switch_ = NULL;
    int _opt_switch_len_ = 0;
    #else
    vector<double*> opt_switch;
    #endif

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



class SixDOFError
{
public:
    SixDOFError( const Matrix4d& _observed__c1_T_c2, const double _weight=1.0 ) : observed__c1_T_c2( _observed__c1_T_c2 )
    {
        observed_c1_q_c2 = Quaterniond( _observed__c1_T_c2.topLeftCorner<3,3>() );
        observed_c1_t_c2 << _observed__c1_T_c2(0,3), _observed__c1_T_c2(1,3), _observed__c1_T_c2(2,3);

        weight = _weight;
    }

    // q1, t1 : w_T_c1
    // q2, t2 : w_T_c2
    template <typename T>
    bool operator() ( const T* const q1, const T* const t1,   const T* const q2, const T* const t2, T* residue_ptr ) const
    {
        // Eigen:
        // Note the order of the arguments: the real w coefficient first,
        // while internally the coefficients are stored in the following order: [x, y, z, w]

        // q1,t1 --> w_T_c1
        Eigen::Map<const Eigen::Matrix<T,3,1> > p_1( t1 );
        Eigen::Map<const Eigen::Quaternion<T> > q_1( q1 );

        // q2,t2 --> w_T_c2
        Eigen::Map<const Eigen::Matrix<T,3,1> > p_2( t2 );
        Eigen::Map<const Eigen::Quaternion<T> > q_2( q2 );

        // relative transforms between the 2 frames
        Quaternion<T> q_1_inverse = q_1.conjugate();
        Quaternion<T> q_12_estimated = q_1_inverse * q_2;
        Matrix<T,3,1> p_12_estimated = q_1_inverse * (p_2 - p_1);

        // compute error between orientations estimates
        Quaternion<T> delta_q = q_12_estimated.conjugate() * observed_c1_q_c2.cast<T>();
        Matrix<T,3,1> delta_t = q_12_estimated.conjugate() * ( observed_c1_t_c2.cast<T>() - p_12_estimated );


        Eigen::Map<Matrix<T,6,1> > residuals( residue_ptr );
        residuals.block(0,0,  3,1) =  delta_t;
        residuals.block(3,0,  3,1) =  T(2.0) * delta_q.vec();


        // Dynamic Covariance Scaling
        T phi = T(5.0);
        // T s = T(2.)*phi / ( phi + residuals.squaredNorm() );
        T s = T(1.0);
        residuals *= s * T(weight);
        return true;

    }


    static ceres::CostFunction* Create( const Matrix4d& _observed__c1_T_c2, const double weight=1.0 )
    {
      return ( new ceres::AutoDiffCostFunction<SixDOFError,6,4,3,4,3>
        (
          new SixDOFError(_observed__c1_T_c2, weight )
        )
      );
    }


private:


    Matrix4d observed__c1_T_c2;
    Quaterniond observed_c1_q_c2;
    Matrix<double,3,1> observed_c1_t_c2;

    double weight;
};


// With ceres you cannot set and then unset parameter blocks as constant. So the get arround
// to that issue is to have pose regularization. The poses that you want to keep as constant
// (for example the 1st pose of the path), just add penalty for its change with a large weight value.
class NodePoseRegularization
{
public:
    NodePoseRegularization( const Matrix4d _nodepose, const double _weight ) : nodepose( _nodepose ), weight( _weight )
    {

    }

    template <typename T>
    bool operator() ( const T* const q1, const T* const t1, T* residue_ptr ) const
    {
        // q1,t1 --> w_T_c1
        Eigen::Map<const Eigen::Matrix<T,3,1> > p_1( t1 );
        Eigen::Map<const Eigen::Quaternion<T> > q_1( q1 );
        Matrix<T,4,4> npose = Matrix<T,4,4>::Identity();
        npose.topLeftCorner(3,3) = q_1.toRotationMatrix();
        npose.col(3).topRows(3) = p_1;

        // nodepose --> T
        Matrix<T,4,4> f = nodepose.cast<T> ();

        Matrix<T,4,4> delta = f.inverse() * npose ;
        Matrix<T,3,3> R = delta.topLeftCorner(3,3);
        Quaternion<T> delta_q( R );


        Eigen::Map<Matrix<T,6,1> > residuals( residue_ptr );
        residuals.block(0,0, 3,1) = T(weight) * delta.col(3).topRows(3);
        residuals.block(3,0, 3,1) = T(weight) * T(2.0) * delta_q.vec();

        return true;
    }

    static ceres::CostFunction* Create( const Matrix4d _observed_node_pose, const double xweight )
    {
      return ( new ceres::AutoDiffCostFunction<NodePoseRegularization,6,4,3>
        (
          new NodePoseRegularization(_observed_node_pose, xweight )
        )
      );
    }

private:
    const Matrix4d nodepose;
    const double weight;
};



class SixDOFErrorWithSwitchingConstraints
{
public:
    SixDOFErrorWithSwitchingConstraints( const Matrix4d& _observed__c1_T_c2, const double _weight=1.0 ) : observed__c1_T_c2( _observed__c1_T_c2 )
    {
        observed_c1_q_c2 = Quaterniond( _observed__c1_T_c2.topLeftCorner<3,3>() );
        observed_c1_t_c2 << _observed__c1_T_c2(0,3), _observed__c1_T_c2(1,3), _observed__c1_T_c2(2,3);

        weight = _weight;
    }

    // q1, t1 : w_T_c1
    // q2, t2 : w_T_c2
    template <typename T>
    bool operator() ( const T* const q1, const T* const t1,
                      const T* const q2, const T* const t2,
                      const T* const switching_var,
                      T* residue_ptr ) const
    {
        // Eigen:
        // Note the order of the arguments: the real w coefficient first,
        // while internally the coefficients are stored in the following order: [x, y, z, w]

        // q1,t1 --> w_T_c1
        Eigen::Map<const Eigen::Matrix<T,3,1> > p_1( t1 );
        Eigen::Map<const Eigen::Quaternion<T> > q_1( q1 );

        // q2,t2 --> w_T_c2
        Eigen::Map<const Eigen::Matrix<T,3,1> > p_2( t2 );
        Eigen::Map<const Eigen::Quaternion<T> > q_2( q2 );

        // relative transforms between the 2 frames
        Quaternion<T> q_1_inverse = q_1.conjugate();
        Quaternion<T> q_12_estimated = q_1_inverse * q_2;
        Matrix<T,3,1> p_12_estimated = q_1_inverse * (p_2 - p_1);

        // compute error between orientations estimates
        Quaternion<T> delta_q = q_12_estimated.conjugate() * observed_c1_q_c2.cast<T>();
        Matrix<T,3,1> delta_t = q_12_estimated.conjugate() * ( observed_c1_t_c2.cast<T>() - p_12_estimated );


        Eigen::Map<Matrix<T,7,1> > residuals( residue_ptr );
        residuals.block(0,0,  3,1) =  delta_t;
        residuals.block(3,0,  3,1) =  T(2.0) * delta_q.vec();
        residuals(6) = T(1.0) * ( T(1.0) - switching_var[0] ); //switching constraint penalty

        // pitch and roll aka residuals(5) and residuals(4) are observable witht he IMU, so
        // increase the penalty for changing those. This will make them to be nearer to initial estimates.
        // residuals(4) *= T(5.);
        // residuals(5) *= T(5.);


        T s = switching_var[0];
        residuals *= s; //* T(weight);
        return true;

    }


    static ceres::CostFunction* Create( const Matrix4d& _observed__c1_T_c2, const double weight=1.0 )
    {
      return ( new ceres::AutoDiffCostFunction<SixDOFErrorWithSwitchingConstraints,7,4,3,4,3,1>
        (
          new SixDOFErrorWithSwitchingConstraints(_observed__c1_T_c2, weight )
        )
      );
    }


private:


    Matrix4d observed__c1_T_c2;
    Quaterniond observed_c1_q_c2;
    Matrix<double,3,1> observed_c1_t_c2;

    double weight;
};
