#include "PoseGraphSLAM.h"

// PoseGraphSLAM::PoseGraphSLAM( const NodeDataManager* _manager ): manager( _manager )
// {
//     solved_until = 0;
// }
PoseGraphSLAM::PoseGraphSLAM( NodeDataManager* _manager ): manager( _manager )
{
    solved_until = 0;

    new_optimize6DOF_disable();
    reinit_ceres_problem_onnewloopedge_optimize6DOF_disable();
    reinit_ceres_problem_onnewloopedge_optimize6DOF_status = -1;


    #if defined(___OPT_AS_DOUBLE_STAR)
    // int n = 3000;
    this->_opt_quat_ = new double [4*30000];
    this->_opt_t_ = new double [3*30000];
    this->_opt_len_ = 0;

    this->_opt_switch_ = new double [5000]; //1 per loop edge
    _opt_switch_len_ = 0;
    #endif

}


//################################################################################
//############## Public Interfaces to retrive optimized poses ####################
//################################################################################



void PoseGraphSLAM::getAllNodePose( vector<Matrix4d>& w_T_ci ) const
{
    w_T_ci.clear();
    for( int i=0 ; i<nNodes() ; i++ )
    {
        w_T_ci.push_back( getNodePose(i) );
    }
}

int PoseGraphSLAM::nNodes() const
{
    std::lock_guard<std::mutex> lk(mutex_opt_vars);
    #if defined(___OPT_AS_DOUBLE_STAR)
    return _opt_len_;
    #else
    return opt_quat.size();
    #endif
}


/* TODO: Removal
bool PoseGraphSLAM::getNodePose( int i, Matrix4d& out_w_T_nodei ) const
{
    std::lock_guard<std::mutex> lk(mutex_opt_vars);

    assert( i < nNodes() );
    if( i<0 || i>= nNodes() )
        return false;

    // qi,ti --> w_T_ci
    Eigen::Map<const Eigen::Quaternion<double> > q_1( opt_quat[i] );
    Eigen::Map<const Eigen::Matrix<double,3,1> > p_1( opt_t[i] );

    out_w_T_nodei = Matrix4d::Identity();
    out_w_T_nodei.topLeftCorner<3,3>() = q_1.toRotationMatrix();
    out_w_T_nodei.col(3).head(3) = p_1;

    return true;
}
*/

// It is on purpose I am returning const Matrix4d and not const Matrix4d&
const Matrix4d PoseGraphSLAM::getNodePose( int i ) const
{

#if defined(___OPT_AS_DOUBLE_STAR)
    std::lock_guard<std::mutex> lk(mutex_opt_vars);
    assert( i>=0 && i<_opt_len_ );
    Matrix4d w_T_cam;
    PoseManipUtils::raw_xyzw_to_eigenmat( (const double*)&_opt_quat_[4*i], (const double*)&_opt_t_[3*i], w_T_cam );
    return w_T_cam;

#else
    std::lock_guard<std::mutex> lk(mutex_opt_vars);
    assert( i>=0 && i <opt_quat.size() );

    Matrix4d w_T_cam;
    PoseManipUtils::raw_xyzw_to_eigenmat( opt_quat[i], opt_t[i], w_T_cam );

    return w_T_cam;
#endif
}

bool PoseGraphSLAM::nodePoseExists( int i ) const //< returns if ith node pose exist
{
#if defined(___OPT_AS_DOUBLE_STAR )
    std::lock_guard<std::mutex> lk(mutex_opt_vars);
    if( i>= 0 && i<_opt_len_ )
        return true;
    return false;
#else
    std::lock_guard<std::mutex> lk(mutex_opt_vars);
    if( i>=0 && i <opt_quat.size() )
        return true;
    return false;
#endif

}


void PoseGraphSLAM::allocate_and_append_new_opt_variable_withpose( const Matrix4d& pose )
{
#if defined(___OPT_AS_DOUBLE_STAR )

    double i_opt_quat[5], i_opt_t[5];
    PoseManipUtils::eigenmat_to_raw_xyzw( pose, (double*)i_opt_quat, (double*)i_opt_t );
    int ppp=nNodes();
    {
        std::lock_guard<std::mutex> lk(mutex_opt_vars);
        _opt_quat_[4*ppp+0] = i_opt_quat[0];
        _opt_quat_[4*ppp+1] = i_opt_quat[1];
        _opt_quat_[4*ppp+2] = i_opt_quat[2];
        _opt_quat_[4*ppp+3] = i_opt_quat[3];

        _opt_t_[3*ppp+0] = i_opt_t[0];
        _opt_t_[3*ppp+1] = i_opt_t[1];
        _opt_t_[3*ppp+2] = i_opt_t[2];
        _opt_len_++;
    }

#else

    // step-1: new i_opt_quat[5], new i_opt_t[5]
    double * i_opt_quat = new double[5];
    double * i_opt_t = new double[5];

    // step-2: pose--> i_opt_quat, i_opt_t
    PoseManipUtils::eigenmat_to_raw_xyzw( pose, i_opt_quat, i_opt_t );

    // step-3: opt_quat.push_back( i_opt_quat ); opt_t.push_back( i_opt_t )
    {
        std::lock_guard<std::mutex> lk(mutex_opt_vars);
        opt_quat.push_back( i_opt_quat );
        opt_t.push_back( i_opt_t );
    }

#endif
}

const int PoseGraphSLAM::n_opt_variables( ) const
{
    std::lock_guard<std::mutex> lk(mutex_opt_vars);
    #if defined(___OPT_AS_DOUBLE_STAR)
    return _opt_len_;
    #else
    return opt_quat.size();
    #endif
}

double * PoseGraphSLAM::get_raw_ptr_to_opt_variable_q( int i ) const
{
    assert( i>=0 && i<n_opt_variables() );
#if defined(___OPT_AS_DOUBLE_STAR)
    return &_opt_quat_[4*i];
#else
    return opt_quat[i];
#endif
}

double * PoseGraphSLAM::get_raw_ptr_to_opt_variable_t( int i ) const
{
    assert( i>=0 && i<n_opt_variables() );
#if defined(___OPT_AS_DOUBLE_STAR)
    return &_opt_t_[3*i];
#else
    return opt_t[i];
#endif
}

bool PoseGraphSLAM::update_opt_variable_with( int i, const Matrix4d& pose ) //< this will set opt_quad[i] and opt_t[i]. Will return false for invalid i
{

#if defined(___OPT_AS_DOUBLE_STAR)
    double i_opt_quat[5], i_opt_t[5];
    PoseManipUtils::eigenmat_to_raw_xyzw( pose, (double*) i_opt_quat,  (double*)i_opt_t );
    if( i>=0 && i<n_opt_variables() )
    {
        std::lock_guard<std::mutex> lk(mutex_opt_vars);
        _opt_quat_[4*i+0] = i_opt_quat[0];
        _opt_quat_[4*i+1] = i_opt_quat[1];
        _opt_quat_[4*i+2] = i_opt_quat[2];
        _opt_quat_[4*i+3] = i_opt_quat[3];

        _opt_t_[3*i+0] = i_opt_t[0];
        _opt_t_[3*i+1] = i_opt_t[1];
        _opt_t_[3*i+2] = i_opt_t[2];
        return true;
    }
    return false;

#else
    double i_opt_quat[5], i_opt_t[5];
    PoseManipUtils::eigenmat_to_raw_xyzw( pose, i_opt_quat, i_opt_t );

    if( i>=0 && i<n_opt_variables() )
    {
        std::lock_guard<std::mutex> lk(mutex_opt_vars);
        for( int k=0 ; k<4 ; k++ )
            (opt_quat[i])[k] = i_opt_quat[k];
        for( int k=0 ; k<3 ; k++ )
            (opt_t[i])[k] = i_opt_t[k];
        return true;
    } else {
        return false;
    }
#endif
}



const int PoseGraphSLAM::n_opt_switch() const {
    #if defined(___OPT_AS_DOUBLE_STAR)
    std::lock_guard<std::mutex> lk(mutex_opt_vars);
    return _opt_switch_len_;
    #else
    std::lock_guard<std::mutex> lk(mutex_opt_vars);
    return opt_switch.size();
    #endif
}

double * PoseGraphSLAM::get_raw_ptr_to_opt_switch( int i ) const {
    #if defined(___OPT_AS_DOUBLE_STAR)
    std::lock_guard<std::mutex> lk(mutex_opt_vars);
    // return &_opt_switch_[i];
    return (i>=0 && i<_opt_switch_len_)?&_opt_switch_[i]:NULL;
    #else
    std::lock_guard<std::mutex> lk(mutex_opt_vars);
    return opt_switch[i];
    #endif
}

void PoseGraphSLAM::allocate_and_append_new_edge_switch_var() {
    #if defined(___OPT_AS_DOUBLE_STAR)
    double tmp[10];
    tmp[0] = 0.99;
    {
        std::lock_guard<std::mutex> lk(mutex_opt_vars);
        _opt_switch_[ _opt_switch_len_ ] = tmp[0];
        _opt_switch_len_++;

    }
    #else
    double *tmp = new double[1];
    tmp[0] = 0.99;
    {
        std::lock_guard<std::mutex> lk(mutex_opt_vars);
        opt_switch.push_back( tmp );
    }
    #endif
}


//######## END Public Interfaces to retrive optimized poses ################


// //TODO: removal of optimize6DOF(), the new_optimize6DOF works and superseeds this function
// #define _DEBUG_LVL_optimize6DOF 1
// void PoseGraphSLAM::optimize6DOF()
// {
//     cout << "#####################################\n";
//     cout << TermColor::BLUE() << "### PoseGraphSLAM::optimize6DOF() ###" << TermColor::RESET() << endl;
//     cout << "### \tinf_loop="<<inf_loop << "###\n"; //if this is set to true (default) will run a infinite loop to monitor the manager.
//     cout << "#####################################\n";
//
//
//
//     int nodesize, old_nodesize; //even though this says size, it is actually indices. Consider renaming. TODO
//     int edgesize, old_edgesize;
//     old_nodesize = 0;//manager->getNodeLen();
//     old_edgesize = 0;//manager->getEdgeLen();
//
//     ceres::Problem problem;
//
//     // Set Solver Options
//     ceres::Solver::Options options;
//     ceres::Solver::Summary summary;
//     // options.linear_solver_type = ceres::SPARSE_SCHUR;
//     options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//
//     options.minimizer_progress_to_stdout = false;
//     options.max_num_iterations = 8;
//
//
//     // ceres::LocalParameterization * quaternion_parameterization = new ceres::QuaternionParameterization;
//     ceres::LocalParameterization * eigenquaternion_parameterization = new ceres::EigenQuaternionParameterization;
//     ceres::LossFunction * loss_function = new ceres::CauchyLoss(1.0);
//
//
//     // There are 3 parts :
//     // while( )
//     //      step-1: Add optimization variables (if new nodes available with manager)
//     //      step-2: Add Odometry Edges (if new available)
//     //      step-3: Add Loop Edges (if new loops available)
//
//     while( ros::ok() )
//     {
//         nodesize = manager->getNodeLen();
//         edgesize = manager->getEdgeLen();
//         // if new nodes are available add odometry edges
//             //continue;
//         if( nodesize > old_nodesize ) //==> new nodes
//         {
//             cout << TermColor::BLUE() << "New nodes (#" << nodesize - old_nodesize << ")" << TermColor::RESET() << endl ;
//
//             ////////////////////////// Add optimization variables ///////////////////////
//             cout << TermColor::BLUE() << "New Optimization Variables for nodes: " << old_nodesize << " to " << nodesize-1 << TermColor::RESET() << endl;
//             for(int u=old_nodesize; u<nodesize ; u++ )
//             {
//                 // cout << ", u="<< u ;
//                 // cout << "old_nodesize="<< old_nodesize << " u="<<u <<";\n";
//                 double * __quat = new double[4];
//                 double * __tran = new double[3];
//
//
//                 // init optimization variables w_T_cam
//                 // if( true )
//                 if( old_nodesize == 0 )
//                 {   // Original code: Set the initial guess as the VIO poses.
//                     // This is note correct if solve() has changed some of the poses in the past. Best is to rely only on relative poses from VIO
//                     Matrix4d w_M_u;
//                     bool status0 = manager->getNodePose(u, w_M_u);
//                     assert( status0 );
//                     // PoseManipUtils::eigenmat_to_raw( w_M_u, __quat, __tran );
//                     PoseManipUtils::eigenmat_to_raw_xyzw( w_M_u, __quat, __tran );
//                 }
//                 else
//                 {
//                     // M : uncorrected poses
//                     // T : corrected poses
//
//                     // Note:
//                     // Say in the previous run 0-248 are optimized and there are new nodes
//                     // from 249 to 290 whose VIO exists but not yet taken into ceres.
//                     // wTM_260 = w_T_248 * 248_M_260
//                     //         = w_T_248 * 248_M_w * w_M_260
//                     //         = w_T_248 * w_M_280.inv() * w_M_260
//
//                     // Getting the relative pose between current last corrected one
//                     Matrix4d w_M_last;
//                     bool status0 = manager->getNodePose( old_nodesize-1, w_M_last );
//
//                     Matrix4d w_M_u;
//                     bool status1 = manager->getNodePose(u, w_M_u);
//
//                     assert( status0 && status1 );
//                     Matrix4d last_M_u = w_M_last.inverse() * w_M_u;
//
//
//                     // Getting pose of last corrected one (in world frame)
//                     Matrix4d w_T_last;
//                     // PoseManipUtils::raw_to_eigenmat( opt_quat[old_nodesize-1], opt_t[old_nodesize-1], w_T_last );
//                     PoseManipUtils::raw_xyzw_to_eigenmat( opt_quat[old_nodesize-1], opt_t[old_nodesize-1], w_T_last );
//
//                     Matrix4d w_TM_u = w_T_last * last_M_u;
//
//                     // PoseManipUtils::eigenmat_to_raw( w_TM_u, __quat, __tran );
//                     PoseManipUtils::eigenmat_to_raw_xyzw( w_TM_u, __quat, __tran );
//
//                 }
//
//
//                 {
//                     std::lock_guard<std::mutex> lk(mutex_opt_vars);
//                     opt_quat.push_back( __quat );
//                     opt_t.push_back( __tran );
//                 }
//
//
//                 problem.AddParameterBlock( __quat, 4 );
//                 // problem.SetParameterization( __quat,  quaternion_parameterization );
//                 problem.SetParameterization( __quat,  eigenquaternion_parameterization );
//                 problem.AddParameterBlock( __tran, 3 );
//             }
//
//
//             //////////////////// Error from Odometry Edges //////////////////////
//             // Add residue blocks for odometry
//             for( int u=old_nodesize; u<nodesize ; u++ )
//             {
//                 #if _DEBUG_LVL_optimize6DOF >= 2
//                 cout << TermColor::RED() << "Odometry Edge: "  << TermColor::RESET();
//                 #endif
//                 for( int f=1 ; f<6 ; f++ )
//                 {
//                     if( u-f < 0 )
//                         continue;
//                     #if _DEBUG_LVL_optimize6DOF >= 2
//                     cout << u << "<-->" << u-f << "    ";
//                     #endif
//
//                     Matrix4d w_M_u, w_M_umf;
//                     bool status0 = manager->getNodePose(u, w_M_u);
//                     bool status1 = manager->getNodePose( u-f, w_M_umf );
//                     assert( status0 && status1 );
//
//                     w_M_umf =  w_M_u.inverse() * w_M_umf ;
//                     ceres::CostFunction * cost_function = SixDOFError::Create( w_M_umf);
//                     problem.AddResidualBlock( cost_function, loss_function, opt_quat[u], opt_t[u],  opt_quat[u-f], opt_t[u-f] );
//
//                     // TODO: Get measure of how accurate was the local odometry. make it a co-variance matrix of size 6x6 and
//                     //       and scale the residue block with the squareroot of the co-variance matrix.
//                 }
//                 #if _DEBUG_LVL_optimize6DOF >= 2
//                 cout << endl;
//                 #endif
//
//                 if( u==0 )
//                 {
//
//                     problem.SetParameterBlockConstant(  opt_quat[0] );
//                     problem.SetParameterBlockConstant(  opt_t[0]  );
//                 }
//
//             }
//         }
//         else
//         {
//             #if _DEBUG_LVL_optimize6DOF >= 3
//             cout << "No new nodes\n";
//             #endif
//         }
//         old_nodesize = nodesize;
//
//
//         ///////////////////// Error from Loop Closure Edges //////////////////////////////
//
//         // if new edges are available add loop edges
//             //solve()
//         if( edgesize > old_edgesize )
//         {
//             cout << TermColor::GREEN() << "New Loop Closure Edges (#" << edgesize - old_edgesize << ")\n" << TermColor::RESET();
//
//             for( int u=old_edgesize ; u<edgesize ; u++ )
//             {
//                 std::pair<int,int> p;
//                 bool status0 = manager->getEdgeIdxInfo( u, p );
//                 Matrix4d pTc;
//                 bool status1 = manager->getEdgePose( u, pTc );
//                 double closure_loop_weight = manager->getEdgeWeight(u);
//                 assert( closure_loop_weight >= 0. && "Loop closure weight need to be non-negative");
//                 // TODO: Get edge weight, make it a co-variance matrix of sixe 6x6 and
//                 //       and scale the residue block with the squareroot of the co-variance matrix.
//
//                 assert( status0 && status1 );
//                 assert( p.first >=0  && p.second >=0 );
//                 assert( p.first < opt_quat.size() );
//                 assert(  p.second < opt_quat.size() );
//
//                 cout << TermColor::GREEN() << "Add Loop Closure "<< u << " : " << p.first << "<-->" << p.second << " | ";
//                 string _tmp;
//                 PoseManipUtils::prettyprintPoseMatrix( pTc, _tmp );
//                 cout << _tmp <<  TermColor::RESET() << endl;
//
//                 ceres::CostFunction * cost_function = SixDOFError::Create( pTc, closure_loop_weight );
//                 problem.AddResidualBlock( cost_function, loss_function, opt_quat[p.first], opt_t[p.first],
//                                                                 opt_quat[p.second], opt_t[p.second]  );
//             }
//
//             cout << "solve()\n";
//             ceres::Solve( options, &problem, &summary );
//             solved_until = nodesize;
//             // cout << summary.FullReport() << endl;
//             cout << summary.BriefReport() << endl;
//             cout << "Solve() took " << summary.total_time_in_seconds << " sec"<< endl;
//             cout << "Poses are Optimized from 0 to "<< old_nodesize << endl;
//             cout << "New nodes in manager which are not yet taken here from idx "<< old_nodesize << " to "<< manager->getNodeLen() << endl;
//         }
//         else
//         {
//             #if _DEBUG_LVL_optimize6DOF >= 3
//             cout << "No new loop closure edges\n";
//             #endif
//         }
//         old_edgesize = edgesize;
//
//         if( inf_loop == false )
//             return;
//
//         std::this_thread::sleep_for( std::chrono::milliseconds(1000) );
//     }
// }


void PoseGraphSLAM::deallocate_optimization_variables()
{
    #if defined(___OPT_AS_DOUBLE_STAR)
    _opt_len_ = 0;
    delete [] _opt_quat_;
    delete [] _opt_t_;
    #else

    std::lock_guard<std::mutex> lk(mutex_opt_vars);
    assert( opt_quat.size() == opt_t.size() );

    for( int i=0 ; i<opt_t.size() ; i++ )
    {
        delete [] opt_quat[i];
        delete [] opt_t[i];
    }
    opt_quat.clear();
    opt_t.clear();

    for( int i=0 ; i<opt_switch.size() ; i++ )
        delete [] opt_switch[i];

    #endif


}





//-------------------------------------------------------------------------------------

void PoseGraphSLAM::init_ceres_optimization_problem()
{
    cout << "[PoseGraphSLAM::init_ceres_optimization_problem]\n";

    // options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 8;


    eigenquaternion_parameterization = new ceres::EigenQuaternionParameterization;
    // robust_norm = new ceres::CauchyLoss(1.0);
    robust_norm = new ceres::HuberLoss(0.1);

}



// #define __PoseGraphSLAM_new_optimize6DOF_odom_debug( msg ) msg;
#define __PoseGraphSLAM_new_optimize6DOF_odom_debug( msg ) ;

#define __PoseGraphSLAM_new_optimize6DOF_odom( msg ) msg;
// #define __PoseGraphSLAM_new_optimize6DOF_odom( msg ) ;

#define __PoseGraphSLAM_new_optimize6DOF_worldcorrection_code 1 // have this to 1 to enable the code, 0 to disable the code

void PoseGraphSLAM::new_optimize6DOF()
{
    cout << "#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#\n";
    cout << TermColor::BLUE() << "Start new_optimize6DOF()" << TermColor::RESET() << endl;
    cout << "#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#\n";

    init_ceres_optimization_problem(); //< Sets all the ceres options, if you need to edit, just go in the function and edit whatever,


    ros::Rate loop_rate(1); // 1 time per sec
    int prev_node_len=0, node_len;
    int prev_loopedge_len=0, loopedge_len;
    solved_until = 1;

    bool marked_previous_nodes_opt_variables_as_constant = false; //< this is used so that I only mark variables as constant 1 time and avoid repeating

    // Whenever kidnap happens, the vins_estimator is restarted. This results in a new co-ordinate frame (new world)
    // for the new incoming poses (after unkidnap)
    // This map stores the estimates of relative transforms between 2 world's. For example map[2,4] will store
    // transform between world-2 and world-4. Look at class Worlds that stores the info on relative poses of 2 worlds


    //////////////////////// Summary of the Loop //////////////////////////////////
    // -1-
    // Are there any new nodes ?
    //      if yes than for each new node
    //            a) add new optimization variables
    //            b) add odometry edges to the optimization problem. Taking care to not add odom edges between 2 different co-ordinate systems (different worlds)
    //
    // -2-
    // Are there any new loopedges,for each new edge e=(a,b) \in E, ?
    //      if yes then
    //              case-a: world_of_a == world_of_b  add loop edges to the optimization problem
    //              case-b: world_of_a != world_of_b
    //                  i) If you already NOT know it, compute relative poses between the worlds. If you know it no action needed
    //
    // -3-
    // Ceres::Solve
    //////////////////////// END Summary of the Loop //////////////////////////////////

    while( new_optimize6DOF_isEnabled )
    {
        cout << "---\n";
        // cout << "[PoseGraphSLAM::new_optimize6DOF]\n";
        // cout << "manager->print_nodes_lengths()\t"; manager->print_nodes_lengths();
        // cout << "PoseGraphSLAM::n_opt_variables = " << this->n_opt_variables() << endl;
        node_len = manager->getNodeLen();
        loopedge_len = manager->getEdgeLen();
        cout << "node_len=" << node_len << "\tloopedge_len=" << loopedge_len << endl;


        // if we got kidnaped, mark the previous world's opt variables as constants
        if( manager->curr_kidnap_status() && marked_previous_nodes_opt_variables_as_constant == false )
        {
            cout << "I think I am kidnapped, mark the previous world's opt variables as constant\n";
            int __start = manager->nodeidx_of_world_i_started( manager->n_worlds() - 1 );
            int __end = manager->nodeidx_of_world_i_ended( manager->n_worlds() - 1 );
            for( int q = __start ; q<=__end ; q++  ) {
                cout << "Mark node#"<< q << " as constant\n";
                problem.SetParameterBlockConstant(  get_raw_ptr_to_opt_variable_q(q) );
                problem.SetParameterBlockConstant(  get_raw_ptr_to_opt_variable_t(q)  );
            }
            marked_previous_nodes_opt_variables_as_constant = true;
        }
        if( !manager->curr_kidnap_status() )
            marked_previous_nodes_opt_variables_as_constant = false;



        //-------------------
        // -1-
        // Are there any new nodes ?
        //      if yes than a) add new optimization variables b) add odometry edges to the optimization problem
        //-------------------
        if( node_len > prev_node_len )
        {
            __PoseGraphSLAM_new_optimize6DOF_odom(
            cout << TermColor::CYAN() << "there are " << node_len - prev_node_len << " new nodes\t";
            cout << " from [" << prev_node_len << ", " << node_len-1 << "]\t" ;
            cout << "t=" << manager->getNodeTimestamp( prev_node_len  ) << " to " << manager->getNodeTimestamp( node_len-1  ) ;
            cout << "  which_world_is_this=" << manager->which_world_is_this( manager->getNodeTimestamp( prev_node_len  ) ) << " to " << manager->which_world_is_this( manager->getNodeTimestamp( node_len-1  ) ) ;
            cout << TermColor::RESET() << endl;
            )

            //###################
            // Add new optimization variables and initialize them correctly.
            //###################
            __PoseGraphSLAM_new_optimize6DOF_odom_debug(
            cout << TermColor::CYAN() << "Add new optimization variables for each node and initialize them correctly. push_back optimization variables for each of these" << TermColor::RESET() << endl;
            cout << "loop from u=" << prev_node_len <<" ; u<" << node_len << " ; u++\n";
            )
            for( int u=prev_node_len ; u<node_len ; u++ ) {
                __PoseGraphSLAM_new_optimize6DOF_odom_debug( cerr << "u=" << u << " " ; )
                if( u==0 ) { //0th node.
                    allocate_and_append_new_opt_variable_withpose( manager->getNodePose(u) );
                }
                else {


                    // M : uncorrected poses
                    // T : corrected poses

                    // Note:
                    // Say in the previous run 0-248 are optimized and there are new nodes
                    // from 249 to 290 whose VIO exists but not yet taken into ceres.
                    // wTM_260 = w_T_248 * 248_M_260
                    //         = w_T_248 * 248_M_w * w_M_260
                    //         = w_T_248 * w_M_280.inv() * w_M_260
                    // Getting the relative pose between current last corrected one
                    Matrix4d w_M_last = manager->getNodePose( (int)this->solvedUntil()-1 );

                    Matrix4d w_M_u = manager->getNodePose(u);
                    Matrix4d last_M_u = w_M_last.inverse() * w_M_u;

                    Matrix4d w_T_last = this->getNodePose( (int)this->solvedUntil()-1 );
                    // Matrix4d w_T_last;
                    // bool status = this->getNodePose(  (int)this->solvedUntil()-1 , w_T_last  );

                    Matrix4d w_TM_u = w_T_last * last_M_u;
                    //       ^^^^ this is in its own world frame.

                    allocate_and_append_new_opt_variable_withpose( w_TM_u );


                    #if __PoseGraphSLAM_new_optimize6DOF_worldcorrection_code
                    // This is the new code added for handling kidnaps #corrected-node-pose-IPSUMLOREM
                    cout << TermColor::iMAGENTA() << "[__PoseGraphSLAM_new_optimize6DOF_worldcorrection_code]\n";
                    int world_of_u = manager->which_world_is_this( manager->getNodeTimestamp( u  ) );
                    int setID_of__world_of_u = manager->getWorldsConstPtr()->find_setID_of_world_i( world_of_u );
                    cout << "world_of_u=" << world_of_u << "\t\tsetID_of__world_of_u=" << setID_of__world_of_u << endl;
                    if( world_of_u >= 0 && setID_of__world_of_u >= 0 ) {
                        if( world_of_u == setID_of__world_of_u ) {
                            ;
                        }
                        else {
                            cout << "Does the pose between world#"<< setID_of__world_of_u << " and world#" << world_of_u << " exists? ";
                            cout << ( (manager->getWorldsConstPtr()->is_exist(setID_of__world_of_u, world_of_u) ) ? "YES":"NO" ) << ", ";

                            if( manager->getWorldsConstPtr()->is_exist(setID_of__world_of_u, world_of_u) ) {
                                cout << "\nI Can initialize this pose in world#" << setID_of__world_of_u << "\n";
                                auto _tmo = manager->getWorldsConstPtr()->getPoseBetweenWorlds(setID_of__world_of_u,world_of_u) * w_TM_u;
                                cout << "w" << setID_of__world_of_u<<  "_T_w" << world_of_u << "=" << PoseManipUtils::prettyprintMatrix4d( manager->getWorldsConstPtr()->getPoseBetweenWorlds(setID_of__world_of_u,world_of_u)  );
                                this->update_opt_variable_with( u, _tmo );

                            }
                            else {
                                cout << "\nthis u is in its own world. no shifying is needed\n";
                            }
                        }
                    }
                    cout << "\n[/__PoseGraphSLAM_new_optimize6DOF_worldcorrection_code]" << TermColor::RESET() << endl;
                    #endif



                }

                __PoseGraphSLAM_new_optimize6DOF_odom_debug( cout << u << "\t" );
                problem.AddParameterBlock( get_raw_ptr_to_opt_variable_q(u), 4 );
                problem.SetParameterization( get_raw_ptr_to_opt_variable_q(u),  eigenquaternion_parameterization );
                problem.AddParameterBlock( get_raw_ptr_to_opt_variable_t(u), 3 );
                // int ____world_of_u =  manager->which_world_is_this( manager->getNodeTimestamp( u  ) );
                // if( u==0 || u == manager->nodeidx_of_world_i_started( ____world_of_u ) ) {
                if( u==0 ) {
                    // problem.SetParameterBlockConstant(  get_raw_ptr_to_opt_variable_q(0) );
                    // problem.SetParameterBlockConstant(  get_raw_ptr_to_opt_variable_t(0)  );
                    // cout << "make the 1st node in this world as constant. world=" << world_of_u << " u=" << u << endl;
                    problem.SetParameterBlockConstant(  get_raw_ptr_to_opt_variable_q(u) );
                    problem.SetParameterBlockConstant(  get_raw_ptr_to_opt_variable_t(u)  );
                }
            }
            cout << "\n";
            __PoseGraphSLAM_new_optimize6DOF_odom_debug( cout << "\\n" << endl );



            //###########################
            // Add odometry edges - residue terms
            //###########################
            __PoseGraphSLAM_new_optimize6DOF_odom_debug(
            cout << TermColor::CYAN() << "Add odometry edges residue terms" << TermColor::RESET() << endl;
            )
            for( int u=prev_node_len ; u<node_len ; u++ ) {
                // add u<-->u-1, add u<-->u-2, add u<-->u-3, add u<-->u-4
                for( int f=1 ; f<=4; f++ ) { // the '4' here is tunable, can try say 5 or 8 or 10.
                    if( u-f < 0 )
                        continue;

                    #if 1
                    // not letting odometry edges between two co-ordinate systems
                    ///////////////// u and u-f various scenarios
                    // (1) ----|        |---um--u  //< OK
                    // (2) ----| um     |--u---    //< don't add odomedge
                    // (3) ----| um  u  |----      //< don't add odomedge
                    // (4) --um--|  u   |----      //< don't add odomedge

                    if( manager->curr_kidnap_status() == false &&
                        manager->getNodeTimestamp( u-f ) >manager->last_kidnap_ended()
                    )
                    {
                        //OK!, ie. add odometry edge
                    }
                    else {
                        // don't add this odometry edge
                        cout << TermColor::iBLUE() << "not adding " << u-f << "<-->" << u << TermColor::RESET() << endl;
                        continue;
                    }
                    // cout << "Add : " << u-f << "<-->" << u << endl;

                    //////////////////
                    #endif

                    // Pose of odometry edge
                    Matrix4d w_M_u = manager->getNodePose(u);
                    Matrix4d w_M_umf = manager->getNodePose(u-f);
                    Matrix4d u_M_umf =  w_M_u.inverse() * w_M_umf ;

                    double odom_edge_weight=1.0;

                    // weight of odometry edge - deminish as it you go farther away 0.7-0.95 is usually a good value
                    odom_edge_weight *= pow(0.9,f);

                    // weight inversely proportional to yaw. More the yaw, less should the weight.
                    // The idea is that, odometry tends to be most error prone on yaws
                    Vector3d __ypr = PoseManipUtils::R2ypr( u_M_umf.topLeftCorner<3,3>() );
                    odom_edge_weight *= exp( -__ypr(0)*__ypr(0)/6. );

                    __PoseGraphSLAM_new_optimize6DOF_odom_debug(
                        cout << u << "<--(" << std::setprecision(3) << odom_edge_weight << ")-->" << u-f << "\t";
                        // cout << "del_yaw = " << __ypr(0) << endl;
                    )


                    ceres::CostFunction * cost_function = SixDOFError::Create( u_M_umf, odom_edge_weight  );
                    problem.AddResidualBlock( cost_function, NULL,
                        get_raw_ptr_to_opt_variable_q(u), get_raw_ptr_to_opt_variable_t(u),
                        get_raw_ptr_to_opt_variable_q(u-f), get_raw_ptr_to_opt_variable_t(u-f) );

                    push_back_odomedge_residue_info( std::make_tuple(u,u-f,odom_edge_weight, "N/A") );
                }
                __PoseGraphSLAM_new_optimize6DOF_odom_debug( cout << "\\n" << endl; )
            }

        }


        //-------------------
        // -2-
        // Are there any new loopedges ?
        //      if yes than add loop edges to the optimization problem
        //-------------------
        if( loopedge_len > prev_loopedge_len )
        {
            cout << TermColor::MAGENTA() << "there are " << loopedge_len - prev_loopedge_len << " new loopedges\t";
            cout << " from [" << prev_loopedge_len << ", " << loopedge_len-1 << "]" << TermColor::RESET() << endl;

            //##############################
            // Add loopedges residues for each edge
            //##############################
            cout <<  TermColor::MAGENTA() << "Add loopedges from [" << prev_loopedge_len << ", " << loopedge_len-1  << "]"<< TermColor::RESET() << endl;
            for( int e=prev_loopedge_len ; e<loopedge_len ; e++ ) {
                MatrixXd bTa = manager->getEdgePose(e); //< This is the edge's observed pose, as received in the edge-message
                double weight = manager->getEdgeWeight(e);
                auto paur = manager->getEdgeIdxInfo(e);

                #if 1
                // Print ordinary info of the edge as it is.
                cout << "\t---\n\t##loopedge e=" << e << "\t";
                cout << "(a,b)==" << paur.first << "<-->" << paur.second << "   weight=" << std::setprecision(4) << weight;
                cout << "\n\tdescription_string=" << manager->getEdgeDescriptionString(e) << "\n";
                cout << "\tbTa(observed)="  << PoseManipUtils::prettyprintMatrix4d(bTa) << endl;;

                // Print world info of the edge
                int __a_world_is = manager->which_world_is_this( manager->getNodeTimestamp( paur.first ) );
                int __b_world_is = manager->which_world_is_this( manager->getNodeTimestamp( paur.second ) );
                if( __b_world_is != __a_world_is ) cout << "\t\t>>>>>><<<<<<==========\n";
                cout << TermColor::iWHITE() ;
                cout << "\ta's world is: "<< __a_world_is
                     << "\t"
                     << "b's world is: " <<  __b_world_is;
                cout << TermColor::RESET() << endl;;
                cout << "world#" << __a_world_is << " is in setID=" << manager->getWorldsPtr()->find_setID_of_world_i( __a_world_is ) << "\t\t" ;
                cout << "world#" << __b_world_is << " is in setID=" << manager->getWorldsPtr()->find_setID_of_world_i( __b_world_is ) << "\n" ;
                // manager->getWorldsPtr()->print_summary();
                #endif



                #if 1
                // code added to take care of kidnaps
                // Move the initial guess of optimization variables if both seem to be from different worlds
                {
                    int _a = paur.first; // current, eg. 386
                    int _b = paur.second; //previous  eg. 165
                    int world_of_a = manager->which_world_is_this( manager->getNodeTimestamp( _a ) );
                    int world_of_b = manager->which_world_is_this( manager->getNodeTimestamp( _b ) );

                    if( world_of_a != world_of_b && world_of_a >=0 && world_of_b >=0 )
                    {
                        // #corrected-node-pose-IPSUMLOREM
                        // the two edge-end-pts are in different worlds.
                        cout << TermColor::BLUE() ;
                        cout << "The two edge-end-pts are in different worlds.\n";


                        if( manager->getWorldsPtr()->is_exist(world_of_b,world_of_a) )
                        {
                            auto rel_wb_T_wa = manager->getWorldsPtr()->getPoseBetweenWorlds( world_of_b, world_of_a );
                            cout << "I already know the relative transforms between the 2 worlds, wa= "<< world_of_a << " ; wb=" << world_of_b << " \n";
                            cout << "[TODO] no action is needed as the initial guesses must be in order by now\n";
                            cout << "rel pose between 2 worlds, wb_T_wa=" << TermColor::iBLUE() << PoseManipUtils::prettyprintMatrix4d(rel_wb_T_wa) << endl;
                        }
                        else {
                            cout << "CURRENT STATUS OF WORLDS (disjoint set) before:\n";
                            manager->getWorldsPtr()->print_summary();
                            cout << "....\n";
                            cout << "I DONOT know the relative transforms between the 2 world, wa= "<< world_of_a << " ; wb=" << world_of_b << " \n";

                            // compute the relative transforms between the 2 worlds
                            Matrix4d wa_T_a = this->getNodePose( _a );
                            Matrix4d wb_T_b = this->getNodePose( _b );
                            Matrix4d b_T_a_observed = manager->getEdgePose(e);

                            Matrix4d wb_T_a = wb_T_b * b_T_a_observed;
                            Matrix4d wb_T_wa = wb_T_a * wa_T_a.inverse();
                            cout << "I just computed the rel pose between 2 worlds, wb_T_wa=" << TermColor::iBLUE() << PoseManipUtils::prettyprintMatrix4d(wb_T_wa) << endl;

                            // set the computed pose into the global (to this thread) data-structure
                            cout << "setting var rel_pose_between_worlds__wb_T_wa[ " << world_of_b << "," << world_of_a << " ] = " << PoseManipUtils::prettyprintMatrix4d(wb_T_wa)  << endl;

                            string info_string = "this pose computed from edge "+ std::to_string(_a) + " <--> " + std::to_string(_b);
                            manager->getWorldsPtr()->setPoseBetweenWorlds( world_of_b, world_of_a, wb_T_wa,  info_string );


                            // set all earlier node poses of [world-a-start, world-a-ends]. Reinitialize the optimization variable.
                            // Those comptimization variables will be in co-ordinate system of world-a.
                            cout << TermColor::RESET() << TermColor::BLUE();
                            cout << "[TODO] Set all earlier node poses (initial gueses) of [world-a-start, world_of_a]\n";
                            cout << "world_of_a=" << world_of_a << "\tworld_of_a starts at nodeidx=" << manager->nodeidx_of_world_i_started(world_of_a) << "\tuntil nodeidx=" << manager->nodeidx_of_world_i_ended(world_of_a) << endl;
                            cout << "world_of_b=" << world_of_b << "\tworld_of_b starts at nodeidx=" << manager->nodeidx_of_world_i_started(world_of_b) << "\tuntil nodeidx=" << manager->nodeidx_of_world_i_ended(world_of_b) << endl;

                            {
                                int __start = manager->nodeidx_of_world_i_started(world_of_a);
                                int __end = manager->nodeidx_of_world_i_ended(world_of_a);
                                for( int q=__start ; q<=__end ; q++ )
                                {
                                    auto wa_T_q = this->getNodePose( q ); // this is wa_T_q
                                    auto __tmo =  wb_T_wa * wa_T_q;
                                    cout << "new_pose = w" << world_of_b << "__T__w" << world_of_a << " x w" << world_of_a << "__T__" << q << endl;
                                    this->update_opt_variable_with( q, __tmo );


                                }
                            }

                            cout << "CURRENT STATUS OF WORLDS (disjoint set) after:\n";
                            manager->getWorldsPtr()->print_summary();
                            cout << "....\n";







                        }

                        cout << TermColor::RESET() << endl;

                    }
                }
                #endif



                // ordinary loop edge residue term
                #if 0
                ceres::CostFunction * cost_function = SixDOFError::Create( bTa, weight );
                problem.AddResidualBlock( cost_function, robust_norm,
                    get_raw_ptr_to_opt_variable_q(paur.second), get_raw_ptr_to_opt_variable_t(paur.second),
                    get_raw_ptr_to_opt_variable_q(paur.first), get_raw_ptr_to_opt_variable_t(paur.first)  );
                #endif


                #if 1
                allocate_and_append_new_edge_switch_var();
                ceres::CostFunction * cost_function = SixDOFErrorWithSwitchingConstraints::Create( bTa, weight );
                problem.AddResidualBlock( cost_function, robust_norm,
                    get_raw_ptr_to_opt_variable_q(paur.second), get_raw_ptr_to_opt_variable_t(paur.second),
                    get_raw_ptr_to_opt_variable_q(paur.first), get_raw_ptr_to_opt_variable_t(paur.first),
                    get_raw_ptr_to_opt_switch(e)
                );
                #endif



                // push_back
                push_back_loopedge_residue_info( std::make_tuple(paur.first,paur.second,weight, "NA1","NA2") );
            }


            #if 1
            cout << TermColor::YELLOW() << "## Loopedges before Solve()\n";
            for( int e=prev_loopedge_len ; e<loopedge_len ; e++ ) {
                auto paur = manager->getEdgeIdxInfo(e);
                int _a = paur.first;
                int _b = paur.second;
                cout << ":::::::::" <<  _a << "<-->" << _b << "\t";
                cout << "edge_switch_variable=" << std::setprecision(4) << get_raw_ptr_to_opt_switch(e)[0] << std::fixed << "\n";

                cout << "wTa(opt_vars) : " <<  PoseManipUtils::prettyprintMatrix4d( this->getNodePose(_a) ) << endl;
                cout << "wTb(opt_vars) : " <<  PoseManipUtils::prettyprintMatrix4d( this->getNodePose(_b) ) << endl;

                auto getEdgePose_after_opt = this->getNodePose(_b).inverse() * this->getNodePose(_a);
                cout << "bTa(opt_vars)" << PoseManipUtils::prettyprintMatrix4d( getEdgePose_after_opt ) << endl;


                auto getEdgePose_after_opt_manager = manager->getNodePose(_b).inverse() * manager->getNodePose(_a);
                cout << std::setprecision(20) << "(manager)a_timestamp=" << manager->getNodeTimestamp( _a ).toSec() << "\t" << "b_timestamp=" << manager->getNodeTimestamp(_b).toSec() << std::fixed <<  endl;
                cout << "bTa(manager,odom)" << PoseManipUtils::prettyprintMatrix4d( getEdgePose_after_opt_manager ) << endl;
            }
            cout << TermColor::RESET() ;
            #endif


            //-------------------------------
            // CERES::SOLVE()
            //-------------------------------
            #if 1
            cout << TermColor::iGREEN() ;
            cout << "solve()\n";
            ElapsedTime timer; timer.tic();
            {
                std::lock_guard<std::mutex> lk(mutex_opt_vars);
                ceres::Solve( options, &problem, &summary );

                cout << "summary.termination_type = "<< (ceres::TerminationType::CONVERGENCE == summary.termination_type) << endl;
                if( summary.termination_type == ceres::TerminationType::CONVERGENCE )
                    solved_until = node_len;
            }
            // cout << summary.FullReport() << endl;

            cout << "Solve() took (milli-sec) : " << timer.toc_milli()  << endl;
            cout << summary.BriefReport() << endl;
            cout << TermColor::RESET() << endl;
            #endif



            #if 1
            // print loopedges after optimization
            cout << TermColor::GREEN() << "## Loopedges after Solve()\n";
            for( int e=prev_loopedge_len ; e<loopedge_len ; e++ ) {
                auto paur = manager->getEdgeIdxInfo(e);
                int _a = paur.first;
                int _b = paur.second;
                cout << ":::::::::" << _a << "<-->" << _b << "\t";
                cout << "edge_switch_variable=" << std::setprecision(4) << get_raw_ptr_to_opt_switch(e)[0] << std::fixed << "\n";

                cout << "wTa(opt_vars) : " <<  PoseManipUtils::prettyprintMatrix4d( this->getNodePose(_a) ) << endl;
                cout << "wTb(opt_vars) : " <<  PoseManipUtils::prettyprintMatrix4d( this->getNodePose(_b) ) << endl;

                auto getEdgePose_after_opt = this->getNodePose(_b).inverse() * this->getNodePose(_a);
                cout << "bTa" << PoseManipUtils::prettyprintMatrix4d( getEdgePose_after_opt ) << endl;
            }
            cout << TermColor::RESET() ;
            #endif

        }


        // book keeping, make sure this executes each time
        prev_node_len = node_len;
        prev_loopedge_len = loopedge_len;
        loop_rate.sleep();
    }

    cout << TermColor::BLUE() << "Done with thread. returning from `PoseGraphSLAM::new_optimize6DOF`" << TermColor::RESET() << endl;


    #if 1
    ///// Next bit of code just prints info


    //// Info on worlds
    cout << TermColor::YELLOW() ;
    cout << "Info on worlds start and end times from NodeDataManager\n";
    cout << "#worlds = " << manager->n_worlds() << endl;
    for( int i=0 ; i<manager->n_worlds() ; i++ ) {
        cout << "world#" << std::setw(2) << i;
        cout << "  start_u=" <<  std::setw(5) << manager->nodeidx_of_world_i_started(i);
        cout << "  end_u  =" <<  std::setw(5) << manager->nodeidx_of_world_i_ended(i);
        cout << "  start_u_stamp=" <<  manager->getNodeTimestamp( manager->nodeidx_of_world_i_started(i) );
        cout << "  end_u_stamp  =" << manager->getNodeTimestamp( manager->nodeidx_of_world_i_ended(i) );
        cout << "  duration=" << manager->getNodeTimestamp( manager->nodeidx_of_world_i_ended(i) ) - manager->getNodeTimestamp( manager->nodeidx_of_world_i_started(i) );
        cout << endl;
    }
    cout << TermColor::RESET() << endl;

    //// Relative transforms between worlds
    manager->getWorldsPtr()->print_summary();


    //// When was I kidnaped
    cout << TermColor::BLUE();
    cout << "Info on Kidnap starts and ends\n";
    cout << "There were a total of " << manager->n_kidnaps() << " kidnaps\n";
    for( int i=0 ; i<manager->n_kidnaps() ; i++ )
    {
        cout << "kidnap#" << std::setw(2)  << i ;
        cout << "\tstart=" << manager->stamp_of_kidnap_i_started(i);
        cout << "\tends =" << manager->stamp_of_kidnap_i_ended(i);
        cout << "\tduration=" << manager->stamp_of_kidnap_i_ended(i) - manager->stamp_of_kidnap_i_started(i) ;
        cout << endl;
    }

    cout << " manager->last_kidnap_started() : "  << manager->last_kidnap_started() << endl;
    cout << " manager->last_kidnap_ended()   : " <<  manager->last_kidnap_ended() << endl;
    cout << TermColor::RESET();




    //// Which world each of the nodes belong to
    cout << "Info on all the Nodes\n";
    int r=0;
    for( int r=0 ; r<manager->getNodeLen() ; r++ )
    {
        ros::Time _t = manager->getNodeTimestamp(r);
        cout << "node#" <<  std::setw(5) << r << " t=" << _t  << " world=" <<  std::setw(3) << manager->which_world_is_this( _t ) ;

        if( r%3 == 0  )
            cout << endl;
        else
            cout << "\t\t";
    }
    cout << endl;

    #endif



}


void PoseGraphSLAM::push_back_odomedge_residue_info( std::tuple<int,int,float,string> m )
{
    std::lock_guard<std::mutex> lk(mutex_residue_info);
    odometry_edges_terms.push_back( m );
}

const std::tuple<int,int,float,string>& PoseGraphSLAM::get_odomedge_residue_info( int i) const
{
    std::lock_guard<std::mutex> lk(mutex_residue_info);
    assert( i>=0 && i<odometry_edges_terms.size() );
    return odometry_edges_terms[i];
}

int PoseGraphSLAM::get_odomedge_residue_info_size() const
{
    std::lock_guard<std::mutex> lk(mutex_residue_info);
    return odometry_edges_terms.size();
}


void PoseGraphSLAM::push_back_loopedge_residue_info( std::tuple<int,int,float,string, string> m )
{
    std::lock_guard<std::mutex> lk(mutex_residue_info);
    loop_edges_terms.push_back( m );
}

const std::tuple<int,int,float,string, string>& PoseGraphSLAM::get_loopedge_residue_info(int i) const
{
    std::lock_guard<std::mutex> lk(mutex_residue_info);
    assert( i>=0 && i<loop_edges_terms.size() );
    return loop_edges_terms[i];

}

int PoseGraphSLAM::get_loopedge_residue_info_size() const
{
    std::lock_guard<std::mutex> lk(mutex_residue_info);
    return loop_edges_terms.size();

}




void PoseGraphSLAM::print_worlds_info( int verbosity )
{

    bool start_ends_u_of_worlds=false, rel_pose_between_worlds=false, kidnap_info=false, which_world_each_node_belong_to=false ;

    switch( verbosity )
    {
        case 0:
        start_ends_u_of_worlds=true;
        rel_pose_between_worlds=false;
        kidnap_info=false;
        which_world_each_node_belong_to=false;
        break;
        case 1:
        start_ends_u_of_worlds=true;
        rel_pose_between_worlds=true;
        kidnap_info=true;
        which_world_each_node_belong_to=false;
        break;
        case 2:
        start_ends_u_of_worlds=true;
        rel_pose_between_worlds=true;
        kidnap_info=true;
        which_world_each_node_belong_to=true;
        break;
        default:
        cout << "[PoseGraphSLAM::print_worlds_info] ERROR invalid verbosity.\n";
        exit(10);
    }

    cout << "---------------------!!!!!  PoseGraphSLAM::print_worlds_info verbosity = " << verbosity << "----------------\n";

    if( start_ends_u_of_worlds ) {
    //// Info on worlds
    cout << TermColor::YELLOW() ;
    cout << "Info on worlds start and end times from NodeDataManager\n";
    cout << "#worlds = " << manager->n_worlds()  << "\t";
    cout << "\tWorlds::n_worlds = " << manager->getWorldsConstPtr()->n_worlds() << "\t";
    cout << "\tWorlds::n_sets = " << manager->getWorldsConstPtr()->n_sets() << "\t";
    cout << endl;
    for( int i=0 ; i<manager->n_worlds() ; i++ ) {
        cout << "world#" << std::setw(2) << i;
        cout << "  start_u=" <<  std::setw(5) << manager->nodeidx_of_world_i_started(i);
        cout << "  end_u=" <<  std::setw(5) << manager->nodeidx_of_world_i_ended(i);
        // cout << "  startstamp=" <<  manager->getNodeTimestamp( manager->nodeidx_of_world_i_started(i) );
        // cout << "  endstamp  =" << manager->getNodeTimestamp( manager->nodeidx_of_world_i_ended(i) );
        cout << " (" <<  manager->getNodeTimestamp( manager->nodeidx_of_world_i_started(i) );
        cout << " to " << manager->getNodeTimestamp( manager->nodeidx_of_world_i_ended(i) ) << ")";
        cout << "  sec=" << manager->getNodeTimestamp( manager->nodeidx_of_world_i_ended(i) ) - manager->getNodeTimestamp( manager->nodeidx_of_world_i_started(i) );
        cout << "  setID=" << std::setw(2) << manager->getWorldsConstPtr()->find_setID_of_world_i( i );
        cout << endl;
    }
    cout << TermColor::RESET() << endl;
    }

    if( rel_pose_between_worlds ) {
    //// Relative transforms between worlds
    manager->getWorldsPtr()->print_summary(2);
    } else {
    manager->getWorldsPtr()->print_summary(0);
    }


    if( kidnap_info ) {
    //// When was I kidnaped
    cout << TermColor::BLUE();
    cout << "Info on Kidnap starts and ends\n";
    cout << "There were a total of " << manager->n_kidnaps() << " kidnaps\n";
    for( int i=0 ; i<manager->n_kidnaps() ; i++ )
    {
        cout << "kidnap#" << std::setw(2)  << i ;
        cout << "\tstart=" << manager->stamp_of_kidnap_i_started(i);
        cout << "\tends =" << manager->stamp_of_kidnap_i_ended(i);
        cout << "\tduration=" << manager->stamp_of_kidnap_i_ended(i) - manager->stamp_of_kidnap_i_started(i) ;
        cout << endl;
    }

    cout << " manager->last_kidnap_started() : "  << manager->last_kidnap_started() << endl;
    cout << " manager->last_kidnap_ended()   : " <<  manager->last_kidnap_ended() << endl;
    cout << TermColor::RESET();
    }



    if( which_world_each_node_belong_to ) {
    //// Which world each of the nodes belong to
    cout << "Info on all the Nodes\n";
    int r=0;
    for( int r=0 ; r<manager->getNodeLen() ; r++ )
    {
        ros::Time _t = manager->getNodeTimestamp(r);
        cout << "node#" <<  std::setw(5) << r << " t=" << _t  << " world=" <<  std::setw(3) << manager->which_world_is_this( _t ) ;

        if( r%3 == 0  )
            cout << endl;
        else
            cout << "\t\t";
    }
    cout << endl;
    }
}



bool PoseGraphSLAM::saveAsJSON(const string base_path)
{
    json all_info;
    const static IOFormat CSVFormat(FullPrecision, DontAlignCols, ",", ";");

    all_info["meta_data"]["nNodes"] = nNodes();


    for( int i=0 ; i<nNodes() ; i++ )
    {
        json opt_var_i;

        // Save Optimed pose
        Matrix4d w_T_c_opt = getNodePose( i );
        // getNodePose( i, w_T_c_opt );

        std::stringstream ss;
        ss << w_T_c_opt.format(CSVFormat);
        opt_var_i["wTc_opt"] = ss.str();
        opt_var_i["wTc_opt_prettyprint"] = PoseManipUtils::prettyprintMatrix4d( w_T_c_opt );


        // save corresponding odom poses
        Matrix4d w_T_c_odom;
        manager->getNodePose( i, w_T_c_odom );
        std::stringstream ss2;
        ss2 << w_T_c_odom.format(CSVFormat);
        opt_var_i["w_T_c_odom"] = ss2.str();
        opt_var_i["w_T_c_odom_prettyprint"] = PoseManipUtils::prettyprintMatrix4d( w_T_c_odom );
        opt_var_i["node_i"] = i;

        all_info["PoseGraphSLAM_nodes"].push_back( opt_var_i );
    }


    // loop edges
    for( int i=0 ; i<manager->getEdgeLen() ; i++ ) {
        json edgeinfo;

        // bTa with odometry
        edgeinfo["getEdge_i"] = i;
        int a = manager->getEdgeIdxInfo( i ).first;
        int b = manager->getEdgeIdxInfo( i ).second;
        edgeinfo["a"] = a;
        edgeinfo["b"] = b;

        edgeinfo["world_of_a"] = manager->which_world_is_this( manager->getNodeTimestamp(a) );
        edgeinfo["world_of_b"] = manager->which_world_is_this( manager->getNodeTimestamp(b) );


        edgeinfo["weight"] = manager->getEdgeWeight(i);
        edgeinfo["description_string"] = manager->getEdgeDescriptionString(i);

        edgeinfo["getEdgePose"] = PoseManipUtils::prettyprintMatrix4d( manager->getEdgePose( i ) );

        auto getEdgePose_after_opt = this->getNodePose(b).inverse() * this->getNodePose(a);
        edgeinfo["getEdgePose_after_opt"] = PoseManipUtils::prettyprintMatrix4d( getEdgePose_after_opt );


        // switching_var
        if( i < n_opt_switch() )
            edgeinfo["switching_var_after_opt"] = get_raw_ptr_to_opt_switch(i)[0];

        all_info["PoseGraphSLAM_loopedgeinfo"].push_back( edgeinfo );

    }


    #if 0
    // odometry edges
    for( int i=0 ; i<this->get_odomedge_residue_info_size() ; i++ ) {
        json odom_edge_info;

        auto m = this->get_odomedge_residue_info(i);
        odom_edge_info["u"] = std::get<0>(m);
        odom_edge_info["u-f"] = std::get<1>(m);
        odom_edge_info["weight"] = std::get<2>(m);
        odom_edge_info["debug_string"] = std::get<3>(m);
        all_info["PoseGraphSLAM_odom_edge_info"].push_back( odom_edge_info );
    }
    #endif



    cout << TermColor::GREEN() << "[PoseGraphSLAM::saveAsJSON]\n" <<  all_info["meta_data"] << endl;
    cout << "Write : "<< base_path+"/log_optimized_poses.json"<< endl;
    cout << "Done! "<< TermColor::RESET() << endl;
    std::ofstream outf(base_path+"/log_optimized_poses.json");
    if( !outf.is_open() ) {
        cout << "[ERROR PoseGraphSLAM::saveAsJSON] Cannot open file\n";
        return false;
    }

    outf << std::setw(4) << all_info << std::endl;
    return true;

}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//---  This is the newer (Feb22 2019) implementation of `new_optimize6DOF`.---
//---  It is an infinite loop and triggers the solve when there are new    ---
//---  loop edges in the manager.                                          ---
//----------------------------------------------------------------------------
// #define __reint_allocation_cout(msg)  msg;
#define __reint_allocation_cout(msg)  ;

// #define __reint_odom_cout(msg) msg;
#define __reint_odom_cout(msg) ;

#define __reinit_loopedge_cout( msg ) msg;
// #define __reinit_loopedge_cout( msg ) ;


#define __reint_gueses_short_info(msg) msg;
// #define __reint_gueses_short_info(msg) ;


// Print info on node regularization. I use node regularization to set
// the start of each worlds as constant. You can tune this as need be.
// The reason I do not use ceres::SetParameterBlockConstant() is that it cannot be
// set to not constant which hinders when the initial guess moves.
#define __reint_node_regularization_info( msg ) msg;
// #define __reint_node_regularization_info( msg ) ;


// Print Ceres Brief Report and other info on solving
#define __reint_ceres_solve_info( msg ) msg;
// #define __reint_ceres_solve_info( msg ) ;



// This thread wakes up when there are new loopedges. This flag controls the printing on trigger.
// When debugging you want to keep the headers 'on'.
#define ___trigger_header( msg ) msg;
// #define ___trigger_header( msg ) ;

void PoseGraphSLAM::reinit_ceres_problem_onnewloopedge_optimize6DOF()
{
    cout << "#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#\n";
    cout << TermColor::BLUE() << "Start reinit_ceres_problem_onnewloopedge_optimize6DOF()" << TermColor::RESET() << endl;
    cout << "#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#\n";

    ros::Rate loop_rate(0.5);
    int prev_loopedge_len = 0;
    int prev_node_len = 0;
    ElapsedTime eta;
    int trigger_id = 0;
    int n_solve_convergences = 0;

    //-----------------------
    //-0- INIT CERES Problem
    //-----------------------
    ceres::Problem reint_problem;
    ceres::Solver::Options reint_options;
    ceres::Solver::Summary reint_summary;
    reint_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    reint_options.minimizer_progress_to_stdout = false;
    reint_options.max_num_iterations = 10;
    // reint_options.enable_fast_removal = true;
    eigenquaternion_parameterization = new ceres::EigenQuaternionParameterization;
    // robust_norm = new ceres::CauchyLoss(1.0);
    robust_norm = new ceres::HuberLoss(0.1);

    //==> key:= worldID, value:= (prev setID of this world, new setID of this world).
    std::map< int , std::tuple<int,int> > changes_to_setid_on_set_union;
    vector<ceres::ResidualBlockId> regularization_terms_list;


    while( reinit_ceres_problem_onnewloopedge_optimize6DOF_isEnabled )
    {
        //--- Header info at start of every wakeup
        int node_len = manager->getNodeLen();
        int loopedge_len = manager->getEdgeLen();

        ___trigger_header(
        // cout << "---\n";
        // cout << "node_len=" << node_len << "\tloopedge_len=" << loopedge_len << "\t\t";
        // cout << "prev_node_len=" << prev_node_len << "\tprev_loopedge_len=" << prev_loopedge_len << endl;
        // cout << "\tso total_new_loopedges=" << loopedge_len - prev_loopedge_len << endl;

        cout << "---";
        cout << "node_len=(" << prev_node_len << "  to  " << node_len << ")\t";
        cout << "loopedge_len=(" << prev_loopedge_len << " to " << prev_loopedge_len << ")\t";
        cout << endl;
        )

        //--- If no new loop edges sleep again!
        if( prev_loopedge_len == loopedge_len ) {
            prev_loopedge_len = loopedge_len;
            ___trigger_header( cout << "No new loop edge, sleep again!\n"; )
            reinit_ceres_problem_onnewloopedge_optimize6DOF_status = 0;
            loop_rate.sleep();
            continue;
        }

        if( manager->curr_kidnap_status() ) {
            cout << "kidnapped!. sleep \n";
            reinit_ceres_problem_onnewloopedge_optimize6DOF_status = 0;
            loop_rate.sleep();
            continue;
        }

        ___trigger_header(
        cout << TermColor::iMAGENTA() << "#@#@#@++#@#@#@++\n#@#@#@++ TRIGGERED " << trigger_id++ << "#@#@#@++\n#@#@#@++#@#@#@++\n" << TermColor::RESET();
        print_worlds_info( 0 );
        cout << "<=========== n_opt_variables() =========>" << n_opt_variables() << endl;
        )


        //--- Create and Solve the problem
        {
        reinit_ceres_problem_onnewloopedge_optimize6DOF_status = 1;



        //---------------------------------
        // -0- Allocate new opt variables (if need be)
        //----------------------------------

        int __new_node_allocations = 0;
        ___trigger_header( cout << "\n[New Opt Variables Allocation starts] for( yp=n_opt_variables()=" << n_opt_variables()<< " ; yp< node_len="<< node_len <<  " ; yp++) "<< endl; )
        for( int yp=n_opt_variables() ; yp< node_len ; yp++ ) {
            allocate_and_append_new_opt_variable_withpose( Matrix4d::Identity() );
            __reint_allocation_cout( cout << "++  allocate_and_append_new_opt_variable_withpose, after allocation size({u})=" << n_opt_variables() << endl; )
            __new_node_allocations++;

            // [IMPORTANT] : remeber to specify (to ceres) the parameter blocks and their parameterization.
            reint_problem.AddParameterBlock( get_raw_ptr_to_opt_variable_q(yp), 4 );
            reint_problem.SetParameterization( get_raw_ptr_to_opt_variable_q(yp),  eigenquaternion_parameterization );
            reint_problem.AddParameterBlock( get_raw_ptr_to_opt_variable_t(yp), 3 );
        }

        int __new_opt_switch_allocations = 0;
        ___trigger_header( cout << "\n[New Switch Variables per edges Allocation starts] for( yp=n_opt_switch()=" << n_opt_switch() << " ; yp< loopedge_len="<< loopedge_len << " ; yp++ )\n"; )
        for( int yp=n_opt_switch() ; yp< loopedge_len ; yp++ ) {
            allocate_and_append_new_edge_switch_var();
            __reint_allocation_cout( cout << "+++ allocate_and_append_new_edge_switch_var, after allocation size({u})=" << n_opt_switch() << endl; )
            __new_opt_switch_allocations++;

            // [IMPORTANT] : remeber to specify (to ceres) the parameter blocks and their parameterization.
            reint_problem.AddParameterBlock( get_raw_ptr_to_opt_switch(yp), 1 );

        }

        ___trigger_header( cout << "[ETA New Allocations] __new_node_allocations="<<__new_node_allocations << "  " << "__new_opt_switch_allocations=" << __new_opt_switch_allocations << endl; )


        //-----------------------
        //-1- Loop Edges (intra world)
        //-----------------------
        //-----------------------
        //-2- Loop edges (inter world)
        //-----------------------
        eta.tic();
        ___trigger_header( cout << "\n[Loop Edges Residues starts] for( e=" << prev_loopedge_len <<"; e< "<< loopedge_len << "; e++) " << endl; )
        // for( int e=0 ; e<loopedge_len ; e++ ) {
        for( int e=prev_loopedge_len ; e<loopedge_len ; e++ ) {
            MatrixXd bTa = manager->getEdgePose(e); //< This is the edge's observed pose, as received in the edge-message
            double weight = manager->getEdgeWeight(e);
            auto paur = manager->getEdgeIdxInfo(e);
            int _a = paur.first; // current, eg. 386
            int _b = paur.second; //previous  eg. 165
            int __a_world_is = manager->which_world_is_this( manager->getNodeTimestamp( paur.first ) );
            int __b_world_is = manager->which_world_is_this( manager->getNodeTimestamp( paur.second ) );
            if( __a_world_is < 0 || __b_world_is < 0 )
                continue; // skip if the edge's 1 node lies in dead-zone
            int __setid_of_world_A = manager->getWorldsPtr()->find_setID_of_world_i( __a_world_is );
            int __setid_of_world_B = manager->getWorldsPtr()->find_setID_of_world_i( __b_world_is );


            bool s1 = (__a_world_is == __b_world_is );
            bool s2 = (__setid_of_world_A == __setid_of_world_B );

            __reinit_loopedge_cout(
            if( s1 && s2 ) cout << "";
            if( !s1 && s2 ) cout << TermColor::CYAN();
            if( s1 && !s2 ) cout << TermColor::BLUE();
            if( !s1 && !s2 ) cout << TermColor::MAGENTA();
            )

            __reinit_loopedge_cout(
            cout << "#e=" << e;
            cout << "\t(a,b)==" << paur.first << "<-->" << paur.second << "   weight=" << std::setprecision(3) << weight;
            cout << " bTa(observed)="  << PoseManipUtils::prettyprintMatrix4d(bTa);
            // cout << "\n\tdescription_string=" << manager->getEdgeDescriptionString(e) ;
            cout << endl;

            cout << "\ta's world is: " << __a_world_is;
            cout << "\tb's world is: " <<  __b_world_is;
            cout << "\tworld#" << __a_world_is << " is in setID=" << __setid_of_world_A << "\t" ;
            cout <<   "world#" << __b_world_is << " is in setID=" << __setid_of_world_B << "\n" ;
            // manager->getWorldsPtr()->print_summary();
            cout << TermColor::RESET();
            )

            if(  __a_world_is < 0 || __b_world_is < 0 ) {
                __reinit_loopedge_cout( cout << "either of the worlds is a kidnapped world (negative id). ignore this edge\n"; )
                continue;
            }

            if( __a_world_is == __b_world_is ) // Intra World
            {
                __reinit_loopedge_cout( cout << "This is intra-world loop edge.\n"; )
            }
            else { //Inter World
                // calculate the relative transforms between the two worlds if you already do not know.
                // If you know the transform between the 2 worlds, no need to recompute.

                if( manager->getWorldsPtr()->is_exist(__b_world_is,__a_world_is) ) {
                    auto rel_wb_T_wa = manager->getWorldsPtr()->getPoseBetweenWorlds( __b_world_is, __a_world_is );
                    __reinit_loopedge_cout(
                    cout << "I already know the relative transforms between the 2 worlds, ";
                    // cout << "wa="<< __a_world_is << ";wb=" << __b_world_is;
                    cout << " wb_T_wa=" << TermColor::iBLUE() << PoseManipUtils::prettyprintMatrix4d(rel_wb_T_wa) << TermColor::RESET() << endl;
                    )
                }
                else {

                    __reinit_loopedge_cout(
                    cout << TermColor::iWHITE() << "I DONOT know the relative transforms between the 2 world, wa= "<< __a_world_is << " ; wb=" << __b_world_is << " \n" << TermColor::RESET();

                    )
                    // compute the relative transforms between the 2 worlds
                    Matrix4d wa_T_a = manager->getNodePose( _a );
                    Matrix4d wb_T_b = manager->getNodePose( _b );
                    Matrix4d b_T_a_observed = manager->getEdgePose(e);

                    Matrix4d wb_T_a = wb_T_b * b_T_a_observed;
                    Matrix4d wb_T_wa = wb_T_a * wa_T_a.inverse();
                    __reinit_loopedge_cout(
                    cout << "I just computed the rel pose between 2 worlds, wb_T_wa=" << TermColor::iBLUE() << PoseManipUtils::prettyprintMatrix4d(wb_T_wa) << TermColor::RESET() << endl;
                    )
                    // Done...!


                    // The following bit of code, records the setID states before `setPoseBetweenWorlds` and after `setPoseBetweenWorlds`
                    // This is done so that we know which world's base changed. This is used later to initialize/reinitialize the guesses for optimization

                    //---- get setID (before)
                    std::map<int,int> mipmap_before_union, mipmap_after_union;
                    __reinit_loopedge_cout(
                    cout << "CURRENT STATUS OF WORLDS (disjoint set) before:\n";
                    manager->getWorldsPtr()->print_summary();
                    )
                    manager->getWorldsPtr()->getWorld2SetIDMap( mipmap_before_union );



                    //---- setPoseBetweenWorlds()
                    __reinit_loopedge_cout(
                    cout << "setting var rel_pose_between_worlds__wb_T_wa[ " << __b_world_is << "," << __a_world_is << " ] = " << PoseManipUtils::prettyprintMatrix4d(wb_T_wa)  << endl;
                    )
                    string info_string = "this pose computed from edge "+ std::to_string(_a) + " <--> " + std::to_string(_b);
                    // THIS IS THE ONLY PLACE WHERE THE 2 SETS CAN MERGE AND THE SETID OF THE WORLDS CAN CHANGE.
                    manager->getWorldsPtr()->setPoseBetweenWorlds( __b_world_is, __a_world_is, wb_T_wa,  info_string );




                    //---- get setID (after)
                    __reinit_loopedge_cout(
                    cout << "CURRENT STATUS OF WORLDS (disjoint set) after:\n";
                    manager->getWorldsPtr()->print_summary();
                    )
                    manager->getWorldsPtr()->getWorld2SetIDMap( mipmap_after_union );



                    //---- findout the changes from `mipmap_before_union` to `mipmap_after_union`
                    __reinit_loopedge_cout(
                    cout << TermColor::iWHITE();
                    cout << "Changes in the setID of the worlds\n";
                    )
                    changes_to_setid_on_set_union.clear();
                    for( auto itt=mipmap_before_union.begin() ; itt!=mipmap_before_union.end() ; itt++ ) {
                        int setid_before =  itt->second;
                        int setid_after = mipmap_after_union.at( itt->first );
                        __reinit_loopedge_cout(
                            cout << "world#" << itt->first  << ": " << setid_before << "--->" << setid_after << "  ";
                        )
                        if( setid_before ==  setid_after ) {
                            __reinit_loopedge_cout( cout << "NOCHANGE\n"; )
                        }
                        else {
                            __reinit_loopedge_cout( cout << "CHANGE\n"; )
                            changes_to_setid_on_set_union[ itt->first ] = std::make_tuple( setid_before, setid_after );
                        }
                    }
                    __reinit_loopedge_cout( cout << TermColor::iWHITE() << "....\n" << TermColor::RESET(); )




                }

            }

            // simply add edge residue
            ceres::CostFunction * cost_function = SixDOFErrorWithSwitchingConstraints::Create( bTa, weight );
            reint_problem.AddResidualBlock( cost_function, NULL,
                get_raw_ptr_to_opt_variable_q(paur.second), get_raw_ptr_to_opt_variable_t(paur.second),
                get_raw_ptr_to_opt_variable_q(paur.first), get_raw_ptr_to_opt_variable_t(paur.first),
                get_raw_ptr_to_opt_switch(e)
            );

        }
        ___trigger_header( cout << "[ETA loopedge_len] ms=" << eta.toc_milli() << " loopedge_len="<< loopedge_len << endl; )


        //-------------------------------
        //-3- Odometry Residues
        //-------------------------------
        eta.tic();
        ___trigger_header( cout << "\n[Add Odom Residues starts] for( u=solvedUntil()+1=" << solvedUntil()+1 <<"; u< "<< node_len << "; u++) " << endl; )
        // for( int u=0 ; u<node_len ; u++ )
        // for( int u=prev_node_len ; u<node_len ; u++ ) // No!!!
        for( int u=solvedUntil()+1 ; u<node_len ; u++ )
        {
            __reint_odom_cout( cout << "---u=" << u << "\t"; )
            int world_of_u = manager->which_world_is_this( manager->getNodeTimestamp(u) );
            int setID_of__world_of_u = manager->getWorldsConstPtr()->find_setID_of_world_i( world_of_u );

            // --------------> add odometry residue :  u <---> u-f
            for( int f=1 ; f<5 ; f++ ){
                int world_of_u_m_f=-1;
                if( u-f >= 0 )
                    world_of_u_m_f = manager->which_world_is_this( manager->getNodeTimestamp(u-f) );
                int setID_of__world_of_u_m_f = manager->getWorldsConstPtr()->find_setID_of_world_i( world_of_u_m_f );

                if( setID_of__world_of_u < 0 || setID_of__world_of_u_m_f < 0 ) {
                    __reint_odom_cout( cout << TermColor::BLUE() << "not add odom coz u is in deadzone "<< u << "<--->"<< u-f << "  \t" << TermColor::RESET(); )
                    continue;
                }

                if( u-f < 0 ) {
                    __reint_odom_cout( cout << TermColor::BLUE() << "not add odom coz u-f is in deadzone "<< u << "<--->"<< u-f << "  \t" << TermColor::RESET() ; )
                    continue;
                }


                __reint_odom_cout( cout << TermColor::GREEN() << "add edge " << u << "<--->" << u-f << "\t" << TermColor::RESET(); )

                // Pose of odometry edge
                Matrix4d w_M_u = manager->getNodePose(u);
                Matrix4d w_M_umf = manager->getNodePose(u-f);
                Matrix4d u_M_umf =  w_M_u.inverse() * w_M_umf ;

                // weight inversely proportional to yaw. More the yaw, less should the weight.
                // The idea is that, odometry tends to be most error prone on yaws
                double odom_edge_weight=1.0;
                odom_edge_weight *= pow(0.9,f);
                Vector3d __ypr = PoseManipUtils::R2ypr( u_M_umf.topLeftCorner<3,3>() );
                odom_edge_weight *= exp( -__ypr(0)*__ypr(0)/6. );

                // cost
                ceres::CostFunction * cost_function = SixDOFError::Create( u_M_umf, odom_edge_weight  );
                reint_problem.AddResidualBlock( cost_function, NULL,
                    get_raw_ptr_to_opt_variable_q(u), get_raw_ptr_to_opt_variable_t(u),
                    get_raw_ptr_to_opt_variable_q(u-f), get_raw_ptr_to_opt_variable_t(u-f) );


            }
            __reint_odom_cout( cout << endl; )
        }
        ___trigger_header( cout << "[ETA odometry residues] done in milli-secs=" << eta.toc_milli() << " node_len="<< node_len << endl; )




        //-----------------------
        //-4- Initial Guesses for the nodes
        //-----------------------
        eta.tic();
        int ____solvedUntil = solvedUntil();
        int ____solvedUntil_worldid = manager->which_world_is_this( manager->getNodeTimestamp(____solvedUntil) );
        bool ____solvedUntil_worldid_is_neg = false;
        if( ____solvedUntil_worldid < 0 ) { ____solvedUntil_worldid = -____solvedUntil_worldid -1; ____solvedUntil_worldid_is_neg=true;}
        ___trigger_header(
        cout << "\n[Set initial guesses of the nodes] for( u=" << 0 << " ; u<" << node_len << " ; u++ ) ";
        cout << "\tsolvedUntil="<< ____solvedUntil << "  ____solvedUntil_worldid=" << ____solvedUntil_worldid <<  endl;
        )
        for( int u=0 ; u<node_len ; u++ ) //TODO: No need to initialize the guess for all the nodes. only the unsolved and the setID changed nodes need to be reinitialized.
        {
            //
            //########## Initial Bookkeeping. worldID, setID, skip kidnapped
            int world_of_u = manager->which_world_is_this( manager->getNodeTimestamp(u) );
            int setID_of__world_of_u = manager->getWorldsConstPtr()->find_setID_of_world_i( world_of_u );


            if( setID_of__world_of_u < 0 ){ // this indicates kidnapped nodes.
                __reint_gueses_short_info( cout << TermColor::iBLUE() << "|" << TermColor::RESET(); )
                continue;
            }

            //
            //########### relative pose between odometry's world-origin and setID's origin
            Matrix4d wset_T_w = Matrix4d::Identity(); //< Pose between the world and its iIDworld;
            if( setID_of__world_of_u != world_of_u ) {
                if( manager->getWorldsConstPtr()->is_exist(setID_of__world_of_u, world_of_u) ) {
                    wset_T_w = manager->getWorldsConstPtr()->getPoseBetweenWorlds( setID_of__world_of_u, world_of_u );
                }
                else {
                    // this is impossible.
                    cout << TermColor::RED() << "manager->getWorldsConstPtr()->is_exist("<<setID_of__world_of_u<<","<< world_of_u<<") gave false. This cannot be happening\n" << TermColor::RESET();
                    exit(3);
                }
            } // will not get the pose if worldid is same as setID.



            //
            //############### --------> Set the initial guess of node pose (in this own world)
            Matrix4d wTu ; //< pose of this node in its own world.

            // When to skip the update
            bool _before_solveduntil = false;
            bool _in_change_set = false;
            if( u<=____solvedUntil )
                _before_solveduntil = true;

            if( changes_to_setid_on_set_union.count(world_of_u) > 0 ) {
                _in_change_set = true;
            __reint_gueses_short_info( cout << (_in_change_set?"T":"F") << (_before_solveduntil?"T":"F") << "."; )
            }



            // The 4 conditions on `_before_solveduntil` , `_in_change_set`
            if( _in_change_set &&  _before_solveduntil ) {
                // this is a challenging case. needs careful implementation
                __reint_gueses_short_info( cout << TermColor::iRED() << "|" << TermColor::RESET(); )

                if( setID_of__world_of_u == ____solvedUntil_worldid ) {
                    cout << "Adifficult case 8. impossible\n";
                    exit(8);
                }else {
                    // cout << "Bdifficult case 8\n";
                    int old_setid = std::get<0>( changes_to_setid_on_set_union[world_of_u] );
                    int new_setid = std::get<1>( changes_to_setid_on_set_union[world_of_u] );
                    // cout << "is_exisit_"<< new_setid << "-->"<< old_setid << " "<< manager->getWorldsConstPtr()->is_exist( new_setid, old_setid ) << endl;

                    Matrix4d wsetnew_T_wsetold = manager->getWorldsConstPtr()->getPoseBetweenWorlds( new_setid, old_setid );
                    Matrix4d wsetnew_T_u = wsetnew_T_wsetold * this->getNodePose(u);
                    __reint_gueses_short_info(
                    cout << TermColor::iRED()  << new_setid << "_T_" << old_setid << "*" << "P_" << u << TermColor::RESET() ;
                    )
                    update_opt_variable_with( u, wsetnew_T_u );
                    // exit(8);
                }

            } else if( _in_change_set &&  !_before_solveduntil ) {

                __reint_gueses_short_info( cout << TermColor::RED(); )
                if( ____solvedUntil_worldid == world_of_u ) {
                    Matrix4d w_M_last = manager->getNodePose( ____solvedUntil );
                    Matrix4d w_M_u = manager->getNodePose(u);
                    Matrix4d last_M_u = w_M_last.inverse() * w_M_u;
                    Matrix4d w_T_last;
                    if( this->nodePoseExists(____solvedUntil ) ) {
                        w_T_last = this->getNodePose( ____solvedUntil );
                    }
                    else {
                        __reint_gueses_short_info( cout << "HU"; )
                    }

                    Matrix4d w_TM_u = w_T_last * last_M_u;
                    // cout << "A";
                    __reint_gueses_short_info( cout << "P_" << ____solvedUntil << "*" << ____solvedUntil << "_odmT_" << u << ","; )
                    update_opt_variable_with( u, w_TM_u );
                } else {
                    Matrix4d wset_T_u = wset_T_w * manager->getNodePose(u);
                    // cout << "B";
                    __reint_gueses_short_info( cout << setID_of__world_of_u << "_T_" << world_of_u << "* odmT_" << u <<","; )
                    update_opt_variable_with( u, wset_T_u );

                }
                __reint_gueses_short_info( cout << TermColor::RESET(); )

            } else if( !_in_change_set &&  _before_solveduntil ) {
                if( ____solvedUntil == 0 ) {
                    __reint_gueses_short_info( cout << TermColor::iYELLOW() << "X" << TermColor::RESET(); )
                    Matrix4d w_M_u = manager->getNodePose(u);
                    update_opt_variable_with( u, w_M_u );
                    continue;
                }


                // skip updating the initial guess of the node
                __reint_gueses_short_info( cout << TermColor::iYELLOW() << "|" << TermColor::RESET(); )

            } else if( !_in_change_set &&  !_before_solveduntil ) {

                __reint_gueses_short_info( cout << TermColor::GREEN(); )
                if( ____solvedUntil_worldid == world_of_u ) {
                    Matrix4d w_M_last = manager->getNodePose( ____solvedUntil );
                    Matrix4d w_M_u = manager->getNodePose(u);
                    Matrix4d last_M_u = w_M_last.inverse() * w_M_u;
                    Matrix4d w_T_last = this->getNodePose( ____solvedUntil );
                    Matrix4d w_TM_u = w_T_last * last_M_u;
                    // cout << "A";
                    __reint_gueses_short_info( cout << "P_" << ____solvedUntil << "*" << ____solvedUntil << "_M_" << u << ","; )
                    update_opt_variable_with( u, w_TM_u );
                } else {
                    Matrix4d wset_T_u = wset_T_w * manager->getNodePose(u);
                    // cout << "B";
                    __reint_gueses_short_info( cout << setID_of__world_of_u << "_T_" << world_of_u << "* odmT_" << u <<","; )
                    update_opt_variable_with( u, wset_T_u );

                }
                __reint_gueses_short_info( cout << TermColor::RESET(); )

            }

            continue;


        }

        ___trigger_header( cout << "\n[ETA initial_guesses] done in milli-secs=" << eta.toc_milli() << " node_len="<< node_len << endl; )


        //-----------------------------
        //  -5- Mark nodes as constant. Possibly also need to mark starts of each worlds as constants. Can also try setting each sets 1st node as constant.
        //------------------------------
        #if 1
        eta.tic();
        for( int v=0 ; v<regularization_terms_list.size() ; v++ ) {
            reint_problem.RemoveResidualBlock( regularization_terms_list[v] );
            __reint_node_regularization_info( cout << "Remove " << v << " th regularization term\n"; )
        }
        regularization_terms_list.clear();

        std::map< int, bool > mark_as_constant;
        // mark_as_constant[0] = true;
        mark_as_constant[1] = true;
        __reint_node_regularization_info(
        cout << "There are " << manager->n_worlds() << " worlds. Loop through each world to get its start node\n";
        cout << "Number of opt_nodes = " << this->n_opt_variables() << endl;
        )
        string reg_debug_info = "";
        for( int ww=0 ; ww<manager->n_worlds(); ww++ ) {
            // if( mark_as_constant.count(ww) ==0  )
                // continue;
            int ww_setid = manager->getWorldsConstPtr()->find_setID_of_world_i(ww);
            int ww_start = manager->nodeidx_of_world_i_started( ww );
            int ww_end = manager->nodeidx_of_world_i_ended( ww );
            if( ww_start < 0 ) {
                __reint_node_regularization_info( cout << TermColor::RED() << "ignore this world since ww_start is negative. Shouldnt be happening but..:(\n" );
                continue;
            }
            __reint_node_regularization_info( cout << TermColor::CYAN() << "&&&&&&&&&&&&world#" << ww << " is in setID=" << ww_setid << " with ww_start=" << ww_start << " ww_end=" << ww_end << TermColor::RESET() << endl; )
            if( (ww_setid >= 0 && ww_setid==ww)  )
            {
            __reint_node_regularization_info( cout << "Mark node#" << ww_start << " as constant. \n"; )
            reg_debug_info += "Mark node#" + to_string(ww_start) + " as constant \n";

            // reint_problem.SetParameterBlockConstant(  get_raw_ptr_to_opt_variable_q(ww_start) );
            // reint_problem.SetParameterBlockConstant(  get_raw_ptr_to_opt_variable_t(ww_start)  );

            // Matrix4d ww_start_pose = manager->getNodePose(ww_start);
            // Functionally similar to marking it as constant
            double regularization_weight = max( 1.1, log( 1+ww_end - ww_start )/2. );

            for( int s=0; s<3 ; s++ )
            {
            __reint_node_regularization_info( cout << "s="<< s << " "; )
            Matrix4d ww_start_pose = this->getNodePose(ww_start+s);
            // Matrix4d ww_start_pose = manager->getNodePose(ww_start+s);
            __reint_node_regularization_info( cout << "regularization_weight=" << regularization_weight << "   ww_start_pose : " << PoseManipUtils::prettyprintMatrix4d( ww_start_pose ) << endl; )
            ceres::CostFunction * regularixa_cost = NodePoseRegularization::Create( ww_start_pose, regularization_weight );
            ceres::ResidualBlockId resi_id = reint_problem.AddResidualBlock( regularixa_cost, NULL,  get_raw_ptr_to_opt_variable_q(ww_start+s), get_raw_ptr_to_opt_variable_t(ww_start+s) );
            regularization_terms_list.push_back( resi_id );
            }

            }
            else {
                __reint_node_regularization_info( cout << "skip\n"; )
            }


            #if 0
            // check if this world is in change set aka `changes_to_setid_on_set_union`. Add regularization if not in changeset
            if( changes_to_setid_on_set_union.count( ww ) > 0  ) {
                __reint_node_regularization_info( cout << "World#" << ww << " is in `changes_to_setid_on_set_union`. This indicates it was changed.So no regularization needed\n"; )
            }
            else {
                if( ww == manager->n_worlds()-1 ) // dont add regularization for current world
                    continue;
                __reint_node_regularization_info( cout << "Add regularization to each node of world#" << ww << endl; )
                for( int q=ww_start ; q<=ww_end ; q++ ) {
                    Matrix4d ww_start_pose = this->getNodePose(q);
                    ceres::CostFunction * regularixa_cost = NodePoseRegularization::Create( ww_start_pose, 1.0 );
                    ceres::ResidualBlockId resi_id = reint_problem.AddResidualBlock( regularixa_cost, NULL,  get_raw_ptr_to_opt_variable_q(q), get_raw_ptr_to_opt_variable_t(q) );
                    regularization_terms_list.push_back( resi_id );
                }
            }
            #endif

        }
        ___trigger_header( cout << "\n[ETA Total elements in `regularization_terms_list` = " << regularization_terms_list.size() << "   done in milli-secs=" << eta.toc_milli() << " node_len="<< node_len << endl; )
        ___trigger_header( cout << "Debug Info: " << reg_debug_info << "Done with debug info\n"; )
        #endif


        changes_to_setid_on_set_union.clear(); //< this is important to clear. When the setID changes (upon union_sets), this map is filled in to indicate the changes. The node initialization uses this to selectively reinitialize old node.

        //-----------------------
        //-6- ceres::Solve()
        //-----------------------
        #if 1
        ___trigger_header( cout << TermColor::iGREEN() << "=====================Ceres::Solve()============\n" << TermColor::RESET() ; )
        __reint_ceres_solve_info( cout << TermColor::iGREEN()  << "ceres::Solve()\n"; )

        reinit_ceres_problem_onnewloopedge_optimize6DOF_status = 2; // solving in progress.
        eta.tic();
        {
            std::lock_guard<std::mutex> lk(mutex_opt_vars);
            ceres::Solve( reint_options, &reint_problem, &reint_summary );

            __reint_ceres_solve_info( cout << "summary.termination_type = "<< (ceres::TerminationType::CONVERGENCE == reint_summary.termination_type) << endl; )
            // if( reint_summary.termination_type == ceres::TerminationType::CONVERGENCE )
                solved_until = node_len-1;
                __reint_ceres_solve_info( cout << "solved_until:=" << node_len-1 << endl; )

            if( reint_summary.termination_type == ceres::TerminationType::CONVERGENCE )
                n_solve_convergences++;
        }
        // cout << reint_summary.FullReport() << endl;
        ___trigger_header( cout << TermColor::iGREEN() << "Solve() took (milli-sec) : " << eta.toc_milli() << "\n" << TermColor::RESET() ; )


        __reint_ceres_solve_info(
        cout << "Solve() took (milli-sec) : " << eta.toc_milli()  << endl;
        cout << reint_summary.BriefReport() << endl;
        cout << TermColor::RESET() << endl;
        )
        #endif


        }







        // Bookkeeping
        reinit_ceres_problem_onnewloopedge_optimize6DOF_status = 0;
        prev_loopedge_len = loopedge_len;
        prev_node_len = node_len;
        loop_rate.sleep();
    }


    cout << TermColor::BLUE() << "Done with thread. returning from `PoseGraphSLAM::reinit_ceres_problem_onnewloopedge_optimize6DOF`" << TermColor::RESET() << endl;
    cout << "n_solve_convergences = " << n_solve_convergences << "\tn_solves="<< trigger_id <<  endl;

    #if 0  // make this '1' to print the current info on world starts/ends
    print_worlds_info(2);
    #endif

}
