#include "PoseGraphSLAM.h"

PoseGraphSLAM::PoseGraphSLAM( const NodeDataManager* _manager ): manager( _manager )
{
    solved_until = 0;
}

//################################################################################
//############## Public Interfaces to retrive optimized poses ####################
//################################################################################

// It is on purpose I am returning const Matrix4d and not const Matrix4d&
const Matrix4d PoseGraphSLAM::getNodePose( int i ) const
{
    std::lock_guard<std::mutex> lk(mutex_opt_vars);
    assert( i>=0 && i <opt_quat.size() );

    Matrix4d w_T_cam;
    PoseManipUtils::raw_xyzw_to_eigenmat( opt_quat[i], opt_t[i], w_T_cam );

    return w_T_cam;
}

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
    return opt_quat.size();
}


void PoseGraphSLAM::getNodePose( int i, Matrix4d& out_w_T_nodei ) const
{
    std::lock_guard<std::mutex> lk(mutex_opt_vars);

    assert( i < nNodes() );

    // qi,ti --> w_T_ci
    Eigen::Map<const Eigen::Quaternion<double> > q_1( opt_quat[i] );
    Eigen::Map<const Eigen::Matrix<double,3,1> > p_1( opt_t[i] );

    out_w_T_nodei = Matrix4d::Identity();
    out_w_T_nodei.topLeftCorner<3,3>() = q_1.toRotationMatrix();
    out_w_T_nodei.col(3).head(3) = p_1;
}

void PoseGraphSLAM::allocate_and_append_new_opt_variable_withpose( const Matrix4d& pose )
{
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
}

const int PoseGraphSLAM::n_opt_variables( ) const
{
    std::lock_guard<std::mutex> lk(mutex_opt_vars);
    return opt_quat.size();
}

double * PoseGraphSLAM::get_raw_ptr_to_opt_variable_q( int i ) const
{
    assert( i>=0 && i<n_opt_variables() );
    return opt_quat[i];
}

double * PoseGraphSLAM::get_raw_ptr_to_opt_variable_t( int i ) const
{
    assert( i>=0 && i<n_opt_variables() );
    return opt_t[i];
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
        Matrix4d w_T_c_opt;
        getNodePose( i, w_T_c_opt );

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



        all_info["PoseGraphSLAM_nodes"].push_back( opt_var_i );
    }


    for( int i=0 ; i<manager->getEdgeLen() ; i++ ) {
        json edgeinfo;

        // bTa with odometry
        edgeinfo["getEdge_i"] = i;
        int a = manager->getEdgeIdxInfo( i ).first;
        int b = manager->getEdgeIdxInfo( i ).second;
        edgeinfo["a"] = a;
        edgeinfo["b"] = b;
        edgeinfo["weight"] = manager->getEdgeWeight(i);
        edgeinfo["description_string"] = manager->getEdgeDescriptionString(i);

        edgeinfo["getEdgePose"] = PoseManipUtils::prettyprintMatrix4d( manager->getEdgePose( i ) );

        auto getEdgePose_after_opt = this->getNodePose(b).inverse() * this->getNodePose(a);
        edgeinfo["getEdgePose_after_opt"] = PoseManipUtils::prettyprintMatrix4d( getEdgePose_after_opt );

        all_info["PoseGraphSLAM_edgeinfo"].push_back( edgeinfo );

    }

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
    std::lock_guard<std::mutex> lk(mutex_opt_vars);
    assert( opt_quat.size() == opt_t.size() );

    for( int i=0 ; i<opt_t.size() ; i++ )
    {
        delete [] opt_quat[i];
        delete [] opt_t[i];
    }
    opt_quat.clear();
    opt_t.clear();


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
// #define __PoseGraphSLAM_new_optimize6DOF_odom_debug( msg ) ;


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
    while( new_optimize6DOF_isEnabled )
    {
        cout << "---\n";
        // cout << "[PoseGraphSLAM::new_optimize6DOF]\n";
        // cout << "manager->print_nodes_lengths()\t"; manager->print_nodes_lengths();
        cout << "PoseGraphSLAM::n_opt_variables = " << this->n_opt_variables() << endl;
        node_len = manager->getNodeLen();
        loopedge_len = manager->getEdgeLen();


        //-------------------
        // Are there any new nodes ?
        //      if yes than a) add new optimization variables b) add odometry edges to the optimization problem
        //-------------------
        if( node_len > prev_node_len )
        {
            __PoseGraphSLAM_new_optimize6DOF_odom(
            cout << TermColor::CYAN() << "there are " << node_len - prev_node_len << " new nodes\t";
            cout << " from [" << prev_node_len << ", " << node_len-1 << "]" << TermColor::RESET() << endl;
            )

            //###################
            // Add new optimization variables and initialize them correctly.
            //###################
            __PoseGraphSLAM_new_optimize6DOF_odom_debug(
            cout << TermColor::CYAN() << "Add new optimization variables for each node and initialize them correctly. push_back optimization variables for each of these" << TermColor::RESET() << endl;
            )
            for( int u=prev_node_len ; u<node_len ; u++ ) {
                if( u==0 ) { //0th node.
                    allocate_and_append_new_opt_variable_withpose( manager->getNodePose(u) );
                }
                else {
                    // TODO think more, probably need to use solved_until and init the pose of opt variables accordingly
                    #if 0
                    allocate_and_append_new_opt_variable_withpose( manager->getNodePose(u) );
                    #else

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
                    Matrix4d w_TM_u = w_T_last * last_M_u;

                    allocate_and_append_new_opt_variable_withpose( w_TM_u );


                    #endif


                }

                __PoseGraphSLAM_new_optimize6DOF_odom_debug( cout << u << "\t" );
                problem.AddParameterBlock( get_raw_ptr_to_opt_variable_q(u), 4 );
                problem.SetParameterization( get_raw_ptr_to_opt_variable_q(u),  eigenquaternion_parameterization );
                problem.AddParameterBlock( get_raw_ptr_to_opt_variable_t(u), 3 );
                if( u==0 ) {
                    problem.SetParameterBlockConstant(  opt_quat[0] );
                    problem.SetParameterBlockConstant(  opt_t[0]  );
                }
            }
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

                }
                __PoseGraphSLAM_new_optimize6DOF_odom_debug( cout << "\\n" << endl; )
            }

        }


        //-------------------
        // Are there any new loopedges ?
        //      if yes than add loop edges to the optimization problem
        //-------------------
        if( loopedge_len > prev_loopedge_len )
        {
            cout << TermColor::MAGENTA() << "there are " << loopedge_len - prev_loopedge_len << " new loopedges\t";
            cout << " from [" << prev_loopedge_len << ", " << loopedge_len-1 << "]" << TermColor::RESET() << endl;

            //##############################
            // TODO Add loopedges residues for each edge
            //##############################
            cout <<  TermColor::MAGENTA() << "Add loopedges from [" << prev_loopedge_len << ", " << loopedge_len-1  << "]"<< TermColor::RESET() << endl;
            for( int e=prev_loopedge_len ; e<loopedge_len ; e++ ) {
                MatrixXd bTa = manager->getEdgePose(e);
                double weight = manager->getEdgeWeight(e);
                auto paur = manager->getEdgeIdxInfo(e);

                cout << "\t##loopedge e=" << e << "\t";
                cout << "(a,b)==" << paur.first << "<-->" << paur.second << "   weight=" << std::setprecision(4) << weight;
                cout << "\n\tdescription_string=" << manager->getEdgeDescriptionString(e) << "\n";
                cout << "\tbTa="  << PoseManipUtils::prettyprintMatrix4d(bTa) << endl;;


                ceres::CostFunction * cost_function = SixDOFError::Create( bTa, weight );
                problem.AddResidualBlock( cost_function, robust_norm,
                    get_raw_ptr_to_opt_variable_q(paur.second), get_raw_ptr_to_opt_variable_t(paur.second),
                    get_raw_ptr_to_opt_variable_q(paur.first), get_raw_ptr_to_opt_variable_t(paur.first)  );
            }


            #if 1
            cout << TermColor::YELLOW() << "## Loopedges before Solve()\n";
            for( int e=prev_loopedge_len ; e<loopedge_len ; e++ ) {
                auto paur = manager->getEdgeIdxInfo(e);
                int _a = paur.first;
                int _b = paur.second;
                cout << _a << "<-->" << _b << "\t";
                auto getEdgePose_after_opt = this->getNodePose(_b).inverse() * this->getNodePose(_a);
                cout << "bTa(opt_vars)" << PoseManipUtils::prettyprintMatrix4d( getEdgePose_after_opt ) << endl;

                auto getEdgePose_after_opt_manager = manager->getNodePose(_b).inverse() * manager->getNodePose(_a);
                cout << std::setprecision(20) << "(manager)a_timestamp=" << manager->getNodeTimestamp( _a ).toSec() << "\t" << "b_timestamp=" << manager->getNodeTimestamp(_b).toSec() << endl;
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
            ceres::Solve( options, &problem, &summary );
            solved_until = node_len;
            // cout << summary.FullReport() << endl;
            cout << summary.BriefReport() << endl;
            cout << TermColor::RESET() << endl;
            #endif



            #if 1
            // print loopedges after optimization
            cout << TermColor::YELLOW() << "## Loopedges after Solve()\n";
            for( int e=prev_loopedge_len ; e<loopedge_len ; e++ ) {
                auto paur = manager->getEdgeIdxInfo(e);
                int _a = paur.first;
                int _b = paur.second;
                cout << _a << "<-->" << _b << "\t";
                auto getEdgePose_after_opt = this->getNodePose(_b).inverse() * this->getNodePose(_a);
                cout << "bTa" << PoseManipUtils::prettyprintMatrix4d( getEdgePose_after_opt ) << endl;
                cout << TermColor::RESET() ;
            }
            #endif

        }


        // book keeping, make sure this executes each time
        prev_node_len = node_len;
        prev_loopedge_len = loopedge_len;
        loop_rate.sleep();
    }

    cout << TermColor::BLUE() << "Done with thread. returning from `PoseGraphSLAM::new_optimize6DOF`" << TermColor::RESET() << endl;
}
