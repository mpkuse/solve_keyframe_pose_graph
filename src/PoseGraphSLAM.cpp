#include "PoseGraphSLAM.h"

PoseGraphSLAM::PoseGraphSLAM( NodeDataManager* _manager )
{
    manager = _manager;
    mutex_opt_vars = new std::mutex();
    solved_until = 0;
}

// Return reference instead of a copy
Matrix4d PoseGraphSLAM::getNodePose( int i )
{
    assert( i>=0 && i <opt_quat.size() );
    // mutex_opt_vars->lock();
    Matrix4d w_T_cam;
    PoseManipUtils::raw_xyzw_to_eigenmat( opt_quat[i], opt_t[i], w_T_cam );
    // mutex_opt_vars->unlock();
    return w_T_cam;
}

void PoseGraphSLAM::getAllNodePose( vector<Matrix4d>& w_T_ci )
{
    w_T_ci.clear();
    for( int i=0 ; i<nNodes() ; i++ )
    {
        w_T_ci.push_back( getNodePose(i) );
    }
}

int PoseGraphSLAM::nNodes()
{
    int n;
    mutex_opt_vars->lock();
    n = opt_quat.size();
    mutex_opt_vars->unlock();
    return n;
}


void PoseGraphSLAM::opt_pose( int i, Matrix4d& out_w_T_nodei )
{
    assert( i < nNodes() );

    // qi,ti --> w_T_ci
    Eigen::Map<const Eigen::Quaternion<double> > q_1( opt_quat[i] );
    Eigen::Map<const Eigen::Matrix<double,3,1> > p_1( opt_t[i] );

    out_w_T_nodei = Matrix4d::Identity();
    out_w_T_nodei.topLeftCorner<3,3>() = q_1.toRotationMatrix();
    out_w_T_nodei.col(3).head(3) = p_1;
}


/*
#define _DEBUG_LVL_optimize6DOF 1
void PoseGraphSLAM::optimize6DOF()
{
    Color::Modifier fg_red(Color::Code::FG_RED);
    Color::Modifier fg_green(Color::Code::FG_GREEN);
    Color::Modifier fg_blue(Color::Code::FG_BLUE);
    Color::Modifier fg_def(Color::Code::FG_DEFAULT);




    int nodesize, old_nodesize; //even though this says size, it is actually indices. Consider renaming. TODO
    int edgesize, old_edgesize;
    old_nodesize = manager->getNodeLen();
    old_edgesize = manager->getEdgeLen();

    ceres::Problem problem;

    // Set Solver Options
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    // options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 8;


    // ceres::LocalParameterization * quaternion_parameterization = new ceres::QuaternionParameterization;
    ceres::LocalParameterization * eigenquaternion_parameterization = new ceres::EigenQuaternionParameterization;
    ceres::LossFunction * loss_function = new ceres::CauchyLoss(1.0);


    // There are 3 parts :
    // while( )
    //      step-1: Add optimization variables (if new nodes available with manager)
    //      step-2: Add Odometry Edges (if new available)
    //      step-3: Add Loop Edges (if new loops available)

    while( ros::ok() )
    {
        nodesize = manager->getNodeLen();
        edgesize = manager->getEdgeLen();
        // if new nodes are available add odometry edges
            //continue;
        if( nodesize > old_nodesize ) //==> new nodes
        {
            cout << fg_blue << "New nodes (#" << nodesize - old_nodesize << ")\n" << fg_def ;

            ////////////////////////// Add optimization variables ///////////////////////
            cout << fg_blue << "New Optimization Variables for nodes: " << old_nodesize << " to " << nodesize-1 << endl;;
            for(int u=old_nodesize; u<nodesize ; u++ )
            {
                // cout << ", u="<< u ;
                // cout << "old_nodesize="<< old_nodesize << " u="<<u <<";\n";
                double * __quat = new double[4];
                double * __tran = new double[3];


                // init optimization variables w_T_cam
                // if( true )
                if( old_nodesize == 0 )
                {   // Original code: Set the initial guess as the VIO poses.
                    // This is note correct if solve() has changed some of the poses in the past. Best is to rely only on relative poses from VIO
                    Matrix4d w_M_u;
                    bool status0 = manager->getNodePose(u, w_M_u);
                    assert( status0 );
                    // PoseManipUtils::eigenmat_to_raw( w_M_u, __quat, __tran );
                    PoseManipUtils::eigenmat_to_raw_xyzw( w_M_u, __quat, __tran );
                }
                else
                {
                    // M : uncorrected poses
                    // T : corrected poses

                    // Note:
                    // Say in the previous run 0-248 are optimized and there are new nodes
                    // from 249 to 290 whose VIO exists but not yet taken into ceres.
                    // wTM_260 = w_T_248 * 248_M_260
                    //         = w_T_248 * 248_M_w * w_M_260
                    //         = w_T_248 * w_M_280.inv() * w_M_260

                    // Getting the relative pose between current last corrected one
                    Matrix4d w_M_last;
                    bool status0 = manager->getNodePose( old_nodesize-1, w_M_last );

                    Matrix4d w_M_u;
                    bool status1 = manager->getNodePose(u, w_M_u);

                    assert( status0 && status1 );
                    Matrix4d last_M_u = w_M_last.inverse() * w_M_u;


                    // Getting pose of last corrected one (in world frame)
                    Matrix4d w_T_last;
                    // PoseManipUtils::raw_to_eigenmat( opt_quat[old_nodesize-1], opt_t[old_nodesize-1], w_T_last );
                    PoseManipUtils::raw_xyzw_to_eigenmat( opt_quat[old_nodesize-1], opt_t[old_nodesize-1], w_T_last );

                    Matrix4d w_TM_u = w_T_last * last_M_u;

                    // PoseManipUtils::eigenmat_to_raw( w_TM_u, __quat, __tran );
                    PoseManipUtils::eigenmat_to_raw_xyzw( w_TM_u, __quat, __tran );

                }


                mutex_opt_vars->lock();
                opt_quat.push_back( __quat );
                opt_t.push_back( __tran );
                mutex_opt_vars->unlock();

                problem.AddParameterBlock( __quat, 4 );
                // problem.SetParameterization( __quat,  quaternion_parameterization );
                problem.SetParameterization( __quat,  eigenquaternion_parameterization );
                problem.AddParameterBlock( __tran, 3 );
            }
            cout << fg_def << endl;


            //////////////////// Error from Odometry Edges //////////////////////
            // Add residue blocks for odometry
            for( int u=old_nodesize; u<nodesize ; u++ )
            {
                #if _DEBUG_LVL_optimize6DOF >= 2
                cout << fg_red << "Odometry Edge: " << fg_def;
                #endif
                for( int f=1 ; f<6 ; f++ )
                {
                    if( u-f < 0 )
                        continue;
                    #if _DEBUG_LVL_optimize6DOF >= 2
                    cout << u << "<-->" << u-f << "    ";
                    #endif

                    Matrix4d w_M_u, w_M_umf;
                    bool status0 = manager->getNodePose(u, w_M_u);
                    bool status1 = manager->getNodePose( u-f, w_M_umf );
                    assert( status0 && status1 );

                    w_M_umf =  w_M_u.inverse() * w_M_umf ;
                    ceres::CostFunction * cost_function = SixDOFError::Create( w_M_umf);
                    problem.AddResidualBlock( cost_function, loss_function, opt_quat[u], opt_t[u],  opt_quat[u-f], opt_t[u-f] );
                }
                #if _DEBUG_LVL_optimize6DOF >= 2
                cout << endl;
                #endif

                if( u==0 )
                {

                    problem.SetParameterBlockConstant(  opt_quat[0] );
                    problem.SetParameterBlockConstant(  opt_t[0]  );
                }

            }
        }
        else
        {
            #if _DEBUG_LVL_optimize6DOF >= 3
            cout << "No new nodes\n";
            #endif
        }
        old_nodesize = nodesize;


        ///////////////////// Error from Loop Closure Edges //////////////////////////////

        // if new edges are available add loop edges
            //solve()
        if( edgesize > old_edgesize )
        {
            cout << fg_green << "New Loop Closure Edges (#" << edgesize - old_edgesize << ")\n" << fg_def;

            for( int u=old_edgesize ; u<edgesize ; u++ )
            {
                std::pair<int,int> p;
                bool status0 = manager->getEdgeIdxInfo( u, p );
                Matrix4d pTc;
                bool status1 = manager->getEdgePose( u, pTc );

                assert( status0 && status1 );
                assert( p.first >=0  && p.second >=0 );
                assert( p.first < opt_quat.size() );
                assert(  p.second < opt_quat.size() );

                cout << fg_green << "Add Loop Closure "<< u << " : " << p.first << "<-->" << p.second << " | ";
                string _tmp;
                PoseManipUtils::prettyprintPoseMatrix( pTc, _tmp );
                cout << _tmp <<  fg_def << endl;

                ceres::CostFunction * cost_function = SixDOFError::Create( pTc );
                problem.AddResidualBlock( cost_function, loss_function, opt_quat[p.first], opt_t[p.first],
                                                                opt_quat[p.second], opt_t[p.second]  );
            }

            cout << "solve()\n";
            // mutex_opt_vars->lock();
            ceres::Solve( options, &problem, &summary );
            solved_until = nodesize;
            // // mutex_opt_vars->unlock();
            // cout << summary.FullReport() << endl;
            cout << summary.BriefReport() << endl;
            cout << "Solve() took " << summary.total_time_in_seconds << " sec"<< endl;
            cout << "Poses are Optimized from 0 to "<< old_nodesize << endl;
            cout << "New nodes in manager which are not yet taken here from idx "<< old_nodesize << " to "<< manager->getNodeLen() << endl;
        }
        else
        {
            #if _DEBUG_LVL_optimize6DOF >= 3
            cout << "No new loop closure edges\n";
            #endif
        }
        old_edgesize = edgesize;



        std::this_thread::sleep_for( std::chrono::milliseconds(1000) );
    }



    // Deallocate optimization variables
    assert( opt_quat.size() == opt_t.size() );
    for( int i=0 ; i<opt_t.size() ; i++ )
    {
        delete [] opt_quat[i];
        delete [] opt_t[i];
    }

}

*/


#define _DEBUG_LVL_optimize6DOF 1
void PoseGraphSLAM::optimize6DOF()
{
    Color::Modifier fg_red(Color::Code::FG_RED);
    Color::Modifier fg_green(Color::Code::FG_GREEN);
    Color::Modifier fg_blue(Color::Code::FG_BLUE);
    Color::Modifier fg_def(Color::Code::FG_DEFAULT);
    cout << "#####################################\n";
    cout << fg_blue<< "### PoseGraphSLAM::optimize6DOF() ###\n" << fg_def;
    cout << "### \tinf_loop="<<inf_loop << "###\n"; //if this is set to true (default) will run a infinite loop to monitor the manager.
    cout << "#####################################\n";



    int nodesize, old_nodesize; //even though this says size, it is actually indices. Consider renaming. TODO
    int edgesize, old_edgesize;
    old_nodesize = 0;//manager->getNodeLen();
    old_edgesize = 0;//manager->getEdgeLen();

    ceres::Problem problem;

    // Set Solver Options
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    // options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 8;


    // ceres::LocalParameterization * quaternion_parameterization = new ceres::QuaternionParameterization;
    ceres::LocalParameterization * eigenquaternion_parameterization = new ceres::EigenQuaternionParameterization;
    ceres::LossFunction * loss_function = new ceres::CauchyLoss(1.0);


    // There are 3 parts :
    // while( )
    //      step-1: Add optimization variables (if new nodes available with manager)
    //      step-2: Add Odometry Edges (if new available)
    //      step-3: Add Loop Edges (if new loops available)

    while( ros::ok() )
    {
        nodesize = manager->getNodeLen();
        edgesize = manager->getEdgeLen();
        // if new nodes are available add odometry edges
            //continue;
        if( nodesize > old_nodesize ) //==> new nodes
        {
            cout << fg_blue << "New nodes (#" << nodesize - old_nodesize << ")\n" << fg_def ;

            ////////////////////////// Add optimization variables ///////////////////////
            cout << fg_blue << "New Optimization Variables for nodes: " << old_nodesize << " to " << nodesize-1 << endl;;
            for(int u=old_nodesize; u<nodesize ; u++ )
            {
                // cout << ", u="<< u ;
                // cout << "old_nodesize="<< old_nodesize << " u="<<u <<";\n";
                double * __quat = new double[4];
                double * __tran = new double[3];


                // init optimization variables w_T_cam
                // if( true )
                if( old_nodesize == 0 )
                {   // Original code: Set the initial guess as the VIO poses.
                    // This is note correct if solve() has changed some of the poses in the past. Best is to rely only on relative poses from VIO
                    Matrix4d w_M_u;
                    bool status0 = manager->getNodePose(u, w_M_u);
                    assert( status0 );
                    // PoseManipUtils::eigenmat_to_raw( w_M_u, __quat, __tran );
                    PoseManipUtils::eigenmat_to_raw_xyzw( w_M_u, __quat, __tran );
                }
                else
                {
                    // M : uncorrected poses
                    // T : corrected poses

                    // Note:
                    // Say in the previous run 0-248 are optimized and there are new nodes
                    // from 249 to 290 whose VIO exists but not yet taken into ceres.
                    // wTM_260 = w_T_248 * 248_M_260
                    //         = w_T_248 * 248_M_w * w_M_260
                    //         = w_T_248 * w_M_280.inv() * w_M_260

                    // Getting the relative pose between current last corrected one
                    Matrix4d w_M_last;
                    bool status0 = manager->getNodePose( old_nodesize-1, w_M_last );

                    Matrix4d w_M_u;
                    bool status1 = manager->getNodePose(u, w_M_u);

                    assert( status0 && status1 );
                    Matrix4d last_M_u = w_M_last.inverse() * w_M_u;


                    // Getting pose of last corrected one (in world frame)
                    Matrix4d w_T_last;
                    // PoseManipUtils::raw_to_eigenmat( opt_quat[old_nodesize-1], opt_t[old_nodesize-1], w_T_last );
                    PoseManipUtils::raw_xyzw_to_eigenmat( opt_quat[old_nodesize-1], opt_t[old_nodesize-1], w_T_last );

                    Matrix4d w_TM_u = w_T_last * last_M_u;

                    // PoseManipUtils::eigenmat_to_raw( w_TM_u, __quat, __tran );
                    PoseManipUtils::eigenmat_to_raw_xyzw( w_TM_u, __quat, __tran );

                }


                mutex_opt_vars->lock();
                opt_quat.push_back( __quat );
                opt_t.push_back( __tran );
                mutex_opt_vars->unlock();

                problem.AddParameterBlock( __quat, 4 );
                // problem.SetParameterization( __quat,  quaternion_parameterization );
                problem.SetParameterization( __quat,  eigenquaternion_parameterization );
                problem.AddParameterBlock( __tran, 3 );
            }
            cout << fg_def << endl;


            //////////////////// Error from Odometry Edges //////////////////////
            // Add residue blocks for odometry
            for( int u=old_nodesize; u<nodesize ; u++ )
            {
                #if _DEBUG_LVL_optimize6DOF >= 2
                cout << fg_red << "Odometry Edge: " << fg_def;
                #endif
                for( int f=1 ; f<6 ; f++ )
                {
                    if( u-f < 0 )
                        continue;
                    #if _DEBUG_LVL_optimize6DOF >= 2
                    cout << u << "<-->" << u-f << "    ";
                    #endif

                    Matrix4d w_M_u, w_M_umf;
                    bool status0 = manager->getNodePose(u, w_M_u);
                    bool status1 = manager->getNodePose( u-f, w_M_umf );
                    assert( status0 && status1 );

                    w_M_umf =  w_M_u.inverse() * w_M_umf ;
                    ceres::CostFunction * cost_function = SixDOFError::Create( w_M_umf);
                    problem.AddResidualBlock( cost_function, loss_function, opt_quat[u], opt_t[u],  opt_quat[u-f], opt_t[u-f] );

                    // TODO: Get measure of how accurate was the local odometry. make it a co-variance matrix of size 6x6 and
                    //       and scale the residue block with the squareroot of the co-variance matrix.
                }
                #if _DEBUG_LVL_optimize6DOF >= 2
                cout << endl;
                #endif

                if( u==0 )
                {

                    problem.SetParameterBlockConstant(  opt_quat[0] );
                    problem.SetParameterBlockConstant(  opt_t[0]  );
                }

            }
        }
        else
        {
            #if _DEBUG_LVL_optimize6DOF >= 3
            cout << "No new nodes\n";
            #endif
        }
        old_nodesize = nodesize;


        ///////////////////// Error from Loop Closure Edges //////////////////////////////

        // if new edges are available add loop edges
            //solve()
        if( edgesize > old_edgesize )
        {
            cout << fg_green << "New Loop Closure Edges (#" << edgesize - old_edgesize << ")\n" << fg_def;

            for( int u=old_edgesize ; u<edgesize ; u++ )
            {
                std::pair<int,int> p;
                bool status0 = manager->getEdgeIdxInfo( u, p );
                Matrix4d pTc;
                bool status1 = manager->getEdgePose( u, pTc );
                double closure_loop_weight = manager->getEdgeWeight(u);
                assert( closure_loop_weight >= 0. && "Loop closure weight need to be non-negative");
                // TODO: Get edge weight, make it a co-variance matrix of sixe 6x6 and
                //       and scale the residue block with the squareroot of the co-variance matrix.

                assert( status0 && status1 );
                assert( p.first >=0  && p.second >=0 );
                assert( p.first < opt_quat.size() );
                assert(  p.second < opt_quat.size() );

                cout << fg_green << "Add Loop Closure "<< u << " : " << p.first << "<-->" << p.second << " | ";
                string _tmp;
                PoseManipUtils::prettyprintPoseMatrix( pTc, _tmp );
                cout << _tmp <<  fg_def << endl;

                ceres::CostFunction * cost_function = SixDOFError::Create( pTc, closure_loop_weight );
                problem.AddResidualBlock( cost_function, loss_function, opt_quat[p.first], opt_t[p.first],
                                                                opt_quat[p.second], opt_t[p.second]  );
            }

            cout << "solve()\n";
            // mutex_opt_vars->lock();
            ceres::Solve( options, &problem, &summary );
            solved_until = nodesize;
            // // mutex_opt_vars->unlock();
            // cout << summary.FullReport() << endl;
            cout << summary.BriefReport() << endl;
            cout << "Solve() took " << summary.total_time_in_seconds << " sec"<< endl;
            cout << "Poses are Optimized from 0 to "<< old_nodesize << endl;
            cout << "New nodes in manager which are not yet taken here from idx "<< old_nodesize << " to "<< manager->getNodeLen() << endl;
        }
        else
        {
            #if _DEBUG_LVL_optimize6DOF >= 3
            cout << "No new loop closure edges\n";
            #endif
        }
        old_edgesize = edgesize;

        if( inf_loop == false )
            return;

        std::this_thread::sleep_for( std::chrono::milliseconds(1000) );
    }



    // Deallocate optimization variables
    mutex_opt_vars->lock();

    assert( opt_quat.size() == opt_t.size() );

    for( int i=0 ; i<opt_t.size() ; i++ )
    {
        delete [] opt_quat[i];
        delete [] opt_t[i];
    }
    opt_quat.clear();
    opt_t.clear();
    mutex_opt_vars->unlock();


}


void PoseGraphSLAM::deallocate_optimization_variables()
{
    mutex_opt_vars->lock();
    assert( opt_quat.size() == opt_t.size() );

    for( int i=0 ; i<opt_t.size() ; i++ )
    {
        delete [] opt_quat[i];
        delete [] opt_t[i];
    }
    opt_quat.clear();
    opt_t.clear();


    mutex_opt_vars->unlock();
}
