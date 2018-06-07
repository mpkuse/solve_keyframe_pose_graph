#include "PoseGraphSLAM.h"

PoseGraphSLAM::PoseGraphSLAM( NodeDataManager* _manager )
{
    manager = _manager;
    mutex_opt_vars = new std::mutex();
}

Matrix4d PoseGraphSLAM::getNodePose( int i )
{
    assert( i>=0 && i <opt_quat.size() );
    mutex_opt_vars->lock();
    Matrix4d w_T_cam;
    PoseManipUtils::raw_to_eigenmat( opt_quat[i], opt_t[i], w_T_cam );
    mutex_opt_vars->unlock();
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

#define _DEBUG_LVL_optimize6DOF 1
void PoseGraphSLAM::optimize6DOF()
{
    Color::Modifier fg_red(Color::Code::FG_RED);
    Color::Modifier fg_green(Color::Code::FG_GREEN);
    Color::Modifier fg_blue(Color::Code::FG_BLUE);
    Color::Modifier fg_def(Color::Code::FG_DEFAULT);




    int nodesize, old_nodesize;
    int edgesize, old_edgesize;
    old_nodesize = manager->getNodeLen();
    old_edgesize = manager->getEdgeLen();

    ceres::Problem problem;

    // Set Solver Options
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 5;
    ceres::LocalParameterization * quaternion_parameterization = new ceres::QuaternionParameterization;

    while( ros::ok() )
    {
        cout << "optimize6DOF():opt_quat.size()" <<  opt_quat.size() << endl;
        nodesize = manager->getNodeLen();
        edgesize = manager->getEdgeLen();
        // if new nodes are available add odometry edges
            //continue;
        if( nodesize > old_nodesize ) //==> new nodes
        {
            cout << fg_blue << "New nodes (#" << nodesize - old_nodesize << ")\n" << fg_def ;

            // Add optimization variables
            cout << fg_blue << "New Optimization Variables for nodes: ";
            for(int u=old_nodesize; u<nodesize ; u++ )
            {
                cout << ", u="<< u ;
                double * __quat = new double[4];
                double * __tran = new double[3];
                // init optimization variables w_T_cam
                Matrix4d w_M_u;
                bool status0 = manager->getNodePose(u, w_M_u);
                assert( status0 );
                PoseManipUtils::eigenmat_to_raw( w_M_u, __quat, __tran );

                mutex_opt_vars->lock();
                opt_quat.push_back( __quat );
                opt_t.push_back( __tran );
                mutex_opt_vars->unlock();

                problem.AddParameterBlock( __quat, 4 );
                problem.SetParameterization( __quat,  quaternion_parameterization );
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
                for( int f=1 ; f<5 ; f++ )
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
                    ceres::CostFunction * cost_function = SixDOFError::Create( w_M_u.inverse() * w_M_umf );
                    problem.AddResidualBlock( cost_function, NULL, opt_quat[u], opt_t[u],  opt_quat[u-f], opt_t[u-f] );
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
            cout << "New Loop Closure Edges (#" << edgesize - old_edgesize << ")\n" ;

            for( int u=old_edgesize ; u<edgesize ; u++ )
            {
                std::pair<int,int> p;
                bool status0 = manager->getEdgeIdxInfo( u, p );
                Matrix4d pTc;
                bool status1 = manager->getEdgePose( u, pTc );

                cout << "Add Loop Closure "<< u << " : " << p.first << "<-->" << p.second << endl;
                assert( status0 && status1 );
                assert( p.first >=0  && p.second >=0 );
                assert( p.first < opt_quat.size() );
                assert(  p.second < opt_quat.size() );


                ceres::CostFunction * cost_function = SixDOFError::Create( pTc );
                problem.AddResidualBlock( cost_function, NULL, opt_quat[p.first], opt_t[p.first],
                                                                opt_quat[p.second], opt_t[p.second]  );
            }

            cout << "solve()\n";
            mutex_opt_vars->lock();
            ceres::Solve( options, &problem, &summary );
            mutex_opt_vars->unlock();
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
