#include "PoseGraphSLAM.h"

PoseGraphSLAM::PoseGraphSLAM( NodeDataManager* _manager )
{
    manager = _manager;


}


void PoseGraphSLAM::optimize6DOF()
{
    Color::Modifier fg_red(Color::Code::FG_RED);
    Color::Modifier fg_green(Color::Code::FG_GREEN);
    Color::Modifier fg_blue(Color::Code::FG_BLUE);
    Color::Modifier fg_def(Color::Code::FG_DEFAULT);


    vector<double*> opt_quat;
    vector<double*> opt_t;

    int nodesize, old_nodesize;
    int edgesize, old_edgesize;
    old_nodesize = manager->getNodeLen();
    old_edgesize = manager->getEdgeLen();

    ceres::Problem problem;

    // Set Solver Options
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 5;
    ceres::LocalParameterization * quaternion_parameterization = new ceres::QuaternionParameterization;

    while( ros::ok() )
    {
        nodesize = manager->getNodeLen();
        edgesize = manager->getEdgeLen();
        // if new nodes are available add odometry edges
            //continue;
        if( nodesize > old_nodesize ) //==> new nodes
        {
            cout << "New nodes (#" << nodesize - old_nodesize << ")\n" ;

            // Add optimization variables
            cout << "New Optimization Variables for node: ";
            for(int u=old_nodesize; u<nodesize ; u++ )
            {
                cout << ", u="<< u ;
                double * __quat = new double[4];
                double * __tran = new double[3];
                // init optimization variables w_T_cam
                Matrix4d w_M_u;
                bool status0 = manager->getNodePose(u, w_M_u);
                assert( status0 );
                eigenmat_to_raw( w_M_u, __quat, __tran );


                opt_quat.push_back( __quat );
                opt_t.push_back( __tran );

                problem.AddParameterBlock( __quat, 4 );
                problem.SetParameterization( __quat,  quaternion_parameterization );
                problem.AddParameterBlock( __tran, 3 );
            }
            cout << endl;


            // Add residue blocks for odometry
            for( int u=old_nodesize; u<nodesize ; u++ )
            {
                cout << fg_red << "Odometry Edge: " << fg_def;
                for( int f=1 ; f<5 ; f++ )
                {
                    if( u-f < 0 )
                        continue;
                    cout << u << "<-->" << u-f << "    ";

                    Matrix4d w_M_u, w_M_umf;
                    bool status0 = manager->getNodePose(u, w_M_u);
                    bool status1 = manager->getNodePose( u-f, w_M_umf );
                    assert( status0 && status1 );
                    ceres::CostFunction * cost_function = SixDOFError::Create( w_M_u.inverse() * w_M_umf );
                    problem.AddResidualBlock( cost_function, NULL, opt_quat[u], opt_t[u],  opt_quat[u-f], opt_t[u-f] );
                }

                if( u==0 )
                {

                    problem.SetParameterBlockConstant(  opt_quat[0] );
                    problem.SetParameterBlockConstant(  opt_t[0]  );
                }
                cout << endl;
            }
        }
        else
        {
            cout << "No new nodes\n";
        }
        old_nodesize = nodesize;


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
                problem.AddResidualBlock( cost_function, NULL, opt_quat[p.first], opt_t[p.first],  opt_quat[p.second], opt_t[p.second]  );
            }

            cout << "solve()\n";
            ceres::Solve( options, &problem, &summary );
        }
        else
        {
            cout << "No new loop closure edges\n";
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




///////////////// Pose compitation related helpers /////////////////

void PoseGraphSLAM::raw_to_eigenmat( const double * quat, const double * t, Matrix4d& dstT )
{
  Quaterniond q = Quaterniond( quat[0], quat[1], quat[2], quat[3] );

  dstT = Matrix4d::Zero();
  dstT.topLeftCorner<3,3>() = q.toRotationMatrix();

  dstT(0,3) = t[0];
  dstT(1,3) = t[1];
  dstT(2,3) = t[2];
  dstT(3,3) = 1.0;
}

void PoseGraphSLAM::eigenmat_to_raw( const Matrix4d& T, double * quat, double * t)
{
  assert( T(3,3) == 1 );
  Quaterniond q( T.topLeftCorner<3,3>() );
  quat[0] = q.w();
  quat[1] = q.x();
  quat[2] = q.y();
  quat[3] = q.z();
  t[0] = T(0,3);
  t[1] = T(1,3);
  t[2] = T(2,3);
}

void PoseGraphSLAM::rawyprt_to_eigenmat( const double * ypr, const double * t, Matrix4d& dstT )
{
  dstT = Matrix4d::Identity();
  Vector3d eigen_ypr;
  eigen_ypr << ypr[0], ypr[1], ypr[2];
  dstT.topLeftCorner<3,3>() = ypr2R( eigen_ypr );
  dstT(0,3) = t[0];
  dstT(1,3) = t[1];
  dstT(2,3) = t[2];
}

void PoseGraphSLAM::eigenmat_to_rawyprt( const Matrix4d& T, double * ypr, double * t)
{
  assert( T(3,3) == 1 );
  Vector3d T_cap_ypr = R2ypr( T.topLeftCorner<3,3>() );
  ypr[0] = T_cap_ypr(0);
  ypr[1] = T_cap_ypr(1);
  ypr[2] = T_cap_ypr(2);

  t[0] = T(0,3);
  t[1] = T(1,3);
  t[2] = T(2,3);
}

Vector3d PoseGraphSLAM::R2ypr( const Matrix3d& R)
{
  Eigen::Vector3d n = R.col(0);
  Eigen::Vector3d o = R.col(1);
  Eigen::Vector3d a = R.col(2);

  Eigen::Vector3d ypr(3);
  double y = atan2(n(1), n(0));
  double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
  double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
  ypr(0) = y;
  ypr(1) = p;
  ypr(2) = r;

  return ypr / M_PI * 180.0;
}


Matrix3d PoseGraphSLAM::ypr2R( const Vector3d& ypr)
{
  double y = ypr(0) / 180.0 * M_PI;
  double p = ypr(1) / 180.0 * M_PI;
  double r = ypr(2) / 180.0 * M_PI;

  // Eigen::Matrix<double, 3, 3> Rz;
  Matrix3d Rz;
  Rz << cos(y), -sin(y), 0,
      sin(y), cos(y), 0,
      0, 0, 1;

  // Eigen::Matrix<double, 3, 3> Ry;
  Matrix3d Ry;
  Ry << cos(p), 0., sin(p),
      0., 1., 0.,
      -sin(p), 0., cos(p);

  // Eigen::Matrix<double, 3, 3> Rx;
  Matrix3d Rx;
  Rx << 1., 0., 0.,
      0., cos(r), -sin(r),
      0., sin(r), cos(r);

  return Rz * Ry * Rx;
}

void PoseGraphSLAM::prettyprintPoseMatrix( const Matrix4d& M )
{
  cout << "YPR      : " << R2ypr(  M.topLeftCorner<3,3>() ).transpose() << "; ";
  cout << "Tx,Ty,Tz : " << M(0,3) << ", " << M(1,3) << ", " << M(2,3) << endl;
}

void PoseGraphSLAM::prettyprintPoseMatrix( const Matrix4d& M, string& return_string )
{
   Vector3d ypr;
   ypr = R2ypr(  M.topLeftCorner<3,3>()  );

  char __tmp[200];
  snprintf( __tmp, 200, ":YPR=(%4.2f,%4.2f,%4.2f)  :TxTyTz=(%4.2f,%4.2f,%4.2f)",  ypr(0), ypr(1), ypr(2), M(0,3), M(1,3), M(2,3) );
  return_string = string( __tmp );
}
