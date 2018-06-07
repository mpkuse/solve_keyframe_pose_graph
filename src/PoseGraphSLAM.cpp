#include "PoseGraphSLAM.h"

PoseGraphSLAM::PoseGraphSLAM( NodeDataManager* _manager )
{
    manager = _manager;
}


void PoseGraphSLAM::optimize6DOF()
{
    vector<double*> opt_quat;
    vector<double*> opt_t;

    while( ros::ok() )
    {
        // if new nodes are available add odometry edges
            //continue;

        // if new edges are available add loop edges
            //solve()

        cout << "solve()\n";
        std::this_thread::sleep_for( std::chrono::milliseconds(1000) );
    }
}
